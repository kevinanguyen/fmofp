"""
Remote Terminal Socket

Handles socket communication between Remote Terminal and Bus Controller, mirroring BC_socket behavior.
"""
import json
import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeoutError
import atexit
import time
import select
import traceback
import FMOFP.Utils.common.fetching as fetching
from FMOFP.MIL_STD_1553B.mil_std_1553B import MIL_STD_1553B_Message
from FMOFP.MIL_STD_1553B.message_schemas import (
    get_schema_for_message_type,
    apply_schema_to_data_words,
    extract_fields_from_data_words,
    MODE_VALUE_MAP,
    MODE_NAME_MAP
)
from FMOFP.MIL_STD_1553B.metadata_codec import MetadataCodec
from FMOFP.MIL_STD_1553B.bus_adapter import get_bus_adapter, get_listen_endpoint
from FMOFP.Utils.common.health import socket_is_bound
from FMOFP.Utils.common.retry import BackoffPolicy
from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()

# How long an accepted connection may sit without sending anything before
# the listener reclaims it. See handle_connection() for why this exists.
CONNECTION_IDLE_TIMEOUT_S = 30.0

# MIL-STD-1553B: one message carries a command (or status) word plus at most
# 32 data words.
# Upper bound on how long a caller will wait for one transmit to finish.
# Deliberately well above BUS_SOCKET_TIMEOUT_S so that a socket-level failure
# reports itself rather than being masked by this.
# Upper bound on a whole block transfer, which is one connect-and-send cycle
# per chunk and so can be thousands of operations. Below the caller's bound so
# the worker gives up first and says how far it got.
BLOCK_TRANSFER_TIMEOUT_S = 20.0

SEND_COMPLETION_TIMEOUT_S = 30.0

MAX_DATA_WORDS_PER_MESSAGE = 32
MAX_FRAMES_PER_MESSAGE = 1 + MAX_DATA_WORDS_PER_MESSAGE          # 33

# A block-transfer chunk spends two of its data words on the sequence number
# and the total count, so only 30 are left for payload.
#
# This used to be 32, and the chunk was then built as
#     [command_word] + [sequence_word, total_word] + 32 payload frames
# which is 35 frames -- above the 33-frame threshold that sends a message down
# the block-transfer path in the first place. So every chunk of a block
# transfer was itself "large" and re-entered the split, which produced another
# 35-frame chunk, forever. Computed over the real arithmetic: 35 frames splits
# into chunks of 35 and 5, and the 35 splits into 35 and 5 again -- a fixed
# point, not a reduction.
#
# The practical effect was that ANY bus message with more than 33 frames never
# completed. Reproduced against a peer that accepts and never reads: eleven
# nested _send_large_message frames on the stack, eleven sockets held open,
# and the caller never returned.
BLOCK_TRANSFER_HEADER_WORDS = 2
MAX_PAYLOAD_FRAMES_PER_CHUNK = (
    MAX_DATA_WORDS_PER_MESSAGE - BLOCK_TRANSFER_HEADER_WORDS)    # 30


# H3: the accept loop used to call handle_connection() inline, so the listener
# served exactly one connection at a time and nothing else was accepted until
# that peer went away. Connections are now served on worker threads, bounded so
# a flood cannot spawn threads without limit. Eight is far above the two peers
# this bus actually has (one BC, one RT); it is a ceiling, not a target.
MAX_CONCURRENT_CONNECTIONS = 8

class RT_sender:
    """
    Remote Terminal sender class that mirrors BC_sender behavior.
    Handles communication from Remote Terminal to Bus Controller.
    """
    def __init__(self, max_workers=5):
        self.destination_ip = "localhost"
        self.destination_port = 5000  # BC_Listener is listening on this port
        # Transport is now behind the bus adapter layer (bus_adapter.py) so a
        # real 1553B hardware interface can be swapped in via
        # busAdapterConfig.xml without touching this class. The default
        # SocketBusAdapter reproduces the previous inline socket code exactly
        # (destination_ip/destination_port above are kept for backwards
        # compatibility of anything that reads them; the adapter owns the
        # live values).
        self._bus_adapter = get_bus_adapter('rt')
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.is_shutting_down = False
        self.shutdown_event = threading.Event()
        self.acknowledged_requests = {}  # Track acknowledged requests
        self.acknowledged_requests_lock = threading.Lock()  # Lock for thread safety
        atexit.register(self.shutdown)
        logger.info(f"RT_sender initialized with max_workers={max_workers}")

    def RT_send_message(self, message):
        """
        Send message to BC, mirroring BC_send_message behavior.
        """
        if self.is_shutting_down or self.executor._shutdown:
            logger.warning("Cannot send message, RT_sender is shutting down")
            return False
            
        logger.info(f"RT_sender submitting message to executor: {message}")
        future = self.executor.submit(self._send_message, message)
        try:
            # Bounded. This used to be a bare future.result(): the worker
            # blocked in sendall() against a socket with no timeout, so the
            # caller's thread was parked for the life of the process. The
            # socket timeout in the bus adapter should fire long before this
            # does -- this is the backstop for anything on the send path that
            # blocks for a reason the socket cannot see.
            result = future.result(timeout=SEND_COMPLETION_TIMEOUT_S)
        except FuturesTimeoutError:
            logger.error(
                f"[RT_SENDER] Send did not complete within "
                f"{SEND_COMPLETION_TIMEOUT_S}s; abandoning it. The worker may "
                f"still be running -- the bus adapter's own socket timeout is "
                f"what stops it."
            )
            return False
        logger.info(f"RT_send_message result: {result}")
        return result

    def _preprocess_message(self, message):
        """
        Convert MIL_STD_1553B_Message objects to proper dictionary format.
        Ensures schema application and field consistency with BC's expectations.
        """
        if isinstance(message, MIL_STD_1553B_Message):
            # Extract all critical fields
            status_word = getattr(message, 'status_word', None)
            request_id = getattr(message, 'request_id', None)
            rt_address = getattr(message, 'rt_address', None)
            sub_address = getattr(message, 'sub_address', None)
            
            # Generate status word if needed
            if not status_word and rt_address is not None:
                from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                rt_construct = RT_construct()
                status_word = rt_construct.construct_status_word(rt_address, sub_address or 1)
                logger.info(f"Generated status word for RT={rt_address}, SA={sub_address}: {status_word}")
            
            # Create properly formatted message dict
            formatted_dict = {
                'status_word': status_word,
                'frames': [status_word] if status_word else [],
                'request_id': request_id,
                'rt_address': rt_address,
                'sub_address': sub_address,
                'metadata': {
                    'rt_address': rt_address,
                    'sub_address': sub_address,
                    'message_type': getattr(message, 'message_type', None),
                    'command_type': getattr(message, 'command_type', None),
                    'command_name': getattr(message, 'command_name', None),
                    'mode': getattr(message, 'mode', None),
                    'mode_value': getattr(message, 'mode_value', None)
                }
            }
            
            # Convert data if available
            if hasattr(message, 'data') and message.data:
                data = message.data
                formatted_dict['data'] = data  # Preserve original data
                
                # If frames only has status word, add data frames
                if len(formatted_dict['frames']) <= 1:
                    # Convert different data types to frames
                    from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                    rt_construct = RT_construct()
                    
                    if isinstance(data, list):
                        for item in data:
                            if isinstance(item, str) and len(item) == 20:
                                formatted_dict['frames'].append(item)
                            elif isinstance(item, (int, float)):
                                data_word = rt_construct.construct_data_word(int(item))
                                formatted_dict['frames'].append(data_word)
                    elif isinstance(data, (int, float)):
                        data_word = rt_construct.construct_data_word(int(data))
                        formatted_dict['frames'].append(data_word)
            
            # Apply schema validation
            message_type = getattr(message, 'message_type', None)
            if message_type:
                schema = get_schema_for_message_type(message_type)
                if schema:
                    # Check required metadata fields
                    required_fields = schema.get('metadata_fields', [])
                    for field in required_fields:
                        if field not in formatted_dict['metadata'] or formatted_dict['metadata'][field] is None:
                            field_value = getattr(message, field, None)
                            if field_value is not None:
                                formatted_dict['metadata'][field] = field_value
                                # Also add at top level for key fields
                                if field in ['rt_address', 'sub_address', 'message_type', 'command_type', 'command_name']:
                                    formatted_dict[field] = field_value
            
            logger.info(f"Preprocessed MIL_STD_1553B_Message object to dictionary format")
            return formatted_dict
        
        return message

    def _send_message(self, message):
        """
        Internal method to send message to BC, EXACTLY mirroring BC's _send_message method.
        """
        if self.is_shutting_down:
            logger.warning("_send_message aborting early due to shutdown")
            return False

        # Preprocess message to ensure consistent format
        message = self._preprocess_message(message)

        # EXACT MIRROR OF BC LOGIC: First check if frames exceed limit
        # This is an exact match to BC_sender's logic as the primary check
        if isinstance(message, dict) and 'frames' in message and isinstance(message['frames'], list):
            # Check if frames exceed MIL-STD-1553B limit (32 data words per message)
            if len(message['frames']) > MAX_FRAMES_PER_MESSAGE:
                logger.info(f"RT_sender detected large message with {len(message['frames'])} frames, using block transfer protocol")
                return self._send_large_message(message)

        # RT-SPECIFIC ENHANCEMENT: Detect large data arrays that need conversion to frames
        # This is specific to RT since BC would have already converted data to frames
        if isinstance(message, dict) and 'data' in message and isinstance(message['data'], list):
            # Use more aggressive threshold for pure data arrays
            if len(message['data']) > 15:  # Set threshold based on log analysis
                logger.info(f"RT_sender detected large data array of {len(message['data'])} items, using block transfer protocol")
                
                # Convert data to frames if needed, ensuring status_word is first
                from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                rt_construct = RT_construct()
                
                status_word = message.get('status_word', '')
                data_frames = []
                
                # Convert data items to 20-bit frames
                for item in message['data']:
                    if isinstance(item, (int, float)):
                        data_frames.append(rt_construct.construct_data_word(int(item)))
                    elif isinstance(item, str) and len(item) == 20:
                        data_frames.append(item)
                    else:
                        data_frames.append(rt_construct.construct_data_word(0))
                
                # Create frame list with status word first
                message_with_frames = message.copy()
                message_with_frames['frames'] = [status_word] + data_frames
                
                # Preserve the original data in message, so it's available to block_transfer_manager
                # But tag it for proper handling to avoid duplication
                message_with_frames['_raw_data'] = message['data']
                message_with_frames['_data_converted_to_frames'] = True
                
                logger.info(f"RT_sender prepared message with {len(message_with_frames['frames'])} frames for block transfer")
                return self._send_large_message(message_with_frames)

        # Third check: RT-specific - Detect precipitation data by command type
        if isinstance(message, dict) and 'command_type' in message:
            if message['command_type'] in ['precipitation_data', 'vil_data'] and 'data' in message:
                # This is precipitation or VIL data, which should always use block transfer
                data_type = message['command_type'].split('_')[0]  # 'precipitation' or 'vil'
                logger.info(f"RT_sender detected {data_type} data that requires block transfer protocol")
                
                # Convert to frames if needed
                if 'frames' not in message or not message['frames']:
                    # Use the same conversion logic as above
                    from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                    rt_construct = RT_construct()
                    
                    status_word = message.get('status_word', '')
                    data_frames = []
                    
                    # Convert data items to 20-bit frames
                    for item in message['data']:
                        if isinstance(item, (int, float)):
                            data_frames.append(rt_construct.construct_data_word(int(item)))
                        elif isinstance(item, str) and len(item) == 20:
                            data_frames.append(item)
                        else:
                            data_frames.append(rt_construct.construct_data_word(0))
                    
                    # Create frame list with status word first
                    message_with_frames = message.copy()
                    message_with_frames['frames'] = [status_word] + data_frames
                    
                    # Preserve the original data, but tag it
                    message_with_frames['_raw_data'] = message['data'] 
                    message_with_frames['_data_converted_to_frames'] = True
                    
                    logger.info(f"RT_sender converted {data_type} data to {len(data_frames)} frames for block transfer")
                    return self._send_large_message(message_with_frames)

        # Extract metadata for logging but don't send it directly
        metadata = None
        request_id = None
        if isinstance(message, dict):
            metadata = message.get('metadata')
            request_id = message.get('request_id')
            if metadata:
                logger.info(f"RT_sender found metadata in message: {metadata}")
                if 'message_type' in metadata:
                    logger.info(f"RT_sender found message_type in metadata: {metadata['message_type']}")

        # Prepare the message to send
        # For dict messages, we need to extract the frames and preserve all fields
        if isinstance(message, dict):
            # Ensure completion messages with status_word also have a frames field
            # This makes RT's message format match exactly what BC_Listener expects
            if 'status_word' in message and 'frames' not in message:
                # Mirror BC's handling by ensuring status_word is also in frames field
                frames_field = [message['status_word']]
                logger.info(f"RT_sender added frames field for status_word message: {frames_field}")
            else:
                frames_field = message.get('frames', [])
                
            # Create transmission_dict with all fields preserved
            # Start with basic fields that mirror BC's approach
            transmission_dict = {
                'frames': frames_field,
                'request_id': request_id
            }
            
            # If this is a status_word message, preserve the status_word field for consistency
            if 'status_word' in message:
                transmission_dict['status_word'] = message['status_word']
            
            # ARCHITECTURAL ENHANCEMENT: Preserve ALL fields from original message
            # This ensures data objects and other critical fields are not lost during transmission
            for key, value in message.items():
                # Don't override already set fields and skip metadata (handled separately)
                if key not in transmission_dict and key != 'metadata':
                    transmission_dict[key] = value
                    logger.info(f"RT_sender preserved field '{key}' from original message")
            
            # EXPERT DATA HANDLING: Intelligent frame generation for complex data
            # If we have a data field but insufficient frames, convert data to frames
            if ('data' in transmission_dict and 
                (not transmission_dict.get('frames') or len(transmission_dict.get('frames', [])) <= 1)):
                
                # Get properly formatted status word
                status_word = message.get('status_word')
                
                # Import RT_construct to create properly formatted MIL-STD-1553B words
                from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                rt_construct = RT_construct()
                
                # Start with status word if available
                frames = [status_word] if status_word else []
                
                # Convert data to frames based on data type
                data = transmission_dict['data']
                logger.info(f"RT_sender generating frames from data field of type {type(data)}")
                
                if isinstance(data, list):
                    # Each item in the list becomes a frame
                    for item in data:
                        if isinstance(item, (int, float)):
                            # Convert numbers to properly formatted 20-bit data words
                            frames.append(rt_construct.construct_data_word(int(item)))
                        elif isinstance(item, str) and len(item) == 20:
                            # Already a valid 20-bit frame
                            frames.append(item)
                        else:
                            # Use standard format for complex items (simple numeric placeholder)
                            frames.append(rt_construct.construct_data_word(0))
                            
                    # Update frames field with generated frames
                    transmission_dict['frames'] = frames
                    logger.info(f"RT_sender generated {len(frames)} frames from data field")
            
            # Only include metadata fields needed for message processing
            if metadata:
                # Create a simplified metadata dict with just the essential fields
                essential_metadata = {
                    'message_type': metadata.get('message_type', ''),
                    'command_type': metadata.get('command_type', ''),
                    'command_name': metadata.get('command_name', ''),
                    # Add mode and mode_value fields to preserve them across the RT-BC boundary
                    'mode': metadata.get('mode'),
                    'mode_value': metadata.get('mode_value'),
                    # Also preserve old_mode and new_mode for mode change completion messages
                    'old_mode': metadata.get('old_mode'),
                    'new_mode': metadata.get('new_mode')
                }
                
                # Log the mode information for debugging
                if 'mode' in metadata:
                    logger.info(f"[RT_SENDER] Including mode in metadata: {metadata['mode']}")
                if 'mode_value' in metadata:
                    logger.info(f"[RT_SENDER] Including mode_value in metadata: {metadata['mode_value']}")
                
                # Add request_id if available
                if request_id:
                    essential_metadata['request_id'] = request_id
                
                transmission_dict['metadata'] = essential_metadata
            
            msg = json.dumps(transmission_dict).encode()
            logger.info(f"RT_sender sending dict message with frames and essential metadata")
        elif isinstance(message, str) and len(message) == 20:
            # Status word
            transmission_dict = {
                'status_word': message,
                'request_id': request_id,
                'timestamp': time.time()
            }
            msg = json.dumps(transmission_dict).encode()
            logger.info(f"RT_sender sending status word: {message}")
        else:
            # List or single frame format
            if not isinstance(message, list):
                message = [message]
            msg = json.dumps(message).encode()
            logger.info(f"RT_sender sending frame list: {message}")
            
        logger.info(f"RT_sender encoded message length: {len(msg)} bytes")
        
        # Check if message exceeds safe socket buffer size
        # Standard socket buffer is 1024 bytes, we'll stay under 900 to be safe
        MAX_SAFE_SIZE = 900  #  buffer safety threshold
        
        try:
            # Wire transfer now goes through the configured bus adapter
            # (socket by default — identical connect/sendall/close semantics
            # and error handling to the inline code this replaced). One
            # deliberate improvement: the block-transfer conversion branch
            # below no longer opens a throwaway TCP connection first just to
            # close it unused before delegating to _send_large_message().
            payload = msg

            #  SAFETY: Use chunked transmission for large messages
            # This prevents buffer overflow and string truncation errors
            if len(msg) > MAX_SAFE_SIZE:
                logger.info(f"Message exceeds safe buffer size ({len(msg)} bytes), using chunked transmission")

                # For large messages, always use the block transfer protocol instead
                # This ensures proper splitting and reassembly
                if isinstance(message, dict) and 'data' in message:
                    logger.info(f"Converting large message to block transfer format")

                    # If message has frames but is still too large for direct transmission
                    # Ensure block transfer is used
                    if 'frames' in message:
                        return self._send_large_message(message)
                    else:
                        # Convert data to frames first
                        from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
                        rt_construct = RT_construct()

                        status_word = message.get('status_word', '')
                        data_frames = []

                        # Convert data items to 20-bit frames
                        if isinstance(message['data'], list):
                            for item in message['data']:
                                if isinstance(item, (int, float)):
                                    data_frames.append(rt_construct.construct_data_word(int(item)))
                                elif isinstance(item, str) and len(item) == 20:
                                    data_frames.append(item)
                                else:
                                    data_frames.append(rt_construct.construct_data_word(0))

                            # Create frame list with status word first
                            message_with_frames = message.copy()
                            message_with_frames['frames'] = [status_word] + data_frames

                            # Use block transfer protocol
                            return self._send_large_message(message_with_frames)

                # Fallback for other message types: limited metadata
                # Create a simplified version with only essential fields
                if isinstance(message, dict):
                    # Create simplified message with only the most essential fields
                    if 'frames' in message:
                        # If we have frames, just send those with request_id
                        simplified_dict = {
                            'frames': message['frames'],
                            'request_id': message.get('request_id')
                        }

                        # Add only minimal metadata
                        if 'metadata' in message:
                            simplified_dict['metadata'] = {
                                'message_type': message['metadata'].get('message_type', ''),
                                'command_type': message['metadata'].get('command_type', ''),
                                'command_name': message['metadata'].get('command_name', '')
                            }
                    else:
                        # Without frames, just send the minimal required fields
                        # (This should rarely happen after our other changes)
                        simplified_dict = {
                            'request_id': message.get('request_id')
                        }

                        # If we have a status word, include it
                        if 'status_word' in message:
                            simplified_dict['status_word'] = message['status_word']

                    payload = json.dumps(simplified_dict).encode()
                    logger.info(f"Sending simplified message with length: {len(payload)} bytes")
                else:
                    # For non-dict messages, send as is and hope for the best
                    logger.info(f"Sending non-dict message directly")
                    payload = msg

            if not self._bus_adapter.transmit(payload):
                return False

            logger.info(f"Message sent successfully")

            # Log message flow trace just like BC
            logger.info(f"Message flow trace: RT_sender -> BC_Listener")

            return True
        except Exception as e:
            logger.error(f"Error sending message: {str(e)}")
            return False

    def _validate_frames(self, frames):
        """
        Validate all frames to ensure they're exactly 20 bits and have valid sync bits.
        
        Args:
            frames: List of frames to validate
            
        Returns:
            List of valid frames (all invalid frames are removed)
        """
        if not frames:
            return []
            
        valid_frames = []
        for i, frame in enumerate(frames):
            # Skip None and non-string frames
            if not isinstance(frame, str):
                logger.error(f"Frame {i} is not a string: {type(frame)}")
                continue
                
            # Validate frame length
            if len(frame) != 20:
                logger.error(f"Frame {i} has invalid length {len(frame)}, expected 20 bits")
                
                # If close to 20, try to fix by padding or truncating
                if 18 <= len(frame) <= 22:
                    if len(frame) < 20:
                        # Pad with parity bit(s)
                        fixed_frame = frame.ljust(20, '0')
                    else:
                        # Truncate to 20 bits
                        fixed_frame = frame[:20]
                    logger.warning(f"Fixed frame {i} to valid length: {fixed_frame}")
                    frame = fixed_frame
                else:
                    # Too far off, skip this frame
                    continue
                    
            # Check sync bits (must be '100' or '001')
            sync_bits = frame[:3]
            if sync_bits not in ['100', '001']:
                logger.error(f"Frame {i} has invalid sync bits: {sync_bits}")
                continue
                
            # Frame is valid, add to list
            valid_frames.append(frame)
            
        # Log validation summary
        if len(valid_frames) != len(frames):
            logger.warning(f"Frame validation: {len(valid_frames)}/{len(frames)} frames are valid")
        else:
            logger.info(f"Frame validation: All {len(frames)} frames are valid")
            
        return valid_frames
    
    def _send_chunk(self, chunk_message):
        """
        Send a single chunk of data that is guaranteed to be within buffer limits.
        Uses direct socket connection to avoid recursive calls and ensure delivery.
        
        Args:
            chunk_message: Message containing frames and metadata for this chunk
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Serialize message to estimate size
            msg = json.dumps(chunk_message).encode()
            msg_length = len(msg)
            logger.info(f"Chunk message size: {msg_length} bytes")
            
            # If the chunk is still too large, simplify further
            if msg_length > 900:
                logger.warning(f"Chunk message exceeds safe buffer size, simplifying")
                
                # Create minimal version with only essential fields
                simplified = {
                    'frames': chunk_message['frames'],
                    'request_id': chunk_message.get('request_id')
                }
                
                # Add critical metadata but keep it minimal
                if 'metadata' in chunk_message and chunk_message['metadata']:
                    simplified['metadata'] = {
                        'message_type': chunk_message['metadata'].get('message_type', ''),
                        'command_type': chunk_message['metadata'].get('command_type', ''),
                        'is_transfer_data': True,
                        'sequence_number': chunk_message['metadata'].get('sequence_number'),
                        'total_sequences': chunk_message['metadata'].get('total_sequences'),
                        'is_final': chunk_message['metadata'].get('is_final', False)
                    }
                
                # Serialize simplified version and check size again
                msg = json.dumps(simplified).encode()
                simplified_length = len(msg)
                logger.info(f"Simplified chunk size: {simplified_length} bytes")
                
                # If still too large, only send the frames
                if simplified_length > 900:
                    logger.warning(f"Simplified chunk still exceeds buffer size, sending frames only")
                    frames_only = {
                        'frames': chunk_message['frames'],
                        'request_id': chunk_message.get('request_id')
                    }
                    msg = json.dumps(frames_only).encode()
                    logger.info(f"Frames-only size: {len(msg)} bytes")
            
            # Use direct adapter transmission to prevent recursive behavior
            # (previously an inline socket; the default SocketBusAdapter has
            # identical connect/sendall/close and error-return semantics)
            if self._bus_adapter.transmit(msg):
                logger.info(f"Chunk sent successfully")
                return True
            return False
        except Exception as e:
            logger.error(f"Error sending chunk: {str(e)}")
            return False
    
    def _send_large_message(self, message):
        """
        Send a large message using MIL-STD-1553B block transfer protocol.
        
        This method implements the standard MIL-STD-1553B approach for handling large data transfers
        by breaking the data into multiple messages with proper sequencing and metadata.
        
        Args:
            message: The message to send, typically a dict with 'frames', 'request_id', and 'metadata'
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            logger.info(f"RT_sender sending large message using block transfer protocol")
            
            # Extract frames, request_id, and metadata from message
            if not isinstance(message, dict) or 'frames' not in message:
                logger.error("Invalid message format for large message transfer")
                return False
                
            frames = message['frames']
            request_id = message.get('request_id')
            metadata = message.get('metadata', {})
            
            # Check for and set transfer flags consistently
            # This ensures all phases of the transfer maintain proper metadata
            if isinstance(metadata, dict):
                # Force one of the transfer flags to be set
                if not metadata.get('is_transfer_init', False) and \
                   not metadata.get('is_transfer_data', False) and \
                   not metadata.get('is_transfer_complete', False):
                    logger.info(f"Setting is_transfer_data flag to ensure proper block transfer handling")
                    metadata['is_transfer_data'] = True
                    
            # Validate all frames to ensure they're exactly 20 bits
            frames = self._validate_frames(frames)
            if not frames:
                logger.error("No valid frames after validation")
                return False
            
            # Calculate number of messages needed (max 32 data words per message)
            max_frames_per_message = MAX_PAYLOAD_FRAMES_PER_CHUNK
            
            # First frame is always the status word - in BC this is command_word
            status_word = frames[0]
            data_frames = frames[1:]
            
            # Calculate number of messages needed - exactly like BC
            num_messages = (len(data_frames) + max_frames_per_message - 1) // max_frames_per_message
            
            logger.info(f"RT_sender breaking large message into {num_messages} messages")
            
            # Import the RT_construct class to create properly formatted MIL-STD-1553B words
            from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.RT_msg import RT_construct
            rt_construct = RT_construct()
            
            # Send initialization message (Mode Code 17: Synchronize)
            # This message contains metadata about the transfer
            # Create properly formatted 20-bit data words with sync bits and parity
            total_messages_word = rt_construct.construct_data_word(num_messages)
            total_frames_word = rt_construct.construct_data_word(len(data_frames))
            
            # Create init message - mirrors BC's format exactly
            init_message = {
                'frames': [
                    status_word,  # Original status word (BC uses command_word)
                    total_messages_word,  # Total number of messages in the transfer (20-bit word)
                    total_frames_word  # Total number of data frames (20-bit word)
                ],
                'request_id': request_id,
                'metadata': {
                    'message_type': metadata.get('message_type', 'block_transfer_init'),
                    'command_type': 'transfer_init',
                    'is_transfer_init': True,
                    'total_messages': num_messages,
                    'total_frames': len(data_frames)
                }
            }
            
            # Check if this request has already been acknowledged - same as BC
            already_acknowledged = False
            with self.acknowledged_requests_lock:
                if request_id in self.acknowledged_requests:
                    # Check if the acknowledgment is recent (within the last 5 seconds)
                    if time.time() - self.acknowledged_requests[request_id] < 5.0:
                        already_acknowledged = True
                        logger.info(f"RT_sender found recent acknowledgment for request ID: {request_id}, skipping initialization message")
            
            if not already_acknowledged:
                # Send initialization message
                init_result = self._send_message(init_message)
                if not init_result:
                    logger.error("Failed to send initialization message")
                    return False
                    
                logger.info(f"RT_sender sent transfer initialization message with request ID: {request_id}")
                
                # Acknowledgment tracking
                # Instead of BC's wait for status word, we just store acknowledgment directly
                # since the RT's architecture doesn't have an equivalent to BC_Listener's status word handling
                with self.acknowledged_requests_lock:
                    self.acknowledged_requests[request_id] = time.time()
                logger.info(f"RT_sender stored acknowledgment time for request ID: {request_id}")
            
            # Send data messages - EXACTLY LIKE BC
            success = True
            # One deadline for the whole transfer. The caller's
            # future.result(timeout=...) bounds how long IT waits, but not
            # this worker: a transfer is thousands of sequential connect-send
            # cycles and the socket timeout bounds each one separately, not
            # the total. Measured without this -- the caller returned False at
            # 30 s while the worker ran on, and the process would not exit,
            # because ThreadPoolExecutor joins its workers at interpreter
            # shutdown.
            transfer_deadline = time.monotonic() + BLOCK_TRANSFER_TIMEOUT_S

            for i in range(num_messages):
                if time.monotonic() > transfer_deadline:
                    logger.error(
                        f"[RT_SENDER] Abandoning block transfer after "
                        f"{BLOCK_TRANSFER_TIMEOUT_S}s: sent {i} of "
                        f"{num_messages} messages"
                    )
                    return False
                # Calculate start and end indices for this chunk
                start_idx = i * max_frames_per_message
                end_idx = min((i + 1) * max_frames_per_message, len(data_frames))
                
                # Extract data chunk
                chunk = data_frames[start_idx:end_idx]
                
                # Add sequence metadata to the beginning of the chunk
                # First, create properly formatted 20-bit data words for sequence metadata
                sequence_number_word = rt_construct.construct_data_word(i + 1)  # Sequence number (1-based)
                total_messages_word = rt_construct.construct_data_word(num_messages)  # Total number of messages
                
                # Add the properly formatted sequence metadata words to the chunk
                chunk_with_metadata = [sequence_number_word, total_messages_word] + chunk

                # Defence in depth against the fixed point described at
                # MAX_PAYLOAD_FRAMES_PER_CHUNK: if a chunk is still "large",
                # _send_message would hand it straight back here and recurse
                # without bound. Fail loudly instead of silently looping.
                if 1 + len(chunk_with_metadata) > MAX_FRAMES_PER_MESSAGE:
                    logger.error(
                        f"[RT_SENDER] Refusing to send block-transfer chunk "
                        f"{i + 1}/{num_messages}: {1 + len(chunk_with_metadata)} "
                        f"frames exceeds the {MAX_FRAMES_PER_MESSAGE}-frame "
                        f"message limit, so it would be split again forever"
                    )
                    return False
                
                # Create data message - IDENTICAL TO BC's format with status_word instead of command_word
                data_message = {
                    'frames': [status_word] + chunk_with_metadata,  # Status word + chunk with metadata
                    'request_id': request_id,
                    'metadata': {
                        'message_type': metadata.get('message_type'),
                        'command_type': metadata.get('command_type'),
                        'is_transfer_data': True,
                        'sequence_number': i + 1,
                        'total_sequences': num_messages,
                        'is_final': (i == num_messages - 1)
                    }
                }
                
                # Use _send_chunk instead of _send_message for data chunks
                # This ensures buffer size control and prevents recursion issues
                logger.info(f"Using _send_chunk for data message {i+1}/{num_messages}")
                data_result = self._send_chunk(data_message)
                if not data_result:
                    logger.error(f"Failed to send data message {i+1}/{num_messages}")
                    success = False
                    break
                    
                logger.info(f"RT_sender sent data message {i+1}/{num_messages} with request ID: {request_id}")
                
                # Small delay between messages to prevent overwhelming the bus - SAME AS BC
                time.sleep(0.01)
            
            # Send completion message (Mode Code 16: Transmit Vector Word) - SAME AS BC
            # This message indicates the end of the transfer
            if success:
                # Create properly formatted 20-bit data words for completion message
                total_messages_word = rt_construct.construct_data_word(num_messages)
                status_code_word = rt_construct.construct_data_word(1)  # Status code (1 = success)
                
                # Completion message format - IDENTICAL TO BC except status_word instead of command_word
                completion_message = {
                    'frames': [
                        status_word,  # Original status word
                        total_messages_word,  # Total number of messages sent (20-bit word)
                        status_code_word  # Status code (1 = success) (20-bit word)
                    ],
                    'request_id': request_id,
                    'metadata': {
                        'message_type': metadata.get('message_type', 'block_transfer_complete'),
                        'command_type': 'transfer_complete',
                        'is_transfer_complete': True,
                        'total_messages': num_messages,
                        'status': 'success'
                    }
                }
                
                # Use _send_chunk for completion message to ensures the same 
                # buffer handling and message formatting throughout the transfer protocol
                logger.info(f"Using _send_chunk for completion message")
                completion_result = self._send_chunk(completion_message)
                if not completion_result:
                    logger.error("Failed to send completion message")
                    success = False
                else:
                    logger.info(f"RT_sender sent transfer completion message with request ID: {request_id}")
            
            return success
        except Exception as e:
            logger.error(f"Error sending large message: {str(e)}")
            return False


    def shutdown(self):
        """
        Shutdown the RT_sender, mirroring BC_sender's shutdown method.
        """
        logger.info("RT_sender initiating shutdown")
        self.is_shutting_down = True
        logger.info("RT_sender waiting for in-progress tasks to complete")
        self.executor.shutdown(wait=True)
        logger.info("RT_sender has been shut down.")
        self.shutdown_event.set()

    def wait_for_shutdown(self):
        """
        Wait for shutdown to complete, mirroring BC_sender's wait_for_shutdown method.
        """
        logger.info("RT_sender waiting for shutdown to complete")
        self.shutdown_event.wait()
        logger.info("RT_sender shutdown complete")

    def check_health(self) -> bool:
        """
        Check the health of the RT_sender, mirroring BC_sender's health check method.
        
        Returns:
            bool: True if the RT_sender is healthy, False otherwise.
        """
        health_status = not self.is_shutting_down and not self.executor._shutdown
        logger.info(f"RT_sender health check: {'healthy' if health_status else 'unhealthy'}")
        return health_status

class RT_Listener:
    """
    Remote Terminal listener class that mirrors BC_Listener behavior.
    Handles communication from Bus Controller to Remote Terminal.
    """
    def __init__(self):
        self.data_received = list()
        self.processed_messages = list()  # Queue for processed messages
        self.running = False
        # Story C8.1: resolved from busAdapterConfig.xml / environment
        # rather than hardcoded, so two instances can coexist on one
        # host. Defaults remain 5001 on loopback.
        self.listen_host, self.port = get_listen_endpoint('rt')
        # True once the retry budget is spent (story C7.1): the listener has
        # stopped and will not rebuild its socket. check_health() already
        # reports False in this state via the unbound socket; this flag makes
        # 'gave up' distinguishable from 'still trying' to anything that asks.
        self._retry_exhausted = False  # The port RT listens on
        self.health_check_interval = 5  # Seconds between health checks
        self.last_activity_time = time.time()
        self.socket_variable = None
        self._lock = threading.Lock()
        # Lock for thread-safe operations on processed_messages
        self.message_lock = threading.Lock()
        # Live connection-handler workers (H3). Guarded by _handler_lock, which
        # is deliberately NOT self._lock: that one is held while appending to
        # self.data_received inside a handler, so reusing it here would have a
        # handler and the accept loop contend on every message.
        self._handler_threads = set()
        self._handler_lock = threading.Lock()
        logger.info(f"RT_Listener initialized on port {self.port}")

    def setup_socket(self):
        """
        Set up the RT_Listener socket, mirroring BC_Listener's setup_socket method.
        """
        try:
            self.socket_variable = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_variable.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            logger.info(f"RT_Listener binding to port {self.port}")
            # Bound to loopback only: BC and RT always connect to each
            # other via destination_ip = "localhost" (see BC_socket.py /
            # RT_socket.py __init__), so there is no legitimate cross-host
            # use case here. Binding to all interfaces on an unauthenticated,
            # unencrypted socket that parses received frames unnecessarily
            # exposed this to any host on the network (production readiness
            # punch list, item 2).
            self.socket_variable.bind((self.listen_host, self.port))
            # With listen_port=0 the OS picks the port; read it back so
            # self.port is the REAL bound port. check_health() compares
            # against it (story C6.1), and a stale 0 would make a healthy
            # listener look unbound forever.
            self.port = self.socket_variable.getsockname()[1]
            self.socket_variable.listen(5)
            self.socket_variable.setblocking(False)
            logger.info(f"RT_Listener socket set up successfully on port {self.port}")
        except Exception as e:
            # Drop the half-built socket (story C6.1). Construction succeeds
            # before bind() raises, so without this the object survives as an
            # UNBOUND socket that check_health() below would otherwise have to
            # reason about. Clearing it makes "not listening" unambiguous.
            try:
                if self.socket_variable is not None:
                    self.socket_variable.close()
            except Exception:
                pass
            self.socket_variable = None
            logger.error(f"Error setting up RT_Listener socket: {str(e)}")
            raise

    def _sleep_interruptible(self, seconds):
        """Sleep, but wake promptly if stop_listener() clears `running`.

        Backoff delays reach 30 s (story C7.1) and shutdown must not have to
        wait that long, so the wait is chunked rather than a single sleep.
        """
        deadline = time.time() + seconds
        while self.running and time.time() < deadline:
            time.sleep(min(0.25, max(0.0, deadline - time.time())))

    def start_listening(self):
        """Accept loop with bounded, backing-off socket retry (story C7.1)."""
        self.running = True
        self._retry_exhausted = False
        policy = BackoffPolicy(name="RT_Listener socket")
        logger.info(f"RT_Listener starting on {self.listen_host}:{self.port}")

        self.last_activity_time = time.time()
        self.consecutive_errors = 0

        # Main listening loop
        while self.running:
            # Invariant: never attempt to accept without a bound socket
            # (story C7.1). The initial setup is inside the loop rather than
            # before it, so first-attempt failure and later failure follow one
            # code path under one backoff policy instead of two.
            if not socket_is_bound(self.socket_variable, self.port):
                try:
                    self.setup_socket()
                    policy.record_success()
                    logger.info(f"RT_Listener listening on {self.listen_host}:{self.port}")
                except Exception as e:
                    delay = policy.record_failure()
                    if policy.should_log():
                        logger.error(
                            f"[RT_LISTENER] {policy.describe(str(e))}; retrying in {delay:.1f}s"
                        )
                    if policy.exhausted:
                        self._retry_exhausted = True
                        logger.error(
                            f"[RT_LISTENER] Giving up: {policy.give_up_reason()}. "
                            f"The listener is now permanently unhealthy and will not "
                            f"retry; check_health() reports False, which blocks system "
                            f"readiness. Set FMOFP_RT_LISTEN_PORT or listen_port in "
                            f"busAdapterConfig.xml if another process holds "
                            f"{self.listen_host}:{self.port}."
                        )
                        break
                    self._sleep_interruptible(delay)
                    continue

            try:
                
                # Wait for connections
                readable, _, _ = select.select([self.socket_variable], [], [], 1.0)
                if readable:
                    connection, client_address = self.socket_variable.accept()
                    logger.info(f"RT_Listener accepted connection from {client_address}")
                    connection.setblocking(False)
                    # Hand off and go straight back to accept(). Serving inline
                    # here was H3: one peer held the listener for the whole life
                    # of its connection, and every other peer waited. Measured
                    # before the change on the real listener -- a silent peer was
                    # accepted, a second peer connected and sent a valid message,
                    # and the second was not accepted until the first
                    # disconnected.
                    if not self._spawn_connection_handler(connection, client_address):
                        try:
                            connection.close()
                        except Exception:
                            pass
                    # Reset error counter on successful connection
                    self.consecutive_errors = 0
                    self.last_activity_time = time.time()
                
            except (socket.error, OSError) as e:
                # Drop the socket and let the top of the loop rebuild it under
                # the backoff policy. This replaces the previous scheme --
                # recreate every third failure, sleep a flat 0.5 s, repeat
                # forever -- which produced ~2 ERROR lines per second for as
                # long as the fault lasted, with no cap and no terminal state.
                self.consecutive_errors += 1
                delay = policy.record_failure()
                if policy.should_log():
                    logger.error(
                        f"[RT_LISTENER] {policy.describe(str(e))}; "
                        f"rebuilding socket in {delay:.1f}s"
                    )
                try:
                    if self.socket_variable is not None:
                        self.socket_variable.close()
                except Exception:
                    pass
                self.socket_variable = None
                if policy.exhausted:
                    self._retry_exhausted = True
                    logger.error(
                        f"[RT_LISTENER] Giving up: {policy.give_up_reason()}. "
                        f"The listener is now permanently unhealthy and will not retry."
                    )
                    break
                self._sleep_interruptible(delay)

            except Exception as e:
                logger.error(f"Non-socket error in RT_Listener main loop: {str(e)}")
                time.sleep(0.1)  # Prevent tight loop on repeated errors

        # Clean up
        if self.socket_variable is not None:
            try:
                self.socket_variable.close()
            except Exception:
                pass
        logger.info("RT_Listener stopped")

    def handle_connection(self, connection, client_address):
        """
        Handle incoming connection from BC, mirroring BC_Listener's handle_connection method.
        """
        logger.info(f"[RT_LISTENER] Handling connection from {client_address}")
        # See BC_Listener.handle_connection()'s matching comment in
        # BC_socket.py for the full explanation: a single logical JSON
        # message sent via one sendall() call can legitimately arrive
        # split across multiple recv(1024) calls here (TCP is a byte
        # stream, not message-oriented), and without accumulation each
        # fragment independently fails json.loads() and is silently
        # dropped. Same fix mirrored here for RT_Listener.
        recv_buffer = b""
        _MAX_RECV_BUFFER_BYTES = 10 * 1024 * 1024  # 10 MB
        # H2: a connection with no idle timeout. recv() here is guarded by a
        # 1 s select, so no single syscall blocks forever -- but the loop runs
        # on self.running, the LISTENER's flag, and only leaves when the peer
        # closes or errors. A peer that connects and then says nothing holds
        # this slot for as long as it stays connected.
        #
        # That matters because the accept loop is single-threaded (H3): while
        # this runs, no other peer is accepted. Measured on the real listener:
        # a silent peer A was accepted, peer B connected and sent a valid
        # message, and B was not accepted until the moment A disconnected 3 s
        # later. A sender that crashes mid-send, or a stalled link, leaves the
        # bus unreachable indefinitely.
        #
        # The deadline is generous because senders here connect, send and close
        # per message (see RT_sender/BC_sender), so a healthy peer is never
        # idle for anything like this long; it exists to bound the pathological
        # case rather than to police normal traffic.
        last_data_time = time.time()
        while self.running:
            try:
                readable, _, _ = select.select([connection], [], [], 1.0)
                if not readable:
                    idle = time.time() - last_data_time
                    if idle >= CONNECTION_IDLE_TIMEOUT_S:
                        logger.warning(
                            f"[RT_LISTENER] Closing idle connection from {client_address} "
                            f"after {idle:.0f}s without data; it was holding "
                            f"one of the {MAX_CONCURRENT_CONNECTIONS} handler "
                            f"slots and will not be sending"
                        )
                        break
                    continue
                if readable:
                    last_data_time = time.time()
                    data = connection.recv(1024)
                    if data:
                        recv_buffer += data
                        if len(recv_buffer) > _MAX_RECV_BUFFER_BYTES:
                            logger.error(
                                f"[RT_LISTENER] Discarding {len(recv_buffer)} buffered bytes from "
                                f"{client_address}: exceeded {_MAX_RECV_BUFFER_BYTES} bytes without "
                                f"a parseable message"
                            )
                            recv_buffer = b""
                            continue
                        try:
                            decoded_data = recv_buffer.decode()
                        except UnicodeDecodeError:
                            continue
                        if decoded_data.startswith('{') or decoded_data.startswith('['):
                            try:
                                json.loads(decoded_data)
                            except (ValueError, SyntaxError):
                                continue
                        recv_buffer = b""
                        try:
                            logger.info(f"[RT_LISTENER] Received raw data from {client_address}: {data}")
                            logger.info(f"[RT_LISTENER] Decoded data: {decoded_data}")
                            
                            try:
                                # Parse the received data
                                if decoded_data.startswith('{'):
                                    # Dict format with request_id
                                    parsed_data = json.loads(decoded_data)
                                    
                                    # Log the parsed data for debugging
                                    logger.info(f"[RT_LISTENER] Parsed data: {parsed_data}")
                                    
                                    # Use MetadataCodec for complex data
                                    # This exactly mirrors how BC handles metadata-encoded fields
                                    if isinstance(parsed_data, dict) and 'data' in parsed_data:
                                        if isinstance(parsed_data['data'], list) and parsed_data['data'] and len(parsed_data['data']) > 0:
                                            # Try to extract metadata from the data field if present
                                            try:
                                                # Extract potential metadata words
                                                potential_metadata = [item for item in parsed_data['data'] 
                                                                     if isinstance(item, str) and len(item) == 20]
                                                
                                                if potential_metadata:
                                                    # Attempt to decode metadata
                                                    metadata_words, remaining_words = MetadataCodec.extract_metadata_words(potential_metadata)
                                                    if metadata_words:
                                                        try:
                                                            # Decode the metadata
                                                            metadata = MetadataCodec.decode_metadata(metadata_words)
                                                            if metadata:
                                                                logger.info(f"[RT_LISTENER] Decoded metadata from data field: {metadata}")
                                                                
                                                                # Add decoded metadata to the parsed data
                                                                if 'metadata' not in parsed_data or not parsed_data['metadata']:
                                                                    parsed_data['metadata'] = {}
                                                                    
                                                                # Merge the decoded metadata with existing metadata
                                                                parsed_data['metadata'].update(metadata)
                                                                logger.info(f"[RT_LISTENER] Updated metadata: {parsed_data['metadata']}")
                                                        except Exception as e:
                                                            logger.error(f"[RT_LISTENER] Error decoding metadata: {str(e)}")
                                            except Exception as e:
                                                logger.error(f"[RT_LISTENER] Error processing potential metadata: {str(e)}")
                                    
                                    # Check for block transfer indicators
                                    if isinstance(parsed_data, dict):
                                        # Check for block transfer indicators in both top-level and metadata
                                        metadata = parsed_data.get('metadata', {})
                                        is_transfer_init = parsed_data.get('is_transfer_init', False) or (
                                            isinstance(metadata, dict) and metadata.get('is_transfer_init', False)
                                        )
                                        is_transfer_data = parsed_data.get('is_transfer_data', False) or (
                                            isinstance(metadata, dict) and metadata.get('is_transfer_data', False)
                                        )
                                        is_transfer_complete = parsed_data.get('is_transfer_complete', False) or (
                                            isinstance(metadata, dict) and metadata.get('is_transfer_complete', False)
                                        )
                                        
                                        if is_transfer_init or is_transfer_data or is_transfer_complete:
                                            # Log block transfer detection
                                            if is_transfer_init:
                                                logger.info(f"[RT_LISTENER] Detected block transfer initialization from BC")
                                            elif is_transfer_data:
                                                # Get sequence info if available
                                                sequence_number = parsed_data.get('sequence_number') or metadata.get('sequence_number', 0)
                                                total_sequences = parsed_data.get('total_sequences') or metadata.get('total_sequences', 0)
                                                logger.info(f"[RT_LISTENER] Detected block transfer data from BC: sequence {sequence_number}/{total_sequences}")
                                            elif is_transfer_complete:
                                                logger.info(f"[RT_LISTENER] Detected block transfer completion from BC")
                                    
                                    frames = parsed_data.get('frames', [])
                                    request_id = parsed_data.get('request_id')
                                elif decoded_data.startswith('['):
                                    # List format (backwards compatibility)
                                    frames = json.loads(decoded_data)
                                    request_id = None
                                else:
                                    # Single frame format
                                    frames = [decoded_data]
                                    request_id = None
                                
                                # Validate frames
                                valid_frames = []
                                for frame in frames:
                                    if len(frame) != 20:
                                        logger.error(f"Invalid frame size: {len(frame)} bits")
                                        continue
                                        
                                    # Check sync pattern
                                    sync_bits = frame[:3]
                                    if sync_bits in ['100', '001']:
                                        valid_frames.append(frame)
                                        logger.info(f"Valid sync bits ({sync_bits}) found in word: {frame}")
                                    else:
                                        logger.warning(f"Invalid sync pattern: {sync_bits}")
                                
                                # Store valid frames with request_id and command_name if available
                                if valid_frames:
                                    with self._lock:
                                        if request_id:
                                            # Create message with frames and request_id
                                            message = {
                                                'frames': valid_frames,
                                                'request_id': request_id
                                            }
                                            
                                            # Add metadata if available in parsed_data
                                            if isinstance(parsed_data, dict):
                                                # Add command_name if available
                                                if 'command_name' in parsed_data:
                                                    message['command_name'] = parsed_data['command_name']
                                                    logger.info(f"RT_Listener preserved command_name: {parsed_data['command_name']}")
                                                
                                                # Add metadata if available
                                                if 'metadata' in parsed_data:
                                                    message['metadata'] = parsed_data['metadata']
                                                    logger.info(f"RT_Listener preserved metadata: {parsed_data['metadata']}")
                                                
                                            self.data_received.append(message)
                                        else:
                                            self.data_received.append(valid_frames)
                                    logger.info(f"RT_Listener stored valid frame, total frames: {len(self.data_received)}")
                                
                            except (ValueError, SyntaxError) as e:
                                logger.error(f"Failed to parse frame data: {e}")
                                
                            self.last_activity_time = time.time()
                            
                        except UnicodeDecodeError as e:
                            logger.error(f"Failed to decode received data: {e}")
                            logger.info(f"Raw data (hex): {data.hex()}")
                    else:
                        logger.warning(f"Connection closed by {client_address}")
                        break
            except Exception as e:
                logger.error(f"Error handling connection from {client_address}: {str(e)}")
                break
        connection.close()
        logger.info(f"Connection closed with {client_address}")


    def _serve_connection(self, connection, client_address):
        """Run one connection's handler, then close it. Runs on a worker thread."""
        try:
            self.handle_connection(connection, client_address)
        except Exception:
            logger.error(
                f"[RT_LISTENER] Unhandled error serving {client_address}",
                exc_info=True)
        finally:
            # handle_connection() closes on its normal paths; close() on an
            # already-closed socket is a documented no-op, so this only matters
            # when something escaped it.
            try:
                connection.close()
            except Exception:
                pass

    def _spawn_connection_handler(self, connection, client_address) -> bool:
        """Start a worker for this connection, unless we are at capacity.

        Returns False when the connection was not taken, so the caller can
        close it rather than leaving a peer believing it is being served.
        """
        with self._handler_lock:
            self._handler_threads = {
                t for t in self._handler_threads if t.is_alive()}
            if len(self._handler_threads) >= MAX_CONCURRENT_CONNECTIONS:
                logger.warning(
                    f"[RT_LISTENER] Refusing connection from {client_address}: "
                    f"{MAX_CONCURRENT_CONNECTIONS} handlers already active"
                )
                return False
            worker = threading.Thread(
                target=self._serve_connection,
                args=(connection, client_address),
                name=f"RT_LISTENER-conn-{client_address}",
                daemon=True,
            )
            self._handler_threads.add(worker)
        worker.start()
        return True

    def _drain_connection_handlers(self, timeout=5.0):
        """Wait briefly for in-flight handlers after self.running goes False.

        They notice within one select() tick, so this is short in practice. It
        exists so a stop does not return while a handler is still appending to
        self.data_received.
        """
        with self._handler_lock:
            workers = list(self._handler_threads)
        deadline = time.time() + timeout
        for worker in workers:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            worker.join(remaining)
        still_running = [w.name for w in workers if w.is_alive()]
        if still_running:
            logger.warning(
                f"[RT_LISTENER] {len(still_running)} connection handler(s) still "
                f"running after {timeout}s: {still_running}"
            )

    def stop_listening(self):
        """
        Stop the RT_Listener, mirroring BC_Listener's stop_listening method.
        """
        logger.info("RT_Listener stopping...")
        self.running = False
        self._drain_connection_handlers()

    def health_monitor(self):
        """
        Monitor the health of the RT_Listener, mirroring BC_Listener's health_monitor method.
        """
        while self.running:
            time.sleep(self.health_check_interval)
            current_time = time.time()
            idle_time = current_time - self.last_activity_time
            logger.info(f"RT_Listener health check - Idle time: {idle_time:.2f}s")
            if idle_time > 60:
                logger.warning("No activity detected in the last interval")
            else:
                logger.debug("Health check: RT_Listener is active")

    def is_healthy(self) -> bool:
        """
        Check if the RT_Listener is healthy, mirroring BC_Listener's is_healthy method.
        
        Returns:
            bool: True if the RT_Listener is healthy, False otherwise.
        """
        health_status = self.running and (time.time() - self.last_activity_time) < 60
        logger.info(f"RT_Listener health status: {'healthy' if health_status else 'unhealthy'}")
        return health_status
    
    def check_health(self) -> bool:
        """
        Report whether the RT_Listener can actually receive, not merely whether
        its thread is alive (story C6.1).

        This previously returned `self.running` alone. That flag is set when the
        listener thread starts and stays True while the thread spins in its
        socket-error retry loop, so a listener that failed to bind -- the exact
        outcome of starting a second instance while the first holds port 5001 --
        reported itself healthy with no socket at all. Confirmed live: the
        message-queue manager logged "RT_Listener=True" while the transport was
        dead and 192 errors had been emitted.

        Readiness needs both halves: the thread is running AND the socket is
        bound to the port we intended.

        Returns:
            bool: True if the RT_Listener is running and bound, False otherwise.
        """
        health_status = bool(self.running) and socket_is_bound(self.socket_variable, self.port)
        logger.debug(
            f"RT_Listener health: running={self.running} "
            f"bound={socket_is_bound(self.socket_variable, self.port)} -> "
            f"{'healthy' if health_status else 'unhealthy'}"
        )
        return health_status

# Global instances
rt_sender = RT_sender()
rt_listener = RT_Listener()

def get_rt_sender():
    """
    Get the global RT_sender instance, mirroring get_bc_sender.
    
    Returns:
        RT_sender: The global RT_sender instance.
    """
    logger.info("Getting RT_sender instance")
    return rt_sender

def get_rt_listener():
    """
    Get the global RT_Listener instance, mirroring get_bc_listener.
    
    Returns:
        RT_Listener: The global RT_Listener instance.
    """
    logger.info("Getting RT_Listener instance")
    return rt_listener
