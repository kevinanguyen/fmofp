"""
FCS Message Handler

Handles LOCAL Flight Control System-related messages using the 1553B messaging system.
Manages communication between the main system and the flight control system.
"""
import asyncio
import sys
import time
import uuid
import traceback
import threading
from typing import Dict, Any, Optional, List, Tuple, Union
import xml.etree.ElementTree as ET
from FMOFP.local_messaging.routing.handlers.sync_handler.AsyncMessageHandler import AsyncMessageHandler
from FMOFP.core.event_driven_communication import get_event_bus
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.local_messaging.message_types_fcs import (
    FCS_CONTROL_INPUT_REQUEST,
    FCS_CONTROL_INPUT_RESPONSE,
    FCS_ORIENTATION_DATA_REQUEST,
    FCS_ORIENTATION_DATA_RESPONSE,
    FCS_STATUS_REQUEST,
    FCS_STATUS_RESPONSE,
    FCS_MODE_CHANGE_REQUEST,
    FCS_MODE_CHANGE_RESPONSE
)
# Import message loop prevention middleware
from FMOFP.Utils.message_loop_prevention.middleware import get_loop_prevention_middleware, MessageLoopPreventionMiddleware
# Import 1553B messaging components
from FMOFP.MIL_STD_1553B.Messaging import send1553Msg
from FMOFP.MIL_STD_1553B.Bus_Controller.BC_messaging.BC_msg import BC_construct
from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS, get_fcs_command_word
from FMOFP.local_messaging.command_word_map import register_command_word, validate_command_word

logger = get_logger()

# The FCS wire encoding, in one place (B6, extended by C25.1).
#
# These were literal dicts here. They are now re-exported from
# FMOFP/local_messaging/fcs_wire_format.py, because the predefined-message
# classes need to produce the same words this handler decodes -- and because
# a second, contradictory FCS encoding existed in those classes'
# encode_1553b() methods. The names are FlightControlModes constants, which
# is what FlightControlSystem.set_mode() validates against; the wire works
# in numbers, the system works in names. See that module's docstring.
from FMOFP.local_messaging.fcs_wire_format import (
    MODE_NAME_TO_VALUE,
    MODE_VALUE_TO_NAME,
    CODE_TO_CONTROL_SURFACE,
    control_input_data_words,
    decode_control_value,
    mode_data_words,
)

# Import command word maps from main module
from FMOFP.local_messaging.command_word_map import (
    MODE_REQUEST_MAP, STATUS_REQUEST_MAP, DATA_REQUEST_MAP
)

class PendingRequest:
    """Class to track pending requests to the FCS"""
    def __init__(self, request_id: str, command_type: str, timestamp: float,
                 timeout: float = 5.0, system_name: str = None, data=None):
        self.request_id = request_id
        self.command_type = command_type
        self.timestamp = timestamp
        self.timeout = timeout
        # What this request WAS, so an expired one can actually be re-issued.
        # Neither field was stored before, which is why the retry branch in
        # _cleanup_pending_requests could only ever be the comment
        # "Implementation would depend on the specific request type": the
        # inputs send_request needs had been thrown away by then. It logged
        # "Retrying request X (retry 1/3)" and retried nothing. Measured: one
        # retry claim in the log, zero calls to send_request.
        self.system_name = system_name
        self.data = data
        self.retry_count = 0
        self.max_retries = 3
        self.response = None
        self.error = None
        self.completed = False
        
    def is_expired(self) -> bool:
        """Check if request has expired"""
        return time.time() > self.timestamp + self.timeout
        
    def should_retry(self) -> bool:
        """Check if request should be retried"""
        return not self.completed and self.retry_count < self.max_retries
        
    def increment_retry(self) -> int:
        """Increment retry count, restamp, and return the new count.

        The restamp is what the radar handler's PendingRequest already does
        and this one did not. Without it a retried request stays expired, so
        the 1 s cleanup tick burned all three retries within three seconds of
        the first timeout instead of giving each attempt its own window.
        """
        self.retry_count += 1
        self.timestamp = time.time()
        return self.retry_count
        
    def set_error_state(self, error: str) -> None:
        """Set error state"""
        self.error = error
        self.completed = True
        
    def set_response(self, response: Dict[str, Any]) -> None:
        """Set response data"""
        self.response = response
        self.completed = True

class FCSMessageHandler:
    """
    Message handler for the Flight Control System
    
    Processes incoming messages and routes them to the appropriate
    components of the Flight Control System.
    """
    def __init__(self):
        self.async_handler = None
        self.response_service = None
        self.fcs_db = None
        self.lock = threading.Lock()
        self.started = False
        self.pending_requests = {}  # request_id -> PendingRequest
        self.cleanup_timer = None
        # The loop timeout notifications are delivered on, captured in start().
        # The cleanup timer runs on a threading.Timer thread, which has no loop
        # of its own -- see _notify_request_timed_out().
        self._loop = None
        self.logger = logger.logger  # Direct access to logger
        self.event_bus = get_event_bus()
        
        # Initialize 1553B messaging components
        self.sendMsg = send1553Msg()
        self.bc_construct = BC_construct()
        
        # Initialize loop prevention middleware
        try:
            self.loop_prevention = get_loop_prevention_middleware()
            
            # Register FCS-specific message categories for loop prevention
            if self.loop_prevention:
                self._register_fcs_loop_prevention_categories()
                logger.info("FCS Message Handler integrated with enhanced loop prevention middleware")
            else:
                # Create a new middleware instance if global one not available
                self.loop_prevention = MessageLoopPreventionMiddleware()
                self._register_fcs_loop_prevention_categories()
                logger.info("Created dedicated FCS loop prevention middleware")
        except Exception as e:
            logger.error(f"Failed to initialize loop prevention middleware: {e}")
            self.loop_prevention = None
            
        # Import FCS components here to avoid circular imports
        from FMOFP.Systems.flightManagementSys.flightControlSys.flight_control_system import get_flight_control_system
        
        # Import the FCS command word mapping to ensure it's registered
        from FMOFP.local_messaging.command_word_map_fcs import register_fcs_command_words
        
        # Ensure FCS command words are registered
        register_fcs_command_words()
        
        self.fcs = get_flight_control_system()
            
    def _register_fcs_loop_prevention_categories(self):
        """Register FCS-specific message categories for loop prevention"""
        if not self.loop_prevention:
            return
            
        # Check if the middleware supports category registration
        if not hasattr(self.loop_prevention, 'register_category'):
            logger.warning("Loop prevention middleware does not support category registration - using default logic")
            return
            
        try:
            # Register message categories for FCS-specific message types
            categories = {
                'fcs_orientation': {'type': 'FCS data', 'priority': 'high', 'max_processing': 1},
                'fcs_control_input': {'type': 'FCS control', 'priority': 'highest', 'max_processing': 1},
                'fcs_status': {'type': 'FCS status', 'priority': 'medium', 'max_processing': 1},
                'fcs_mode': {'type': 'FCS control', 'priority': 'highest', 'max_processing': 1},
                'fcs_command': {'type': 'FCS control', 'priority': 'highest', 'max_processing': 1}
            }
            
            for category, settings in categories.items():
                self.loop_prevention.register_category(
                    category,
                    category_type=settings['type'],
                    priority=settings['priority'],
                    max_simultaneous_processing=settings['max_processing']
                )
                
            logger.info(f"Registered {len(categories)} FCS-specific categories with loop prevention middleware")
        except Exception as e:
            logger.error(f"Failed to register FCS categories with loop prevention middleware: {e}")
        
    async def send_request(self, system_name: str, request_type: str, data: Any = None, metadata: Optional[Dict[str, Any]] = None, *, inherited_retries: int = 0) -> Optional[str]:
        """
        Send a request to the FCS system.
        
        Args:
            system_name (str): Target system name (usually "flight_control_system")
            request_type (str): Type of request ("mode_change", "control_input", etc.)
            data (Any): Request data (mode name for mode_change, parameters for other types)
            metadata (dict): Optional metadata for request tracking
            
        Returns:
            str: Request ID for tracking and response correlation
        """
        logger.info(f"[FCS_MSGR] Sending {request_type} request to {system_name}")
        
        # Generate unique request ID for tracking
        request_id = str(uuid.uuid4())
        timestamp = time.time()
        
        # Initialize metadata if not provided
        if metadata is None:
            metadata = {}
        
        # Add request ID to metadata for tracing
        metadata["request_id"] = request_id
        metadata["timestamp"] = timestamp
        
        # Store pending request for tracking and timeout management
        pending_request = PendingRequest(
            request_id=request_id,
            command_type=request_type,
            timestamp=timestamp,
            system_name=system_name,
            data=data,
        )
        # Seeded on a re-issue so max_retries actually binds: send_request
        # mints a new request_id, so without carrying the count forward every
        # retry would start the budget again at zero.
        pending_request.retry_count = inherited_retries
        # Under the lock: _cleanup_pending_requests iterates this dict on a
        # threading.Timer thread while this runs on the event loop thread.
        # Only the cleanup side used to take the lock, which is worse than no
        # lock at all because it reads as guarded. Reproduced in 4 s:
        # "RuntimeError: dictionary changed size during iteration".
        # Narrow by design -- never held across an await (H5).
        with self.lock:
            self.pending_requests[request_id] = pending_request
        
        try:
            # Determine appropriate command word and data words based on request type
            command_word = None
            data_words = []
            msg_type = None
            
            if request_type == "mode_change":
                # Get appropriate command word from FCS_COMMAND_WORDS
                cmd_word_obj = get_fcs_command_word(FCS_MODE_CHANGE_REQUEST)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')
                else:
                    logger.error(f"[FCS_MSGR] No mode change command word found")
                    return None
                    
                # Pack mode data into binary format
                data_words = self._pack_mode_data(data)
                msg_type = "flight_control_systemCommand"
                logger.info(f"[FCS_MSGR] Using command word {command_word} for mode change to {data}")
                
            elif request_type == "status":
                # Get appropriate command word from FCS_COMMAND_WORDS
                cmd_word_obj = get_fcs_command_word(FCS_STATUS_REQUEST)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')
                else:
                    logger.error(f"[FCS_MSGR] No status request command word found")
                    return None
                    
                msg_type = "flight_control_systemStatusRequest"
                logger.info(f"[FCS_MSGR] Using command word {command_word} for status request")
                
            elif request_type == "control_input":
                # Get appropriate command word from FCS_COMMAND_WORDS
                cmd_word_obj = get_fcs_command_word(FCS_CONTROL_INPUT_REQUEST)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')
                else:
                    logger.error(f"[FCS_MSGR] No control input command word found")
                    return None
                    
                # Pack control input data into data words
                data_words = self._pack_control_input_data(data)
                msg_type = "flight_control_systemControlInputRequest"
                logger.info(f"[FCS_MSGR] Using command word {command_word} for control input")
                
            elif request_type == "orientation_data":
                # Get appropriate command word from FCS_COMMAND_WORDS
                cmd_word_obj = get_fcs_command_word(FCS_ORIENTATION_DATA_REQUEST)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')
                else:
                    logger.error(f"[FCS_MSGR] No orientation data command word found")
                    return None
                    
                msg_type = "flight_control_systemOrientationDataRequest"
                logger.info(f"[FCS_MSGR] Using command word {command_word} for orientation data request")
                
            else:
                logger.error(f"[FCS_MSGR] Unknown request type: {request_type}")
                return None
            
            # Create appropriate metadata for 1553B message
            msg_metadata = {
                'message_header': request_type,
                'command_name': f"FCS_{request_type.upper()}",
                'command_type': request_type,
                'message_type': msg_type,
                'sending_system': "FCSMessageHandler",
                'destination': system_name,
                'request_id': request_id
            }
            
            # Update with user-provided metadata
            if metadata:
                msg_metadata.update(metadata)
                
            # Send message through 1553B bus
            logger.info(f"[FCS_MSGR] Sending 1553B message: cmd={command_word}, data={data_words}")
            result = await self.sendMsg.send_message(command_word, data_words, request_id, msg_metadata)
            
            if result is None:
                logger.error("[FCS_MSGR] Failed to send message through 1553B")
                with self.lock:
                    self.pending_requests.pop(request_id, None)
                return None
                
            logger.info("[FCS_MSGR] Successfully sent message through 1553B")
            return request_id
            
        except Exception as e:
            logger.error(f"[FCS_MSGR] Error sending request: {str(e)}")
            logger.error(traceback.format_exc())
            
            # Clean up pending request on error
            with self.lock:
                self.pending_requests.pop(request_id, None)
                
            return None
            
    def _pack_mode_data(self, data: Any) -> list:
        """
        Pack FCS mode data into data words.

        Delegates to the shared encoding. The inline version here mapped an
        unrecognised mode name to 0, and 0 is NORMAL -- so a request for a
        mode the FCS does not have was transmitted as a command to enter
        NORMAL, and reported as sent. mode_data_words() raises instead; the
        caller is inside send_request's try/except, which drops the pending
        request and returns None, so the request is refused rather than
        silently turned into a different command.
        """
        if isinstance(data, dict):
            mode_name = data.get('mode_name', data.get('mode'))
            extra = data
        else:
            mode_name = data
            extra = None

        words = mode_data_words(mode_name, extra_params=extra)
        logger.info(f"[FCS_MSGR] Packed mode {mode_name} as {words}")
        return words

    def _pack_control_input_data(self, data: Any) -> list:
        """
        Pack control input data into data words.

        Delegates to the shared encoding. The inline version returned
        [0, 0] for a missing control type and code 0 for an unrecognised
        one; code 0 is not a surface, so the receiving end answered "no
        control surface specified" and ERROR -- after the request had been
        accepted, sent and given an ID. control_input_data_words() raises,
        which send_request's try/except turns into a refusal before
        anything reaches the bus.
        """
        if not isinstance(data, dict):
            raise ValueError(
                f"control input data must be a dict with 'control_type' and "
                f"'value', got {type(data).__name__}"
            )

        words = control_input_data_words(
            data.get('control_type'),
            data.get('value', 0.0),
            rate=data.get('rate'),
        )
        logger.info(
            f"[FCS_MSGR] Packed control input "
            f"{data.get('control_type')}={data.get('value', 0.0)} as {words}"
        )
        return words

    def set_async_handler(self, async_handler):
        """Set the async message handler"""
        self.async_handler = async_handler
        self._register_message_handlers()
        logger.info("FCS Message Handler connected to async handler")
        
    def set_response_service(self, response_service):
        """Set the response service"""
        self.response_service = response_service
        logger.info("FCS Message Handler connected to response service")
        
    def _register_message_handlers(self):
        """Register message handlers with the async handler"""
        if not self.async_handler:
            logger.warning("Cannot register FCS message handlers - async handler not set")
            return
            
        # Register the FCS system with the async handler first
        fcs_system = self.async_handler.register_system("flight_control_system")
        
        # Register handlers for various FCS message types with command words from FCS_COMMAND_WORDS
        # Map handlers to command words
        command_handlers = {
            FCS_MODE_CHANGE_REQUEST: self._handle_mode_change_request,
            FCS_MODE_CHANGE_RESPONSE: self._handle_mode_change_response,
            FCS_CONTROL_INPUT_REQUEST: self._handle_control_input_request,
            FCS_CONTROL_INPUT_RESPONSE: self._handle_control_input_response,
            FCS_ORIENTATION_DATA_REQUEST: self._handle_orientation_data_request,
            FCS_ORIENTATION_DATA_RESPONSE: self._handle_orientation_data_response,
            FCS_STATUS_REQUEST: self._handle_status_request,
            FCS_STATUS_RESPONSE: self._handle_status_response
        }
        
        # Register each handler with the appropriate command word
        for command_name, handler in command_handlers.items():
            try:
                cmd_word_obj = get_fcs_command_word(command_name)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')  # Convert to binary string
                    fcs_system.register_handler(command_word, handler)
                    logger.info(f"Registered handler for {command_name} with command word {command_word}")
                else:
                    logger.warning(f"Command name {command_name} not found in FCS_COMMAND_WORDS")
            except Exception as e:
                logger.error(f"Error registering handler for {command_name}: {e}")
        
        logger.info("FCS message handlers registered")
        
    async def _handle_mode_change_request(self, message):
        """
        Handle FCS mode change request
        
        Args:
            message: The mode change request
            
        Returns:
            dict: Mode change result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                # BLOCKER B6/B7 (gate): this passed transaction_id= and
                # category= to process_message(), whose signature is
                # (message, service_name) -- in the middleware, in the prevention
                # service beneath it, and at every other call site in the codebase.
                # Every FCS message arriving over 1553B therefore raised TypeError
                # here, at the loop-prevention gate, before reaching any FCS code
                # at all -- mode change, control input, orientation data and status
                # alike. The category is derived from the message by the prevention
                # service itself (_extract_category), and the transaction id travels
                # in the message metadata, which is how RadarMessageHandler does it.
                transaction_id = self._create_transaction_id(message, "mode_change")
                if isinstance(message, dict):
                    message.setdefault('metadata', {})
                    if isinstance(message['metadata'], dict):
                        message['metadata'].setdefault('transaction_id', transaction_id)
                should_process, enhanced_message = self.loop_prevention.process_message(
                    message,
                    "fcs_mode_handler"
                )
                if not should_process:
                    logger.warning(f"Breaking loop - FCS mode change message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                    
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
                
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid mode change message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract mode value from the binary data word
            try:
                mode_value = None
                if 'data' in message and isinstance(message['data'], str):
                    # Data is a binary string
                    mode_value = int(message['data'], 2)
                    logger.info(f"[FCS] Extracted mode value from binary data: {mode_value}")
                elif 'parameters' in message and 'mode_name' in message['parameters']:
                    # Data is in parameters dictionary
                    mode_name = message['parameters']['mode_name']
                    # Third inline copy of this table, now the shared one
                    # (fcs_wire_format.MODE_NAME_TO_VALUE). The `.get(..., 0)`
                    # default this replaces mapped any unrecognised mode name
                    # to 0, which is NORMAL -- so a command naming a mode the
                    # FCS does not have put the aircraft into NORMAL and
                    # answered SUCCESS. An unknown name is now refused, which
                    # is what the numeric path a few lines down already does
                    # for an out-of-range mode value.
                    mode_value = MODE_NAME_TO_VALUE.get(str(mode_name).upper())
                    if mode_value is None:
                        logger.warning(
                            f"[FCS] '{mode_name}' is not an FCS mode; the FCS "
                            f"modes are "
                            f"{sorted(n for n in MODE_NAME_TO_VALUE if n != 'STANDBY')}")
                        return {"status": "ERROR",
                                "message": f"Unknown mode: {mode_name}"}
                    logger.info(f"[FCS] Extracted mode value from parameters: {mode_value} ({mode_name})")
                else:
                    logger.warning("[FCS] No mode value found in message")
                    return {"status": "ERROR", "message": "No mode value specified"}
            except Exception as e:
                logger.error(f"[FCS] Error extracting mode value: {e}")
                return {"status": "ERROR", "message": f"Error extracting mode value: {str(e)}"}
                
            request_id = message.get('request_id', None)
            
            logger.info(f"[FCS] Handling mode change request with mode value: {mode_value} (request_id: {request_id})")
            
            # Process mode change through FCS
            if self.fcs:
                old_mode = self.fcs.mode

                # BLOCKER B6: this was `result = self.fcs.change_mode(mode_value)`
                # followed by `result.get('success', False)`. FlightControlSystem
                # has no change_mode() -- the name appeared exactly once in the
                # whole repository, at that call site -- so EVERY mode change
                # command arriving over 1553B raised AttributeError, was
                # swallowed by the outer handler, and returned ERROR. Worse, the
                # exception fired before send_response() below, so the requester
                # got no response at all and waited out its timeout. The real
                # method is set_mode(), and it takes a mode NAME while everything
                # upstream of here works in the numeric wire encoding, so the
                # value has to be mapped back before the call.
                mode_name = MODE_VALUE_TO_NAME.get(mode_value)
                if mode_name is None:
                    logger.warning(
                        f"[FCS] Mode value {mode_value} is outside the defined "
                        f"range {sorted(MODE_VALUE_TO_NAME)}")
                    success = False
                    detail = f"Unknown mode value: {mode_value}"
                else:
                    # set_mode returns a plain bool (declared `-> bool`), not a
                    # dict. Reading it as a dict is what B7 does in the control
                    # input path below.
                    success = bool(self.fcs.set_mode(mode_name, request_id=request_id))
                    detail = (f"Mode set to {mode_name}" if success
                              else f"FCS rejected mode {mode_name}")

                new_mode = self.fcs.mode

                # Create response
                response = {
                    "status": "SUCCESS" if success else "ERROR",
                    "message": detail,
                    "request_id": request_id,
                    "old_mode": old_mode,
                    "new_mode": new_mode
                }
                
                # Send response
                if self.response_service:
                    await self.response_service.send_response(request_id, response)
                
                logger.info(f"[FCS] Mode change {'successful' if success else 'failed'}: {old_mode} -> {new_mode}")
                
                # Return result for internal processing
                return response
            else:
                logger.warning("[FCS] Flight Control System not available, mode change not processed")
                return {"status": "ERROR", "message": "Flight Control System not available"}
                
        except Exception as e:
            logger.error(f"[FCS] Error handling mode change request: {e}")
            logger.error(traceback.format_exc())
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_mode_change_response(self, message):
        """
        Handle FCS mode change response
        
        Args:
            message: The mode change response
            
        Returns:
            dict: Status of processing
        """
        try:
            # This method mainly processes incoming responses from the FCS back to requestors
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid mode change response format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            # Complete pending request if it exists
            with self.lock:
                pending_request = self.pending_requests.pop(request_id, None)
            if pending_request is not None:
                pending_request.set_response(message)
                logger.info(f"[FCS] Completed pending request: {request_id}")
                
            return {"status": "SUCCESS", "message": "Mode change response processed"}
            
        except Exception as e:
            logger.error(f"[FCS] Error handling mode change response: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_control_input_request(self, message):
        """
        Handle FCS control input request
        
        Args:
            message: The control input request
            
        Returns:
            dict: Control input result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                # BLOCKER B6/B7 (gate): this passed transaction_id= and
                # category= to process_message(), whose signature is
                # (message, service_name) -- in the middleware, in the prevention
                # service beneath it, and at every other call site in the codebase.
                # Every FCS message arriving over 1553B therefore raised TypeError
                # here, at the loop-prevention gate, before reaching any FCS code
                # at all -- mode change, control input, orientation data and status
                # alike. The category is derived from the message by the prevention
                # service itself (_extract_category), and the transaction id travels
                # in the message metadata, which is how RadarMessageHandler does it.
                transaction_id = self._create_transaction_id(message, "control_input")
                if isinstance(message, dict):
                    message.setdefault('metadata', {})
                    if isinstance(message['metadata'], dict):
                        message['metadata'].setdefault('transaction_id', transaction_id)
                should_process, enhanced_message = self.loop_prevention.process_message(
                    message,
                    "fcs_control_input_handler"
                )
                if not should_process:
                    logger.warning(f"Breaking loop - FCS control input message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                    
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
                
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid control input message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract control input data
            control_surface = None
            control_value = 0.0
            
            # Check if control data is in parameters or data words
            if 'parameters' in message:
                control_surface = message['parameters'].get('control_type')
                control_value = message['parameters'].get('value', 0.0)
            elif 'data' in message:
                # Try to parse binary data words
                data_words = message.get('data_words', [])
                if len(data_words) >= 2:
                    try:
                        control_code = int(data_words[0], 2)
                        scaled_value = int(data_words[1], 2)

                        # Both the code table and the scaling now come from
                        # fcs_wire_format, so the encoder on the other end
                        # cannot drift from this decoder. The dict and the
                        # /500.0 - 1.0 arithmetic used to be written out
                        # here, which is how a second encoding in the
                        # predefined-message classes managed to disagree
                        # with this one on every field.
                        control_surface = CODE_TO_CONTROL_SURFACE.get(control_code)
                        control_value = decode_control_value(scaled_value)
                    except Exception as e:
                        logger.error(f"[FCS] Error parsing control input data words: {e}")
                        return {"status": "ERROR", "message": f"Error parsing control input data: {str(e)}"}
            
            if not control_surface:
                logger.warning("[FCS] No control surface specified in control input request")
                return {"status": "ERROR", "message": "No control surface specified"}
                
            request_id = message.get('request_id', None)
            
            logger.info(f"[FCS] Handling control input request: {control_surface}={control_value} (request_id: {request_id})")
            
            # Process control input through FCS
            if self.fcs:
                # BLOCKER B7: this was `success = result.get('success', False)`.
                # set_control_input is declared `-> bool` and returns True/False,
                # so `.get` on it raised AttributeError -- caught by the outer
                # handler, reported as ERROR, and send_response() below never
                # reached. The control surface had ALREADY been moved by then,
                # so a stick or throttle command took effect on the aircraft
                # while the console reported failure and the request never
                # completed. For a training system that is the worst available
                # outcome: trainee and instructor see different aircraft.
                success = bool(self.fcs.set_control_input(control_surface, control_value,
                                                          request_id=request_id))

                # set_control_input clamps: throttle to [0, 1], everything else
                # to [-1, 1]. Report what the FCS actually holds rather than
                # echoing the request back as if it were applied verbatim.
                actual_value = self.fcs.control_inputs.get(control_surface, control_value)

                # Create response
                response = {
                    "status": "SUCCESS" if success else "ERROR",
                    "message": (f"{control_surface} set to {actual_value}" if success
                                else f"FCS rejected control input for {control_surface}"),
                    "request_id": request_id,
                    "control_surface": control_surface,
                    "control_value": control_value,
                    "actual_value": actual_value
                }
                
                # Send response
                if self.response_service:
                    await self.response_service.send_response(request_id, response)
                
                logger.info(f"[FCS] Control input {'set successfully' if success else 'failed'}")
                
                # Return result for internal processing
                return response
            else:
                logger.warning("[FCS] Flight Control System not available, control input not processed")
                return {"status": "ERROR", "message": "Flight Control System not available"}
                
        except Exception as e:
            logger.error(f"[FCS] Error handling control input request: {e}")
            logger.error(traceback.format_exc())
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_control_input_response(self, message):
        """
        Handle FCS control input response
        
        Args:
            message: The control input response
            
        Returns:
            dict: Status of processing
        """
        try:
            # This method mainly processes incoming responses from the FCS back to requestors
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid control input response format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            # Complete pending request if it exists
            with self.lock:
                pending_request = self.pending_requests.pop(request_id, None)
            if pending_request is not None:
                pending_request.set_response(message)
                logger.info(f"[FCS] Completed pending request: {request_id}")
                
            return {"status": "SUCCESS", "message": "Control input response processed"}
            
        except Exception as e:
            logger.error(f"[FCS] Error handling control input response: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_orientation_data_request(self, message):
        """
        Handle FCS orientation data request
        
        Args:
            message: The orientation data request
            
        Returns:
            dict: Orientation data result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                # BLOCKER B6/B7 (gate): this passed transaction_id= and
                # category= to process_message(), whose signature is
                # (message, service_name) -- in the middleware, in the prevention
                # service beneath it, and at every other call site in the codebase.
                # Every FCS message arriving over 1553B therefore raised TypeError
                # here, at the loop-prevention gate, before reaching any FCS code
                # at all -- mode change, control input, orientation data and status
                # alike. The category is derived from the message by the prevention
                # service itself (_extract_category), and the transaction id travels
                # in the message metadata, which is how RadarMessageHandler does it.
                transaction_id = self._create_transaction_id(message, "orientation_data")
                if isinstance(message, dict):
                    message.setdefault('metadata', {})
                    if isinstance(message['metadata'], dict):
                        message['metadata'].setdefault('transaction_id', transaction_id)
                should_process, enhanced_message = self.loop_prevention.process_message(
                    message,
                    "fcs_orientation_data_handler"
                )
                if not should_process:
                    logger.warning(f"Breaking loop - FCS orientation data message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                    
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
                
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid orientation data message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            logger.info(f"[FCS] Handling orientation data request (request_id: {request_id})")
            
            # Get orientation data through FCS
            if self.fcs:
                orientation_data = self.fcs.get_orientation_data()
                
                # Create response
                response = {
                    "status": "SUCCESS",
                    "message": "Orientation data retrieved",
                    "request_id": request_id,
                    "data": orientation_data
                }
                
                # Send response
                if self.response_service:
                    await self.response_service.send_response(request_id, response)
                
                logger.info(f"[FCS] Orientation data retrieved successfully")
                
                # Return result for internal processing
                return response
            else:
                logger.warning("[FCS] Flight Control System not available, orientation data not retrieved")
                return {"status": "ERROR", "message": "Flight Control System not available"}
                
        except Exception as e:
            logger.error(f"[FCS] Error handling orientation data request: {e}")
            logger.error(traceback.format_exc())
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_orientation_data_response(self, message):
        """
        Handle FCS orientation data response
        
        Args:
            message: The orientation data response
            
        Returns:
            dict: Status of processing
        """
        try:
            # This method mainly processes incoming responses from the FCS back to requestors
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid orientation data response format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            # Complete pending request if it exists
            with self.lock:
                pending_request = self.pending_requests.pop(request_id, None)
            if pending_request is not None:
                pending_request.set_response(message)
                logger.info(f"[FCS] Completed pending request: {request_id}")
                
            return {"status": "SUCCESS", "message": "Orientation data response processed"}
            
        except Exception as e:
            logger.error(f"[FCS] Error handling orientation data response: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_status_request(self, message):
        """
        Handle FCS status request
        
        Args:
            message: The status request
            
        Returns:
            dict: Status result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                # BLOCKER B6/B7 (gate): this passed transaction_id= and
                # category= to process_message(), whose signature is
                # (message, service_name) -- in the middleware, in the prevention
                # service beneath it, and at every other call site in the codebase.
                # Every FCS message arriving over 1553B therefore raised TypeError
                # here, at the loop-prevention gate, before reaching any FCS code
                # at all -- mode change, control input, orientation data and status
                # alike. The category is derived from the message by the prevention
                # service itself (_extract_category), and the transaction id travels
                # in the message metadata, which is how RadarMessageHandler does it.
                transaction_id = self._create_transaction_id(message, "status")
                if isinstance(message, dict):
                    message.setdefault('metadata', {})
                    if isinstance(message['metadata'], dict):
                        message['metadata'].setdefault('transaction_id', transaction_id)
                should_process, enhanced_message = self.loop_prevention.process_message(
                    message,
                    "fcs_status_handler"
                )
                if not should_process:
                    logger.warning(f"Breaking loop - FCS status message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                    
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
                
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid status request format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            logger.info(f"[FCS] Handling status request (request_id: {request_id})")
            
            # Get status through FCS
            if self.fcs:
                status_data = self.fcs.get_status()
                
                # Create response
                response = {
                    "status": "SUCCESS",
                    "message": "Status retrieved",
                    "request_id": request_id,
                    "data": status_data
                }
                
                # Send response
                if self.response_service:
                    await self.response_service.send_response(request_id, response)
                
                logger.info(f"[FCS] Status retrieved successfully")
                
                # Return result for internal processing
                return response
            else:
                logger.warning("[FCS] Flight Control System not available, status not retrieved")
                return {"status": "ERROR", "message": "Flight Control System not available"}
                
        except Exception as e:
            logger.error(f"[FCS] Error handling status request: {e}")
            logger.error(traceback.format_exc())
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
            
    async def _handle_status_response(self, message):
        """
        Handle FCS status response
        
        Args:
            message: The status response
            
        Returns:
            dict: Status of processing
        """
        try:
            # This method mainly processes incoming responses from the FCS back to requestors
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid status response format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            # Complete pending request if it exists
            with self.lock:
                pending_request = self.pending_requests.pop(request_id, None)
            if pending_request is not None:
                pending_request.set_response(message)
                logger.info(f"[FCS] Completed pending request: {request_id}")
                
            return {"status": "SUCCESS", "message": "Status response processed"}
            
        except Exception as e:
            logger.error(f"[FCS] Error handling status response: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    def _create_transaction_id(self, message, message_subtype):
        """
        Create a unique transaction ID for message loop prevention
        
        Args:
            message (dict): The message to create a transaction ID for
            message_subtype (str): The subtype of the message (e.g., 'command', 'mode_change')
            
        Returns:
            str: A unique transaction ID
        """
        # Extract key fields for transaction ID
        if isinstance(message, dict):
            message_type = message.get('message_type', '')
            request_id = message.get('request_id', '')
            command_type = message.get('command_type', '')
            
            # Use existing transaction ID if available
            if message.get('metadata', {}).get('transaction_id'):
                existing_id = message.get('metadata', {}).get('transaction_id')
                # Only reuse if it's for the same message type to avoid cross-type loop issues
                if f"_{message_subtype}" in existing_id:
                    return existing_id
            
            # Create a unique composite ID
            return f"{request_id}_{message_type}_{command_type}_{message_subtype}"
        
        # Fallback for non-dict messages
        return f"{id(message)}_{message_subtype}"
    

    def _remember_loop(self):
        """Record the event loop to deliver timeout notifications on.

        start() is awaited from SystemManager.start_async_component, so it runs
        on the loop thread and there is a running loop to capture here. The
        cleanup timer that needs it runs on a threading.Timer thread, where
        there is not -- which was H7.
        """
        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            # Started off the loop (a direct call, or a test). Notifications
            # cannot be delivered until a loop is known; say so once, here,
            # rather than raising once per expired request later.
            self._loop = None
            logger.warning(
                f"[FCS_MSG_HNDLR] Started without a running event loop: request "
                f"timeout notifications will be dropped until one is set"
            )

    def _notify_request_timed_out(self, request_id, response) -> bool:
        """Deliver a timeout notification from the cleanup timer's thread.

        Returns False when it could not be delivered. Never raises: a failure
        here must not abort the rest of the expired batch, which is half of
        what made H7 bite.
        """
        loop = getattr(self, '_loop', None)
        if loop is None or loop.is_closed():
            logger.error(
                f"[FCS_MSG_HNDLR] Cannot notify {request_id} of its timeout: no "
                f"usable event loop. The request was still expired and removed."
            )
            return False
        if self.response_service is None:
            logger.error(
                f"[FCS_MSG_HNDLR] Cannot notify {request_id} of its timeout: no "
                f"response service"
            )
            return False
        try:
            # The coroutine is built only once the loop is known to be usable,
            # so there is no path that creates one and drops it unawaited.
            asyncio.run_coroutine_threadsafe(
                self.response_service.send_response(request_id, response), loop)
            return True
        except Exception as e:
            logger.error(
                f"[FCS_MSG_HNDLR] Failed to notify {request_id} of its timeout: "
                f"{e}"
            )
            return False

    def _start_cleanup_timer(self):
        """Arm the one-shot timer that expires stale pending requests.

        The guard here used to be `if self.cleanup_timer.is_alive(): return`.
        This method is re-armed from the `finally` block of
        _cleanup_pending_requests -- which runs ON THE TIMER'S OWN THREAD, where
        that Timer's is_alive() is True. So the guard returned early every
        single time and the timer never re-armed: cleanup ran ONCE, one second
        after start(), and never again. Measured directly -- one pass, one
        "Retrying request" line, then `cleanup_timer.is_alive() == False`
        forever.

        The replacement compares against the thread actually running, so a
        re-arm from inside the timer proceeds while a duplicate call from
        elsewhere is still refused.
        """
        existing = self.cleanup_timer
        if (existing is not None and existing.is_alive()
                and existing is not threading.current_thread()):
            return  # a different, live timer is already pending

        if not self.started:
            return  # stopped between the pass finishing and this re-arm

        # Start the timer BEFORE publishing it, not after.
        #
        # This was three statements on self.cleanup_timer: construct, set
        # daemon, start. A freshly constructed Thread is not alive until
        # start() returns, and start() blocks until the new thread is
        # actually running -- so for that whole window self.cleanup_timer
        # held a timer whose is_alive() was False, visible to anything
        # checking whether a cleanup pass was still pending.
        #
        # Measured on the real handler with a tight sampler over six
        # passes: 43,196 of 27,462,452 samples (0.16% of wall time) saw the
        # slot holding a not-alive timer, all of it clustered on the
        # one-second firing boundaries. test_request_expiry samples exactly
        # there -- it sleeps 4.0s, a whole multiple of the interval -- and
        # CI caught it: "FAIL and a timer is still pending at the end",
        # once, with the other two checks in the same test passing.
        #
        # Binding locally and publishing last removes the window entirely:
        # until the assignment an observer sees the OUTGOING timer, whose
        # thread is still running this very method from its finally block,
        # so is_alive() is True throughout. There is never an instant where
        # the slot holds a dead timer while the chain is in fact alive.
        timer = threading.Timer(1.0, self._cleanup_pending_requests)
        timer.daemon = True
        timer.start()
        self.cleanup_timer = timer
    
    def _cleanup_pending_requests(self):
        """Clean up expired pending requests"""
        try:
            with self.lock:
                expired_requests = []
                retry_requests = []
                
                # Identify expired and retry requests
                for request_id, request in list(self.pending_requests.items()):
                    if request.is_expired():
                        if request.should_retry():
                            retry_requests.append(request_id)
                        else:
                            expired_requests.append(request_id)
                
                # Give an expired request another window, and say plainly
                # that nothing was re-sent.
                #
                # This branch used to log "Retrying request X (retry 1/3)" with
                # no re-issue behind it -- measured: one retry claim in the log,
                # zero calls to send_request. PendingRequest did not store
                # system_name or data, so there was nothing to re-send from;
                # that is what the comment "Implementation would depend on the
                # specific request type" was really recording. Both fields are
                # captured now, so a re-issue is finally POSSIBLE.
                #
                # It is still not DONE, deliberately. Nothing marks a request
                # complete when its response arrives -- set_response() and
                # set_completed() have no callers in this handler -- so every
                # successfully-sent request sits here until it expires whether
                # or not it was answered. Measured: a request the bus accepted
                # was still pending 28 s later with no response recorded.
                # Re-sending on that basis would put four copies of every
                # command on a 1553B bus. The missing completion path is the
                # blocker and is tracked separately; until it exists, this
                # waits rather than retransmits, and the log says so.
                for request_id in retry_requests:
                    request = self.pending_requests[request_id]
                    attempt = request.increment_retry()
                    logger.warning(
                        f"Request {request_id} ({request.command_type}) timed "
                        f"out with no response; extending its window "
                        f"({attempt}/{request.max_retries}). NOT re-sent: this "
                        f"handler has no response-completion path, so a resend "
                        f"could duplicate a command that did arrive."
                    )
                
                # Remove expired requests
                for request_id in expired_requests:
                    request = self.pending_requests.pop(request_id)
                    # request.max_retries here always printed "3", so a
                    # request that expired on its first window reported
                    # "expired after 3 retries". The real count is retry_count.
                    logger.warning(
                        f"Request {request_id} ({request.command_type}) "
                        f"expired after {request.retry_count} of "
                        f"{request.max_retries} windows with no response"
                    )
                    
                    # Notify of failure
                    if request.command_type and self.response_service:
                        response = {
                            "status": "ERROR",
                            "request_id": request_id,
                            "message": "Request timed out",
                            "data": {}
                        }
                        
                        # Send failure response.
                        #
                        # H7: this used to pass asyncio.get_event_loop() here.
                        # _cleanup_pending_requests runs on a threading.Timer
                        # thread, and get_event_loop() is per-thread: with no
                        # loop set on THAT thread it raises
                        # "RuntimeError: There is no current event loop in
                        # thread 'Thread-N'" -- measured on 3.11, and it raises
                        # even while a loop is running happily in another
                        # thread, which is exactly this program's shape.
                        #
                        # Two things were lost each time. The notification: the
                        # coroutine argument is built BEFORE get_event_loop() is
                        # evaluated, so it was created and discarded unawaited
                        # ("coroutine 'send_response' was never awaited") -- the
                        # same defect class as H4. And the rest of the batch:
                        # the exception escaped to the outer handler, so every
                        # expired request after the first was skipped, having
                        # already been popped above.
                        self._notify_request_timed_out(request_id, response)
        except Exception as e:
            logger.error(f"Error cleaning up pending requests: {e}")
        finally:
            # Restart the timer if still running
            if self.started:
                self._start_cleanup_timer()
    
    def start(self):
        """Start the FCS message handler"""
        if self.started:
            logger.warning("FCS Message Handler already started")
            return False
            
        self.started = True
        
        # Capture the loop BEFORE the timer that needs it is started.
        self._remember_loop()

        # Start cleanup timer
        self._start_cleanup_timer()
        
        logger.info("FCS Message Handler started")
        return True
    
    def stop(self):
        """Stop the FCS message handler"""
        if not self.started:
            logger.warning("FCS Message Handler already stopped")
            return False
            
        self.started = False
        
        # Stop cleanup timer
        if self.cleanup_timer:
            self.cleanup_timer.cancel()
            self.cleanup_timer = None
        
        logger.info("FCS Message Handler stopped")
        return True
    
    def is_healthy(self):
        """Check if handler is healthy"""
        fcs_healthy = self.fcs and hasattr(self.fcs, 'check_health') and self.fcs.check_health()
        
        return self.started and fcs_healthy

# Singleton instance
_fcs_message_handler = None

def get_fcs_message_handler():
    """Get singleton instance of FCS Message Handler"""
    global _fcs_message_handler
    if _fcs_message_handler is None:
        _fcs_message_handler = FCSMessageHandler()
    return _fcs_message_handler
