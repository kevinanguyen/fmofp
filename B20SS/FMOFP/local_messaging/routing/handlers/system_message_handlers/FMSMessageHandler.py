"""
FMS Message Handler

Handles LOCAL FMS-related messages using the 1553B messaging system.
Manages LOCAL communication between the main system and the flight management system.
NOT a remote communication system. (See fmsMessenger.py for FMS Messenger)
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
from FMOFP.local_messaging.messageConfigurations.fms_messages import (
    create_fms_message, 
    FMSModeChangeRequest, 
    FMSCommandRequest
)
# Import message loop prevention middleware
from FMOFP.Utils.message_loop_prevention.middleware import get_loop_prevention_middleware, MessageLoopPreventionMiddleware
# Import 1553B messaging components
from FMOFP.MIL_STD_1553B.Messaging import send1553Msg
from FMOFP.MIL_STD_1553B.Bus_Controller.BC_messaging.BC_msg import BC_construct
from FMOFP.local_messaging.command_word_map_fms import FMS_COMMAND_WORDS
from FMOFP.local_messaging.command_word_map import register_command_word, validate_command_word

logger = get_logger()

# Import command word maps from main module
# The control-input encoding is the FCS's: this handler forwards control
# input to the FCS (see the FCS_CONTROL_INPUT_REQUEST command word below).
# The mode encoding is the FMS's own, which is a different vocabulary -- see
# _pack_data.
from FMOFP.local_messaging.fcs_wire_format import control_input_data_words
from FMOFP.local_messaging.messageConfigurations.fms_tactical_data import (
    fms_mode_data_words,
)
from FMOFP.local_messaging.command_word_map import (
    MODE_REQUEST_MAP, STATUS_REQUEST_MAP, DATA_REQUEST_MAP
)

class PendingRequest:
    """Class to track pending requests to the FMS"""
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

class FMSMessageHandler:
    """
    Message handler for the Flight Management System

    Processes incoming messages and routes them to the appropriate
    components of the Flight Management System.

    WHAT IS AND IS NOT LIVE HERE (measured 2026-10-25, on d0e8abc)
    --------------------------------------------------------------
    Read this before adding to the request/response machinery below. It was
    picked off the backlog as "nothing correlates an inbound response to an
    FMS-originated request", which is true and also beside the point: nothing
    ORIGINATES requests through this handler either.

      * send_request() has ZERO callers in the package. The predefined-message
        facade (Interfaces/predefinedMessages/fms_messages.py) sends all five
        of its FMS families through RadarMessageHandler.send_request instead.
      * Consequently PendingRequest, set_response(), set_completed(),
        _cleanup_pending_requests() and the cleanup timer form a parallel
        request/response subsystem with no production traffic. Measured: after
        two facade sends that both returned request IDs, this handler's
        pending_requests held 0 entries.
      * None of the seven _handle_* methods below fires on a live boot.
        Measured by wrapping all seven and booting the real application
        through Tests/live_system.py: 0 calls each.
      * Only three of the seven are even registered, because the FMS's 18
        command-word NAMES collapse onto 6 distinct VALUES --
        register_command_word() derives the word count from the message type,
        and all six FMS data types fall through to the same default of 3 --
        while AsyncMessageHandler.register_handler silently skips a word that
        is already taken. The surviving map is:

            ...0001  _handle_status_request    FMS_STATUS_REQUEST
            ...0010  _handle_fms_command       FMS_MODE_CHANGE, FMS_COMMAND
            ...0011  _handle_attitude_update   FMS_FLIGHT_DATA, UPDATE_ATTITUDE,
                                               UPDATE_NAVIGATION, UPDATE_VELOCITY,
                                               TACTICAL_STATUS, EXECUTE_MANEUVER

        so _handle_mode_change, _handle_navigation_update,
        _handle_execute_maneuver and _handle_generic_fms_message are
        unreachable. _handle_mode_change is additionally registered under the
        key "FMS_SET_MODE", which is not a name in FMS_COMMAND_WORDS at all,
        so it is skipped with a warning.
      * No FMS RESPONSE command word is registered as a handler.

    The live FMS request path does correlate: those three facade sends had
    their pending entries removed from RadarMessageHandler within 1 s, checked
    again at +5 s and +15 s.

    So the gap is not a missing completion step. Before building one here,
    decide whether this handler is meant to serve inbound FMS 1553B traffic at
    all. If it is, the word collision and the FMS_SET_MODE key have to be
    fixed first, and C24.1 with them -- waking _handle_attitude_update means
    an inbound execute-maneuver command gets handled by the attitude updater,
    which reports SUCCESS for an update it never applied. Making these
    handlers reachable while that holds would turn a latent defect into a live
    one.
    """
    def __init__(self):
        self.async_handler = None
        self.response_service = None
        self.fms_db = None
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
            
            # Register FMS-specific message categories for loop prevention
            if self.loop_prevention:
                self._register_fms_loop_prevention_categories()
                logger.info("FMS Message Handler integrated with enhanced loop prevention middleware")
            else:
                # Create a new middleware instance if global one not available
                self.loop_prevention = MessageLoopPreventionMiddleware()
                self._register_fms_loop_prevention_categories()
                logger.info("Created dedicated FMS loop prevention middleware")
        except Exception as e:
            logger.error(f"Failed to initialize loop prevention middleware: {e}")
            self.loop_prevention = None
            
        # Import FMS components here to avoid circular imports
        from FMOFP.Systems.flightManagementSys.flightManagementSystem import get_flightManagementSystem
        from FMOFP.Systems.flightManagementSys.fmsControl import get_fms_control
        from FMOFP.Systems.flightManagementSys.fmsMessenger import get_fms_messenger
        
        # Import the FMS command word mapping to ensure it's registered
        from FMOFP.local_messaging.command_word_map_fms import register_fms_command_words
        
        # Ensure FMS command words are registered
        register_fms_command_words()
        
        self.fms = get_flightManagementSystem()
        self.fms_control = get_fms_control()
        self.fms_messenger = get_fms_messenger()
        
        # Connect components
        self.fms_messenger.set_fms(self.fms)
        self.fms_messenger.set_fms_control(self.fms_control)
            
    def _register_fms_loop_prevention_categories(self):
        """Register FMS-specific message categories for loop prevention"""
        if not self.loop_prevention:
            return
            
        # Check if the middleware supports category registration
        if not hasattr(self.loop_prevention, 'register_category'):
            logger.warning("Loop prevention middleware does not support category registration - using default logic")
            return
            
        try:
            # Register message categories for FMS-specific message types
            categories = {
                'fms_attitude': {'type': 'FMS data', 'priority': 'high', 'max_processing': 1},
                'fms_navigation': {'type': 'FMS data', 'priority': 'high', 'max_processing': 1},  
                'fms_velocity': {'type': 'FMS data', 'priority': 'high', 'max_processing': 1},
                'fms_tactical': {'type': 'FMS data', 'priority': 'high', 'max_processing': 1},
                'fms_status': {'type': 'FMS status', 'priority': 'medium', 'max_processing': 1},
                'fms_mode': {'type': 'FMS control', 'priority': 'highest', 'max_processing': 1},
                'fms_command': {'type': 'FMS control', 'priority': 'highest', 'max_processing': 1},
                'fms_maneuver': {'type': 'FMS control', 'priority': 'highest', 'max_processing': 1}
            }
            
            for category, settings in categories.items():
                self.loop_prevention.register_category(
                    category,
                    category_type=settings['type'],
                    priority=settings['priority'],
                    max_simultaneous_processing=settings['max_processing']
                )
                
            logger.info(f"Registered {len(categories)} FMS-specific categories with loop prevention middleware")
        except Exception as e:
            logger.warning(f"Failed to register FMS categories with loop prevention middleware: {e}")
        
    async def send_request(self, system_name: str, request_type: str, data: Any = None, metadata: Optional[Dict[str, Any]] = None, *, inherited_retries: int = 0) -> Optional[str]:
        """
        Send a request to the FMS system.
        
        Args:
            system_name (str): Target system name (usually "flightManagementSystem")
            request_type (str): Type of request ("mode_change", "status", etc.)
            data (Any): Request data (mode name for mode_change, parameters for other types)
            metadata (dict): Optional metadata for request tracking
            
        Returns:
            str: Request ID for tracking and response correlation
        """
        logger.info(f"[FMS_MSGR_HNDLR] Sending {request_type} request to {system_name}")
        
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
                # Use FMS_SET_MODE command word
                command_word = MODE_REQUEST_MAP.get(system_name)
                if not command_word:
                    logger.error(f"[FMS_MSGR_HNDLR] No mode command word found for {system_name}")
                    return None
                    
                # Pack mode data into binary format
                data_words = self._pack_data(data)
                msg_type = f"{system_name}Command"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for mode change to {data}")
                
            elif request_type == "status":
                command_word = STATUS_REQUEST_MAP.get(system_name)
                if not command_word:
                    logger.error(f"[FMS_MSGR_HNDLR] No status command word found for {system_name}")
                    return None
                    
                msg_type = f"{system_name}Status"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for status request")
                
            elif request_type == "attitude_update":
                # Use FMS_UPDATE_ATTITUDE command word
                command_word = format(FMS_COMMAND_WORDS["FMS_UPDATE_ATTITUDE"], '016b')
                
                # Pack parameters into data words
                if isinstance(data, dict):
                    # Format parameters based on expected structure
                    data_words = [format(int(data.get('roll', 0)), '016b'),
                                 format(int(data.get('pitch', 0)), '016b'),
                                 format(int(data.get('yaw', 0)), '016b')]
                else:
                    logger.error(f"[FMS_MSGR_HNDLR] Invalid attitude data format: {type(data)}")
                    return None
                    
                msg_type = f"{system_name}Data"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for attitude update")
                
            elif request_type == "navigation_update":
                # Use FMS_UPDATE_NAVIGATION command word
                command_word = format(FMS_COMMAND_WORDS["FMS_UPDATE_NAVIGATION"], '016b')
                
                # Pack parameters into data words
                if isinstance(data, dict):
                    # Format parameters based on expected structure
                    data_words = [format(int(data.get('altitude', 0)), '016b'),
                                 format(int(data.get('heading', 0)), '016b'),
                                 format(int(data.get('speed', 0)), '016b')]
                else:
                    logger.error(f"[FMS_MSGR_HNDLR] Invalid navigation data format: {type(data)}")
                    return None
                    
                msg_type = f"{system_name}Data"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for navigation update")
                
            elif request_type == "execute_maneuver":
                # Use FMS_EXECUTE_MANEUVER command word
                command_word = format(FMS_COMMAND_WORDS["FMS_EXECUTE_MANEUVER"], '016b')
                
                # Pack maneuver type into data words
                maneuver_type = data
                if isinstance(data, dict):
                    maneuver_type = data.get('maneuver_type')
                
                if not maneuver_type:
                    logger.error(f"[FMS_MSGR_HNDLR] Missing maneuver type")
                    return None
                
                # Map maneuver type to value - default is 0
                maneuver_map = {"TAKEOFF": 1, "LANDING": 2, "CLIMB": 3, "DESCENT": 4, "TURN": 5}
                maneuver_value = maneuver_map.get(maneuver_type.upper(), 0)
                
                data_words = [format(maneuver_value, '016b')]
                msg_type = f"{system_name}Data"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for maneuver {maneuver_type}")
                
            elif request_type == "control_input":
                # Get appropriate command word from FCS_COMMAND_WORDS for control input
                from FMOFP.local_messaging.message_types_fcs import FCS_CONTROL_INPUT_REQUEST
                from FMOFP.local_messaging.command_word_map_fcs import get_fcs_command_word

                cmd_word_obj = get_fcs_command_word(FCS_CONTROL_INPUT_REQUEST)
                if cmd_word_obj:
                    command_word = format(cmd_word_obj.value, '016b')
                else:
                    logger.error(f"[FMS_MSGR_HNDLR] No control input command word found")
                    return None
                    
                # Pack control input data into data words
                data_words = self._pack_control_input_data(data)
                msg_type = "flight_control_systemControlInputRequest"
                logger.info(f"[FMS_MSGR_HNDLR] Using command word {command_word} for control input")
                
            else:
                logger.error(f"[FMS_MSGR_HNDLR] Unknown request type: {request_type}")
                return None
            
            # Create request message using helper
            request = self._create_request(system_name, request_type, data, command_word, msg_type)
            if not request:
                logger.error("[FMS_MSGR_HNDLR] Failed to create request")
                return None
                
            # Create metadata for 1553B message
            msg_metadata = {
                'message_header': request.get('message_header'),
                'command_name': request.get('command_name'),
                'command_type': request_type,
                'message_type': msg_type,
                'sending_system': "FMSMessageHandler",
                'destination': system_name,
                'command': request.get('command'),
                'request_id': request_id
            }
            
            # Update with user-provided metadata
            if metadata:
                msg_metadata.update(metadata)
                
            # Send message through 1553B bus
            logger.info(f"[FMS_MSGR_HNDLR] Sending 1553B message: cmd={command_word}, data={data_words}")
            result = await self.sendMsg.send_message(command_word, data_words, request_id, msg_metadata)
            
            if result is None:
                logger.error("[FMS_MSGR_HNDLR] Failed to send message through 1553B")
                with self.lock:
                    self.pending_requests.pop(request_id, None)
                return None
                
            logger.info("[FMS_MSGR_HNDLR] Successfully sent message through 1553B")
            return request_id
            
        except Exception as e:
            logger.error(f"[FMS_MSGR_HNDLR] Error sending request: {str(e)}")
            logger.error(traceback.format_exc())
            
            # Clean up pending request on error
            with self.lock:
                self.pending_requests.pop(request_id, None)
                
            return None
            
    def _pack_data(self, data: Any) -> list:
        """Pack FMS mode data into binary format for message transmission."""
        mode_value = None
        mode_name = None
        
        # Extract mode name from data based on its type.
        #
        # 'mode' is accepted as well as 'mode_name'. It was not, and callers
        # pass it: {"mode": "STEALTH"} matched none of these branches, so
        # mode_name stayed None and the `.get(..., 0)` default below turned
        # it into 0 -- NORMAL. The mode was not merely lost, it was replaced
        # with a different command, and the send reported success.
        if isinstance(data, dict) and ('mode_name' in data or 'mode' in data):
            mode_name = data.get('mode_name', data.get('mode'))
            logger.info(f"[FMS_MSGR_HNDLR] Mode name extracted from dictionary: {mode_name}")
        elif hasattr(data, 'value'):
            mode_value = data.value
            mode_name = data.name if hasattr(data, 'name') else str(data.value)
            logger.info(f"[FMS_MSGR_HNDLR] Mode value found from enum: {mode_name} {mode_value}")
        elif isinstance(data, str):
            mode_name = data
            logger.info(f"[FMS_MSGR_HNDLR] Mode name as string: {mode_name}")
        
        # Map mode name to value.
        #
        # The table written out here carried the comment "aligned with
        # FlightControlSystem modes", and it was not: it listed the FCS's
        # NORMAL/COMBAT/PRECISION/AUTOPILOT/TERRAIN/EMERGENCY while this is
        # the FMS's mode packer, and the FMS's own five modes
        # (FlightManagementSystem.set_mode's valid_modes, and the encoder and
        # decoder in fms_tactical_data) are
        # NORMAL/COMBAT/STEALTH/TRAINING/EMERGENCY. So STEALTH and TRAINING,
        # which are real FMS modes, were not in the table at all and fell
        # through the `.get(..., 0)` default to 0 -- NORMAL. An FMS mode
        # change to STEALTH was transmitted as a change to NORMAL.
        # EMERGENCY, the one that matters most, was encoded as the FCS's 5
        # where the FMS decoder reads 4 and so names it as nothing.
        #
        # fms_mode_data_words() is the FMS vocabulary, shared with the
        # tactical-data codec, and it raises rather than defaulting: the
        # caller is inside send_request's try/except, so an unknown mode is
        # refused instead of being turned into a different command.
        if mode_value is None:
            words = fms_mode_data_words(mode_name)
            logger.info(f"[FMS_MSGR_HNDLR] Mapped mode {mode_name} to {words}")
            return words

        # Format as 16-bit binary string
        return [format(mode_value, '016b')]
        
    def _pack_control_input_data(self, data: Any) -> list:
        """
        Pack control input data into data words.

        Fifth copy of the FCS control-input encoding, now the shared one in
        fcs_wire_format. Same codes, same scaling; the difference is that a
        missing or unrecognised control type used to become code 0 (which is
        not a surface, so the FCS answered "no control surface specified"
        after the command had already been sent) and is now refused before
        anything reaches the bus.
        """
        if not isinstance(data, dict):
            raise ValueError(
                f"control input data must be a dict with 'control_type' and "
                f"'value', got {type(data).__name__}"
            )

        return control_input_data_words(
            data.get('control_type'),
            data.get('value', 0.0),
            rate=data.get('rate'),
        )
    
    def _create_request(self, system_name: str, request_type: str, data: Any, command_word: str, msg_type: Optional[str] = None) -> Optional[Dict]:
        """
        Create a request object based on type and system
        
        Args:
            system_name: Name of the system
            request_type: Type of request (status, mode_change, etc.)
            data: Request data
            command_word: MIL-STD-1553B command word
            message_type: Optional message type override
        """
        try:
            # Determine command name from request type
            command_name = None
            
            if request_type == "status":
                command_name = "FMS_STATUS_REQUEST"
                return {
                    "message_header": "status_request",
                    "sending_system": "FMSMessageHandler",
                    "destination": system_name,
                    "message_type": f"{system_name}Status",
                    "command_word": command_word,
                    "command_name": command_name
                }
            elif request_type == "mode_change":
                command_name = "FMS_SET_MODE"
                mode_str = data.name if hasattr(data, 'name') else str(data)
                return {
                    "message_header": "mode_change",
                    "sending_system": "FMSMessageHandler",
                    "destination": system_name,
                    "message_type": f"{system_name}Command",
                    "command": f"set_mode {mode_str}",
                    "command_word": command_word,
                    "command_name": command_name
                }
            elif request_type == "attitude_update":
                command_name = "FMS_UPDATE_ATTITUDE"
                return {
                    "message_header": "data_request",
                    "sending_system": "FMSMessageHandler",
                    "destination": system_name,
                    "message_type": msg_type or f"{system_name}Data",
                    "data_type": "attitude",
                    "command_word": command_word,
                    "command_name": command_name
                }
            elif request_type == "control_input":
                command_name = "FCS_CONTROL_INPUT_REQUEST"
                control_info = ""
                if isinstance(data, dict):
                    control_info = f"{data.get('control_type', 'unknown')}={data.get('value', 0)}"
                return {
                    "message_header": "control_input",
                    "sending_system": "FMSMessageHandler",
                    "destination": system_name,
                    "message_type": msg_type,
                    "command_word": command_word,
                    "command_name": command_name,
                    "command": f"set_control {control_info}"
                }
            # Add other request types as needed
            
            return None
        except Exception as e:
            logger.error(f"Error creating request: {str(e)}")
            return None
    
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
        
    def set_async_handler(self, async_handler):
        """Set the async message handler"""
        self.async_handler = async_handler
        self._register_message_handlers()
        logger.info("FMS Message Handler connected to async handler")
        
    def set_response_service(self, response_service):
        """Set the response service"""
        self.response_service = response_service
        logger.info("FMS Message Handler connected to response service")
        
    def _register_message_handlers(self):
        """Register message handlers with the async handler"""
        if not self.async_handler:
            logger.warning("Cannot register FMS message handlers - async handler not set")
            return
            
        # Register the FMS system with the async handler first
        fms_system = self.async_handler.register_system("flightManagementSystem")
        
        # Register handlers for various FMS message types with command words from FMS_COMMAND_WORDS
        from FMOFP.local_messaging.command_word_map_fms import FMS_COMMAND_WORDS
        
        # Map handlers to command words
        command_handlers = {
            "FMS_COMMAND": self._handle_fms_command,
            "FMS_SET_MODE": self._handle_mode_change,
            "FMS_UPDATE_ATTITUDE": self._handle_attitude_update,
            "FMS_STATUS_REQUEST": self._handle_status_request, 
            "FMS_UPDATE_NAVIGATION": self._handle_navigation_update,
            "FMS_EXECUTE_MANEUVER": self._handle_execute_maneuver,
            "FMS_FLIGHT_DATA": self._handle_generic_fms_message
        }
        
        # Register special handler for Flight Control System messages
        # This is to route FCS messages to the appropriate handler
        self._register_fcs_message_handler()
        
        # Register each handler with the appropriate command word for FMS messages
        for command_name, handler in command_handlers.items():
            try:
                if command_name in FMS_COMMAND_WORDS:
                    command_word = format(FMS_COMMAND_WORDS[command_name], '016b')  # Convert to binary string
                    fms_system.register_handler(command_word, handler)
                    logger.info(f"Registered handler for {command_name} with command word {command_word}")
                else:
                    logger.warning(f"Command name {command_name} not found in FMS_COMMAND_WORDS")
            except Exception as e:
                logger.error(f"Error registering handler for {command_name}: {e}")
        
        logger.info("FMS message handlers registered")
    
    def _register_fcs_message_handler(self):
        """
        Register handlers for Flight Control System (FCS) messages
        
        This method registers handlers for messages that need to be routed to the
        Flight Control System component. It allows the FMS Message Handler to act
        as a router for FCS messages.
        """
        if not self.async_handler:
            logger.warning("Cannot register FCS message handlers - async handler not set")
            return
            
        try:
            # Get access to the FCS message handler
            from FMOFP.local_messaging.routing.handlers.system_message_handlers.FCSMessageHandler import get_fcs_message_handler
            fcs_handler = get_fcs_message_handler()
            
            # Register the flight_control_system with the async handler
            flight_control_system = self.async_handler.register_system("flight_control_system")
            
            # Create a handler that routes FCS mode change messages to the FCS handler
            async def handle_fcs_mode_change(message):
                """Route mode change messages to FCS handler"""
                logger.info(f"[FMS_HANDLER] Routing mode change message to FCS handler")
                if not fcs_handler:
                    logger.error("[FMS_HANDLER] FCS Message Handler not available")
                    return {"status": "ERROR", "message": "FCS Message Handler not available"}
                return await fcs_handler._handle_mode_change_request(message)
            
            # Create a handler that routes FCS control input messages to the FCS handler
            async def handle_fcs_control_input(message):
                """Route control input messages to FCS handler"""
                logger.info(f"[FMS_HANDLER] Routing control input message to FCS handler")
                if not fcs_handler:
                    logger.error("[FMS_HANDLER] FCS Message Handler not available")
                    return {"status": "ERROR", "message": "FCS Message Handler not available"}
                return await fcs_handler._handle_control_input_request(message)
                
            # Create a handler that routes FCS status request messages to the FCS handler
            async def handle_fcs_status(message):
                """Route status request messages to FCS handler"""
                logger.info(f"[FMS_HANDLER] Routing status request message to FCS handler")
                if not fcs_handler:
                    logger.error("[FMS_HANDLER] FCS Message Handler not available")
                    return {"status": "ERROR", "message": "FCS Message Handler not available"}
                return await fcs_handler._handle_status_request(message)
                
            # Create a handler that routes FCS orientation data messages to the FCS handler
            async def handle_fcs_orientation(message):
                """Route orientation data messages to FCS handler"""
                logger.info(f"[FMS_HANDLER] Routing orientation data message to FCS handler")
                if not fcs_handler:
                    logger.error("[FMS_HANDLER] FCS Message Handler not available")
                    return {"status": "ERROR", "message": "FCS Message Handler not available"}
                return await fcs_handler._handle_orientation_data_request(message)
            
            # Register each handler with specific command words for FCS message types
            # Get command words from FCS_COMMAND_WORDS
            from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS
            from FMOFP.local_messaging.message_types_fcs import (
                FCS_MODE_CHANGE_REQUEST,
                FCS_CONTROL_INPUT_REQUEST,
                FCS_STATUS_REQUEST,
                FCS_ORIENTATION_DATA_REQUEST
            )
            
            # Get command words and register handlers
            mode_change_word = format(FCS_COMMAND_WORDS[FCS_MODE_CHANGE_REQUEST].value, '016b')
            control_input_word = format(FCS_COMMAND_WORDS[FCS_CONTROL_INPUT_REQUEST].value, '016b')
            status_word = format(FCS_COMMAND_WORDS[FCS_STATUS_REQUEST].value, '016b')
            orientation_word = format(FCS_COMMAND_WORDS[FCS_ORIENTATION_DATA_REQUEST].value, '016b')
            
            # Register handlers with system using regular register_handler method
            flight_control_system.register_handler(mode_change_word, handle_fcs_mode_change)
            flight_control_system.register_handler(control_input_word, handle_fcs_control_input)
            flight_control_system.register_handler(status_word, handle_fcs_status)
            flight_control_system.register_handler(orientation_word, handle_fcs_orientation)
            
            logger.info("FCS message handlers registered in FMS handler")
        except Exception as e:
            logger.error(f"Error registering FCS message handlers: {e}")
            logger.error(traceback.format_exc())
        
    async def _handle_fms_command(self, message):
        """
        Handle FMS command message
        
        Args:
            message: The command message
            
        Returns:
            dict: Command result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                # Create a unique transaction ID based on message properties
                transaction_id = self._create_transaction_id(message, "command")
                
                # Attempt to use enhanced features, fall back to basic if not supported
                try:
                    # Try to use the advanced parameters
                    should_process, enhanced_message = self.loop_prevention.process_message(
                        message, 
                        "fms_command_handler", 
                        **{"transaction_id": transaction_id, "category": "fms_command"}
                    )
                except TypeError:
                    # If TypeError (wrong signature), use the standard call
                    should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_command_handler")
                except Exception as e:
                    # Any other error, log and use standard call
                    logger.warning(f"Error using enhanced loop prevention features: {e}")
                    should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_command_handler")
                
                if not should_process:
                    logger.warning(f"Breaking loop - FMS command message already processed: {transaction_id}")
                    return {"status": "SKIPPED", "message": "Message loop detected", "transaction_id": transaction_id}
                    
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid FMS command message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract command details
            command_type = message.get('command_type')
            parameters = message.get('parameters', {})
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing FMS command: {command_type} (request_id: {request_id})")
            
            # Process command
            if self.fms_control:
                result = self.fms_control.process_command(command_type, parameters)
                
                # Create response
                response = create_fms_message(
                    "fms_commandResponse",
                    status="SUCCESS" if result.get('status') == 'SUCCESS' else "ERROR",
                    request_id=request_id,
                    message=result.get('message', ''),
                    data=result.get('data', {})
                )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, command not processed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling FMS command: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_mode_change(self, message):
        """
        Handle FMS mode change message
        
        Args:
            message: The mode change message
            
        Returns:
            dict: Mode change result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_mode_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS mode change message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid mode change message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract mode_name from parameters dictionary
            mode_name = None
            if 'parameters' in message:
                mode_name = message['parameters'].get('mode_name')
            
            if not mode_name:
                logger.warning("No mode_name specified in mode change request")
                return {"status": "ERROR", "message": "No mode_name specified in mode change request"}
                
            parameters = {"mode": mode_name}  # Parameter key remains 'mode' for compatibility with FMS control
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing mode change to {mode_name} (request_id: {request_id})")
            
            # Set mode via FMS Control
            if self.fms_control:
                old_mode = self.fms_control.mode
                result = self.fms_control.process_command("SET_MODE", parameters)
                
                # Create response
                response = create_fms_message(
                    "fms_modeChangeResponse",
                    status="SUCCESS" if result.get('status') == 'SUCCESS' else "ERROR",
                    request_id=request_id,
                    message=result.get('message', ''),
                    data=result.get('data', {}),
                    old_mode=old_mode,
                    new_mode=mode_name if result.get('status') == 'SUCCESS' else old_mode
                )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, mode change not processed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling mode change: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_attitude_update(self, message):
        """
        Handle attitude update message
        
        Args:
            message: The attitude update message
            
        Returns:
            dict: Attitude update result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_attitude_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS attitude message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid attitude update message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract attitude parameters
            parameters = message.get('parameters', {})
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing attitude update (request_id: {request_id})")
            
            # Update attitude via FMS Control
            if self.fms_control:
                result = self.fms_control.process_command("UPDATE_ATTITUDE", parameters)
                
                # Create response
                response = create_fms_message(
                    "fms_commandResponse",
                    status="SUCCESS" if result.get('status') == 'SUCCESS' else "ERROR",
                    request_id=request_id,
                    message=result.get('message', ''),
                    data=result.get('data', {})
                )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, attitude update not processed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling attitude update: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_navigation_update(self, message):
        """
        Handle navigation update message
        
        Args:
            message: The navigation update message
            
        Returns:
            dict: Navigation update result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_navigation_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS navigation message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid navigation update message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract navigation parameters
            parameters = message.get('parameters', {})
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing navigation update (request_id: {request_id})")
            
            # Update navigation via FMS Control
            if self.fms_control:
                result = self.fms_control.process_command("SET_AUTOPILOT", parameters)
                
                # Create response
                response = create_fms_message(
                    "fms_commandResponse",
                    status="SUCCESS" if result.get('status') == 'SUCCESS' else "ERROR",
                    request_id=request_id,
                    message=result.get('message', ''),
                    data=result.get('data', {})
                )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, navigation update not processed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling navigation update: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_status_request(self, message):
        """
        Handle status request message
        
        Args:
            message: The status request message
            
        Returns:
            dict: Status information
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_status_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS status message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid status request message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing status request (request_id: {request_id})")
            
            # Get status via FMS Control
            if self.fms_control:
                result = self.fms_control.process_command("GET_STATUS", {})
                
                # Create response
                if result.get('status') == 'SUCCESS':
                    tactical_status = result.get('data', {}).get('tactical_status', {})
                    
                    response = create_fms_message(
                        "fms_statusResponse",
                        status="SUCCESS",
                        request_id=request_id,
                        message="FMS status retrieved",
                        data=result.get('data', {}),
                        mode=tactical_status.get('mode', ''),
                        health=tactical_status.get('health', 'UNKNOWN'),
                        warnings=tactical_status.get('envelope_warnings', [])
                    )
                else:
                    response = create_fms_message(
                        "fms_statusResponse",
                        status="ERROR",
                        request_id=request_id,
                        message=result.get('message', 'Failed to get status'),
                        data={}
                    )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, status request not processed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_execute_maneuver(self, message):
        """
        Handle execute maneuver message
        
        Args:
            message: The execute maneuver message
            
        Returns:
            dict: Maneuver execution result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_maneuver_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS maneuver message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid execute maneuver message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            # Extract maneuver parameters
            maneuver_type = message.get('maneuver_type')
            parameters = message.get('parameters', {})
            if maneuver_type and 'maneuver_type' not in parameters:
                parameters['maneuver_type'] = maneuver_type
                
            request_id = message.get('request_id', None)
            
            logger.info(f"Processing execute maneuver: {maneuver_type} (request_id: {request_id})")
            
            # Execute maneuver via FMS Control
            if self.fms_control:
                result = self.fms_control.process_command("EXECUTE_MANEUVER", parameters)
                
                # Create response
                response = create_fms_message(
                    "fms_commandResponse",
                    status="SUCCESS" if result.get('status') == 'SUCCESS' else "ERROR",
                    request_id=request_id,
                    message=result.get('message', ''),
                    data=result.get('data', {})
                )
                
                # Send response to requestor if response service is available
                if self.response_service:
                    await self.response_service.send_response(request_id, response.to_dict())
                
                # Return result
                return result
            else:
                logger.warning("FMS Control not available, maneuver not executed")
                return {"status": "ERROR", "message": "FMS Control not available"}
                
        except Exception as e:
            logger.error(f"Error handling execute maneuver: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    
    async def _handle_generic_fms_message(self, message):
        """
        Handle generic FMS message
        
        Args:
            message: The generic FMS message
            
        Returns:
            dict: Processing result
        """
        try:
            # Check for message loops if middleware is available
            if self.loop_prevention:
                should_process, enhanced_message = self.loop_prevention.process_message(message, "fms_generic_handler")
                if not should_process:
                    logger.warning(f"Breaking loop - FMS generic message already processed")
                    return {"status": "SKIPPED", "message": "Message loop detected"}
                # Use the enhanced message with loop prevention metadata
                message = enhanced_message
            if not isinstance(message, dict):
                if hasattr(message, 'to_dict'):
                    message = message.to_dict()
                else:
                    logger.warning(f"Invalid generic FMS message format: {type(message)}")
                    return {"status": "ERROR", "message": "Invalid message format"}
            
            message_type = message.get('message_type', '')
            
            logger.info(f"Processing generic FMS message: {message_type}")
            
            # Route to appropriate handler based on message type
            if message_type.startswith('fms_mode'):
                return await self._handle_mode_change(message)
            elif message_type.startswith('fms_attitude'):
                return await self._handle_attitude_update(message)
            elif message_type.startswith('fms_navigation'):
                return await self._handle_navigation_update(message)
            elif message_type.startswith('fms_status'):
                return await self._handle_status_request(message)
            elif message_type.startswith('fms_maneuver'):
                return await self._handle_execute_maneuver(message)
            elif message_type.startswith('fms_command'):
                return await self._handle_fms_command(message)
            else:
                logger.warning(f"Unknown FMS message type: {message_type}")
                return {"status": "ERROR", "message": f"Unknown message type: {message_type}"}
                
        except Exception as e:
            logger.error(f"Error handling generic FMS message: {e}")
            return {"status": "ERROR", "message": f"Exception: {str(e)}"}
    

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
                f"[FMS_MSG_HNDLR] Started without a running event loop: request "
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
                f"[FMS_MSG_HNDLR] Cannot notify {request_id} of its timeout: no "
                f"usable event loop. The request was still expired and removed."
            )
            return False
        if self.response_service is None:
            logger.error(
                f"[FMS_MSG_HNDLR] Cannot notify {request_id} of its timeout: no "
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
                f"[FMS_MSG_HNDLR] Failed to notify {request_id} of its timeout: "
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
                        response = create_fms_message(
                            "fms_commandResponse",
                            status="ERROR",
                            request_id=request_id,
                            message="Request timed out",
                            data={}
                        )
                        
                        # Send failure response. See the note in
                        # FCSMessageHandler._cleanup_pending_requests: this was
                        # H7, asyncio.get_event_loop() on a threading.Timer
                        # thread, which raises, discards the notification
                        # coroutine unawaited, and aborts the rest of the
                        # expired batch.
                        self._notify_request_timed_out(request_id, response.to_dict())
        except Exception as e:
            logger.error(f"Error cleaning up pending requests: {e}")
        finally:
            # Restart the timer if still running
            if self.started:
                self._start_cleanup_timer()
    
    def start(self):
        """Start the FMS message handler"""
        if self.started:
            logger.warning("FMS Message Handler already started")
            return False
            
        self.started = True
        
        # Start messenger
        if self.fms_messenger:
            self.fms_messenger.start()
        
        # Capture the loop BEFORE the timer that needs it is started.
        self._remember_loop()

        # Start cleanup timer
        self._start_cleanup_timer()
        
        logger.info("FMS Message Handler started")
        return True
    
    def stop(self):
        """Stop the FMS message handler"""
        if not self.started:
            logger.warning("FMS Message Handler already stopped")
            return False
            
        self.started = False
        
        # Stop cleanup timer
        if self.cleanup_timer:
            self.cleanup_timer.cancel()
            self.cleanup_timer = None
        
        # Stop messenger
        if self.fms_messenger:
            self.fms_messenger.stop()
        
        logger.info("FMS Message Handler stopped")
        return True
    
    def is_healthy(self):
        """Check if handler is healthy"""
        messenger_healthy = self.fms_messenger and self.fms_messenger.is_healthy()
        fms_healthy = self.fms and hasattr(self.fms, 'check_health') and self.fms.check_health()
        
        return self.started and messenger_healthy and fms_healthy

# Singleton instance
_fms_message_handler = None

def get_fms_message_handler():
    """Get singleton instance of FMS Message Handler"""
    global _fms_message_handler
    if _fms_message_handler is None:
        _fms_message_handler = FMSMessageHandler()
    return _fms_message_handler
