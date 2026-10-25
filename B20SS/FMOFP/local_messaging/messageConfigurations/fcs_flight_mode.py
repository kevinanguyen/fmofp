"""
Flight Control System Flight Mode Messages

Defines message types for FCS flight mode management.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional
import time
import uuid

from FMOFP.local_messaging.messageConfigurations.base_message import BaseMessage, register_message_type
from FMOFP.local_messaging.command_word_map_fcs import get_fcs_command_word
from FMOFP.local_messaging.message_types_fcs import (
    FCS_MODE_CHANGE_REQUEST,
    FCS_MODE_CHANGE_RESPONSE
)

class fcs_flightModeRequest(BaseMessage):
    """Flight mode request message"""
    def __init__(self, message_header, sending_system, destination, request_uuid=None, 
                 mode_data=None, **kwargs):
        """
        Initialize flight mode request.
        
        Args:
            message_header: Message header
            sending_system: Sending system ID
            destination: Destination system ID
            request_uuid: Unique identifier for request
            mode_data: Dictionary with flight mode data
        """
        super().__init__(
            message_type="fcs_flightModeRequest",
            message_header=message_header,
            sending_system=sending_system,
            destination=destination,
            command_name=FCS_MODE_CHANGE_REQUEST,
            **kwargs
        )
        self.request_uuid = request_uuid or str(uuid.uuid4())
        self.mode_data = mode_data or {}
        
    def validate(self):
        """Validate flight mode request message"""
        super().validate()
        if not self.request_uuid:
            raise ValueError("request_uuid is required")
        if not isinstance(self.mode_data, dict):
            raise ValueError("mode_data must be a dictionary")
        if "mode" not in self.mode_data:
            raise ValueError("mode_data must contain a 'mode' key")
            
    def to_data_words(self):
        """
        The data words for this flight mode request, as 16-bit binary
        strings. See fcs_data.fcs_controlSurfaceRequest.to_data_words() for
        why this did not exist, and fcs_wire_format's docstring for the
        encoding that used to live in encode_1553b() below and disagreed
        with the live one on every mode value.

        The mode branch of RadarMessageHandler.send_request() prefers this
        over _pack_data(), which starts from RADAR_MODE_MAP and raises
        KeyError for anything that is not one of the five radars.
        """
        from FMOFP.local_messaging.fcs_wire_format import mode_data_words

        return mode_data_words(self.mode_data.get("mode"), extra_params=self.mode_data)

    def encode_1553b(self):
        """
        Encode message for 1553B transmission: [command_word, *data_words].

        Delegates to to_data_words() so there is one FCS mode encoding.
        The layout this method used to carry numbered the modes
        normal 1 .. emergency 7 over a set of names the FCS does not have
        (approach, takeoff, landing, cruise); the FCS decodes NORMAL 0 ..
        EMERGENCY 5. Returns ints, as before.
        """
        command_word = get_fcs_command_word(FCS_MODE_CHANGE_REQUEST).value
        return [command_word] + [int(word, 2) for word in self.to_data_words()]

class fcs_flightModeResponse(BaseMessage):
    """Flight mode response message"""
    def __init__(self, message_header, sending_system, destination, request_uuid, 
                 response_uuid=None, status=None, mode_data=None, **kwargs):
        """
        Initialize flight mode response.
        
        Args:
            message_header: Message header
            sending_system: Sending system ID
            destination: Destination system ID
            request_uuid: Original request UUID
            response_uuid: Unique identifier for response
            status: Status code ("SUCCESS", "ERROR", etc.)
            mode_data: Dictionary with current flight mode data
        """
        super().__init__(
            message_type="fcs_flightModeResponse",
            message_header=message_header,
            sending_system=sending_system,
            destination=destination,
            command_name=FCS_MODE_CHANGE_RESPONSE,
            **kwargs
        )
        self.request_uuid = request_uuid
        self.response_uuid = response_uuid or str(uuid.uuid4())
        self.status = status or "SUCCESS"
        self.mode_data = mode_data or {}
        
    def validate(self):
        """Validate flight mode response message"""
        super().validate()
        if not self.request_uuid:
            raise ValueError("request_uuid is required")
        if not self.response_uuid:
            raise ValueError("response_uuid is required")
        if not isinstance(self.mode_data, dict):
            raise ValueError("mode_data must be a dictionary")

# Register message types
register_message_type("fcs_flightModeRequest", fcs_flightModeRequest)
register_message_type("fcs_flightModeResponse", fcs_flightModeResponse)
