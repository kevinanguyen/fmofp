"""
Message Loop Prevention Module

This module provides the central service for preventing message loops.
Enhanced with specialized tracking of message ID + type combinations.
"""

import time
import uuid
import logging
from typing import Dict, Any, Optional, Tuple, TypeVar

from FMOFP.Utils.message_loop_prevention.identifier import MessageIdentifier
from FMOFP.Utils.message_loop_prevention.registry import MessageRegistry
from FMOFP.Utils.message_loop_prevention.tracking_library import MessageTrackingLibrary
from FMOFP.Utils.message_loop_prevention.config import get_config

# Import local messaging configurations
try:
    from FMOFP.local_messaging.command_name_registry import COMMAND_NAMES
except ImportError:
    # Create empty dict if import fails
    COMMAND_NAMES = {}

# Imported unconditionally. This was wrapped in try/except ImportError with a
# six-member stand-in enum defined in the handler -- "Create dummy enum if
# import fails".
#
# The canonical RadarDisplayMode has 35 members. The only use of it here is
# the loop below, which adds every member NAME to the category set, so the
# stand-in did not corrupt any value at this site -- it silently narrowed
# loop prevention to 6 categories out of 35, leaving 29 radar display modes
# unrecognised by the thing whose job is recognising them. (The values were
# wrong too -- SURVEILLANCE 1 against the canonical 10, MAPPING 2 against 11,
# NORMAL 5 against 1, five of the six disagreeing -- which would have
# mattered to any other consumer of the name.)
#
# A fallback is only worth having when degrading is better than failing. An
# enum that silently answers for a third of itself is not degrading, it is
# answering wrongly, and the module it guards imports nothing but
# `enum.Enum`, so the ImportError it catches cannot realistically happen
# without the package being broken outright. Failing loudly is the honest
# behaviour.
from FMOFP.local_messaging.radar_display_modes import RadarDisplayMode

# Type variable for generic message types
T = TypeVar('T')

# Get logger from system logger
try:
    from FMOFP.Utils.logger.sys_logger import get_logger
    logger = get_logger()
except ImportError:
    # Fall back to standard logging if system logger not available
    logger = logging.getLogger(__name__)

class MessageLoopPrevention:
    """Central service for preventing message loops."""
    
    _instance = None  # Singleton instance
    
    @classmethod
    def get_instance(cls):
        """Get singleton instance."""
        if cls._instance is None:
            cls._instance = MessageLoopPrevention()
        return cls._instance
    
    def __init__(self):
        """Initialize the message loop prevention service."""
        self._registry = MessageRegistry()
        self._identifier = MessageIdentifier()
        self._config = get_config()
        
        # Initialize the tracking library for ID + type tracking
        self._tracking_library = MessageTrackingLibrary()
        
        # Extract message types from command name registry
        self._message_types = set()
        for cmd_name, cmd_info in COMMAND_NAMES.items():
            if isinstance(cmd_info, dict) and 'message_type' in cmd_info:
                self._message_types.add(cmd_info['message_type'])
        
        # Common categories based on system components
        self._common_categories = {
            # Radar data types
            'vil', 'precipitation', 'turbulence', 'windshear', 'echo_top', 'shear',
            # Radar systems
            'weather_radar', 'tfr_radar', 'sar_radar', 'targeting_radar', 'aewc_radar',
            # Message operations
            'mode_change', 'status', 'command', 'configuration', 'data', 'maintenance',
            'initialization', 'shutdown', 'error', 'alert', 'heartbeat', 'update',
            'acknowledgment', 'diagnostic', 'log',
            # Display systems
            'display', 'mfd', 'radar_display',
            # FMS data types
            'attitude', 'velocity', 'tactical', 'maneuver',
            # FMS systems
            'fms', 'flightManagementSystem', 'flight_management',
            # Other systems
            'navigation', 'targeting', 'communication'
        }
        
        # Add radar display modes as categories
        for mode in RadarDisplayMode:
            self._common_categories.add(mode.name.lower())

        # Metadata supplied by register_category(). Recorded, not acted upon
        # -- see that method's docstring.
        self._category_metadata = {}
        
        self._stats = {
            'loops_detected': 0,
            'messages_processed': 0,
            'messages_by_category': {},
            'duplicate_combinations': 0  # Track duplicate request_id + message_type combinations
        }
        
        logger.info("MessageLoopPrevention initialized")
        
    def process_message(self, message: T, service_name: str) -> Tuple[bool, T]:
        """Process a message and determine if it should be handled or skipped.
        
        Args:
            message: The message to process
            service_name: Name of the service processing the message
            
        Returns:
            Tuple containing:
                - Boolean indicating if message should be processed
                - Enhanced message with loop prevention metadata
        """
        # Check if service is enabled
        if not self._config.is_service_enabled(service_name):
            logger.debug(f"[LOOP_PREVENTION] Service {service_name} is disabled, skipping loop prevention")
            return True, message
            
        # Extract request_id and message_type for enhanced loop detection
        request_id = self._extract_request_id(message)
        message_type = self._extract_message_type(message)
        
        # Generate message ID for backward compatibility
        message_id = self._identifier.generate_message_id(message)
        
        # Extract message type for category
        category = self._extract_category(message)
        
        # Check if category is enabled
        if not self._config.is_category_enabled(category):
            logger.debug(f"[LOOP_PREVENTION] Category {category} is disabled, skipping loop prevention")
            return True, message
        
        # First check for duplicate request_id + message_type combinations
        if request_id and message_type:
            # Special handling for all message flows
            is_status_or_completion = self._is_status_or_completion(message, message_type)

            # Allow duplicate status words and completion messages for any message flow
            if self._tracking_library.has_seen_message(request_id, message_type) and not is_status_or_completion:
                # We've seen this exact combination before - it's a loop
                # Exception: Allow status words and completion messages for precipitation data
                count = self._tracking_library.get_count(request_id, message_type)
                logger.warning(f"[LOOP_PREVENTION] Breaking loop - same message type '{message_type}' "
                             f"for request_id '{request_id}' seen {count} times")
                
                # Log what other message types we've seen for this request_id
                types = self._tracking_library.get_message_types(request_id)
                logger.info(f"[LOOP_PREVENTION] Message types for request_id '{request_id}': {types}")
                
                self._stats['loops_detected'] += 1
                self._stats['duplicate_combinations'] += 1
                return False, message
            
            # Record that we've seen this combination
            self._tracking_library.record_message(request_id, message_type)
            logger.debug(f"[LOOP_PREVENTION] Recorded message: request_id='{request_id}', type='{message_type}'")
            
        # Also check using the traditional method for backward compatibility
        if self._registry.is_processed(message_id, category):
            logger.warning(f"[LOOP_PREVENTION] Breaking loop - message already processed by {service_name} (category: {category})")
            self._stats['loops_detected'] += 1
            return False, message
            
        # Mark as processed in the registry for backward compatibility
        self._registry.mark_processed(message_id, category)
        self._stats['messages_processed'] += 1
        
        # Update category stats
        if category not in self._stats['messages_by_category']:
            self._stats['messages_by_category'][category] = 0
        self._stats['messages_by_category'][category] += 1
        
        # Add processing metadata
        enhanced_message = self._add_processing_metadata(message, service_name)
        
        return True, enhanced_message
        
    def register_category(self, category: str,
                          category_type: str = None,
                          priority: str = None,
                          max_simultaneous_processing: int = None) -> str:
        """Make `category` a name this service can classify messages as.

        Story B3.1. Three handlers -- FCS, FMS and Radar -- each had a
        _register_*_loop_prevention_categories() method that began:

            if not hasattr(self.loop_prevention, 'register_category'):
                logger.warning("Loop prevention middleware does not support
                                category registration - using default logic")
                return

        and `register_category` existed nowhere. So the method logged and
        returned, and the 21 category registrations below it -- 5 FCS, 8 FMS,
        8 radar -- were dead code. Measured on a live boot: that warning
        exactly 7 times, which is the figure the PI's own risk register
        recorded five months earlier.

        WHAT IS ACTED UPON: the NAME, which joins _common_categories and so
        becomes something _extract_category can return.

        WHAT IS RECORDED BUT NOT ACTED UPON: category_type, priority and
        max_simultaneous_processing. The handlers pass all three, and nothing
        in this package reads any of them -- verified by search, zero hits.
        Implementing them would mean inventing a priority scheduler and a
        concurrency limiter for message categories, which is a feature, not a
        fix. They are kept in _category_metadata so the declared intent is
        retrievable rather than discarded, and this docstring is where that
        distinction is written down.
        """
        name = str(category).lower()
        self._common_categories.add(name)
        self._category_metadata[name] = {
            'category_type': category_type,
            'priority': priority,
            'max_simultaneous_processing': max_simultaneous_processing,
            'note': 'name is used for classification; the other fields are '
                    'recorded only -- nothing consumes them',
        }
        return name

    def get_category_metadata(self) -> Dict[str, Dict[str, Any]]:
        """What register_category() was told, for inspection."""
        return dict(self._category_metadata)

    def _best_category_match(self, haystack: str):
        """The LONGEST registered category appearing in `haystack`, or None.

        _extract_category used to return the first match while iterating
        _common_categories, which is a SET -- so the answer depended on
        string hash order and therefore on PYTHONHASHSEED. Measured before
        this change, with the finer names registered:
        _extract_category({'message_type': 'fcs_statusResponse'}) returned
        'fcs_status' on 2 of 5 seeds and 'status' on the other 3. The same
        message, categorised differently from run to run.

        That mattered more once B3.1 registered the 21 intended names,
        because 16 of them are compound forms of names already present
        ('fcs_status' beside 'status', 'fms_mode' beside 'mode_change'), so
        registering them would have multiplied the overlaps rather than
        sharpened the classification.

        Longest wins, ties broken alphabetically: deterministic, and the
        most specific answer rather than an arbitrary one.
        """
        best = None
        for category in self._common_categories:
            if category in haystack:
                if best is None or (len(category), best) > (len(best), category):
                    best = category
        return best

    def _extract_category(self, message: Any) -> str:
        """Extract a category from a message.
        
        Args:
            message: The message to extract a category from
            
        Returns:
            A category string
        """
        # Try to extract message type
        message_type = self._identifier._extract_message_type(message)
        
        # Check if message type is in known message types
        if message_type in self._message_types:
            # Find the command name for this message type
            for cmd_name, cmd_info in COMMAND_NAMES.items():
                if isinstance(cmd_info, dict) and cmd_info.get('message_type') == message_type:
                    # Extract category from command name (e.g., WEATHER_RADAR_VIL_DATA -> vil)
                    parts = cmd_name.lower().split('_')
                    for part in parts:
                        if part in self._common_categories:
                            return part
        
        # Check if message type contains any common category
        match = self._best_category_match(message_type.lower())
        if match:
            return match
        
        # Try to extract from command type
        command_type = None
        if isinstance(message, dict):
            command_type = message.get('command_type')
        elif hasattr(message, 'command_type'):
            command_type = getattr(message, 'command_type')
            
        if command_type:
            match = self._best_category_match(str(command_type).lower())
            if match:
                return match
        
        # Default to message type if no common category found
        return message_type
        
    def _add_processing_metadata(self, message: Any, service_name: str) -> Any:
        """Add processing metadata to message.
        
        Args:
            message: The message to add metadata to
            service_name: Name of the service processing the message
            
        Returns:
            The message with added metadata
        """
        # Create a copy to avoid modifying the original
        if isinstance(message, dict):
            message_copy = message.copy()
            
            # Ensure metadata exists
            if 'metadata' not in message_copy:
                message_copy['metadata'] = {}
                
            # Add processing flags
            message_copy['metadata'][f'_processed_by_{service_name}'] = True
            message_copy['metadata']['_processing_timestamp'] = time.time()
            
            # Add transaction ID if not present
            if 'transaction_id' not in message_copy['metadata']:
                message_copy['metadata']['transaction_id'] = str(uuid.uuid4())
                
            return message_copy
        else:
            # Handle non-dict messages (e.g., objects)
            try:
                # Try to make a copy if possible
                import copy
                message_copy = copy.copy(message)
            except Exception:
                # Fall back to original if copy fails
                message_copy = message
                
            # Ensure metadata attribute exists
            if not hasattr(message_copy, 'metadata'):
                setattr(message_copy, 'metadata', {})
            elif message_copy.metadata is None:
                message_copy.metadata = {}
                
            # Add processing flags
            message_copy.metadata[f'_processed_by_{service_name}'] = True
            message_copy.metadata['_processing_timestamp'] = time.time()
            
            # Add transaction ID if not present
            if not hasattr(message_copy.metadata, 'transaction_id') or not message_copy.metadata.get('transaction_id'):
                message_copy.metadata['transaction_id'] = str(uuid.uuid4())
                
            return message_copy
            
    def _extract_request_id(self, message: Any) -> Optional[str]:
        """Extract request ID from message.
        
        Args:
            message: The message to extract from
            
        Returns:
            The request ID as a string, or None if not found
        """
        # Check common locations for request ID
        locations = [
            lambda m: m.get('request_id') if isinstance(m, dict) else None,
            lambda m: m.get('metadata', {}).get('request_id') if isinstance(m, dict) else None,
            lambda m: m.get('additional_info', {}).get('request_id') if isinstance(m, dict) else None,
            lambda m: getattr(m, 'request_id', None) if hasattr(m, 'request_id') else None,
            lambda m: getattr(m, 'metadata', {}).get('request_id') if hasattr(m, 'metadata') else None,
            lambda m: getattr(m, 'additional_info', {}).get('request_id') if hasattr(m, 'additional_info') else None,
            lambda m: m.get('id') if isinstance(m, dict) else None,
            lambda m: getattr(m, 'id', None) if hasattr(m, 'id') else None,
            lambda m: m.get('transaction_id') if isinstance(m, dict) else None,
            lambda m: m.get('metadata', {}).get('transaction_id') if isinstance(m, dict) else None,
            lambda m: getattr(m, 'transaction_id', None) if hasattr(m, 'transaction_id') else None,
        ]
        
        for extractor in locations:
            try:
                request_id = extractor(message)
                if request_id:
                    return str(request_id)
            except (AttributeError, TypeError, ValueError):
                continue
                
        return None

    def _is_status_or_completion(self, message: Any, message_type: str) -> bool:
        """Determine if this message is a status word or completion notification.
        
        Status words and completion messages are legitimate follow-up messages
        that should be allowed even for the same request_id.
        
        Args:
            message: The original message
            message_type: The extracted message type
            
        Returns:
            bool: True if this is a status or completion message
        """
        # Check message type for status or completion indicators
        message_type_lower = message_type.lower()
        if ('statusword' in message_type_lower or 
            'status_word' in message_type_lower or 
            'completion' in message_type_lower or 
            'complete' in message_type_lower):
            return True
            
        # Check command_type for status or completion indicators
        command_type = None
        if isinstance(message, dict):
            command_type = message.get('command_type')
        elif hasattr(message, 'command_type'):
            command_type = getattr(message, 'command_type')
            
        if command_type:
            command_type_lower = str(command_type).lower()
            if ('status' in command_type_lower or 
                'completion' in command_type_lower or 
                'complete' in command_type_lower):
                return True
                
        # Check metadata for completion indicators
        metadata = None
        if isinstance(message, dict):
            metadata = message.get('metadata', {})
        elif hasattr(message, 'metadata'):
            metadata = getattr(message, 'metadata')
            
        if isinstance(metadata, dict):
            if (metadata.get('is_transfer_complete') or 
                metadata.get('completion_message')):
                return True
                
        return False
                
    def _extract_message_type(self, message: Any) -> str:
        """Extract and normalize message type from message.
        
        Args:
            message: The message to extract from
            
        Returns:
            The normalized message type string
        """
        # Check common locations for message type
        locations = [
            lambda m: m.get('message_type') if isinstance(m, dict) else None,
            lambda m: m.get('metadata', {}).get('message_type') if isinstance(m, dict) else None,
            lambda m: m.get('additional_info', {}).get('message_type') if isinstance(m, dict) else None,
            lambda m: getattr(m, 'message_type', None) if hasattr(m, 'message_type') else None,
            lambda m: getattr(m, 'metadata', {}).get('message_type') if hasattr(m, 'metadata') else None,
            lambda m: getattr(m, 'additional_info', {}).get('message_type') if hasattr(m, 'additional_info') else None,
            lambda m: m.get('command_type') if isinstance(m, dict) else None,
            lambda m: getattr(m, 'command_type', None) if hasattr(m, 'command_type') else None,
        ]
        
        message_type = None
        for extractor in locations:
            try:
                message_type = extractor(message)
                if message_type:
                    break
            except (AttributeError, TypeError, ValueError):
                continue
        
        # Return unknown_type if not found
        if not message_type:
            return "unknown_type"
        
        # Normalize the message type
        # This handles standardizing between:
        # - weather_radarCommand
        # - weather_radarModeChangeCompletion
        message_type_lower = str(message_type).lower()
        
        # Special handling for weather radar message types
        if 'weather_radar' in message_type_lower:
            if 'completion' in message_type_lower or 'complete' in message_type_lower:
                if 'mode' in message_type_lower or 'mode_change' in message_type_lower:
                    return "weather_radarModeChangeCompletion"
            elif 'command' in message_type_lower:
                return "weather_radarCommand"
            elif 'status' in message_type_lower:
                return "weather_radarStatus"
        
        # For all other message types, use as-is
        return str(message_type)
        
    def get_stats(self) -> Dict[str, Any]:
        """Get statistics about message processing.
        
        Returns:
            Dictionary of statistics
        """
        # Get registry stats
        registry_stats = self._registry.get_stats()
        
        # Combine stats
        stats = self._stats.copy()
        stats['registry'] = registry_stats
        stats['config'] = self._config.get_config()
        
        # Add tracking library stats
        stats['tracking'] = {
            'duplicate_combinations': self._stats.get('duplicate_combinations', 0)
        }
        
        return stats
        
    def clear_registry(self, category: Optional[str] = None) -> None:
        """Clear the registry.
        
        Args:
            category: Optional category to clear
        """
        self._registry.clear(category)
        
    def reset_stats(self) -> None:
        """Reset statistics."""
        self._stats = {
            'loops_detected': 0,
            'messages_processed': 0,
            'messages_by_category': {}
        }
        self._registry.reset_stats()
