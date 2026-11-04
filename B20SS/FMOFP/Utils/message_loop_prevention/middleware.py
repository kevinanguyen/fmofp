"""
Message Loop Prevention Middleware

This module provides a middleware component for integrating message loop prevention
with the routing pipeline.
"""

import logging
from typing import Dict, Any, Tuple, TypeVar, Optional

from FMOFP.Utils.message_loop_prevention.prevention import MessageLoopPrevention

# Type variable for generic message types
T = TypeVar('T')

# Get logger
try:
    from FMOFP.Utils.logger.sys_logger import get_logger
    logger = get_logger()
except ImportError:
    # Fall back to standard logging if system logger not available
    logger = logging.getLogger(__name__)

class MessageLoopPreventionMiddleware:
    """Middleware for message loop prevention in the routing pipeline."""
    
    _instance = None  # Singleton instance
    
    @classmethod
    def get_instance(cls):
        """Get singleton instance."""
        if cls._instance is None:
            cls._instance = MessageLoopPreventionMiddleware()
        return cls._instance
    
    def __init__(self):
        """Initialize the middleware."""
        self._prevention = MessageLoopPrevention.get_instance()
        self._config = {
            'enabled': True,
            'log_level': 'warning',
            'max_tracked_messages': 1000,
            'expiration_time': 60.0  # seconds
        }
        logger.info("MessageLoopPreventionMiddleware initialized")
        
    def process_message(self, message: T, service_name: str) -> Tuple[bool, T]:
        """
        Process a message and determine if it should be handled or skipped.
        
        Args:
            message: The message to process
            service_name: Name of the service processing the message
            
        Returns:
            Tuple containing:
                - Boolean indicating if message should be processed
                - Enhanced message with loop prevention metadata
        """
        if not self._config['enabled']:
            # If middleware is disabled, always process the message
            return True, message
            
        # Use the prevention service to process the message
        return self._prevention.process_message(message, service_name)
        
    async def process_message_async(self, message: T, service_name: str) -> Tuple[bool, T]:
        """
        Process a message asynchronously and determine if it should be handled or skipped.
        
        Args:
            message: The message to process
            service_name: Name of the service processing the message
            
        Returns:
            Tuple containing:
                - Boolean indicating if message should be processed
                - Enhanced message with loop prevention metadata
        """
        # For now, this just calls the synchronous version
        # In the future, this could be enhanced with async-specific functionality
        return self.process_message(message, service_name)
        
    def configure(self, config: Dict[str, Any]) -> None:
        """
        Configure the middleware.
        
        Args:
            config: Configuration dictionary
        """
        self._config.update(config)
        logger.info(f"MessageLoopPreventionMiddleware configured: {self._config}")
        
    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about message processing.
        
        Returns:
            Dictionary of statistics
        """
        stats = self._prevention.get_stats()
        stats['middleware_config'] = self._config
        return stats
        
    def register_category(self, category: str,
                          category_type: Optional[str] = None,
                          priority: Optional[str] = None,
                          max_simultaneous_processing: Optional[int] = None) -> str:
        """Register a message category with the prevention service.

        Story B3.1. The FCS, FMS and Radar handlers all probe
        `hasattr(self.loop_prevention, 'register_category')` before
        registering their categories, and this is the middleware they probe.
        The method did not exist, so each handler logged "Loop prevention
        middleware does not support category registration - using default
        logic" and returned, leaving 21 registrations dead. Measured on a
        live boot: that warning exactly 7 times.

        Delegates to MessageLoopPrevention.register_category, whose docstring
        records which of these arguments are acted upon (the name) and which
        are recorded only (the other three, because nothing in the package
        consumes them).
        """
        return self._prevention.register_category(
            category,
            category_type=category_type,
            priority=priority,
            max_simultaneous_processing=max_simultaneous_processing,
        )

    def get_category_metadata(self) -> Dict[str, Dict[str, Any]]:
        """What register_category() was told, for inspection."""
        return self._prevention.get_category_metadata()

    def clear_registry(self, category: Optional[str] = None) -> None:
        """
        Clear the registry.
        
        Args:
            category: Optional category to clear
        """
        self._prevention.clear_registry(category)
        
    def reset_stats(self) -> None:
        """Reset statistics."""
        self._prevention.reset_stats()
        
    def is_enabled(self) -> bool:
        """
        Check if middleware is enabled.
        
        Returns:
            Boolean indicating if middleware is enabled
        """
        return self._config['enabled']
        
    def enable(self) -> None:
        """Enable middleware."""
        self._config['enabled'] = True
        logger.info("MessageLoopPreventionMiddleware enabled")
        
    def disable(self) -> None:
        """Disable middleware."""
        self._config['enabled'] = False
        logger.info("MessageLoopPreventionMiddleware disabled")

def get_loop_prevention_middleware() -> MessageLoopPreventionMiddleware:
    """Get the singleton instance of MessageLoopPreventionMiddleware."""
    return MessageLoopPreventionMiddleware.get_instance()
