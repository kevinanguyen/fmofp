"""
Base module for predefined messages.

Contains base class and initialization logic.
"""


# Story C10.1: a second import bound `get_display_message_handler` to
# DisplayMessageHandler's function on the line above this one. Two DIFFERENT
# modules were bound to one name, and the alias below — being later — is what
# has always won. The dead import was actively misleading: a reader following
# `get_display_message_handler` in this file would land in the wrong module.
from FMOFP.Interfaces.userInterface.messaging.interface_display_message_handler import get_interface_display_message_handler as get_display_message_handler
from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()


class PredefinedMessagesBase:
    """
    Base class for predefined messages.
    Contains initialization logic and common methods.
    """

    def __init__(self):
        """Initialize with None handlers that will be set later."""
        self.radar_handler = None
        self.display_handler = None
        self.async_handler = None
        self.initialized = False
        self.routing_service = None
        self.logger = get_logger()
        
    async def initialize(self, radar_handler=None):
        """
        Asynchronously initialize the class with required handlers.
        This should be called after creating an instance.
        
        Args:
            radar_handler: Optional radar message handler to use
                          If not provided, will get from system manager
        """
        # Get system manager and handlers
        if radar_handler:
            self.radar_handler = radar_handler
            self.logger.info("Using provided radar message handler")
        else:
            from FMOFP.core.system_manager import get_system_manager
            system_manager = get_system_manager()
            # Get radar message handler
            self.radar_handler = system_manager.components.get('radar_message_handler')
            self.logger.info("Using radar message handler from system manager")
        
        # Get display message handler
        self.display_handler = get_display_message_handler()
        
        # Get async message handler if available through system manager
        from FMOFP.core.system_manager import get_system_manager
        system_manager = get_system_manager()
        self.async_handler = system_manager.components.get('async_message_handler')
        
        # Get routing service
        from FMOFP.local_messaging.routing.MessageRoutingService import get_message_routing_service
        self.routing_service = get_message_routing_service()
        
        self.initialized = True
        self.logger.info("PredefinedMessages initialized")
        
        # Verify handlers are available
        await self._verify_handlers()
        
    async def _verify_handlers(self):
        """Verify that all required handlers are available."""
        if not all([self.radar_handler, self.display_handler, self.async_handler, self.routing_service]):
            missing = []
            if not self.radar_handler:
                missing.append("radar_message_handler")
            if not self.display_handler:
                missing.append("display_message_handler")
            if not self.async_handler:
                missing.append("async_message_handler")
            if not self.routing_service:
                missing.append("routing_service")
                
            self.logger.error(f"Missing required handlers: {', '.join(missing)}")
            raise RuntimeError(f"Missing required handlers: {', '.join(missing)}")
