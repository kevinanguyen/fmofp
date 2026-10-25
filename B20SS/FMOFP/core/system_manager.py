"""
System Manager for Flight Management Operating Flight Program
"""
import threading
import time
import traceback
import asyncio
import FMOFP.Utils.common.fetching as fetching
from FMOFP.Systems.radarManagement.radarControl import get_radar_management_system
from FMOFP.Systems.radarManagement.radar_messaging.radarMessenger import get_radar_messenger
from FMOFP.Interfaces.userInterface.messaging.displayMessenger import get_display_messenger
from FMOFP.Systems.flightManagementSys.flightManagementSystem import get_flightManagementSystem
from FMOFP.Systems.flightManagementSys.fmsMessenger import get_fms_messenger
from FMOFP.local_messaging.routing.handlers.system_message_handlers.FMSMessageHandler import get_fms_message_handler
from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_messaging.message_queue_manager import get_message_queue_manager
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Utils.common.thread_manager import thread_manager
from FMOFP.Utils.common.system_states import SystemState
from FMOFP.Utils.common.system_state_manager import get_system_state_manager
from FMOFP.Utils.common.health import (
    CRITICAL_COMPONENTS,
    DelegatedHealth,
    HealthRegistry,
    HealthState,
    probe_all,
)
from FMOFP.Utils.debug.userCLI import get_user_cli
from FMOFP.core.systemsStartUp import systemsStartup
from FMOFP.MIL_STD_1553B.Bus_Controller.BC import get_Bus_Controller
from FMOFP.MIL_STD_1553B.Remote_Terminal.RT import Remote_Terminal
from FMOFP.local_messaging.routing.handlers.sync_handler.AsyncMessageHandler import get_Async_message_handler, HandlerState
from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import get_radar_message_handler
from FMOFP.local_messaging.routing.handlers.system_message_handlers.DisplayMessageHandler import get_display_message_handler
from FMOFP.local_messaging.routing.handlers.system_message_handlers.CommsMessageHandler import get_comms_message_handler
from FMOFP.local_messaging.routing.handlers.system_message_handlers.MissionMessageHandler import get_mission_message_handler
from FMOFP.Interfaces.userInterface.displays.display_nodes.display_tree_manager import get_display_tree_manager
from FMOFP.core.event_driven_communication import get_event_bus
from FMOFP.MIL_STD_1553B.Messaging import get_schedule_message
from FMOFP.Interfaces.userInterface.managers.display_manager import get_display_manager
from PyQt6.QtCore import QTimer
from FMOFP.Utils.common.operation_tracker import track_operation

logger = get_logger()

#: Seconds between repeats of an unchanged readiness-refusal log line.
#: A changed reason always logs immediately, so progress through boot stays
#: visible; only a stuck condition is throttled.
_READINESS_LOG_INTERVAL = 10.0



# Subsystems whose absence means a function the operator may command is simply
# not there: flight control, propulsion and the energy that feeds them, terrain
# protection, and navigation. Each of these is started inside a
# `try/except ... (non-fatal)` block, and a failure there used to be one log
# line. Measured before this change, with FCC.start() forced to raise:
#
#     is_system_ready():                  True
#     system state:                       SystemState.NORMAL
#     flight_control_computer registered: False
#
# The names are the labels used in those except blocks, not component keys,
# because a subsystem that fails to start is never registered as a component
# at all -- the registration line sits after the call that raised.
#
# Deliberately excluded, as degraded capability rather than an unflyable
# aircraft: PerformanceMonitor, FlightDataMonitoring, FlightManagementFitness,
# SoftwareConfigManager, ScenarioEngine, MissionService, CommsService,
# DefensiveService. Genuinely arguable and left out for now, so that adding
# them is a deliberate act rather than a default: SensorService,
# AirframeSystemManager, ECSControl.
SAFETY_CRITICAL_SUBSYSTEMS = frozenset({
    "FCC",                          # flight control computer
    "FlightSurfaceService",         # the surfaces it commands
    "HydraulicSystemController",    # which the surfaces need
    "ECU",                          # engine control
    "ThrustManagementSystem",
    "FuelManagementSystem",
    "PowerManagementSystem",
    "GCAS",                         # ground collision avoidance
    "NavService",
})


class SystemManager:
    def __init__(self):
        self.components = {}
        self.state_manager = get_system_state_manager()
        self.systems_startup = systemsStartup()
        self.running = False
        self.health_check_interval = 5  # seconds
        self.shutdown_event = threading.Event()
        # Handles for coroutine stop() calls scheduled during shutdown (B3).
        self._stop_waiters = []
        self.event_bus = get_event_bus()
        self.system_started = False
        self._thread_lock = threading.Lock()
        self.logger = get_logger().logger
        self.display_manager = None
        self.display_timer = None
        self._initialization_complete = False
        # Most recent health sweep (stories C4.1 / C5.1). Written by the
        # health_monitor thread, read from the Qt timer that drives readiness,
        # hence the lock inside HealthRegistry.
        self.health = HealthRegistry()
        # How stale a sweep may be before is_system_ready() refreshes it inline.
        # Much shorter than health_check_interval (5 s) because readiness is
        # polled every 100 ms during boot and quantising it to the monitor's
        # cadence added ~6 s of dead time to every start. Measured cost of a
        # sweep across 51 components is ~0.05 ms, so refreshing at up to 2 Hz is
        # free; the monitor thread remains the steady-state sweeper.
        self.readiness_probe_max_age = 0.5
        # Rate limiting for the readiness-refusal log line. Readiness is polled
        # ~20x/s during boot (Main.py's QTimer plus its await loop), so logging
        # every refusal produced 2,491 lines in a 178 s failure run -- the same
        # log-flood pattern story C7.1 exists to remove, and it would be
        # inconsistent to fix it in the listeners and leave it here. A refusal
        # is logged when its REASON changes, or every _READINESS_LOG_INTERVAL
        # seconds while unchanged, so the condition stays visible without
        # drowning the log.
        # Subsystems whose start() raised. Populated by
        # _record_startup_failure(); the safety-critical ones block
        # readiness. A set, because a subsystem is started once and the
        # same failure must not accumulate.
        self._failed_subsystem_starts = set()
        self._last_readiness_reason = None
        self._last_readiness_log = 0.0

    def get_component(self, component_name):
        """Get a component by name"""
        return self.components.get(component_name)

    def register_component(self, component_name, component):
        """Register a component with the system manager"""
        logger.info(f"Registering component: {component_name}")
        self.components[component_name] = component
        return True

    def register_message_handler(self, handler_name, handler_function):
        """Register a message handler function with the system manager"""
        logger.info(f"Registering message handler: {handler_name}")
        self.components[f"handler_{handler_name}"] = handler_function
        return True

    def initialize_components(self):
        if self._initialization_complete:

            return

        logger.info("Initializing system components")
        try:
            # Initialize event bus first
            logger.info("Initializing event bus")
            self.components['event_bus'] = self.event_bus

            # Initialize message queue manager first
            logger.info("Initializing message queue manager")
            queue_manager = get_message_queue_manager()
            self.components['message_queue_manager'] = queue_manager

            # Initialize async message handler
            logger.info("Initializing async message handler")
            async_handler = get_Async_message_handler()
            self.components['async_message_handler'] = async_handler

            # Initialize message routing service
            logger.info("Initializing message routing service")
            from FMOFP.local_messaging.routing.MessageRoutingService import get_message_routing_service
            routing_service = get_message_routing_service()
            self.components['message_routing_service'] = routing_service

            # Initialize display response service
            logger.info("Initializing display response service")
            from FMOFP.local_messaging.routing.response_services.system_response_services.DisplayResponseService import get_display_response_service
            display_response_service = get_display_response_service()
            self.components['display_response_service'] = display_response_service

            # Initialize VIL response service
            logger.info("Initializing VIL response service")
            from FMOFP.storage.DBM import DatabaseManager
            db_manager = DatabaseManager('FMOFP/dbConfig.xml')
            radar_db = db_manager.get_system_db('radar_management')
            from FMOFP.local_messaging.routing.response_services.data_response_services.vil_response_service import VILResponseService
            vil_response_service = VILResponseService(radar_db)
            self.components['vil_response_service'] = vil_response_service

            # Connect services
            routing_service.set_display_response_service(display_response_service)
            routing_service._vil_service = vil_response_service

            # Initialize flight management and radar systems first
            logger.info("Initializing flight management system")
            flight_management = get_flightManagementSystem()
            self.components['flightManagementSystem'] = flight_management

            # Initialize FMS messaging components
            logger.info("Initializing FMS messaging components")
            self.components['fms_message_handler'] = get_fms_message_handler()
            self.components['fms_messenger'] = get_fms_messenger()

            # Initialize Comms and Mission message handlers
            logger.info("Initializing Comms message handler")
            self.components['comms_message_handler'] = get_comms_message_handler()
            logger.info("Initializing Mission message handler")
            self.components['mission_message_handler'] = get_mission_message_handler()

            # Initialize radar management
            logger.info("Initializing radar management")
            radar_management = get_radar_management_system()
            self.components['radar_management'] = radar_management

            # Initialize radar messaging components in order
            logger.info("Initializing radar messaging components")
            self.components['radar_message_handler'] = get_radar_message_handler()
            self.components['radar_messenger'] = get_radar_messenger()

            # Set radar response service in routing service
            routing_service.set_radar_response_service(self.components['radar_message_handler'].response_service)

            # Initialize display components in order
            logger.info("Initializing display components")

            # Create display tree manager first
            display_tree = get_display_tree_manager()
            self.components['display_tree_manager'] = display_tree
            logger.info("Display tree manager initialized")

            # Create display message handler (but don't set async handler yet)
            display_handler = get_display_message_handler()
            display_handler.display_tree = display_tree  # Set tree manager reference
            self.components['display_message_handler'] = display_handler
            logger.info("Display message handler created")

            # Create display messenger
            display_messenger = get_display_messenger()
            self.components['display_messenger'] = display_messenger
            logger.info("Display messenger created")

            # Verify display components
            if not self.components['display_tree_manager']:
                raise RuntimeError("Display tree manager not properly initialized")
            if not self.components['display_message_handler']:
                raise RuntimeError("Display message handler not properly initialized")
            if not self.components['display_messenger']:
                raise RuntimeError("Display messenger not properly initialized")

            logger.info("All display components initialized successfully")

            # Initialize unified routing system
            logger.info("Initializing unified routing system")
            from FMOFP.local_messaging.routing.system_integration import initialize_routing_system
            from FMOFP.local_messaging.routing.unified_router import get_unified_router
            initialize_routing_system()
            unified_router = get_unified_router()
            self.components['unified_router'] = unified_router

            # Initialize remaining components
            logger.info("Initializing remaining components")
            self.components['UserCLI_Control'] = get_user_cli()
            self.components['UserCLI_Input'] = get_user_cli()
            self.components['UserCLI_Processing'] = get_user_cli()
            self.components['UserCLI_Output'] = get_user_cli()
            self.components['user_cli'] = get_user_cli()
            self.components['schedule_message'] = get_schedule_message()
            logger.info("All components constructed successfully")
            # B2: this used to be `self._initialization_complete = True`, set
            # here -- before initialize_system()'s loop had called
            # initialize() on a single one of them. A component failing in that
            # loop therefore left the flag reading "initialization complete".
            # It is now set by initialize_system(), once the loop has finished.
        except Exception as e:
            logger.error(f"Error initializing components: {str(e)}")
            raise

    def setup_display_timer(self):
        """Set up display update timer"""
        try:
            # Create timer for display updates
            self.display_timer = QTimer()
            self.display_timer.setInterval(16)  # ~60 FPS
            self.display_timer.timeout.connect(self._update_displays)
            self.display_timer.start()
        except Exception as e:
            logger.error(f"Error setting up display timer: {str(e)}")
            raise

    def _update_displays(self):
        """Update displays from the main thread via timer"""
        try:
            if self.display_manager and self.display_manager._running:
                for display_id, display in self.display_manager.displays.items():
                    # Only process string keys to avoid duplicates
                    if not isinstance(display_id, str):
                        continue

                    # Check if container is running
                    if not display.is_running():
                        continue

                    # Update container
                    display.update()

                    # Also update the actual display inside the container if needed
                    if display_id == 'pfd' and hasattr(display, 'pfd') and display.pfd:
                        if hasattr(display.pfd, 'update'):
                            display.pfd.update()
                    elif display_id == 'mfd' and hasattr(display, 'mfd') and display.mfd:
                        if hasattr(display.mfd, 'update'):
                            display.mfd.update()
                    elif display_id == 'radar_display' and hasattr(display, 'display') and display.display:
                        if hasattr(display.display, 'update'):
                            display.display.update()
                    elif display_id == 'hud' and hasattr(display, 'hud') and display.hud:
                        if hasattr(display.hud, 'update'):

                            display.hud.update()
                    elif display_id == 'hud' and hasattr(display, 'hud') and display.hud:
                        if hasattr(display.hud, 'update'):
                            display.hud.update()
        except Exception as e:
            logger.error(f"Error updating displays: {str(e)}")

    async def initialize_system(self):
        if self._initialization_complete:

            return

        logger.info("Initializing system")
        try:
            # Set up display timer
            self.setup_display_timer()

            # Initialize components
            self.initialize_components()

            # Initialize components
            for component_name, component in self.components.items():
                try:
                    if hasattr(component, 'initialize'):
                        if asyncio.iscoroutinefunction(component.initialize):
                            await component.initialize()
                        else:
                            component.initialize()
                    logger.info(f"Initialized {component_name}")
                except Exception as e:
                    # BLOCKER B2: this used to `return` here, silently leaving
                    # every component after the failing one uninitialised --
                    # while _initialization_complete had ALREADY been set True
                    # by initialize_components() above, so the flag said boot
                    # had finished. Raise instead: the caller sets ERROR and
                    # the failure reaches Main.initialize(), which is the only
                    # place that can decline to mark the system initialised.
                    logger.error(f"Error initializing {component_name}: {str(e)}")
                    raise RuntimeError(
                        f"Component {component_name} failed to initialize: {e}") from e

            # Initialize display system
            logger.info("Starting display system")
            if not self.display_manager:
                self.display_manager = get_display_manager()

                # Initialize display tree first
                logger.info("Initializing display tree manager")
                display_tree = self.components.get('display_tree_manager')
                if not display_tree:
                    logger.error("Display tree manager not found in components")
                    raise RuntimeError("Display tree manager not found")

                # Initialize display tree
                if not display_tree._initialized:
                    await display_tree.initialize()
                    if not display_tree._initialized:
                        raise RuntimeError("Display tree manager failed to initialize")
                    logger.info("Display tree manager initialization verified")

                # Initialize display manager
                logger.info("Initializing display manager")
                await self.display_manager.initialize()
                if not hasattr(self.display_manager, '_init_complete') or not self.display_manager._init_complete:
                    raise RuntimeError("Display manager failed to initialize")
                logger.info("Display manager initialization verified")

                # Start display manager
                logger.info("Starting display manager")
                await self.display_manager.start()
                if not self.display_manager._running:
                    raise RuntimeError("Display manager failed to start")
                logger.info("Display manager startup verified")

                # Connect display manager to messenger
                logger.info("Connecting display manager to messenger")
                display_messenger = self.components.get('display_messenger')
                if not display_messenger:
                    raise RuntimeError("Display messenger not found in components")
                display_messenger.set_display_manager(self.display_manager)
                logger.info("Display manager connected to messenger")

                # Verify displays are running with enhanced container verification
                for display_id in ['pfd', 'mfd', 'radar_display', 'hud']:
                    if display_id not in self.display_manager.displays:
                        raise RuntimeError(f"Display {display_id} not found")

                    display_container = self.display_manager.displays[display_id]

                    # First check if the container is running
                    if not display_container.is_running():
                        logger.error(f"Display container {display_id} reports not running")

                        # Check if container is initialized
                        if hasattr(display_container, '_initialized') and not display_container._initialized:
                            logger.error(f"Display container {display_id} is not initialized")
                            raise RuntimeError(f"Display container {display_id} is not initialized")

                        # Check if actual display exists in container
                        if display_id == 'pfd' and (not hasattr(display_container, 'pfd') or not display_container.pfd):
                            logger.error(f"PFD container has no display instance")
                            raise RuntimeError(f"PFD container has no display instance")
                        elif display_id == 'mfd' and (not hasattr(display_container, 'mfd') or not display_container.mfd):
                            logger.error(f"MFD container has no display instance")
                            raise RuntimeError(f"MFD container has no display instance")
                        elif display_id == 'radar_display' and (not hasattr(display_container, 'display') or not display_container.display):
                            logger.error(f"Radar display container has no display instance")
                            raise RuntimeError(f"Radar display container has no display instance")
                        elif display_id == 'hud' and (not hasattr(display_container, 'hud') or not display_container.hud):
                            logger.error(f"HUD container has no display instance")
                            raise RuntimeError(f"HUD container has no display instance")

                        # If we get here, the container exists but is not running
                        raise RuntimeError(f"Display {display_id} not running")

                    # Additional verification for actual display inside container
                    if display_id == 'pfd':
                        if not hasattr(display_container, 'pfd') or not display_container.pfd:
                            logger.error(f"PFD container has no display instance")
                            raise RuntimeError(f"PFD container has no display instance")
                        if hasattr(display_container.pfd, 'is_running') and not display_container.pfd.is_running():
                            logger.error(f"PFD display inside container is not running")
                            raise RuntimeError(f"PFD display inside container is not running")
                    elif display_id == 'mfd':
                        if not hasattr(display_container, 'mfd') or not display_container.mfd:
                            logger.error(f"MFD container has no display instance")
                            raise RuntimeError(f"MFD container has no display instance")
                        if hasattr(display_container.mfd, 'is_running') and not display_container.mfd.is_running():
                            logger.error(f"MFD display inside container is not running")
                            raise RuntimeError(f"MFD display inside container is not running")
                    elif display_id == 'radar_display':
                        if not hasattr(display_container, 'display') or not display_container.display:
                            logger.error(f"Radar display container has no display instance")
                            raise RuntimeError(f"Radar display container has no display instance")
                        if hasattr(display_container.display, 'is_running') and not display_container.display.is_running():
                            logger.error(f"Radar display inside container is not running")
                            raise RuntimeError(f"Radar display inside container is not running")
                    elif display_id == 'hud':
                        if not hasattr(display_container, 'hud') or not display_container.hud:
                            logger.error(f"HUD container has no display instance")
                            raise RuntimeError(f"HUD container has no display instance")
                        if hasattr(display_container.hud, 'is_running') and not display_container.hud.is_running():
                            logger.error(f"HUD display inside container is not running")
                            raise RuntimeError(f"HUD display inside container is not running")

                    logger.info(f"Display {display_id} running verified (both container and display)")

            self.state_manager.set_state(SystemState.INITIALIZED)
            self.running = True
            # B2: set here, at the end of the real initialization sequence,
            # rather than in initialize_components() before any component had
            # been initialized at all.
            self._initialization_complete = True
            logger.info("System initialization completed")
        except Exception as e:
            # BLOCKER B2: this swallowed the failure. Main.initialize() awaits
            # this method and then sets self._initialized = True
            # unconditionally, so a config, database or display failure
            # produced one ERROR line and boot carried straight on into
            # start_FM_system() with an arbitrary subset of components missing
            # -- and reported itself started. Re-raise so the one caller that
            # can act on it gets the chance.
            logger.error(f"Error during system initialization: {str(e)}")
            self.state_manager.set_state(SystemState.ERROR)
            raise

    async def start_async_component(self, component_name, component):
        """Start an async component and handle its coroutine properly."""
        try:
            if asyncio.iscoroutinefunction(component.start):
                await component.start()
            else:
                component.start()
            logger.info(f"Started {component_name}")
            return True
        except Exception as e:
            logger.error(f"Error starting {component_name}: {str(e)}")
            return False

    def start_thread_if_not_running(self, thread_name, target):
        """Start a thread if it's not already running, with proper locking."""
        with self._thread_lock:
            if not thread_manager.is_thread_alive(thread_name):
                thread_manager.add_thread(thread_name, target=target)
                success = thread_manager.start_thread(thread_name)
                if success:
                    logger.info(f"Thread '{thread_name}' started successfully")
                else:
                    logger.warning(f"Failed to start thread '{thread_name}'")
            else:

                pass

    async def start_async_components(self):
        """Start all async components."""
        logger.info("Starting async components")

        # Start event bus first.
        #
        # Story C9.1: this used to ALSO spawn `_process_events` directly as a
        # thread named "Event_Bus" on the line above `event_bus.start()`, so two
        # threads were launched on the same target. Both entered the function --
        # "Event processing thread started" appears twice in every pre-fix boot
        # log -- and the system survived only on timing: `start()` is what sets
        # `running = True`, so the first thread usually reached its
        # `while self.running` check before that happened and exited at once.
        #
        # That is an accident, not a design. The two statements race: if the OS
        # scheduled the first thread after `start()` had already set the flag,
        # two threads would drain one queue. Queue.get() is thread-safe so
        # nothing would corrupt, but events would be dispatched by whichever
        # thread won, task_done() accounting would double-count, and the
        # message-loop-prevention layer would see interleaved processing it was
        # never designed for -- an intermittent, near-unreproducible class of bug.
        #
        # `EventBus.start()` is the correct single entry point: it is guarded by
        # the bus's own lock, checks `started` and thread liveness, names the
        # thread "EventBus_Processor" (the name the display widgets' comments
        # already refer to), and sets `_health_status`. The direct spawn added
        # nothing except the race.
        logger.info("Starting event bus")
        self.event_bus.start()

        # Start message queue manager
        logger.info("Starting message queue manager")
        queue_manager = self.components['message_queue_manager']
        queue_manager.start()
        logger.info(f"MessageQueueManager started with instance ID: {id(queue_manager)}")

        # Log RT_Listener instance ID for comparison
        from FMOFP.MIL_STD_1553B.Remote_Terminal.RT_connect.RT_socket import get_rt_listener
        rt_listener = get_rt_listener()
        logger.info(f"System Manager using RT_Listener instance: {id(rt_listener)}")

        # Start async message handler
        logger.info("Starting async message handler")
        async_handler = self.components['async_message_handler']

        singleton_handler = get_Async_message_handler()

        # If we have different instances, use the singleton
        if id(async_handler) != id(singleton_handler):
            logger.warning(f"AsyncMessageHandler instance mismatch: {id(async_handler)} != {id(singleton_handler)}")
            logger.warning("Using singleton instance instead")
            async_handler = singleton_handler
            self.components['async_message_handler'] = singleton_handler

        # Check if already started
        if not async_handler.started or not async_handler.running:
            await self.start_async_component("AsyncMessageHandler", async_handler)


        # Wait for async handler to be fully started and in RUNNING state
        if not async_handler.started or not async_handler.running or (hasattr(async_handler, '_state') and async_handler._state not in [2, 4]):  # 2=RUNNING, 4=DEGRADED
            logger.warning("Waiting for AsyncMessageHandler to start...")
            for _ in range(30):  # Try for 3 seconds
                if async_handler.started and async_handler.running and hasattr(async_handler, '_state') and async_handler._state in [2, 4]:
                    logger.info(f"AsyncMessageHandler is now running in state: {HandlerState.to_string(async_handler._state)}")
                    break
                await asyncio.sleep(0.1)
            else:
                logger.error("AsyncMessageHandler failed to start within timeout")
                # Force restart with more aggressive approach
                logger.warning("Attempting to restart AsyncMessageHandler with aggressive approach")

                # First stop completely
                try:
                    async_handler.stop()
                    # Wait for stop to complete
                    for _ in range(20):  # Wait up to 2 seconds
                        if not async_handler.running:
                            break
                        await asyncio.sleep(0.1)
                except Exception as e:
                    logger.error(f"Error stopping AsyncMessageHandler: {e}")

                # Then start fresh
                try:
                    # Reset state to STOPPED
                    if hasattr(async_handler, '_set_state'):
                        async_handler._set_state(0)  # 0 = STOPPED

                    # Start with fresh workers
                    async_handler.workers = []
                    async_handler._worker_ready_events = []

                    # Start with fresh executor
                    if hasattr(async_handler, 'executor') and async_handler.executor:
                        try:
                            async_handler.executor.shutdown(wait=False)
                        except Exception:
                            logger.debug("SystemManager: executor shutdown error during restart", exc_info=True)
                    async_handler.executor = None

                    # Now start
                    await self.start_async_component("AsyncMessageHandler", async_handler)

                    # Wait for start to complete
                    for _ in range(30):  # Wait up to 3 seconds
                        if async_handler.started and async_handler.running and hasattr(async_handler, '_state') and async_handler._state in [2, 4]:
                            logger.info(f"AsyncMessageHandler successfully restarted in state: {HandlerState.to_string(async_handler._state)}")
                            break
                        await asyncio.sleep(0.1)
                    else:
                        raise RuntimeError("AsyncMessageHandler failed to restart after aggressive approach")
                except Exception as e:
                    logger.error(f"Error in aggressive restart of AsyncMessageHandler: {e}")
                    raise RuntimeError(f"AsyncMessageHandler failed to start after aggressive restart: {e}")

        # Initialize database manager after async handler is running
        logger.info("Initializing database manager")

        def _initialize_database_manager():
            from FMOFP.storage.DBM import DatabaseManager
            dbm = DatabaseManager('FMOFP/dbConfig.xml')
            radar_db = dbm.get_system_db('radar_management')
            async_handler.radar_db = radar_db
            async_handler._db_initialized.set()  # Signal database is ready
            logger.info("Database initialization complete")
            return dbm

        # Track this operation to ensure it only happens once
        track_operation('db_init', 'system_manager', _initialize_database_manager)

        # Start FMS messaging components in order
        logger.info("Starting FMS messaging components")

        # 1. Start FMS message handler
        logger.info("Starting FMS message handler")
        fms_message_handler = self.components['fms_message_handler']
        fms_message_handler.set_async_handler(async_handler)
        await self.start_async_component("FMSMessageHandler", fms_message_handler)

        # 2. Start Comms message handler
        logger.info("Starting Comms message handler")
        comms_message_handler = self.components.get('comms_message_handler')
        if comms_message_handler:
            comms_message_handler.start()
            logger.info("Comms message handler started")

        # 3. Start Mission message handler
        logger.info("Starting Mission message handler")
        mission_message_handler = self.components.get('mission_message_handler')
        if mission_message_handler:
            mission_message_handler.start()
            logger.info("Mission message handler started")

        # 2. Start FMS messenger
        logger.info("Starting FMS messenger")
        fms_messenger = self.components['fms_messenger']
        await self.start_async_component("FMSMessenger", fms_messenger)

        # Start FMS system
        logger.info("Starting FMS system")
        flight_management = self.components['flightManagementSystem']
        flight_management.set_messenger(fms_messenger)
        await self.start_async_component("flightManagementSystem", flight_management)

        # Start radar messaging components in order
        logger.info("Starting radar messaging components")

        # 1. Start radar message handler
        logger.info("Starting radar message handler")
        radar_message_handler = self.components['radar_message_handler']
        radar_message_handler.set_async_handler(async_handler)
        await self.start_async_component("RadarMessageHandler", radar_message_handler)

        # 2. Start radar messenger
        # started by radar control

        # Start display messaging components in order
        logger.info("Starting display messaging components")

        # 1. Start display message handler
        logger.info("Starting display message handler")
        display_message_handler = self.components['display_message_handler']

        # Get interface display message handler
        # Stored on self (not just a local var) so stop_system() can reach
        # it later to call stop() -- see stop_system() and this class's
        # own stop() method, both added because this handler previously had
        # no stop() at all despite being the one that actually owns the
        # "DisplayMessageProcessor"/"DisplayMessageCleanup" threads.
        # NOTE: previously flagged as "should be started in displays
        # somewhere" instead of here. Deliberately NOT relocated: this
        # exact start ordering (interface handler started before
        # DisplayMessageHandler below) is what determines which of the two
        # competing implementations wins the "DisplayMessageProcessor"/
        # "DisplayMessageCleanup" thread-name registration race (see the
        # comment above and this class's stop_system()/stop() - both were
        # fixed specifically because this handler previously had no stop()
        # despite owning those threads). Moving where this gets constructed
        # and started risks silently reversing that ordering and
        # reintroducing the exact bug that was root-caused and fixed.
        # Central orchestration in system_manager.py, matching every other
        # component start below, is the correct place for this.
        from FMOFP.Interfaces.userInterface.messaging.interface_display_message_handler import get_interface_display_message_handler
        interface_handler = get_interface_display_message_handler()
        self.interface_display_handler = interface_handler

        # Set async handler on the main display handler only
        logger.info("Setting async handler on display message handler")
        try:
            # Only set async_handler on the main display handler
            display_message_handler.set_async_handler(async_handler)
            logger.info("Successfully set async_handler on display message handler")
        except Exception as e:
            logger.error(f"Error setting async_handler on display handler: {e}")
            raise RuntimeError(f"Failed to set async handler: {e}")

        # Start interface handler first
        logger.info("Starting interface display message handler")
        await interface_handler.start()
        if not interface_handler.started:
            logger.error("Interface display message handler failed to start")
            raise RuntimeError("Interface display message handler failed to start")
        logger.info("Interface display message handler running")

        # Now start the regular display message handler
        logger.info("Starting main display message handler")
        await self.start_async_component("DisplayMessageHandler", display_message_handler)
        if not display_message_handler.started:
            logger.error("DisplayMessageHandler failed to start")
            raise RuntimeError("DisplayMessageHandler failed to start")
        logger.info("Display message handler running")

        # 2. Start display messenger
        logger.info("Starting display messenger")
        await self.start_async_component("DisplayMessenger", self.components['display_messenger'])

        # Start unified router
        logger.info("Starting unified router")
        unified_router = self.components.get('unified_router')
        if unified_router:
            # Create a wrapper function that doesn't require arguments
            def unified_router_thread():
                logger.info("Unified router thread started")
                # This is just a placeholder thread function
                # The actual routing is done by calling route_message with a message
                while self.running:
                    try:
                        time.sleep(1)  # Just keep the thread alive
                    except Exception as e:
                        logger.error(f"Error in unified router thread: {e}")
                        break
                logger.info("Unified router thread stopped")

            # Start unified router thread with the wrapper function
            self.start_thread_if_not_running("UnifiedRouter", unified_router_thread)
            logger.info("Unified router started")
        else:
            logger.warning("Unified router component not found")

        # Start remaining components
        logger.info("Starting remaining components")
        for component_name, component in self.components.items():
            if component_name not in ['event_bus', 'async_message_handler',
                                    'radar_message_handler', 'radar_messenger',
                                    'display_message_handler', 'display_messenger',
                                    'unified_router']:
                if component_name == 'schedule_message':
                    self.start_thread_if_not_running("ScheduleMessage", component.start)
                elif hasattr(component, 'start'):
                    if asyncio.iscoroutinefunction(component.start):
                        await self.start_async_component(component_name, component)
                    else:
                        self.start_thread_if_not_running(component_name, component.start)

        logger.info("All async components started successfully")
        return True

    def _log_fire_and_forget_task_exception(self, task, task_name: str) -> None:
        """Retrieve and log the exception (if any) from a fire-and-forget
        asyncio.Task created via loop.create_task() and never otherwise
        awaited. Without this, a failure in such a task is only ever
        reported by asyncio's generic default exception handler at an
        unpredictable future garbage-collection time, with no indication
        which boot-time task failed or that the system may now be in a
        degraded state.
        """
        try:
            exc = task.exception()
        except asyncio.CancelledError:
            return
        if exc is not None:
            logger.error(f"[BOOT] Fire-and-forget task {task_name} failed: {exc}", exc_info=exc)

    def start_FM_system(self):
        if self.system_started:
            logger.warning("System has already been started. Ignoring repeated start attempt.")
            return

        logger.info("Starting system components")
        self.state_manager.set_state(SystemState.STARTING)

        try:
            # Get the existing event loop
            try:
                loop = asyncio.get_event_loop()
                logger.info("Using existing event loop")
            except RuntimeError:
                logger.error("No event loop found in current thread")
                raise RuntimeError("Event loop not properly initialized")

            # Initialize system using create_task
            init_task = loop.create_task(self.initialize_system())
            # Fire-and-forget task: if initialize_system() raises, asyncio's
            # default handler logs "Task exception was never retrieved" (via
            # the generic 'asyncio' logger, only once this task object is
            # garbage collected) with no application-level context and no
            # indication that boot may be in a degraded state. Attach a
            # done-callback -- the same pattern already used correctly in
            # the response_services classes' _handle_task_done -- so a
            # failure here is retrieved and logged immediately, with clear
            # context, through this module's own logger.
            init_task.add_done_callback(
                lambda t: self._log_fire_and_forget_task_exception(t, "initialize_system()")
            )

            # Start the main loop
            self.start_thread_if_not_running("Main_Loop", self.main_loop)

            # Start BC and RT threads
            self.logger.info("Starting BC threads...")
            self.listenerBC = get_Bus_Controller()
            self.start_thread_if_not_running("BC Listener", self.listenerBC.start_listener)

            self.logger.info("Starting RT threads...")
            # Stored on self (not just a local var) so stop_system() can
            # reach it later to call stop_listener() -- see stop_system().
            self.listenerRT = Remote_Terminal()
            self.start_thread_if_not_running("RT Listener", self.listenerRT.start_listener)

            # Story C3.1: also register both listeners as components.
            #
            # They were previously reachable ONLY as these attributes, and
            # check_component_health() iterates self.components -- so the bus
            # transport, the single most load-bearing thing in the system, was
            # outside the health sweep entirely. Fixing the sweep (C4.1) and
            # readiness (C5.1) without this would still have left the
            # two-instance port-conflict failure undetected, because neither
            # listener would ever have been asked.
            #
            # The attribute references above are deliberately kept: stop_system()
            # uses them (see the getattr(self, 'listenerBC', None) block), and
            # the names differ from the component keys on purpose so the two
            # access paths stay easy to tell apart when reading.
            # Registered through DelegatedHealth rather than directly: the two
            # objects above are wrappers with no health interface, BC_Listener
            # is not a singleton, and the BC listener is constructed inside its
            # own thread so it does not exist yet at this point. See
            # DelegatedHealth's docstring for the full reasoning.
            self.components['bc_listener'] = DelegatedHealth(
                'bc_listener', lambda: getattr(self.listenerBC, '_listener', None)
            )
            self.components['rt_listener'] = DelegatedHealth(
                'rt_listener', lambda: getattr(self.listenerRT, 'rt_listener', None)
            )

            # Start User CLI threads
            for component_name, component in self.components.items():
                if component_name == 'UserCLI_Control':
                    self.start_thread_if_not_running(f"{component_name}", component.commandLineThreadControl)
                elif component_name == 'UserCLI_Input':
                    self.start_thread_if_not_running(f"{component_name}", component.get_commands)
                elif component_name == 'UserCLI_Processing':
                    self.start_thread_if_not_running(f"{component_name}", component.process_commands)
                elif component_name == 'UserCLI_Output':
                    self.start_thread_if_not_running(f"{component_name}", component.output_commands)
                elif component_name == 'user_cli':
                    self.enable_user_cli()

            # Start async components using create_task
            start_components_task = loop.create_task(self.start_async_components())
            # Same fire-and-forget exception-visibility gap as init_task above.
            start_components_task.add_done_callback(
                lambda t: self._log_fire_and_forget_task_exception(t, "start_async_components()")
            )

            # Start FMS and radar management systems
            #
            # 'flightManagementSystem' is populated by initialize_components(),
            # which initialize_system() calls synchronously at its own start.
            # The intended sequence (Main.py's start_fmofp: await
            # fmofp.initialize() fully, then fmofp.start()) already guarantees
            # initialize_system() has completed by the time start_FM_system()
            # runs. But the init_task = loop.create_task(self.initialize_system())
            # a few lines above is a fire-and-forget task: if start_FM_system()
            # is ever called before initialize_system() has actually run (e.g.
            # called directly, out of sequence, or from a different caller than
            # Main.py), that task has not had a chance to execute yet and this
            # dict access raises KeyError immediately -- confirmed live: 'Error
            # during system start: 'flightManagementSystem'' fires on every
            # such call. initialize_components() is idempotent (it no-ops if
            # already run), so call it directly here as a synchronous guard
            # instead of assuming a precondition the type system doesn't
            # enforce (production readiness deep dive, found via live testing
            # of start_FM_system() in isolation).
            if 'flightManagementSystem' not in self.components:
                logger.warning(
                    "flightManagementSystem not in components at start_FM_system(); "
                    "initialize_system() has not completed yet -- running "
                    "initialize_components() synchronously before proceeding"
                )
                self.initialize_components()
            logger.info("Starting flight management system")
            flight_management = self.components['flightManagementSystem']
            flight_management.start()

            logger.info("Starting radar management system")
            radar_management = self.components['radar_management']
            radar_management.start()

            logger.info("Adding RadarMain thread")
            thread_manager.add_thread(name="RadarMain", target=radar_management._update_loop)
            logger.info("Starting RadarMain thread")
            self.start_thread_if_not_running("RadarMain", radar_management._update_loop)

            logger.info("Adding RadarManagement thread")
            thread_manager.add_thread(name="RadarManagement", target=radar_management.update)
            self.start_thread_if_not_running("RadarManagement", radar_management.update)

            # ── Phase 2: aircraft subsystems ──────────────────────────────────

            # Flight Control Computer
            try:
                logger.info("Starting Flight Control Computer")
                from FMOFP.Systems.flightControlSys.flightControlComputer.flightControlComputer import get_flight_control_computer
                fcc = get_flight_control_computer()
                fcc.start()
                self.components['flight_control_computer'] = fcc
                logger.info("Flight Control Computer started")
            except Exception as e:
                self._record_startup_failure("FCC", e)

            # Navigation Service (GPS + INS + fusion)
            try:
                logger.info("Starting Navigation Service")
                from FMOFP.Systems.nav.navService import get_nav_service
                nav_svc = get_nav_service()
                nav_svc.start()
                self.components['nav_service'] = nav_svc
                logger.info("Navigation Service started")
            except Exception as e:
                self._record_startup_failure("NavService", e)

            # Engine Control Unit
            try:
                logger.info("Starting Engine Control Unit")
                from FMOFP.Systems.engineManagement.ecu.engineControlUnit import get_engine_control_unit
                ecu = get_engine_control_unit()
                ecu.start()
                self.components['engine_control_unit'] = ecu
                logger.info("Engine Control Unit started")
            except Exception as e:
                self._record_startup_failure("ECU", e)

            # Communications Service
            try:
                logger.info("Starting Communications Service")
                from FMOFP.Systems.comms.messaging_service import get_comms_service
                comms = get_comms_service()
                comms.start()
                self.components['comms_service'] = comms
                logger.info("Communications Service started")
                # Wire into routing service so route_comms_message can reach it
                try:
                    from FMOFP.local_messaging.routing.MessageRoutingService import get_message_routing_service
                    get_message_routing_service().set_comms_service(comms)
                    logger.info("CommsService registered with MessageRoutingService")
                except Exception as e:
                    logger.warning(f"CommsService routing registration failed (non-fatal): {e}")
            except Exception as e:
                self._record_startup_failure("CommsService", e)

            # Mission Planning Service
            try:
                logger.info("Starting Mission Planning Service")
                from FMOFP.Systems.missionPlanning.missionService import get_mission_service
                mission = get_mission_service()
                mission.start()
                self.components['mission_service'] = mission
                logger.info("Mission Planning Service started")
                # Wire into routing service so route_mission_message can reach it
                try:
                    from FMOFP.local_messaging.routing.MessageRoutingService import get_message_routing_service
                    get_message_routing_service().set_mission_service(mission)
                    logger.info("MissionService registered with MessageRoutingService")
                except Exception as e:
                    logger.warning(f"MissionService routing registration failed (non-fatal): {e}")
            except Exception as e:
                self._record_startup_failure("MissionService", e)

            # ── end Phase 2 ───────────────────────────────────────────────────

            # ── Phase 3: new systems ──────────────────────────────────────────

            # Defensive Systems (RWR, countermeasures, EW)
            try:
                logger.info("Starting Defensive Service")
                from FMOFP.Systems.defensiveSys.defensiveService import get_defensive_service
                dfs = get_defensive_service()
                dfs.start()
                self.components['defensive_service'] = dfs
                logger.info("Defensive Service started")
            except Exception as e:
                self._record_startup_failure("DefensiveService", e)

            # Flight Surfaces
            try:
                logger.info("Starting Flight Surface Service")
                from FMOFP.Systems.flightSurfaces.flightSurfaceService import get_flight_surface_service
                fss = get_flight_surface_service()
                fss.start()
                self.components['flight_surface_service'] = fss
                logger.info("Flight Surface Service started")
            except Exception as e:
                self._record_startup_failure("FlightSurfaceService", e)

            # GCAS
            try:
                logger.info("Starting GCAS")
                from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys import get_gcas
                gcas = get_gcas()
                gcas.start()
                self.components['gcas'] = gcas
                logger.info("GCAS started")
            except Exception as e:
                self._record_startup_failure("GCAS", e)

            # Performance Monitor
            try:
                logger.info("Starting Performance Monitor")
                from FMOFP.Systems.flightControlSys.performaneMonitoring.performaneMonitoring import get_performance_monitor
                perf = get_performance_monitor()
                perf.start()
                self.components['performance_monitor'] = perf
                logger.info("Performance Monitor started")
            except Exception as e:
                self._record_startup_failure("PerformanceMonitor", e)

            # Power Management System (batteries, avionics cooling loop, HVAC)
            try:
                logger.info("Starting Power Management System")
                from FMOFP.Systems.powerManagement.elec.powerManagementSystem import get_power_management_system
                pms = get_power_management_system()
                pms.start()
                self.components['power_management_system'] = pms
                logger.info("Power Management System started")
            except Exception as e:
                self._record_startup_failure("PowerManagementSystem", e)

            # Fuel Management System (tanks, engines, monitor, transfer manager)
            try:
                logger.info("Starting Fuel Management System")
                from FMOFP.Systems.engineManagement.fuelManagement.fuelControl import get_fuel_management_system
                fuel_mgmt = get_fuel_management_system()
                fuel_mgmt.start()
                self.components['fuel_management_system'] = fuel_mgmt
                logger.info("Fuel Management System started")
            except Exception as e:
                self._record_startup_failure("FuelManagementSystem", e)

            # Thrust Management System (twin-engine coordination, autothrottle)
            try:
                logger.info("Starting Thrust Management System")
                from FMOFP.Systems.engineManagement.thrustManagement.thrustManagement import get_thrust_management_system
                thrust_mgmt = get_thrust_management_system()
                thrust_mgmt.start()
                self.components['thrust_management_system'] = thrust_mgmt
                logger.info("Thrust Management System started")
            except Exception as e:
                self._record_startup_failure("ThrustManagementSystem", e)

            # Hydraulic System Controller (primary/backup pressure monitoring)
            try:
                logger.info("Starting Hydraulic System Controller")
                from FMOFP.Systems.hydraulics.hydrControl import get_hydraulic_system_controller
                hydraulics = get_hydraulic_system_controller()
                hydraulics.start()
                self.components['hydraulic_system_controller'] = hydraulics
                logger.info("Hydraulic System Controller started")
            except Exception as e:
                self._record_startup_failure("HydraulicSystemController", e)

            # Airframe System Manager (structural sensors, landing gear)
            try:
                logger.info("Starting Airframe System Manager")
                from FMOFP.Systems.airframeSystemManagement.airframeControl import get_airframe_system_manager
                airframe = get_airframe_system_manager()
                airframe.start()
                self.components['airframe_system_manager'] = airframe
                logger.info("Airframe System Manager started")
            except Exception as e:
                self._record_startup_failure("AirframeSystemManager", e)

            # Environmental Control System
            try:
                logger.info("Starting Environmental Control System")
                from FMOFP.Systems.enviornmentalControlSystem.ecsControl import get_ecs_control
                ecs = get_ecs_control()
                ecs.start()
                self.components['ecs_control'] = ecs
                logger.info("Environmental Control System started")
            except Exception as e:
                self._record_startup_failure("ECSControl", e)

            # Flight Data Monitoring (flight data / cockpit voice recorders)
            try:
                logger.info("Starting Flight Data Monitoring")
                from FMOFP.Systems.flightDataMonitoring.fdmControl import get_flight_data_monitoring
                fdm = get_flight_data_monitoring()
                fdm.start()
                self.components['flight_data_monitoring'] = fdm
                logger.info("Flight Data Monitoring started")
            except Exception as e:
                self._record_startup_failure("FlightDataMonitoring", e)

            # Flight Management Fitness (component health / redundancy monitoring)
            try:
                logger.info("Starting Flight Management Fitness")
                from FMOFP.Systems.fmFitness.fmStatus import get_flight_management_fitness
                fmf = get_flight_management_fitness()
                fmf.start()
                self.components['flight_management_fitness'] = fmf
                logger.info("Flight Management Fitness started")
            except Exception as e:
                self._record_startup_failure("FlightManagementFitness", e)

            # Software Configuration Manager (component/data-load inventory;
            # queryable on demand, does not run a background thread -- see
            # get_software_config_manager()'s docstring)
            try:
                logger.info("Starting Software Configuration Manager")
                from FMOFP.Systems.configurationManagement.swConfigure import get_software_config_manager
                swcm = get_software_config_manager()
                self.components['software_config_manager'] = swcm
                logger.info("Software Configuration Manager started")
            except Exception as e:
                self._record_startup_failure("SoftwareConfigManager", e)

            # ── Phase 5: scenario engine + sensor service ─────────────────────
            try:
                from FMOFP.Interfaces.scenarios.scenarioEngine import get_scenario_engine
                scenario_engine = get_scenario_engine()
                self.components['scenario_engine'] = scenario_engine
                logger.info("ScenarioEngine registered (call load() + start() to run a scenario)")
            except Exception as e:
                self._record_startup_failure("ScenarioEngine", e)

            try:
                from FMOFP.Systems.sensorManagement.sensorService import get_sensor_service
                sensor_service = get_sensor_service()
                sensor_service.start()
                self.components['sensor_service'] = sensor_service
                logger.info("SensorService started")
            except Exception as e:
                self._record_startup_failure("SensorService", e)
            # ── end Phase 5 ───────────────────────────────────────────────────

            # ── end Phase 3 ───────────────────────────────────────────────────

            # Start display management system
            logger.info("Starting display management system")
            if self.display_manager:
                logger.info("Display system already running via Qt timer")

            # Start the health monitor
            self.start_thread_if_not_running("HealthMonitor", self.health_monitor)

            # Start remote systems. systemsStartup.start() is a coroutine:
            # the previous bare `self.systems_startup.start()` call just
            # created a coroutine object and dropped it ("RuntimeWarning:
            # coroutine 'systemsStartup.start' was never awaited" on every
            # boot), so _start_radar_system() and _start_displays() never
            # actually executed (found August 2026 live-boot
            # re-verification). Schedule it on the running loop with the
            # same retained-task + logged-exception pattern used for
            # initialize_system()/start_async_components() above; the task
            # reference is kept on self because asyncio holds only weak
            # task references.
            self._systems_startup_task = loop.create_task(
                self.systems_startup.start()
            )
            self._systems_startup_task.add_done_callback(
                lambda t: self._log_fire_and_forget_task_exception(
                    t, "systems_startup.start()"
                )
            )

            self.state_manager.set_state(SystemState.RUNNING)
            logger.info("All system components started")

            active_threads = thread_manager.get_active_threads()
            logger.debug(f"Active threads: {active_threads}")

            self.system_started = True

        except Exception as e:
            logger.error(f"Error during system start: {str(e)}")
            self.state_manager.set_state(SystemState.ERROR)

    def main_loop(self):
        logger.info(f"Main loop started. Thread ID: {threading.get_ident()}")
        while self.running:
            try:
                current_state = self.state_manager.get_state()
                if current_state == SystemState.ERROR:
                    logger.error("System entered ERROR state. Initiating shutdown.")
                    self.stop()
                    break
                time.sleep(1)
            except Exception as e:
                logger.error(f"Error in main loop: {str(e)}")
                self.state_manager.set_state(SystemState.ERROR)
                break
        logger.info(f"Main loop ended. Thread ID: {threading.get_ident()}")

    def health_monitor(self):
        logger.info(f"Health monitor started. Thread ID: {threading.get_ident()}")
        while self.running:
            try:
                self.check_component_health()
                time.sleep(self.health_check_interval)
            except Exception as e:
                logger.error(f"Error in health monitor: {str(e)}")
        logger.info(f"Health monitor ended. Thread ID: {threading.get_ident()}")

    def check_component_health(self):
        """Sweep every registered component and publish the result (story C4.1).

        This method previously collected `unhealthy_components` and then threw
        the list away: both branches of its final `if` logged the same
        "All components are healthy" line and one of them was a bare `pass`,
        under a `# remove later when health manager is functional` comment. The
        health monitor thread therefore ran on its timer, did the work, and
        discarded the answer -- so nothing in the system could observe a
        component failure even in principle.

        It also asked only for `check_health`, missing the 17 classes that
        expose `is_healthy` instead, and silently skipped anything implementing
        neither. Probing now goes through the health contract
        (Utils/common/health.py), which resolves both spellings and reports
        "no health interface" as UNKNOWN rather than as tacit success.

        The result is published to `self.health` for `is_system_ready()`
        (story C5.1) and for anything else that needs to ask.
        """
        reports = probe_all(self.components)
        self.health.update(reports)

        unhealthy = [r for r in reports if r.is_unhealthy]
        unknown = [r for r in reports if r.state is HealthState.UNKNOWN]

        if unhealthy:
            critical = [r.name for r in unhealthy if r.name in CRITICAL_COMPONENTS]
            detail = "; ".join(f"{r.name}: {r.reason}" for r in unhealthy)
            if critical:
                # Severity depends on whether we have ever declared ourselves
                # operational. During STARTING/RUNNING the components come up in
                # sequence and a critical one being briefly unhealthy is the
                # normal shape of boot, not a fault -- logging that at ERROR
                # would break the project's clean-boot-zero-errors property for
                # no informational gain. Once the system has reached NORMAL, the
                # same condition means something that was working has failed,
                # which is a genuine error.
                reached_normal = self.state_manager.get_state() == SystemState.NORMAL
                log_at = logger.error if reached_normal else logger.info
                log_at(
                    f"Health sweep: {len(unhealthy)} of {len(reports)} components unhealthy, "
                    f"including critical {sorted(critical)} -- {detail}"
                )
            else:
                logger.warning(
                    f"Health sweep: {len(unhealthy)} of {len(reports)} components unhealthy "
                    f"(none critical) -- {detail}"
                )
        else:
            logger.debug(
                f"Health sweep: all {len(reports) - len(unknown)} interrogable components "
                f"healthy ({len(unknown)} expose no health interface)"
            )

        return unhealthy

    def get_health_snapshot(self):
        """Most recent health sweep, as a list of HealthReport (story C4.1).

        Empty until the first sweep has run; use `self.health.has_swept()` to
        tell "no failures" apart from "never checked".
        """
        return self.health.snapshot()

    # ── shutdown task scheduling (BLOCKER B3) ────────────────────────────
    #
    # Every coroutine stop() below used to be handled the same wrong way:
    #
    #     loop = asyncio.get_event_loop()
    #     if not loop.is_closed():
    #         loop.create_task(component.stop())
    #
    # Two defects in three lines.
    #
    # First, asyncio.get_event_loop() in a thread with no loop set is a
    # DeprecationWarning on 3.10 and a RuntimeError on 3.12+. stop_system()
    # runs on the "Main_Loop" background thread whenever shutdown is triggered
    # by SystemState.ERROR, and the outer handler re-raises -- so on the error
    # path everything after the first coroutine stop (radar management) never
    # ran at all: display manager, every Phase 2/3/5 subsystem, routing, the
    # async handler, the queue manager, the event bus, thread_manager's sweep,
    # the DatabaseManager close, and the SHUTDOWN transition itself. The loop
    # was therefore never stopped and the process hung with sockets and threads
    # live.
    #
    # Second, even on the happy path these were fire-and-forget tasks, and
    # stop_system() then transitioned to SHUTDOWN immediately -- which stops
    # the loop synchronously -- so none of them had been given a chance to run.
    # Sockets, executors and routing services were never actually shut down;
    # the 5-second os._exit watchdog was doing the real work, which made
    # "graceful shutdown" a hard kill with extra steps.
    #
    # Now: one loop, resolved from the Initializer that owns it; the right
    # scheduling call for the calling thread; and every handle recorded so it
    # can be drained before the loop goes away.

    def _shutdown_loop(self):
        """The application's event loop, or None if there isn't a usable one."""
        loop = None
        try:
            from FMOFP.core.initializer import get_initializer
            loop = get_initializer().get_loop()
        except Exception:
            loop = None
        if loop is None:
            try:
                loop = asyncio.get_event_loop_policy().get_event_loop()
            except Exception:
                return None
        if loop.is_closed():
            return None
        return loop

    def _schedule_stop(self, component, label):
        """Run a coroutine stop() and record a handle for the drain below.

        Returns True if the stop was scheduled (or run), False if there was no
        loop to run it on -- in which case the caller has already logged.
        """
        loop = self._shutdown_loop()
        if loop is None:
            logger.warning(f"No usable event loop to stop {label} on; skipping")
            return False

        coro = component.stop()
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None

        if running is loop:
            # On the loop thread: a task, drained by await_stop_tasks().
            handle = loop.create_task(coro)
            handle.add_done_callback(
                lambda t, _l=label: self._log_fire_and_forget_task_exception(t, f"{_l}.stop()")
            )
        else:
            # Off the loop thread (the ERROR path runs on "Main_Loop"):
            # submit across threads and block on it in wait_for_stop_tasks().
            handle = asyncio.run_coroutine_threadsafe(coro, loop)
        self._stop_waiters.append((label, handle))
        return True

    def wait_for_stop_tasks(self, timeout=5.0):
        """Block until the scheduled stops finish. NOT for the loop thread."""
        import concurrent.futures
        deadline = time.time() + timeout
        for label, handle in list(self._stop_waiters):
            if not isinstance(handle, concurrent.futures.Future):
                continue
            remaining = max(0.0, deadline - time.time())
            try:
                handle.result(timeout=remaining)
            except concurrent.futures.TimeoutError:
                logger.warning(f"Stop of {label} did not complete within the shutdown budget")
            except Exception as e:
                logger.error(f"Error stopping {label}: {e}")
        self._stop_waiters = []

    async def await_stop_tasks(self, timeout=5.0):
        """Await the scheduled stops. For callers running ON the loop."""
        tasks = [h for _, h in self._stop_waiters if isinstance(h, asyncio.Task)]
        self._stop_waiters = []
        if not tasks:
            return
        done, pending = await asyncio.wait(tasks, timeout=timeout)
        for task in pending:
            logger.warning("A component stop did not complete within the shutdown budget; cancelling")
            task.cancel()

    def stop(self):
        logger.info(f"Stopping Flight Management Operating Flight Program. Thread ID: {threading.get_ident()}")
        self.running = False

        # Stop display timer
        if self.display_timer and self.display_timer.isActive():
            self.display_timer.stop()

        self.stop_system()

        logger.info(f"Flight Management Operating Flight Program stopped. Thread ID: {threading.get_ident()}")

    def stop_system(self):
        """Stop all system components with proper async handling."""
        logger.info("Stopping system components")
        self.state_manager.set_state(SystemState.SHUTTING_DOWN)

        try:
            # Stop the BC/RT 1553B bus listener threads ("BC Listener",
            # "BC Listening", "RT Listener", "RT Listening"). Nothing in
            # this method called Bus_Controller.stop_listener() or
            # Remote_Terminal.stop_listener() before -- confirmed live
            # neither had any caller anywhere in the codebase, so those
            # four threads never received a stop signal and could only be
            # abandoned (not cleanly joined) by thread_manager's bounded
            # join timeout later in this method. Stopping them here, before
            # that generic sweep runs, lets them exit on their own in the
            # few hundred ms their loops already poll at instead of hitting
            # the timeout (see BC.py / RT.py for what each stop_listener()
            # now actually stops).
            if getattr(self, 'listenerBC', None):
                logger.info("Stopping BC listener threads")
                self.listenerBC.stop_listener()
            if getattr(self, 'listenerRT', None):
                logger.info("Stopping RT listener threads")
                self.listenerRT.stop_listener()
            # See InterfaceDisplayMessageHandler.stop()'s docstring for why
            # this is a separate call from self.components['display_message_
            # handler'].stop() below -- they are two different objects, and
            # this one actually owns the "DisplayMessageProcessor"/
            # "DisplayMessageCleanup" threads.
            if getattr(self, 'interface_display_handler', None):
                logger.info("Stopping interface display message handler")
                self.interface_display_handler.stop()
            # The "UserCLI_Control"/"UserCLI_Input"/"UserCLI_Processing"/
            # "UserCLI_Output" threads (all four run methods on the same
            # get_user_cli() singleton) all loop on `while not
            # self.stop_threads:`, but nothing anywhere in this codebase
            # ever called UserCLI.stop_cli_threads() -- the only thing that
            # sets self.stop_threads = True. Confirmed live: even after
            # making each thread's blocking wait interruptible (see
            # userCLI.py), all four still hung for the full shutdown
            # timeout, because the loop condition itself was never
            # satisfied. self.cli_threads (also touched by
            # stop_cli_threads()) is unused/always empty elsewhere in this
            # class, so this call is a plain, cheap flag-set with no
            # blocking risk of its own.
            user_cli = self.components.get('user_cli')
            if user_cli and hasattr(user_cli, 'stop_cli_threads'):
                logger.info("Stopping UserCLI threads")
                user_cli.stop_cli_threads()

            # Stop FMS and radar system components
            # Stop FMS components first
            flight_management = self.components.get('flightManagementSystem')
            if flight_management:
                logger.info("Stopping flight management system")
                flight_management.stop()
                # Stop FMS messaging components
                if 'fms_messenger' in self.components:
                    logger.info("Stopping FMS messaging components")
                    self.components['fms_messenger'].stop()
                if 'fms_message_handler' in self.components:
                    self.components['fms_message_handler'].stop()

            # Stop radar components
            radar_management = self.components.get('radar_management')
            if radar_management:
                logger.info("Stopping radar management system")
                # Signal the background threads to exit their update loops
                # BEFORE joining them. thread_manager.stop_thread() below does
                # a blocking, timeout-less thread.join() -- if that runs before
                # anything clears RadarManagementSystem._running, the join
                # waits forever for a thread that was never told to stop,
                # hanging the entire shutdown sequence indefinitely (this was
                # reproduced live: shutdown stalls permanently right after
                # logging "Stopping radar management system", the process
                # never reaches SystemState.SHUTDOWN, and the app requires
                # SIGKILL to terminate -- see PLANNING.md).
                if hasattr(radar_management, '_running'):
                    radar_management._running.clear()
                # RadarManagementSystem.initialize() also spins up a
                # dedicated "RadarControlEventLoop" thread running
                # self._loop.run_forever() (used to host RadarMessenger's
                # async work). Nothing ever called self._loop.stop() on
                # shutdown, so run_forever() -- and therefore this thread --
                # never returned under any circumstance. thread_manager's
                # later stop_all_threads() sweep does a blocking, timeout-
                # less thread.join() on every registered thread including
                # this one, so it would hang forever (confirmed live: this
                # was the last and worst of several shutdown-hang causes
                # found and fixed together; see PLANNING.md). Stop it here,
                # the same way Initializer._initiate_shutdown() stops the
                # main loop: call_soon_threadsafe(loop.stop) from outside
                # the loop's own thread.
                radar_loop = getattr(radar_management, '_loop', None)
                if radar_loop and not radar_loop.is_closed():
                    try:
                        radar_loop.call_soon_threadsafe(radar_loop.stop)
                    except RuntimeError:
                        pass
                thread_manager.stop_thread("RadarMain")
                thread_manager.stop_thread("RadarManagement")
                thread_manager.stop_thread("RadarControlEventLoop")
                # Run the rest of RadarManagementSystem.stop() (messenger,
                # cross-radar data fusion, etc.). It's a coroutine (it awaits
                # radar_messenger.stop()), but this method is synchronous, so
                # calling it bare as `radar_management.stop()` -- as this code
                # previously did -- only constructs a coroutine object and
                # immediately discards it without ever running its body
                # (confirmed live: "=== Stopping Radar Management System ==="
                # never appeared in the log). Schedule it as a task instead,
                # matching the same coroutine-handling pattern already used
                # for the other components stopped later in this method.
                if asyncio.iscoroutinefunction(radar_management.stop):
                    self._schedule_stop(radar_management, "radar_management")
                else:
                    radar_management.stop()
                # Stop radar messaging components
                if 'radar_messenger' in self.components:
                    logger.info("Stopping radar messaging components")
                    self.components['radar_messenger'].stop()
                if 'radar_message_handler' in self.components:
                    self.components['radar_message_handler'].stop()

            # Stop display system components
            if self.display_manager:
                logger.info("Stopping display management system")
                # Stop display manager threads first
                thread_manager.stop_thread("DisplayManager")
                # Then stop the display manager itself
                self.display_manager.stop()
                # Stop display messaging components
                if 'display_messenger' in self.components:
                    logger.info("Stopping display messaging components")
                    self.components['display_messenger'].stop()
                if 'display_message_handler' in self.components:
                    self.components['display_message_handler'].stop()

            # ── Phase 2 subsystem stops ───────────────────────────────────────
            for name in ('flight_control_computer', 'engine_control_unit',
                         'nav_service', 'comms_service', 'mission_service'):
                comp = self.components.get(name)
                if comp and hasattr(comp, 'stop'):
                    try:
                        logger.info(f"Stopping {name}")
                        comp.stop()
                    except Exception as e:
                        logger.warning(f"Error stopping {name}: {e}")
            # ── end Phase 2 stops ─────────────────────────────────────────────

            # ── Phase 3 subsystem stops ───────────────────────────────────────
            # 'software_config_manager' intentionally excluded from this list --
            # it has no stop() (no background thread to stop), see
            # get_software_config_manager()'s docstring.
            for name in ('defensive_service', 'flight_surface_service',
                         'gcas', 'performance_monitor', 'power_management_system',
                         'fuel_management_system', 'thrust_management_system',
                         'hydraulic_system_controller', 'airframe_system_manager',
                         'ecs_control', 'flight_data_monitoring',
                         'flight_management_fitness'):
                comp = self.components.get(name)
                if comp and hasattr(comp, 'stop'):
                    try:
                        logger.info(f"Stopping {name}")
                        comp.stop()
                    except Exception as e:
                        logger.warning(f"Error stopping {name}: {e}")
            # ── Phase 5 subsystem stops ──────────────────────────────────────
            for name in ('scenario_engine', 'sensor_service'):
                comp = self.components.get(name)
                if comp and hasattr(comp, 'stop'):
                    try:
                        logger.info(f"Stopping {name}")
                        comp.stop()
                    except Exception as e:
                        logger.warning(f"Error stopping {name}: {e}")
            # ── end Phase 5 stops ─────────────────────────────────────────────

            # ── end Phase 3 stops ─────────────────────────────────────────────

            # Stop display response service
            display_response_service = self.components.get('display_response_service')
            if display_response_service:
                logger.info("Stopping display response service")
                if asyncio.iscoroutinefunction(display_response_service.stop):
                    self._schedule_stop(display_response_service, "display_response_service")
                else:
                    display_response_service.stop()

            # Stop VIL response service
            vil_response_service = self.components.get('vil_response_service')
            if vil_response_service:
                logger.info("Stopping VIL response service")
                if asyncio.iscoroutinefunction(vil_response_service.stop):
                    self._schedule_stop(vil_response_service, "vil_response_service")
                else:
                    vil_response_service.stop()

            # Stop message routing service
            routing_service = self.components.get('message_routing_service')
            if routing_service:
                logger.info("Stopping message routing service")
                if asyncio.iscoroutinefunction(routing_service.stop):
                    self._schedule_stop(routing_service, "routing_service")
                else:
                    routing_service.stop()

            # Stop async message handler after dependent components
            async_handler = self.components.get('async_message_handler')
            if async_handler:
                logger.info("Stopping async message handler")
                if asyncio.iscoroutinefunction(async_handler.stop):
                    self._schedule_stop(async_handler, "async_handler")
                else:
                    async_handler.stop()

            # Stop message queue manager
            queue_manager = self.components.get('message_queue_manager')
            if queue_manager:
                logger.info("Stopping message queue manager")
                queue_manager.stop()

            # Stop event bus last
            event_bus = self.components.get('event_bus')
            if event_bus:
                logger.info("Stopping event bus")
                event_bus.stop()

            # Stop remaining components in reverse order
            for component_name, component in reversed(list(self.components.items())):
                if component_name not in ['async_message_handler', 'event_bus']:
                    try:
                        if hasattr(component, 'stop'):
                            if asyncio.iscoroutinefunction(component.stop):
                                self._schedule_stop(component, component_name)
                            else:
                                component.stop()
                        logger.info(f"Stopped {component_name}")
                    except Exception as e:
                        logger.error(f"Error stopping {component_name}: {str(e)}")

            # Clean up any remaining threads
            thread_manager.stop_all_threads()

            # NOTE (production readiness re-analysis, August 2026): DatabaseManager
            # (storage/DBM.py) has a well-implemented shutdown() that closes every
            # system's pooled SQLite connections and shuts down its internal
            # ThreadPoolExecutor, but nothing anywhere in the codebase ever called
            # it -- confirmed via a repo-wide search. Every subsystem above has now
            # stopped writing (this line runs after all of Phase 2/3/5's stop()
            # calls and the reversed-order remaining-components loop), so this is a
            # safe point to close the shared DatabaseManager singleton's connections
            # cleanly instead of leaving them for the OS to reclaim at process exit.
            # Non-fatal if it errors, matching every other shutdown step here.
            try:
                logger.info("Shutting down DatabaseManager")
                from FMOFP.storage.DBM import DatabaseManager
                DatabaseManager('FMOFP/dbConfig.xml').shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down DatabaseManager: {e}")

            # BLOCKER B3: drain the coroutine stops scheduled above BEFORE the
            # SHUTDOWN transition, because that transition synchronously stops
            # the event loop -- so anything still pending on it simply never
            # runs. Off the loop thread (the ERROR path, which runs on
            # "Main_Loop") the handles are cross-thread futures and can be
            # waited on here with a bounded budget. On the loop thread they are
            # asyncio Tasks that this synchronous method cannot await, so
            # Initializer._initiate_shutdown drains them just before it stops
            # the loop instead.
            try:
                asyncio.get_running_loop()
                on_loop_thread = True
            except RuntimeError:
                on_loop_thread = False
            if not on_loop_thread:
                self.wait_for_stop_tasks(timeout=5.0)

            self.state_manager.set_state(SystemState.SHUTDOWN)
            logger.info("All system components stopped")
            self.shutdown_event.set()
            self.system_started = False
            self._initialization_complete = False

        except Exception as e:
            logger.error(f"Error during system shutdown: {str(e)}")
            raise

    def enable_user_cli(self):
        user_cli = self.get_component('user_cli')
        if user_cli:
            user_cli.enable_cli()
            logger.info("UserCLI enabled")
        else:
            logger.error("UserCLI component not found")


    def _record_startup_failure(self, label: str, exc: Exception) -> None:
        """Log a subsystem that failed to start, and remember it if it matters.

        Replaces a bare `logger.error(f"{label} startup failed (non-fatal)")`
        in twenty places. "Non-fatal" was true of the boot sequence and false
        of the aircraft: the flight control computer could fail to start and
        the system would still report ready and transition to NORMAL.

        No health interface is involved. A subsystem that failed to start is
        not registered as a component, so the health sweep never sees it --
        and 33 of 51 components expose no health interface anyway, every
        safety-critical one among them. This uses what the except block
        already knows, at the moment it knows it.
        """
        self._failed_subsystem_starts.add(label)
        if label in SAFETY_CRITICAL_SUBSYSTEMS:
            logger.error(
                f"{label} startup FAILED: {exc}. This is a safety-critical "
                f"subsystem, so the system will not report ready."
            )
        else:
            logger.error(
                f"{label} startup failed (non-fatal, capability degraded): {exc}"
            )

    def failed_safety_critical_starts(self) -> list:
        """Safety-critical subsystems that did not start, worst-case first."""
        return sorted(self._failed_subsystem_starts & SAFETY_CRITICAL_SUBSYSTEMS)

    def is_system_ready(self):
        """Report whether the system can actually do useful work (story C5.1).

        Readiness is what Main.py's 100 ms QTimer polls before moving the system
        to NORMAL, so it is the signal any operator, orchestrator or health
        endpoint would trust. It previously consulted only two things:

          * the state-machine value, and
          * `user_cli.is_cli_ready()`.

        Neither said anything about whether the system worked. The CLI clause was
        weaker still than it looked: `is_cli_ready()` ends in
        `all(t.is_alive() for t in self.cli_threads)`, and `cli_threads` is
        assigned `[]` at userCLI.py:98 and :822 and appended to nowhere, so that
        conjunct is `all([])` -- always True. Whether the CLI's threads are alive
        has never been able to fail readiness.

        The observable consequence, reproduced live: a second instance whose bus
        listeners both failed to bind (port conflict), with the transport dead
        and errors accruing at ~2/s indefinitely, still satisfied this function
        and logged "System is in normal operation".

        Readiness now additionally requires that every component in
        CRITICAL_COMPONENTS has been affirmatively observed HEALTHY. Note the
        asymmetry with the health sweep's logging: UNKNOWN blocks readiness.
        "We have not established that the bus works" is not a basis for
        declaring the system operational, and during the boot window -- before
        the BC listener thread has constructed its listener -- UNKNOWN is
        exactly the honest answer. It resolves on its own within a sweep or two.

        The CLI check is deliberately dropped rather than repaired: it is not a
        statement about the simulation's ability to function, and a debug console
        should not gate operational readiness.
        """
        state_check = self.state_manager.get_state() in [SystemState.RUNNING, SystemState.NORMAL]
        if not state_check:
            return False

        # A safety-critical subsystem that failed to start is not coming back
        # on its own, and is not visible to the health sweep either, because a
        # failed start never reaches its registration line. Checked first: it
        # is a cheap, certain answer, and it does not need a sweep.
        missing = self.failed_safety_critical_starts()
        if missing:
            reason = "safety-critical subsystems did not start: " + ", ".join(missing)
            now = time.time()
            if (reason != self._last_readiness_reason
                    or now - self._last_readiness_log > 5.0):
                logger.error(f"System not ready: {reason}")
                self._last_readiness_reason = reason
                self._last_readiness_log = now
            return False

        # The health monitor sweeps every `health_check_interval` seconds, but it
        # starts late in the boot sequence and readiness is polled every 100 ms.
        # Sweep inline when there is no result yet or the last one has aged out,
        # so readiness is never decided on stale or absent data.
        if (not self.health.has_swept()
                or (time.time() - self.health.swept_at) > self.readiness_probe_max_age):
            try:
                self.check_component_health()
            except Exception as e:
                logger.error(f"Health sweep during readiness check failed: {e}")
                return False

        snapshot = {r.name: r for r in self.health.snapshot()}
        not_ready = []
        for name in sorted(CRITICAL_COMPONENTS):
            report = snapshot.get(name)
            if report is None:
                not_ready.append(f"{name}: not registered")
            elif not report.is_healthy:
                not_ready.append(f"{name}: {report.state.value} ({report.reason})")

        if not_ready:
            reason = '; '.join(not_ready)
            now = time.time()
            if (reason != self._last_readiness_reason
                    or (now - self._last_readiness_log) >= _READINESS_LOG_INTERVAL):
                logger.info(f"System not ready -- critical components: {reason}")
                self._last_readiness_reason = reason
                self._last_readiness_log = now
            return False

        if self._last_readiness_reason is not None:
            logger.info("System ready -- all critical components healthy")
            self._last_readiness_reason = None

        return True

    def wait_for_shutdown(self, timeout=None):
        """Block until the SHUTDOWN transition fires. True if it did.

        shutdown_event is set by stop() immediately after
        set_state(SystemState.SHUTDOWN). Every path that reaches stop() and
        raises before that line leaves the event unset forever, so callers
        must pass a timeout -- an unbounded wait here hangs the process (B1b).
        """
        return self.shutdown_event.wait(timeout)

system_manager = SystemManager()

def get_system_manager():
    return system_manager
