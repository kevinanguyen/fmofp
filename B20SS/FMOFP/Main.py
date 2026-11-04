"""
Main Entry Point for Flight Management Operating Flight Program (FMOFP)

This module initializes and starts the FMOFP system, including all subsystems
and components. It sets up the event-driven communication system and manages
the overall program lifecycle.
"""

import os
import sys
import threading
import signal
import asyncio

# ── Path/working-directory bootstrap (same as SystemStart.py) ────────────────
# `python FMOFP/Main.py` is the launch command documented by install.py and
# the README, but running this file directly puts only FMOFP/ on sys.path,
# so the `import FMOFP...` lines below raised ModuleNotFoundError — the
# documented entry point simply did not work (found August 2026 live-boot
# re-verification; SystemStart.py already had this bootstrap, Main.py never
# did). Put the distribution root on sys.path so `import FMOFP...` resolves.
#
# Story C12: B20SS/FMOFP used to be added as a SECOND entry, which let the same
# file be imported under two dotted names and produce duplicate module and class
# objects. Every import is FMOFP-prefixed now, so one entry is enough and the
# dual-path shim that compensated for it is gone.
_HERE = os.path.dirname(os.path.abspath(__file__))          # …/B20SS/FMOFP
_ROOT = os.path.dirname(_HERE)                               # …/B20SS
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
# Story C11b: the os.chdir(_ROOT) that used to sit here is gone. It existed
# because config and database paths were written CWD-relative ('FMOFP/dbConfig.xml',
# os.path.join('FMOFP','storage','databases',...)), so the process had to relocate
# itself before anything could be read. Those are now resolved against the package
# (Utils/common/fetching.resolve_resource / resolve_data_dir), so the working
# directory no longer changes program behaviour.
#
# chdir() is process-global: it changed the CWD for anything embedding this code,
# and once the project became pip-installable it pointed the process at
# site-packages, where an installed run proceeded to create 11 SQLite databases
# and a log file. Relocating that data out of the package entirely is story C13.
# ─────────────────────────────────────────────────────────────────────────────

from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Utils.common.system_states import SystemState
from FMOFP.core.system_manager import get_system_manager
from FMOFP.core.event_driven_communication import get_event_bus
from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import get_radar_message_handler
from PyQt6.QtCore import QTimer
from FMOFP.core.initializer import get_initializer

# Initialize SysLogger
logger = get_logger()
logger.debug("Main.py execution started - This is a test debug message")

# How long shutdown() waits for the SHUTDOWN transition before giving up and
# finishing teardown anyway. Tests lower this; see test_blocker_lifecycle.
_SHUTDOWN_WAIT_TIMEOUT = 5.0


class Flight_Management_Operating_Flight_Program:
    def __init__(self):
        logger.info(f"Flight_Management_Operating_Flight_Program __init__ called. Thread ID: {threading.get_ident()}")
        self.system_manager = get_system_manager()
        self.event_bus = get_event_bus()
        self.radar_message_handler = get_radar_message_handler()
        self._initialized = False
        self._started = False
        self.running = False
        # True once shutdown() has begun, so a second signal does not
        # re-run the stop sequence (B1).
        self._shutdown_begun = False
        self.shutdown_event = threading.Event()
        self._keep_running = True
        self._check_timer = None
        
        # Get existing instances from Initializer
        self._initializer = get_initializer()
        self._app = self._initializer.get_app()
        self._loop = self._initializer.get_loop()

    async def initialize(self):
        if self._initialized:
            # Silenced logging
            return

        logger.info(f"Initializing Flight Management Operating Flight Program. Thread ID: {threading.get_ident()}")
        logger.info("Initializing system components")
        try:
            # Initialize system asynchronously
            await self.system_manager.initialize_system()
            self._initialized = True
            logger.info(f"Flight Management Operating Flight Program initialized. Thread ID: {threading.get_ident()}")
        except Exception as e:
            logger.error(f"Error during initialization: {str(e)}. Thread ID: {threading.get_ident()}")
            await self.shutdown(failed=True)

    def start(self):
        if not self._initialized:
            logger.error("Attempt to start uninitialized system. Please call initialize() first.")
            return

        if self._started:
            logger.warning("Flight Management Operating Flight Program already started. Ignoring repeated start attempt.")
            return

        logger.info(f"Flight_Management_Operating_Flight_Program start method called. Thread ID: {threading.get_ident()}")
        
        try:
            # Ensure we're in the main thread
            if threading.current_thread() is not threading.main_thread():
                raise RuntimeError("System must be started from the main thread")
            
            # Start the system manager
            self.system_manager.start_FM_system()
            
            # Set up check timer in main thread
            self._check_timer = QTimer()
            self._check_timer.setInterval(100)  # 100ms interval
            self._check_timer.timeout.connect(self._check_system_ready_slot)
            self._check_timer.start()
            
            # System is now running
            self.running = True
            self._started = True
            
        except Exception as e:
            logger.error(f"Error during system start: {str(e)}. Thread ID: {threading.get_ident()}")
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(self.shutdown(failed=True)))

    def _check_system_ready_slot(self):
        """Qt slot for checking system readiness"""
        try:
            if self.system_manager.is_system_ready():
                # Schedule state change in event loop thread
                self._loop.call_soon_threadsafe(
                    lambda: self.system_manager.state_manager.set_state(SystemState.NORMAL)
                )
                logger.info(f"System is in normal operation. Thread ID: {threading.get_ident()}")
                self._check_timer.stop()
        except Exception as e:
            logger.error(f"Error checking system readiness: {str(e)}")
            self._loop.call_soon_threadsafe(lambda: asyncio.create_task(self.shutdown(failed=True)))

    async def shutdown(self, failed=False):
        # BLOCKER B1: this used to open with
        #     if not self._started: ... return
        # and every early-failure path routes here BEFORE _started is ever set
        # -- initialize()'s handler, start()'s handler, and
        # _check_system_ready_slot's handler. All three hit that guard and
        # returned immediately, so nothing tore anything down and, critically,
        # nothing ever stopped the event loop: SystemState.SHUTDOWN is the only
        # thing that reaches Initializer._initiate_shutdown, and that state is
        # set at the very end of SystemManager.stop_system(), which was never
        # reached. The process sat in run_forever() forever with two 60 FPS
        # QTimers still firing -- no exit, no status, and a supervisor seeing a
        # healthy long-running process. It needed SIGKILL.
        #
        # The guard that IS wanted here is re-entrancy, not "did we start":
        # two signals arriving close together each schedule their own
        # shutdown() task (see below), and the stop sequence must run once.
        if self._shutdown_begun:
            logger.debug("Shutdown already in progress or complete; ignoring.")
            return
        self._shutdown_begun = True

        started = self._started
        if not started:
            logger.warning(
                "Shutting down a system that never finished starting; "
                "tearing down whatever was built.")

        # Flip _started to False immediately, before any of the real
        # shutdown work below (not at the end, as this previously did).
        # shutdown() is scheduled via loop.call_soon_threadsafe(lambda:
        # asyncio.create_task(fmofp.shutdown())) from the SIGINT/SIGTERM
        # signal handler in start_fmofp() below -- two signals arriving
        # close together (e.g. a duplicate SIGTERM, or SIGINT followed by
        # SIGTERM, both plausible from process managers/operators) each
        # schedule their own shutdown() task. The guard above only
        # protects against a *second* signal arriving after the *first*
        # shutdown() has fully finished; the previous code left
        # `self._started = False` until after `self.system_manager.stop()`,
        # `await asyncio.sleep(0.1)`, and `self.system_manager.
        # wait_for_shutdown()` had all already run, so a second shutdown()
        # task starting during that window would still see `_started ==
        # True`, pass the guard, and call `self.system_manager.stop()` a
        # second time on top of the first (live-reproduced with a stand-in
        # mirroring this exact guard/await structure: two overlapping
        # shutdown() calls both passed the guard and both invoked the
        # stop-work, confirmed via a call counter). Setting the flag here,
        # before the first await point, closes that window: a second
        # shutdown() task now sees `_started == False` immediately and
        # returns via the guard instead of re-running the stop sequence.
        self._started = False

        logger.info(f"Shutting down Flight Management Operating Flight Program. Thread ID: {threading.get_ident()}")
        self.running = False
        self._keep_running = False
        
        try:
            # Stop check timer
            if self._check_timer and self._check_timer.isActive():
                self._check_timer.stop()

            # Stop system components. On a failed start this runs against a
            # half-built system, which is the point -- whatever came up has to
            # come back down -- so a failure here must not prevent the loop
            # from being stopped in the finally block below.
            try:
                self.system_manager.stop()
            except Exception as e:
                logger.error(f"Error stopping system components: {str(e)}")
                failed = True
            try:
                self.event_bus.stop()
            except Exception as e:
                logger.error(f"Error stopping the event bus: {str(e)}")
                failed = True

            # Give components time to clean up
            await asyncio.sleep(0.1)

            # Only meaningful once the system actually started: on a failed
            # start stop_system() may never have reached its SHUTDOWN
            # transition, and this waits on the event that transition sets.
            if started:
                # BLOCKER B1b: this was a bare self.system_manager.
                # wait_for_shutdown() -- threading.Event.wait() with NO
                # timeout -- called from inside the qasync event loop, since
                # shutdown() is a coroutine. The event is only set by stop()'s
                # SHUTDOWN transition, so whenever stop() raised before
                # reaching it (caught just above, failed=True) nothing would
                # ever set it: the process blocked forever, holding the event
                # loop thread, after every test had already passed. Seen as
                # test_fms_live timing out at 420s roughly one run in three,
                # having already printed "63 passed, 0 failed".
                #
                # Bound the wait, and run it off the loop so the loop keeps
                # servicing any component stops still queued on it.
                loop = asyncio.get_running_loop()
                completed = await loop.run_in_executor(
                    None, self.system_manager.wait_for_shutdown,
                    _SHUTDOWN_WAIT_TIMEOUT)
                if not completed:
                    logger.warning(
                        "Shutdown transition did not complete within "
                        f"{_SHUTDOWN_WAIT_TIMEOUT}s; continuing teardown")
                    failed = True
            self.shutdown_event.set()
            # self._started was already set to False at the top of this
            # method (see comment there) -- not repeated here.
            self._initialized = False

            logger.info(f"Flight Management Operating Flight Program shut down. Thread ID: {threading.get_ident()}")
        except Exception as e:
            logger.error(f"Error during shutdown: {str(e)}")
            failed = True
            raise
        finally:
            # BLOCKER B1: the loop must stop on every path out of here,
            # including the ones where stop_system() never ran and therefore
            # never transitioned to SHUTDOWN. request_stop() is idempotent.
            self._initializer.request_stop(failed=failed or not started)

    def wait_for_shutdown(self):
        self.shutdown_event.wait()

def global_exception_handler(exctype, value, tb):
    logger.critical(f"Uncaught exception: {exctype.__name__}: {value}")

async def start_fmofp():
    sys.excepthook = global_exception_handler

    # Get existing instances from Initializer
    initializer = get_initializer()
    app = initializer.get_app()
    loop = initializer.get_loop()
    
    if not app or not loop:
        raise RuntimeError("Application or event loop not properly initialized")

    fmofp = Flight_Management_Operating_Flight_Program()
    
    def signal_handler(sig, frame):
        logger.info(f"Received signal {sig} to shutdown")
        loop.call_soon_threadsafe(lambda: asyncio.create_task(fmofp.shutdown()))

    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Initialize and start the system
        await fmofp.initialize()
        fmofp.start()
        
        # Wait for system to be ready
        while fmofp.running and not fmofp.system_manager.is_system_ready():
            await asyncio.sleep(0.1)
            
        # System is ready, let it run
        while fmofp.running:
            await asyncio.sleep(0.1)
            
    except Exception as e:
        logger.error(f"Error in FMOFP: {str(e)}")
        await fmofp.shutdown(failed=True)
    finally:
        if fmofp.running:
            await fmofp.shutdown()

def main() -> int:
    """Console entry point for FMOFP (story C11a).

    Extracted from what used to be an inline `if __name__ == "__main__":` block
    so the application has a real callable entry point. `pyproject.toml` exposes
    it as the `fmofp` command; running `python FMOFP/Main.py` still works and
    goes through the same path, so the documented invocation is unchanged.

    Two defects in the original block are fixed here:

      * `initializer` was assigned inside the `try`, but `finally` called
        `initializer.cleanup()` unconditionally. If `get_initializer()` or
        `initialize()` raised -- the boot-deadlock case this project has hit
        more than once -- the `finally` raised NameError on top of the real
        error, hiding it.

      * The process always exited 0, including after a fatal error, so nothing
        supervising it could tell a clean shutdown from a crash. It now returns
        a real exit status.

    Returns:
        0 on clean shutdown or Ctrl-C, 1 on an unhandled error.
    """
    logger.info(f"Main thread ID: {threading.get_ident()}")

    initializer = None
    status = 0
    try:
        # Initialize the system through Initializer
        initializer = get_initializer()
        initializer.initialize()

        # Get the event loop from initializer
        loop = initializer.get_loop()

        # Run the main coroutine and Qt event loop together
        loop.create_task(start_fmofp())
        loop.run_forever()

    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Error in main: {str(e)}")
        status = 1
    finally:
        # Let the initializer handle cleanup. Guarded: see docstring.
        if initializer is not None:
            initializer.cleanup()
            # BLOCKER B1: a failure recorded during shutdown -- an ERROR state
            # transition, a component that could not be stopped, a start that
            # never completed -- has to reach the exit status too, not only the
            # exceptions main() happens to catch itself.
            if initializer.get_exit_status() != 0:
                status = initializer.get_exit_status()
        else:
            logger.error("Initializer was never constructed; nothing to clean up")

    return status


if __name__ == "__main__":
    sys.exit(main())
