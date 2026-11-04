"""
Initializer singleton for managing shared system resources
"""
import logging
import os
import sys
import threading
import asyncio
from PyQt6.QtWidgets import QApplication
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Utils.common.system_state_manager import SystemStateManager
from FMOFP.Utils.common.system_states import SystemState
from qasync import QEventLoop

logger = get_logger()

class SingletonMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class Initializer(metaclass=SingletonMeta):
    def __init__(self):
        self.initialized = False
        self._app = None
        self._loop = None
        self._state_manager = None
        self._shutdown_started = False
        # Process exit status; 1 once anything has failed (B1).
        self._exit_status = 0

    def initialize(self):
        if not self.initialized:
            try:
                # Log system information
                logger.info(f"Current working directory: {os.getcwd()}")
                logger.info(f"Main thread ID: {threading.get_ident()}")
                
                # Initialize Qt application (singleton)
                if not QApplication.instance():
                    self._app = QApplication(sys.argv)
                else:
                    self._app = QApplication.instance()
                
                # Create event loop that works with Qt (singleton)
                if not self._loop:
                    self._loop = QEventLoop(self._app)
                    asyncio.set_event_loop(self._loop)
                
                # Initialize SystemStateManager
                self._state_manager = SystemStateManager()
                self._state_manager.initialize()
                
                # Set up state transition handler
                self._state_manager.add_state_change_handler(self._handle_state_change)
                
                self.initialized = True
                logger.info("System initialization completed successfully.")
            except Exception as e:
                logger.error(f"Error during system initialization: {str(e)}")
                raise

    def _handle_state_change(self, old_state, new_state):
        """Handle system state transitions"""
        logger.info(f"System state transition: {old_state} -> {new_state}")
        
        if new_state == SystemState.SHUTDOWN:
            self._initiate_shutdown()
        elif new_state == SystemState.ERROR:
            self._handle_error_state()

    def _initiate_shutdown(self, failed=False):
        """Initiate graceful shutdown sequence

        NOTE: This is invoked synchronously from _handle_state_change(), which
        is itself fired synchronously by state_manager.set_state(SHUTDOWN).
        That call happens from *inside* system_manager.stop_system(), which is
        running as part of the still-executing Flight_Management_Operating_Flight_Program.shutdown()
        asyncio task. That means this method's call stack is still nested inside
        an active task on the running event loop.

        Previously this method called self.cleanup() synchronously here, and
        cleanup() calls self._loop.run_until_complete(...) and self._loop.close()
        on that SAME loop. Calling run_until_complete() reentrantly while
        another task on the loop hasn't finished executing is illegal in
        asyncio and raised:
            RuntimeError: Cannot enter into task <...MessageRoutingService.stop()...>
            while another task <...Flight_Management_Operating_Flight_Program.shutdown()...>
            is being executed.
        followed by "Error during cleanup: Event loop stopped before Future
        completed." The loop was then left in an inconsistent state where
        run_forever() never actually returned, so the process kept running
        (Qt timers/animations kept firing) long after "All system components
        stopped" was logged.

        The fix: only schedule loop.stop() here (via call_soon_threadsafe,
        which is safe to call from any context). Do NOT call cleanup() here.
        Once the current task chain finishes unwinding and control returns to
        run_forever(), the scheduled stop() will actually take effect and
        run_forever() will return. SystemStart.py's own `finally:
        initializer.cleanup()` block then runs cleanup() safely, outside of
        any actively-executing task, where run_until_complete()/close() are
        legal.

        Watchdog: even with the fix above, live testing showed that
        confirmed-successful calls to loop.stop() (self._loop.is_running()
        correctly reports False immediately afterward, and Qt's
        QApplication.exit() is confirmed invoked by qasync's QEventLoop.stop())
        do not always cause the underlying QApplication.exec() call inside
        qasync's run_forever() to actually return - some UI widgets run their
        own free-running QTimers (e.g. weather radar animation, ~60fps) that
        keep Qt's native event loop pumping. By this point in shutdown, every
        thread and resource this codebase owns has already been stopped
        cleanly (verified via live SIGTERM testing: thread_manager reports
        zero "did not stop within timeout" errors and all system components
        report stopped within ~4 seconds of receiving the signal) - the only
        remaining thing keeping the process alive is Qt's own event pump. A
        bounded watchdog is the standard, safe way to guarantee the process
        actually exits: if we haven't returned from run_forever() a few
        seconds after every real component has already finished stopping,
        force-exit. This mirrors how orchestrators (systemd/k8s/docker) treat
        graceful shutdown themselves: try to exit cleanly, then hard-kill
        after a bounded grace period.
        """
        # B1: record a failure even if shutdown is already under way -- an
        # ERROR arriving during a shutdown still means this process failed.
        if failed:
            self._exit_status = 1

        if self._shutdown_started:
            return

        self._shutdown_started = True
        logger.info("Initiating graceful shutdown sequence")
        
        try:
            # Stop the event loop if it's running. This is scheduled via
            # call_soon_threadsafe so it only takes effect once the current
            # task chain finishes and control returns to run_forever() -
            # cleanup() (run_until_complete/close) must NOT be called here,
            # see docstring above.
            loop_was_running = bool(self._loop and self._loop.is_running())
            if loop_was_running:
                # BLOCKER B3: this used to schedule loop.stop() directly, which
                # halted the loop with every component's stop() coroutine still
                # pending on it -- sockets, executors and routing services were
                # never actually shut down, and the watchdog below did the real
                # work. Drain those tasks first, with a bounded budget, then
                # stop.
                self._loop.call_soon_threadsafe(self._drain_then_stop)

            # Safety-net watchdog: force the process to exit if it hasn't
            # already done so shortly after shutdown was initiated. All
            # component/thread cleanup happens well within this window (see
            # docstring above); this only ever fires if something below the
            # asyncio layer (e.g. a Qt native event loop / timer) is keeping
            # the process alive after every real resource has been released.
            def _watchdog_force_exit():
                logger.warning(
                    "Shutdown watchdog: process did not exit naturally within "
                    "the grace period after all components stopped; forcing exit "
                    f"with status {self._exit_status}."
                )
                # BLOCKER B1: os._exit skips every atexit hook and every
                # logging handler flush, so the lines explaining why the
                # process is dying were routinely lost. Flush first.
                try:
                    logging.shutdown()
                except Exception:
                    pass
                # BLOCKER B1: this was a hardcoded os._exit(0). _handle_error_state
                # routes fatal errors through here, so a crash terminated with
                # SUCCESS status -- systemd, k8s, docker and any supervising
                # script read that as a clean stop and neither restarted nor
                # alerted.
                os._exit(self._exit_status)

            # Only arm the watchdog if there was actually a running loop to
            # stop. Its entire job is to force the process out when Qt's native
            # event pump keeps running after the asyncio loop was asked to
            # stop; with no loop running there is nothing for it to rescue, and
            # arming it anyway means any code path that initiates a shutdown
            # without a loop -- a test, a tool, an embedded use -- gets its
            # process killed five seconds later.
            if loop_was_running:
                watchdog = threading.Timer(5.0, _watchdog_force_exit)
                watchdog.daemon = True
                watchdog.start()
            else:
                logger.info(
                    "No running event loop to stop; shutdown watchdog not armed")
        except Exception as e:
            logger.error(f"Error during shutdown: {str(e)}")

    def _drain_then_stop(self):
        """Give the scheduled component stops a bounded chance, then stop (B3)."""
        async def _drain():
            try:
                from FMOFP.core.system_manager import get_system_manager
                await get_system_manager().await_stop_tasks(timeout=5.0)
            except Exception as e:
                logger.error(f"Error draining component stop tasks: {e}")
            finally:
                logger.info("Component stops drained; stopping the event loop")
                self._loop.stop()

        try:
            self._loop.create_task(_drain())
        except Exception as e:
            # If the drain cannot even be scheduled, stopping the loop is still
            # strictly better than not stopping it.
            logger.error(f"Could not schedule shutdown drain: {e}")
            self._loop.stop()

    def request_stop(self, failed=False):
        """Stop the application loop, even if the system never fully started.

        BLOCKER B1: SystemState.SHUTDOWN was the only thing that ever reached
        _initiate_shutdown, and that state is only set at the very end of
        SystemManager.stop_system(). Every early-failure path in Main.py
        therefore left the loop running forever: the process sat in
        run_forever() with two 60 FPS QTimers still firing, never exiting and
        never returning a status, so a supervisor saw a healthy long-running
        process. Main.shutdown() now calls this directly.
        """
        self._initiate_shutdown(failed=failed)

    def get_exit_status(self):
        """The status the process should exit with (0 unless something failed)."""
        return self._exit_status

    def _handle_error_state(self):
        """Handle transition to error state"""
        logger.error("System entered ERROR state")
        self._initiate_shutdown(failed=True)

    def get_app(self):
        return self._app

    def get_loop(self):
        return self._loop

    def get_state_manager(self):
        return self._state_manager

    def cleanup(self):
        """Clean up resources"""
        if not self._shutdown_started:
            self._shutdown_started = True
            
        try:
            logger.info("Starting cleanup sequence")
            
            # Stop the event loop
            if self._loop and self._loop.is_running():
                self._loop.stop()
                
            # Close the event loop
            if self._loop and not self._loop.is_closed():
                # Run any pending callbacks
                pending = asyncio.all_tasks(self._loop)
                if pending:
                    logger.info(f"Cleaning up {len(pending)} pending tasks")
                    self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                    
                self._loop.close()
                logger.info("Event loop closed")
                
            # Clean up Qt application
            if self._app:
                self._app.quit()
                logger.info("Qt application quit")
                
            logger.info("Cleanup completed successfully")
        except Exception as e:
            logger.error(f"Error during cleanup: {str(e)}")

# Singleton instance
_initializer = None

def get_initializer():
    global _initializer
    if _initializer is None:
        _initializer = Initializer()
    return _initializer
