"""
System startup entry point
"""
import sys
import os

# ── Path bootstrap ────────────────────────────────────────────────────────────
# Put the distribution root on sys.path so `import FMOFP...` resolves however
# this file is launched (debugger, CLI, venv).
#
# Story C12: B20SS/FMOFP used to be added as a second entry so bare imports
# (Utils.*, storage.*, core.*) would also resolve. That is exactly what allowed
# one source file to load under two dotted names as two separate module and
# class objects. All imports are FMOFP-prefixed now; the shim is deleted.
_HERE = os.path.dirname(os.path.abspath(__file__))          # …/B20SS/FMOFP
_ROOT = os.path.dirname(_HERE)                               # …/B20SS
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# ── Working-directory bootstrap ───────────────────────────────────────────────
# All relative config paths in this codebase (e.g. 'FMOFP/dbConfig.xml',
# 'FMOFP/local_messaging/...') are written relative to B20SS/.
# Normalise CWD to B20SS/ regardless of where the user launched from,
# so those paths resolve correctly whether SystemStart.py is run as:
#   cd B20SS      && py FMOFP/SystemStart.py   (CWD already correct)
#   cd B20SS/FMOFP && py SystemStart.py        (CWD was wrong — fixed here)
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
from FMOFP.core.initializer import get_initializer

logger = get_logger()

async def main():
    try:
        # Initialize the system
        initializer = get_initializer()
        initializer.initialize()  # This must complete before proceeding

        # Verify initialization
        app = initializer.get_app()
        loop = initializer.get_loop()
        if not app or not loop:
            raise RuntimeError("Failed to initialize application or event loop")

        logger.info("Starting Flight Management Operating Flight Program")

        # Import here to avoid circular imports
        from Main import start_fmofp

        # Start FMOFP
        await start_fmofp()

    except Exception as e:
        logger.critical(f"Critical error in SystemStart: {str(e)}")

        # Try to get system manager and set error state
        try:
            system_manager = get_system_manager()
            system_manager.state_manager.set_state(SystemState.ERROR)
        except Exception as cleanup_error:
            logger.critical(f"Error during cleanup: {str(cleanup_error)}")

        sys.exit(1)

if __name__ == "__main__":
    # Import threading here to avoid circular imports
    import threading

    try:
        # Check if we're in the main thread
        if threading.current_thread() is not threading.main_thread():
            logger.critical("SystemStart.py must be run in the main thread")
            sys.exit(1)

        # Initialize system first
        initializer = get_initializer()
        initializer.initialize()

        # Get Qt application and event loop
        app = initializer.get_app()
        loop = initializer.get_loop()

        if not app or not loop:
            raise RuntimeError("Failed to initialize application or event loop")

        try:
            # Run the main coroutine
            loop.run_until_complete(main())

            # Start Qt event loop
            with loop:  # Ensure proper cleanup of event loop
                loop.run_forever()

        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
        finally:
            # Clean up
            initializer.cleanup()

    except Exception as e:
        logger.critical(f"Fatal error: {str(e)}")
        sys.exit(1)
