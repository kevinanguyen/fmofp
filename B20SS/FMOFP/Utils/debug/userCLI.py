"""
User CLI for Flight Management Operating Flight Program
"""

import asyncio
import os
import sys
import select
import time
import queue
import threading
import xml.etree.ElementTree as ET
import traceback
import click
import FMOFP.Utils.common.fetching as fetching
from FMOFP.Utils.common.paths import get_paths_instance
from FMOFP.MIL_STD_1553B.Messaging import send1553Msg
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Utils.common.system_states import userCLIStates
from FMOFP.Utils.common.system_state_manager import SystemStateManager
from FMOFP.local_messaging.routing.handlers.system_message_handlers.RadarMessageHandler import RadarMessageHandler
from FMOFP.local_messaging.routing.handlers.sync_handler.AsyncMessageHandler import AsyncMessageHandler
from FMOFP.Systems.radarManagement.radar_enums import RadarMode
from FMOFP.Systems.radarManagement.weather.weather_radar import weather_radarMode
from FMOFP.Systems.radarManagement.radarControl import get_radar_management_system
from FMOFP.local_messaging.command_word_map import RADAR_TYPES, COMMAND_REGISTRY
from FMOFP.Utils.common.fetching import resolve_resource

logger = get_logger()


@click.group()
def cli():
    pass

@cli.command()
def test():
    """Run system tests"""
    cli = get_user_cli()
    cli._process_command("test")

class UserCLI:
    _instance = None
    _test_lock = threading.Lock()
    _test_running = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        logger.info(f"UserCLI __init__ called. Thread ID: {threading.get_ident()}")
        # BLOCKER B12: this was `self.paths = paths()`, which built the whole
        # source index -- an os.walk + MD5 + ast.parse of every .py file under
        # the resolved project root -- as part of constructing the CLI. UserCLI
        # is instantiated during the live boot sequence by
        # core/system_manager.py, so every start paid for it, and on an
        # installed package the root resolved to site-packages. Nothing read
        # this attribute: the only consumer, get_import_statement below, is a
        # `pass`. It is now a lazily-built property, so the cost is paid only
        # if the index is genuinely asked for.
        self.command_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.command_processed = threading.Event()
        self.command_received = threading.Event()
        self.command_printed = threading.Event()
        self.sendMsg = send1553Msg()
        self.command_processed.set()  # Initially, no command is being processed
        self.prompt_printed = False
        self.debugging = False
        self.commandInterface = False
        self.stop_threads = False
        self.command = ""
        self.cli_enabled = False
        self.state_manager = SystemStateManager()
        self.prompt_shown = False
        self.cli_threads = []
        # Set when input() hits EOF (e.g. no interactive terminal attached --
        # headless/CI/automated runs). Distinct from cli_enabled, which
        # reflects the commandInterface config setting: this tracks a
        # runtime fact discovered while running, not configuration.
        self._stdin_eof = False
        self.load_config()

        # These will be initialized later when needed
        self._radar_message_handler = None
        self._async_handler = None
        self._radar_handler = None

    @property
    def radar_message_handler(self):
        if not self._radar_message_handler:
            self._radar_message_handler = RadarMessageHandler()
        return self._radar_message_handler

    @property
    def async_handler(self):
        if not self._async_handler:
            self._async_handler = AsyncMessageHandler()
        return self._async_handler

    @property
    def radar_handler(self):
        if not self._radar_handler:
            self._radar_handler = RadarMessageHandler()
        return self._radar_handler

    def initialize(self):
        logger.info(f"UserCLI initialize called. Thread ID: {threading.get_ident()}")
        if self._initialized:

            return

        # Initialize handlers only when needed
        self._radar_message_handler = RadarMessageHandler()
        self._async_handler = AsyncMessageHandler()
        self._radar_handler = RadarMessageHandler()

        self._initialized = True
        logger.info(f"UserCLI initialized. Thread ID: {threading.get_ident()}")

    def start(self):
        logger.info("UserCLI: Starting user interface")
        # Registered as a background thread (name "user_cli") by the
        # generic per-component thread-starting loop, so `except
        # KeyboardInterrupt` here never actually fires -- Python only
        # delivers KeyboardInterrupt to the main thread, not to a
        # background thread like this one. This was previously `while
        # True:` with no way to ever exit at all (not even checking
        # self.stop_threads), so this thread ran forever and could only
        # ever be abandoned, not stopped, on shutdown -- confirmed live:
        # "Thread 'user_cli' did not stop within 2.0s timeout" on every
        # run, now bounded only by thread_manager's defense-in-depth join
        # timeout rather than ever actually exiting cleanly. Checking
        # self.stop_threads (the same flag stop_cli_threads() sets, used
        # by all the other UserCLI_* threads) and sleeping in 1s
        # increments instead of one 10s block fixes that.
        while not self.stop_threads:
            time.sleep(1)
        logger.info("UserCLI: Exiting user interface")

    def send(self):
        logger.info("UserCLI: Initiating send command process")
        radar_options = {
            1: "weather_radar",
            2: "tfr_radar",
            3: "sar_radar",
            4: "targeting_radar",
            5: "aewc_radar"
        }
        radar_num = int(input("Which radar would you like to select? (1-5)? "))
        selected_radar = radar_options.get(radar_num)

        logger.info("  Which command would you like to send?")
        logger.info("  1) Radar System Status Request")
        logger.info("  2) Radar Mode Change Request")
        command_num = int(input("Enter command (1-2)? "))

        if command_num == 1:
            logger.info(f"UserCLI: Sending status request for {selected_radar}")
            try:
                from FMOFP.Systems.radarManagement.radarControl import get_radar_management_system
                rms = get_radar_management_system()
                status = None
                for radar in rms.radars.values():
                    radar_type = type(radar).__name__.lower()
                    if selected_radar.replace('_radar', '') in radar_type:
                        status = radar.get_status()
                        break
                if status:
                    logger.info(f"--- {selected_radar} status ---")
                    for key, val in status.items():
                        logger.info(f"  {key}: {val}")
                else:
                    logger.info(f"No radar found matching '{selected_radar}'")
            except Exception as e:
                logger.error(f"Error retrieving status for {selected_radar}: {str(e)}")
        elif command_num == 2:
            mode = input("Enter mode (STANDBY, SURVEILLANCE, MAPPING): ")
            logger.info(f"UserCLI: Sending mode change request for {selected_radar}: {mode}")
            try:
                self.radar_handler.send_radar_request(selected_radar, "mode_change", mode)
            except Exception as e:
                logger.error(f"Error in send command: {str(e)}")
                logger.info(f"Error: {str(e)}")
        else:
            logger.info("Invalid command number")

    def load_config(self):
        try:
            config_file = os.path.join(fetching.fetch_fmofp_path(), 'startupConfiguration.xml')
            tree = ET.parse(config_file)
            root = tree.getroot()
            logging_config = root.find('logging')
            if logging_config is not None:
                command_interface_elem = logging_config.find('commandInterface')
                debugging_elem = logging_config.find('debugging')
                prompt_printed_elem = logging_config.find('promptPrinted')

                self.cli_enabled = command_interface_elem is not None and command_interface_elem.text.lower() == 'true'
                self.debugging = debugging_elem is not None and debugging_elem.text.lower() == 'true'
                self.prompt_printed = prompt_printed_elem is not None and prompt_printed_elem.text.lower() == 'true'

            command_registry_file = resolve_resource(os.path.join('FMOFP', 'local_messaging', 'messageConfigurations', 'command_registry.xml'))
            command_registry_tree = ET.parse(command_registry_file)
            command_registry_root = command_registry_tree.getroot()
            command_words_config = command_registry_root.find('command_words')
            if command_words_config is not None:
                for command in command_words_config.findall('command'):
                    name = command.get('name')
                    value = command.get('value')
                    COMMAND_REGISTRY[name] = value

            logger.info(f"Configuration loaded successfully from {config_file} and {command_registry_file}")
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            traceback.print_exc()

    def commandLineThreadControl(self):
        logger.info(f"Entering commandLineThreadControl. Thread ID: {threading.get_ident()}")
        try:
            while not self.stop_threads:
                if not self.command_received.is_set():
                    # Bounded wait -- previously a plain wait() with no
                    # timeout, which blocks here indefinitely whenever no
                    # command has arrived yet (e.g. headless/CI runs, or
                    # simply idle time between commands in a real session),
                    # since self.stop_threads is never rechecked while
                    # blocked inside wait(). Confirmed live this made
                    # "UserCLI_Control" hang for the full shutdown timeout
                    # on every run. Timing out just means "go recheck
                    # stop_threads and try again" -- identical behavior to
                    # before whenever the event actually gets set in time.
                    if not self.command_received.wait(timeout=0.5):
                        continue

                if self.command_received.is_set():
                    self.command_received.clear()
                    self.command_processed.set()

                if not self.command_processed.is_set():
                    if not self.command_processed.wait(timeout=0.5):
                        continue

                if self.command_processed.is_set():
                    self.command_processed.clear()
                    self.command_printed.set()

                if self.command_received.is_set() and self.command_processed.is_set():
                    self.command_received.clear()
                    self.command_processed.clear()
                    self.command_printed.set()
        except Exception as error:
            logger.error(f"Exception occurred in commandLineThreadControl. Thread ID: {threading.get_ident()}", exc_info=True)
        finally:
            logger.info(f"Exiting commandLineThreadControl. Thread ID: {threading.get_ident()}")

    def _process_command(self, command):
        """Process a single command."""
        if not command:
            return
        logger.info(f"Processing command '{command}'")
        try:
            if command.startswith("help"):
                self._print_help()

            elif command == "test":
                # Check if test is already running
                if UserCLI._test_running:
                    logger.info("A test is already running. Please wait for it to complete.")
                    self.output_queue.put("\nA test is already running. Please wait for it to complete.")
                    return

                # Set test running flag
                UserCLI._test_running = True

                try:
                    # Clear any existing output
                    while not self.output_queue.empty():
                        self.output_queue.get()

                    # Use asyncio.run() instead of manually managing event loop

                    # Choose which test to run
                    test_options = {
                        "1": "Combined Precipitation & VIL Flow Test",
                        "2": "FMS System Test",
                        "3": "Flight Control System Test",
                        "4": "Predefined Messages Test",
                        "5": "Weather Radar Test (All Modes)",
                        "6": "TFR Radar Test (All Modes)",
                        "7": "SAR Radar Test (All Modes)",
                        "8": "Targeting Radar Test (All Modes)",
                        "9": "AEWC Radar Test (All Modes)"
                    }

                    print("\nAvailable tests:")
                    for key, name in test_options.items():
                        print(f"{key}) {name}")

                    test_choice = input(f"\nSelect a test to run (1-{len(test_options)}): ")

                    if test_choice == "1":
                        asyncio.run(self.combined_precipitation_vil_flow_test())
                    elif test_choice == "2":
                        asyncio.run(self.fms_system_test())
                    elif test_choice == "3":
                        asyncio.run(self.flight_control_system_test())
                    elif test_choice == "4":
                        asyncio.run(self.predefined_messages_test())
                    elif test_choice == "5":
                        asyncio.run(self.weather_radar_all_modes_test())
                    elif test_choice == "6":
                        asyncio.run(self.tfr_radar_all_modes_test())
                    elif test_choice == "7":
                        asyncio.run(self.sar_radar_all_modes_test())
                    elif test_choice == "8":
                        asyncio.run(self.targeting_radar_all_modes_test())
                    elif test_choice == "9":
                        asyncio.run(self.aewc_radar_all_modes_test())
                    else:
                        logger.error(f"Invalid test selection: {test_choice}")
                        self.output_queue.put(f"\nInvalid test selection: {test_choice}")

                except Exception as e:
                        # Log error and return immediately
                        logger.error(f"Test failed: {str(e)}")
                        self.output_queue.put(f"\nTest failed: {str(e)}")
                        return

                except Exception as e:
                    logger.error(f"Error running tests: {str(e)}", exc_info=True)
                    self.output_queue.put(f"\nError running tests: {str(e)}")
                finally:
                    # Reset test running flag
                    UserCLI._test_running = False

            elif command == "get_import_statement":
                function_name = input("Enter a function name: ")
                file_path = input("Enter a file path: ")
                self.get_import_statement(function_name, file_path)
            elif command == "list_tables":
                self.list_tables()
            elif command == "get_table":
                table_name = input("Enter a table name: ")
                self.get_table(table_name)
            elif command == "scenario":
                self._handle_scenario_command()
            elif command == "status":
                self._handle_status_command()
            else:
                self.output_queue.put("Unknown command. Type 'help' to see a list of available commands.")
        except Exception as error:
            logger.error(f"Exception occurred in _process_command: {error}", exc_info=True)
            self.output_queue.put(f"Error processing command: {str(error)}")
        finally:
            # Ensure command is marked as processed
            self.command_processed.set()

    async def _run_weather_radar_live(self, title: str):
        """Run the weather radar state-assertion suite against the RUNNING system.

        Stories C14.2 and C14.7 deleted three log-scraping suites that this menu
        used to reach -- radar_tests/weather_radar_test.py,
        weather_radar_surveillance_mode_test.py and
        combined_precipitation_vil_flow_test.py. Their subjects are all covered
        by Tests/test_weather_radar_live.py, which asserts state instead of
        regex-matching log prose.

        That suite's body takes a started SystemManager, so the CLI can run the
        very same assertions in-place without booting a second system -- the CI
        harness and this menu are now two entry points to one test body, rather
        than two test bodies that drift apart.
        """
        try:
            from FMOFP.Tests.test_weather_radar_live import body as weather_radar_live_body
            from FMOFP.core.system_manager import get_system_manager

            logger.info(f"\nStarting {title}...")
            failures = await weather_radar_live_body(get_system_manager())

            if failures == 0:
                logger.info(f"\n{title} completed successfully!")
            else:
                logger.error(f"\n{title} failed: {failures} assertion(s)")
                raise RuntimeError(f"{title} failed: {failures} assertion(s)")

        except Exception as e:
            logger.error(f"Test suite error: {str(e)}", exc_info=True)
            raise

    async def combined_precipitation_vil_flow_test(self):
        """Run the precipitation/VIL data-flow assertions (see _run_weather_radar_live)."""
        await self._run_weather_radar_live("Weather Radar Data Flow Test")

    async def fms_system_test(self):
        """Run the FMS state assertions against the RUNNING system (story C14.4).

        Replaces fms_system_test.py, which verified by regex over captured log
        output. Same entry point the CI suite uses.
        """
        try:
            from FMOFP.Tests.test_fms_live import body as fms_body
            from FMOFP.core.system_manager import get_system_manager

            logger.info("\nStarting Flight Management System Test...")
            failures = await fms_body(get_system_manager())

            if failures == 0:
                logger.info("\nFMS Test completed successfully!")
            else:
                logger.error(f"\nFMS Test failed: {failures} assertion(s)")
                raise RuntimeError(f"FMS Test failed: {failures} assertion(s)")

        except Exception as e:
            logger.error(f"Test suite error: {str(e)}", exc_info=True)
            raise

    async def flight_control_system_test(self):
        """Run the flight control state assertions against the RUNNING system (C14.5)."""
        try:
            from FMOFP.Tests.test_flight_control_live import body as fcs_body
            from FMOFP.core.system_manager import get_system_manager

            logger.info("\nStarting Flight Control System Test...")
            failures = await fcs_body(get_system_manager())

            if failures == 0:
                logger.info("\nFlight Control Test completed successfully!")
            else:
                logger.error(f"\nFlight Control Test failed: {failures} assertion(s)")
                raise RuntimeError(f"Flight Control Test failed: {failures} assertion(s)")

        except Exception as e:
            logger.error(f"Test suite error: {str(e)}", exc_info=True)
            raise

    async def predefined_messages_test(self):
        """Run the predefined-message assertions against the RUNNING system (C14.6)."""
        try:
            from FMOFP.Tests.test_predefined_messages_live import body as pm_body
            from FMOFP.core.system_manager import get_system_manager

            logger.info("\nStarting Predefined Messages Test...")
            failures = await pm_body(get_system_manager())

            if failures == 0:
                logger.info("\nPredefined Messages Test completed successfully!")
            else:
                logger.error(f"\nPredefined Messages Test failed: {failures} assertion(s)")
                raise RuntimeError(f"Predefined Messages Test failed: {failures} assertion(s)")

        except Exception as e:
            logger.error(f"Test suite error: {str(e)}", exc_info=True)
            raise

    async def weather_radar_all_modes_test(self):
        """Run the Weather Radar mode assertions (see _run_weather_radar_live)."""
        await self._run_weather_radar_live("Weather Radar Mode Test")

    async def _run_radar_modes_live(self, title: str):
        """Run the four-radar state-assertion suite against the RUNNING system.

        Story C14.3 replaced radar_tests/{targeting,tfr,aewc,sar}_radar_test.py
        -- four structural clones, 1,416 lines of live code verifying by regex
        over captured log output -- with one parameterised suite in
        Tests/test_radar_modes_live.py that asserts state.

        The four menu entries below all reach it, because the replacement covers
        all four radars in a single pass: it is cheaper to run them together than
        to boot the checks four times, and the interesting assertions are about
        how the radars differ from one another.

        As with the weather radar entry, this runs the same body CI runs, against
        the system the operator already has up.
        """
        try:
            from FMOFP.Tests.test_radar_modes_live import body as radar_modes_body
            from FMOFP.core.system_manager import get_system_manager

            logger.info(f"\nStarting {title}...")
            failures = await radar_modes_body(get_system_manager())

            if failures == 0:
                logger.info(f"\n{title} completed successfully!")
            else:
                logger.error(f"\n{title} failed: {failures} assertion(s)")
                raise RuntimeError(f"{title} failed: {failures} assertion(s)")

        except Exception as e:
            logger.error(f"Test suite error: {str(e)}", exc_info=True)
            raise

    async def tfr_radar_all_modes_test(self):
        """Run the radar mode assertions (covers TFR; see _run_radar_modes_live)."""
        await self._run_radar_modes_live("Radar Mode Test (TFR, targeting, AEWC, SAR)")

    async def sar_radar_all_modes_test(self):
        """Run the radar mode assertions (covers SAR; see _run_radar_modes_live)."""
        await self._run_radar_modes_live("Radar Mode Test (SAR, targeting, TFR, AEWC)")

    async def targeting_radar_all_modes_test(self):
        """Run the radar mode assertions (covers targeting; see _run_radar_modes_live)."""
        await self._run_radar_modes_live("Radar Mode Test (targeting, TFR, AEWC, SAR)")

    async def aewc_radar_all_modes_test(self):
        """Run the radar mode assertions (covers AEWC; see _run_radar_modes_live)."""
        await self._run_radar_modes_live("Radar Mode Test (AEWC, targeting, TFR, SAR)")

    def _handle_test_results(self, results):
        """Handle and display test results."""
        logger.info("Processing test results")
        self.output_queue.put("\nTest Results:")
        for result in results:
            status_symbol = "✓" if result['status'] == 'PASS' else "✗"
            msg = f"{status_symbol} {result['name']}"
            logger.info(msg)
            self.output_queue.put(msg)
            if result['message']:
                logger.info(f"    {result['message']}")
                self.output_queue.put(f"    {result['message']}")

        # Summary
        pass_count = sum(1 for r in results if r['status'] == 'PASS')
        total_count = len(results)
        summary = f"\nSummary: {pass_count}/{total_count} tests passed"
        logger.info(summary)
        self.output_queue.put(summary)

    def _read_command_posix(self, timeout: float = 0.5):
        """POSIX-only: wait up to `timeout` seconds for a line on stdin,
        instead of blocking indefinitely in input(). Returns the command
        string (stripped of its trailing newline, matching input()'s own
        return value), or None if nothing arrived before the timeout --
        callers should treat None as "no input yet, go check stop_threads
        and loop again" rather than as an empty command. Raises EOFError
        on a closed stdin, matching input()'s own contract, so the
        existing `except EOFError` handling in get_commands() still
        applies completely unchanged.

        This exists because a bare `command = input()` blocks the whole
        thread until a real line of input arrives, with no way to notice
        self.stop_threads being set in the meantime. Confirmed live: this
        made "UserCLI_Control"/"UserCLI_Input"/"UserCLI_Processing"/
        "UserCLI_Output" (and the UserCLI component's own stop(), which
        waits on them) each hit thread_manager's bounded shutdown timeout
        every time a real/interactive-like stdin was attached, adding
        several seconds to every shutdown. select.select() on stdin is
        POSIX-only (it doesn't work on pipes/console input on Windows), so
        this is only used when os.name == 'posix'; Windows keeps the
        original blocking input() and relies on the daemon-thread +
        bounded-join safety net in thread_manager.py to still let the
        process exit -- see PLANNING.md.
        """
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        line = sys.stdin.readline()
        if line == '':
            raise EOFError
        return line.rstrip('\n')

    def get_commands(self):
        """Get commands from user input."""
        # Per-iteration error containment. The whole loop used to sit
        # inside a single try/except, so ONE unexpected exception ended
        # this thread for good -- and because a threading.Thread cannot
        # be restarted, nothing could bring command input back for the
        # rest of the session. Each iteration is now guarded on its own:
        # transient failures are logged and the loop continues, with a
        # circuit breaker so a persistently failing iteration cannot spin
        # hot forever. (The EOF-on-stdin case is still handled inline
        # below -- it is expected on headless runs, not an error.)
        consecutive_errors = 0
        last_error_time = 0.0
        _MAX_CONSECUTIVE_ERRORS = 10
        while not self.stop_threads:
            try:
                    if self.state_manager.cli_state_node is not None:
                        if (self.cli_enabled and not self._stdin_eof
                                and userCLIStates.ACCEPTING_COMMANDS.name in self.state_manager.get_cli_state().name):
                            if not self.prompt_shown:
                                print("\nEnter a command: ", end='', flush=True)
                                self.prompt_shown = True
                            try:
                                if os.name == 'posix':
                                    command = self._read_command_posix()
                                    if command is None:
                                        # Nothing arrived within the poll
                                        # window -- loop back around so the
                                        # outer `while not self.stop_threads`
                                        # check gets a chance to run instead of
                                        # staying blocked in input().
                                        continue
                                else:
                                    command = input()
                            except EOFError:
                                # No interactive stdin attached (headless/CI/
                                # automated run). Previously this propagated up
                                # to the broad except below, which logged an
                                # ERROR and let the whole thread exit -- and the
                                # startup sequence would then try to restart
                                # this same Thread object, which always failed
                                # with "threads can only be started once" (a
                                # Python Thread can only ever be started once).
                                # Handle it here instead: log once, disable
                                # further input attempts, and keep the thread
                                # alive in its idle loop so nothing downstream
                                # thinks it needs restarting.
                                logger.info(
                                    "CLI input: no interactive terminal attached "
                                    "(EOF on stdin) -- command input disabled for "
                                    "this session; other systems are unaffected."
                                )
                                self._stdin_eof = True
                                self.prompt_shown = False
                                continue
                            if command:
                                # Handle test command directly
                                if command == "test":
                                    self._process_command(command)
                                else:
                                    self.command_queue.put(command)
                                    self.command_received.set()
                                self.prompt_shown = False
                            time.sleep(0.1)
                        else:
                            time.sleep(0.1)
                    else:
                        time.sleep(0.1)
            except Exception:
                now = time.time()
                # Errors more than 30s apart are unrelated blips, not a
                # stuck loop -- restart the count rather than creeping
                # toward the breaker over a long, healthy session.
                if now - last_error_time > 30:
                    consecutive_errors = 1
                else:
                    consecutive_errors += 1
                last_error_time = now
                logger.error(
                    f'Exception occurred in get_commands (consecutive: {consecutive_errors}/{_MAX_CONSECUTIVE_ERRORS})',
                    exc_info=True,
                )
                if consecutive_errors >= _MAX_CONSECUTIVE_ERRORS:
                    logger.error(
                        'get_commands: too many consecutive failures -- '
                        'disabling CLI command input for this session.'
                    )
                    self.cli_enabled = False
                    return
                time.sleep(0.5)

    def output_commands(self):
        try:
            while not self.stop_threads:
                if self.cli_enabled:
                    if not self.output_queue.empty():
                        result = self.output_queue.get()
                        logger.info(result)
                        self.prompt_shown = False
                    if self.command_printed.is_set():
                        self.command_printed.clear()
                        self.command_processed.set()
                    time.sleep(0.1)
                else:
                    time.sleep(0.1)
        except Exception as error:
            logger.error("Exception occurred in output_commands", exc_info=True)

    def enable_cli(self):
        self.cli_enabled = True
        self.state_manager.set_cli_state(userCLIStates.ACCEPTING_COMMANDS)
        logger.info(f"CLI enabled and accepting commands. Thread ID: {threading.get_ident()}")

    def disable_cli(self):
        self.state_manager.set_cli_state(userCLIStates.NOT_ACCEPTING_COMMANDS)
        logger.info(f"CLI disabled and not accepting commands. Thread ID: {threading.get_ident()}")

    def stop_cli_threads(self):
        logger.info(f"Stopping CLI threads. Thread ID: {threading.get_ident()}")
        self.stop_threads = True
        for thread in self.cli_threads:
            thread.join()
        self.cli_threads = []
        logger.info(f"CLI threads stopped. Thread ID: {threading.get_ident()}")

    def is_cli_ready(self):
        return self._initialized and self.cli_enabled and all(thread.is_alive() for thread in self.cli_threads)

    def _get_radar_options(self):
        options = [f"  {i+1}) {radar_type}" for i, radar_type in enumerate(RADAR_TYPES)]
        return "  Which radar would you like to send a request?\n" + "\n".join(options)

    def _get_radar_selection(self):
        radar_types = {str(i+1): radar_type for i, radar_type in enumerate(RADAR_TYPES)}

        while True:
            radar_input = input("Which radar would you like to select? (1-5)? ")
            radar_name = radar_types.get(radar_input)
            if radar_name:
                return radar_name
            else:
                self.output_queue.put("Invalid radar selected. Please try again.")

    def _get_command_options(self):
        return "\n".join([
            "  Available commands:",
            "  1) Radar System Status Request",
            "  2) Radar Mode Change Request"
        ])

    def _get_command_selection(self):
        """Get command selection from user input."""
        command_types = {
            "1": "status",
            "2": "mode_change"
        }

        while True:
            command_input = input("Which command would you like to send? (1-2)? ")
            command = command_types.get(command_input)
            if command:
                return command
            else:
                self.output_queue.put("Invalid command selected. Please try again.")

    def _get_mode_options(self, radar_name):
        if radar_name.lower() == "weather_radar":
            return [mode.name for mode in weather_radarMode], weather_radarMode
        else:
            return [mode.name for mode in RadarMode], RadarMode

    def _display_radar_state(self, radar_name):
        logger.info(f"UserCLI: Displaying current state for {radar_name}")
        radar_management_system = get_radar_management_system()
        radar = radar_management_system.radars.get(radar_name)
        if radar:
            state = radar.get_status()
            self.output_queue.put(f"Current {radar_name} State: {state}")
        else:
            self.output_queue.put(f"{radar_name} not found in the system.")
        self.command_processed.set()

    def generate_random_data_word(self):
        import random
        return ''.join(random.choice('01') for _ in range(16))

    def _handle_status_command(self):
        """Display full system status: system state, all radars, FMS, FCS, Nav, Comms."""
        lines = []
        sep = "-" * 52

        # ── System state ──────────────────────────────────────
        lines.append(sep)
        lines.append("  SYSTEM STATUS")
        lines.append(sep)
        try:
            state = self.state_manager.get_state()
            lines.append(f"  System state : {state.name}")
        except Exception:
            lines.append("  System state : UNKNOWN")

        try:
            cli_state = self.state_manager.get_cli_state()
            lines.append(f"  CLI state    : {cli_state.name}")
        except Exception:
            lines.append("  CLI state    : UNKNOWN")

        # ── Radar subsystems ──────────────────────────────────
        lines.append("")
        lines.append("  RADAR SYSTEMS")
        lines.append(sep)
        try:
            from FMOFP.Systems.radarManagement.radarControl import get_radar_management_system
            rms = get_radar_management_system()
            if rms and rms.radars:
                for name, radar in rms.radars.items():
                    try:
                        st = radar.get_status()
                        mode = st.get("mode", st.get("current_mode", "UNKNOWN"))
                        state_str = st.get("state", st.get("operational_state", "UNKNOWN"))
                        lines.append(f"  {name:<22} mode={mode}  state={state_str}")
                    except Exception as e:
                        lines.append(f"  {name:<22} ERROR: {e}")
            else:
                lines.append("  No radars registered")
        except Exception as e:
            lines.append(f"  Radar management unavailable: {e}")

        # ── Flight Management System ──────────────────────────
        lines.append("")
        lines.append("  FLIGHT MANAGEMENT SYSTEM")
        lines.append(sep)
        try:
            from FMOFP.Systems.flightManagementSys.flightManagementSystem import get_flightManagementSystem
            fms = get_flightManagementSystem()
            st = fms.get_status() if hasattr(fms, "get_status") else {}
            if st:
                for key, val in st.items():
                    lines.append(f"  {key:<22} {val}")
            else:
                lines.append("  FMS running (no status dict available)")
        except Exception as e:
            lines.append(f"  FMS unavailable: {e}")

        # ── Flight Control System ─────────────────────────────
        lines.append("")
        lines.append("  FLIGHT CONTROL SYSTEM")
        lines.append(sep)
        try:
            from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys import get_gcas
            gcas = get_gcas()
            alerts = gcas.get_alerts()
            lines.append(f"  GCAS alerts  : {len(alerts)} active")
            for a in alerts[:3]:
                lines.append(f"    [{a.get('severity','?')}] {a.get('message','')}")
            if len(alerts) > 3:
                lines.append(f"    ... and {len(alerts)-3} more")
        except Exception as e:
            lines.append(f"  GCAS unavailable: {e}")

        try:
            from FMOFP.Systems.flightControlSys.performaneMonitoring.performaneMonitoring import get_performance_monitor
            exceedances = get_performance_monitor().get_exceedances()
            lines.append(f"  Exceedances  : {len(exceedances)} active")
        except Exception as e:
            lines.append(f"  Performance monitor unavailable: {e}")

        # ── Navigation ────────────────────────────────────────
        lines.append("")
        lines.append("  NAVIGATION")
        lines.append(sep)
        try:
            from FMOFP.Systems.nav.dataFusion.navDataFusion import get_nav_data_fusion
            ndf = get_nav_data_fusion()
            nd = ndf.get_data() if hasattr(ndf, "get_data") else {}
            pos = nd.get("position", {})
            if pos:
                lines.append(f"  Lat/Lon      : {pos.get('lat','?'):.4f} / {pos.get('lon','?'):.4f}")
                lines.append(f"  Altitude ft  : {pos.get('alt_ft','?')}")
            else:
                lines.append("  Nav data fusion running (no position yet)")
        except Exception as e:
            lines.append(f"  Nav unavailable: {e}")

        # ── Communications ────────────────────────────────────
        lines.append("")
        lines.append("  COMMUNICATIONS")
        lines.append(sep)
        try:
            from FMOFP.Systems.comms.messaging_service import get_comms_service
            cd = get_comms_service().get_data()
            radio = cd.get("radio", {})
            satcom = cd.get("satcom", {})
            lines.append(f"  Radio        : freq={radio.get('frequency','?')}  active={radio.get('active','?')}")
            lines.append(f"  SatCom       : link={satcom.get('link_quality','?')}")
        except Exception as e:
            lines.append(f"  Comms unavailable: {e}")

        # ── MIL-STD-1553B Bus ────────────────────────────────
        lines.append("")
        lines.append("  MIL-STD-1553B BUS")
        lines.append(sep)
        try:
            from FMOFP.MIL_STD_1553B.Bus_Controller.BC import get_Bus_Controller
            bc = get_Bus_Controller()
            bc_status = bc.get_status() if hasattr(bc, "get_status") else {}
            if bc_status:
                lines.append(f"  BC state     : {bc_status.get('state','UNKNOWN')}")
                lines.append(f"  Messages tx  : {bc_status.get('messages_transmitted', '?')}")
                lines.append(f"  Errors       : {bc_status.get('error_count', '?')}")
            else:
                lines.append("  Bus Controller running")
        except Exception as e:
            lines.append(f"  Bus Controller unavailable: {e}")

        lines.append(sep)
        self.output_queue.put("\n".join(lines))
        logger.info("UserCLI: status command completed")

    def _print_help(self):
        help_message = "\n".join([
            "Available commands:",
            "  status     - Show full system status (state, radars, FMS, FCS, Nav, Comms, Bus)",
            "  send       - Send radar commands",
            "  msg        - Run basic messaging test",
            "  test       - Run all system tests",
            "  test_1553b - Run comprehensive MIL-STD-1553B protocol tests",
            "  scenario   - Load and run training or failure scenarios",
            "  help       - Show this help message"
        ])
        self.output_queue.put(help_message)

    def _print_command_help(self, command_name):
        help_messages = {
            "send": "send - Send commands to radar systems",
            "test": "test - Run all system tests including weather radar, messaging, and MIL-STD-1553B tests"
        }
        self.output_queue.put(help_messages.get(command_name, f"Unknown command '{command_name}'. Type 'help' to see a list of available commands."))

    def _handle_scenario_command(self):
        """Interactive scenario engine control."""
        try:
            from FMOFP.Interfaces.scenarios.scenarioEngine import get_scenario_engine
            engine = get_scenario_engine()
        except Exception as exc:
            self.output_queue.put(f"Scenario engine unavailable: {exc}")
            return

        self.output_queue.put("\n".join([
            "\nScenario Engine",
            "  1) Load training scenario",
            "  2) Load failure scenario",
            "  3) Start loaded scenario",
            "  4) Stop scenario",
            "  5) Show scenario status",
            "  q) Cancel"
        ]))

        choice = input("Select option: ").strip().lower()

        if choice == "1":
            ok = engine.load("trainingScenario.xml")
            self.output_queue.put(
                "Training scenario loaded — type 'scenario' then '3' to start."
                if ok else "Failed to load training scenario (check log for details)."
            )

        elif choice == "2":
            ok = engine.load("failureScenario.xml")
            self.output_queue.put(
                "Failure scenario loaded — type 'scenario' then '3' to start."
                if ok else "Failed to load failure scenario (check log for details)."
            )

        elif choice == "3":
            status = engine.get_status()
            if not status.get("loaded"):
                self.output_queue.put("No scenario loaded. Load one first (options 1 or 2).")
            elif status.get("running"):
                self.output_queue.put("A scenario is already running.")
            else:
                engine.start()
                self.output_queue.put(
                    f"Scenario started: {status.get('scenario_file', 'unknown')}"
                )

        elif choice == "4":
            engine.stop()
            self.output_queue.put("Scenario stopped.")

        elif choice == "5":
            status = engine.get_status()
            lines = [
                "\n=== Scenario Status ===",
                f"  Loaded:   {status.get('loaded', False)}",
                f"  File:     {status.get('scenario_file', 'none')}",
                f"  Running:  {status.get('running', False)}",
                f"  Events:   {status.get('event_count', 0)} total",
                f"  Progress: {status.get('events_fired', 0)} fired",
            ]
            self.output_queue.put("\n".join(lines))

        elif choice == "q":
            self.output_queue.put("Cancelled.")

        else:
            self.output_queue.put("Invalid option.")

    @property
    def paths(self):
        """The source import index, built on first access (B12)."""
        return get_paths_instance()

    def get_import_statement(self, function_name, file_path):
        pass

    def get_table(self, table_name):
        table_data = self.sdb.read_table(table_name)
        if table_data:
            self.output_queue.put(f"Table '{table_name}' contents:\n{table_data}")
        else:
            self.output_queue.put("Invalid radar selected. Please try again.")

    def check_health(self) -> bool:
        """
        Check the health of the UserCLI.
        :return: True if the UserCLI is healthy, False otherwise.
        """
        return self._initialized and self.cli_enabled and all(thread.is_alive() for thread in self.cli_threads)

    def list_tables(self):
        """
        List available tables in the database.
        Note: The current DatabaseManager class doesn't have a method to list all tables.
        This is a placeholder that needs to be implemented.
        """
        self.output_queue.put("Table listing functionality not yet implemented")

    def process_commands(self):
        """Main command processing entry point"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            self.get_commands()
        except Exception as e:
            logger.error(f"Error running async process commands: {e}", exc_info=True)
        finally:
            try:
                loop.close()
            except Exception as e:
                logger.error(f"Error closing event loop: {e}", exc_info=True)

# Lazy initialization of singleton
_user_cli_instance = None

def get_user_cli():
    global _user_cli_instance
    if _user_cli_instance is None:
        _user_cli_instance = UserCLI()
    return _user_cli_instance

if __name__ == '__main__':
    cli()
    cli = get_user_cli()
    cli.process_commands()
