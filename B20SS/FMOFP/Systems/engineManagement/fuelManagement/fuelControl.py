import threading
import time
import random
from typing import Dict, List
from collections import defaultdict
import FMOFP.Utils.common.fetching as fetching
from FMOFP.storage.DBM import DatabaseManager
from FMOFP.MIL_STD_1553B.Messaging import ScheduleMessage
from FMOFP.MIL_STD_1553B.mil_std_1553B  import MIL_STD_1553B_Message
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Systems.engineManagement.fuelManagement.fuelMonitor import FuelMonitor
from FMOFP.Systems.engineManagement.fuelManagement.fuelTransfer import FuelTransferManager, TransferRule

logger = get_logger()

_fms_instance = None

class FuelTank:
    def __init__(self, name: str, capacity: float):
        self.name = name
        self.capacity = capacity
        self.current_level = capacity
        self.flow_rate = 0.0

    def consume(self, amount: float) -> float:
        if amount > self.current_level:
            consumed = self.current_level
            self.current_level = 0
        else:
            consumed = amount
            self.current_level -= amount
        return consumed

    def refill(self, amount: float) -> float:
        space_available = self.capacity - self.current_level
        if amount > space_available:
            filled = space_available
            self.current_level = self.capacity
        else:
            filled = amount
            self.current_level += amount
        return filled

class Engine:
    def __init__(self, name: str, max_thrust: float):
        self.name = name
        self.max_thrust = max_thrust
        self.current_thrust = 0.0
        self.fuel_consumption_rate = 0.0

    def set_thrust(self, thrust_percentage: float):
        self.current_thrust = self.max_thrust * (thrust_percentage / 100)
        self.fuel_consumption_rate = self.current_thrust * 0.1  # Simplified fuel consumption model

class FuelSystem:
    def __init__(self):
        self.tanks: Dict[str, FuelTank] = {}
        self.engines: Dict[str, Engine] = {}
        self.fuel_lines: Dict[str, List[str]] = defaultdict(list)
        self.transfer_rates: Dict[str, float] = {}
        self.lock = threading.Lock()
        # dbConfig.xml has no registered system named "engine" (confirmed
        # live: get_system_db('engine') raises "ValueError: No database
        # configured for system: engine" immediately on construction, the
        # same class of bug already fixed for PowerManagementSystem's
        # 'power' -> 'power_management_system' typo earlier this session).
        # CORRECTION (round 18): the note this comment used to carry here --
        # that this class is dead code superseded by
        # Systems/fuelSystems/fuelControl.py's FuelSystemController -- was
        # backwards and is now confirmed wrong. Grepping the whole tree shows
        # FuelSystemController is never imported anywhere outside its own
        # file (that one is the actual dead code), while THIS class (FuelSystem,
        # via FuelManagementSystem) is constructed every boot by
        # get_fuel_management_system(), which system_manager.py calls directly
        # in its live startup sequence ("Starting Fuel Management System").
        # Falling back to the generic 'default' system database remains the
        # right call -- there's still no dedicated dbConfig.xml entry for this
        # class -- but it's live, not dead.
        self.db = DatabaseManager('FMOFP/dbConfig.xml').get_system_db('default')
        self._setup_database()
        self.messaging = ScheduleMessage()
        self.rt_address = 3  # Assign a unique RT address for the Fuel Management System

    def _setup_database(self):
        """
        add_tank()/add_engine()/connect_tank_to_engine()/transfer_fuel()/
        update() all call insert_into_table()/update_table() against
        'fuel_tanks'/'engines'/'fuel_lines', but nothing ever created those
        tables -- this class was dead code (never instantiated) until it
        was wired into system_manager.py's boot sequence this round, so the
        gap was entirely latent. Confirmed live via a full boot+SIGTERM
        test: every add_tank()/update() call flooded the log with
        "ERROR - [DBM] Error during query execution: no such table:
        fuel_tanks" (260+ ERROR lines in a single 15s run). Creating the
        tables here, matching the column set each insert/update call
        actually uses.
        """
        try:
            self.db.create_table('fuel_tanks', {
                'name': 'TEXT PRIMARY KEY',
                'capacity': 'REAL',
                'current_level': 'REAL',
            })
            self.db.create_table('engines', {
                'name': 'TEXT PRIMARY KEY',
                'max_thrust': 'REAL',
                'current_thrust': 'REAL',
            })
            self.db.create_table('fuel_lines', {
                'id': 'INTEGER PRIMARY KEY AUTOINCREMENT',
                'tank_name': 'TEXT',
                'engine_name': 'TEXT',
                'transfer_rate': 'REAL',
            })
        except Exception as e:
            logger.error(f"FuelSystem database setup failed: {e}")

    def add_tank(self, name: str, capacity: float):
        self.tanks[name] = FuelTank(name, capacity)
        # INSERT OR REPLACE, not insert_into_table()'s plain INSERT: setup_aircraft()
        # re-registers this same fixed set of tank names on every process boot via
        # the get_fuel_management_system() singleton factory (system_manager.py's
        # live boot sequence), against fuel_tanks.name TEXT PRIMARY KEY in the
        # on-disk default.db. That table is created with CREATE TABLE IF NOT
        # EXISTS, so its rows survive process restarts even though the in-memory
        # _fms_instance does not. A plain INSERT here live-reproduced
        # "sqlite3.IntegrityError: UNIQUE constraint failed: fuel_tanks.name" on
        # every boot after the very first (caught this round via the CI test
        # suite running against a default.db already populated by an earlier
        # session run). insert_into_table() routes through execute_query_async(),
        # which retries 3x then only logs the failure, so nothing crashed
        # visibly -- but every restart after the first silently failed to persist
        # this tank's starting capacity/level to disk while flooding the log with
        # retry/rollback ERROR lines (260+ per boot, the same log-noise class
        # already fixed for the missing-table gap documented above). INSERT OR
        # REPLACE makes add_tank() idempotent across restarts and matches the
        # existing precedent for this exact pattern in Utils/common/paths.py and
        # DisplayResponseService.py.
        query = 'INSERT OR REPLACE INTO "fuel_tanks" ("name", "capacity", "current_level") VALUES (?, ?, ?)'
        self.db.execute_query_async(query, (name, capacity, capacity), query_type='insert')

    def add_engine(self, name: str, max_thrust: float):
        self.engines[name] = Engine(name, max_thrust)
        # See add_tank() above -- identical UNIQUE-constraint-on-restart failure
        # live-reproduced for engines.name this round, same INSERT OR REPLACE fix.
        query = 'INSERT OR REPLACE INTO "engines" ("name", "max_thrust") VALUES (?, ?)'
        self.db.execute_query_async(query, (name, max_thrust), query_type='insert')

    def connect_tank_to_engine(self, tank_name: str, engine_name: str, transfer_rate: float):
        self.fuel_lines[tank_name].append(engine_name)
        self.transfer_rates[(tank_name, engine_name)] = transfer_rate
        self.db.insert_into_table('fuel_lines', {'tank_name': tank_name, 'engine_name': engine_name, 'transfer_rate': transfer_rate})

    def set_engine_thrust(self, engine_name: str, thrust_percentage: float):
        if engine_name in self.engines:
            self.engines[engine_name].set_thrust(thrust_percentage)
            self.db.update_table('engines', {'current_thrust': self.engines[engine_name].current_thrust}, {'name': engine_name})

    def transfer_fuel(self, source_tank: str, destination_tank: str, amount: float):
        with self.lock:
            transferred = self.tanks[source_tank].consume(amount)
            self.tanks[destination_tank].refill(transferred)
            self.db.update_table('fuel_tanks', {'current_level': self.tanks[source_tank].current_level}, {'name': source_tank})
            self.db.update_table('fuel_tanks', {'current_level': self.tanks[destination_tank].current_level}, {'name': destination_tank})

    def update(self, delta_time: float):
        with self.lock:
            for tank_name, tank in self.tanks.items():
                for engine_name in self.fuel_lines[tank_name]:
                    engine = self.engines[engine_name]
                    fuel_required = engine.fuel_consumption_rate * delta_time
                    fuel_consumed = tank.consume(fuel_required)
                    if fuel_consumed < fuel_required:
                        logger.info(f"Warning: Engine {engine_name} is not receiving enough fuel from tank {tank_name}")
                    self.db.update_table('fuel_tanks', {'current_level': tank.current_level}, {'name': tank_name})

    def get_fuel_status(self) -> Dict[str, float]:
        return {tank.name: tank.current_level for tank in self.tanks.values()}

    def send_fuel_status(self):
        status = self.get_fuel_status()
        message = MIL_STD_1553B_Message(self.rt_address, 0, status)
        self.messaging.send_message(message)

    def receive_message(self, message):
        data = message.data
        if 'engine_thrust' in data:
            engine_name = data['engine_name']
            thrust_percentage = data['engine_thrust']
            self.set_engine_thrust(engine_name, thrust_percentage)
        elif 'fuel_transfer' in data:
            source_tank = data['source_tank']
            destination_tank = data['destination_tank']
            amount = data['amount']
            self.transfer_fuel(source_tank, destination_tank, amount)

class FuelManagementSystem:
    #: Seconds to back off after a failed update pass. Long enough that an
    #: always-raising call cannot spin the CPU, short enough that a
    #: transient error costs one pass. Waited on the stop Event, so stop()
    #: interrupts it.
    ERROR_BACKOFF_S = 5.0

    #: How long an unbroken failure streak must last before health_report()
    #: reports UNHEALTHY. This was UNHEALTHY_AFTER_FAILURES = 3, a count,
    #: which does not survive being applied to the other eight subsystems:
    #: their loop cadences run from 0.02 s to 1.0 s and only half of them
    #: back off on error, so three passes means 15 s on the FCC and 0.3 s on
    #: the nav service. A duration means the same thing everywhere. None
    #: here defers to health.DEFAULT_FAILURE_GRACE_S.
    FAILURE_GRACE_S = None

    def __init__(self):
        self.fuel_system = FuelSystem()
        self.monitor = FuelMonitor(self.fuel_system)
        self.transfer_manager = FuelTransferManager(self.fuel_system)
        # A stop Event, not a bool -- see health_report(). Cleared by
        # start(), set by stop(), and waited on for the backoff so stop()
        # interrupts it.
        self._running = threading.Event()
        self._running.set()          # not running until start() clears it
        # Defined here, not only in start(): get_status() and health_report()
        # read it, and a constructed-but-not-started instance is a supported
        # state -- test_power_fuel_thrust calls get_status() on one. Leaving
        # it to start() raised "'FuelManagementSystem' object has no
        # attribute '_thread'" from get_status().
        self._thread = None
        # The monotonic time of the first failure in the current unbroken
        # streak, or None when the last pass succeeded.
        self._failing_since = None
        self._last_error = ""
        self.update_interval = 0.1  # seconds
        self._start_lock = threading.Lock()

    def setup_aircraft(self):
        # Set up fuel tanks
        self.fuel_system.add_tank("Main Left", 10000)
        self.fuel_system.add_tank("Main Right", 10000)
        self.fuel_system.add_tank("Center", 20000)

        # Set up engines
        self.fuel_system.add_engine("Engine 1", 50000)
        self.fuel_system.add_engine("Engine 2", 50000)

        # Connect tanks to engines
        self.fuel_system.connect_tank_to_engine("Main Left", "Engine 1", 100)
        self.fuel_system.connect_tank_to_engine("Main Right", "Engine 2", 100)
        self.fuel_system.connect_tank_to_engine("Center", "Engine 1", 50)
        self.fuel_system.connect_tank_to_engine("Center", "Engine 2", 50)

        # NOTE: deliberately NOT registering a FuelTransferManager
        # TransferRule here for the same Center->Mains policy that
        # perform_fuel_transfer() below already implements by hand. Tried
        # that first and it double-drained the Center tank (both the
        # hand-rolled transfer AND the rule-based transfer firing every
        # tick), which in turn made FuelMonitor's leak-detection heuristic
        # (which only knows about engine burn, not transfers) misfire
        # "LEAK_SUSPECTED" continuously. perform_fuel_transfer() remains the
        # single source of truth for this balancing policy; transfer_manager
        # is still wired up and callable (e.g. balance_lateral()) but starts
        # with zero rules so update() is a no-op unless rules are added
        # elsewhere.

    def update_fuel_system(self):
        self.fuel_system.update(self.update_interval)

    def check_fuel_levels(self):
        status = self.fuel_system.get_fuel_status()
        for tank, level in status.items():
            if level < 1000:  # Example threshold
                logger.info(f"Warning: Low fuel level in {tank}: {level}")

    def perform_fuel_transfer(self):
        # Example: transfer fuel from center tank to main tanks if they are low
        status = self.fuel_system.get_fuel_status()
        if status["Center"] > 5000:
            if status["Main Left"] < 5000:
                self.fuel_system.transfer_fuel("Center", "Main Left", 1000)
            if status["Main Right"] < 5000:
                self.fuel_system.transfer_fuel("Center", "Main Right", 1000)

    def run(self):
        # NOTE: previously called self.scheduler.start()/stop(), but
        # self.scheduler was never defined anywhere in this class -- calling
        # run() (as the __main__ block below does, via
        # threading.Thread(target=fms.run)) always raised
        # "AttributeError: 'FuelManagementSystem' object has no attribute
        # 'scheduler'" immediately. This class is not currently instantiated
        # anywhere else in the codebase, so the bug was entirely latent.
        # run() is invoked as a thread target, so it needs to be the actual
        # blocking update loop, matching the polling-loop pattern used by
        # the other subsystem singletons (e.g. EngineControlUnit._update_loop).
        # The loop body is guarded as a WHOLE now. It used to call
        # update_fuel_system(), check_fuel_levels() and
        # perform_fuel_transfer() outside the try below, which covered only
        # the monitor/transfer pair -- so anything the first three raised
        # killed this daemon thread permanently and silently. Measured: ONE
        # transient error injected on the second pass, and the worker was
        # gone, with get_status() still reporting running=True.
        #
        # An AST sweep of all nine safety-critical subsystems found this was
        # the only one with unguarded calls in its worker loop; the other
        # eight wrap the whole body, log, back off and continue, which is
        # the shape adopted here.
        #
        # The backoff waits on the stop Event rather than sleeping, so
        # stop() interrupts it. See the note in
        # FlightControlComputer._update_loop: a plain time.sleep(5) backoff
        # is not interruptible, so stop()'s join(timeout=2) returns while
        # the thread is still alive and logs "stopped" up to ~3 s early.
        # run() is also a supported direct entry point -- the __main__ block
        # at the foot of this module does threading.Thread(target=fms.run) --
        # and the loop condition is now an Event that __init__ leaves SET.
        # The old code set `self.running = True` here for exactly this
        # reason; clearing the Event is the same thing.
        #
        # Only when called directly, though. When run() IS the target that
        # start() spawned, start() has already cleared the Event, and
        # clearing it again here would override a stop() that landed in
        # between -- the race the old bool had. Comparing against the thread
        # start() recorded distinguishes the two cases, the same way
        # _start_cleanup_timer's re-arm guard does.
        if self._thread is not threading.current_thread():
            self._running.clear()

        while not self._running.is_set():
            try:
                self.update_fuel_system()
                self.check_fuel_levels()
                self.perform_fuel_transfer()
                # FuelMonitor/FuelTransferManager were built against this
                # exact class's FuelSystem API (tanks/get_fuel_status()/
                # transfer_fuel()/fuel_lines/engines) but, like every other
                # subsystem built this session, had no live driver calling
                # poll()/update() on a schedule. This is that driver.
                self.monitor.poll(self.update_interval)
                self.transfer_manager.update(self.update_interval)
            except Exception as e:
                # Counted, not just logged. Guarding the loop turns "the
                # worker died" into "the worker is alive and achieving
                # nothing", and a liveness-only health check calls that
                # HEALTHY. health_report() reads these two fields so a
                # subsystem failing every pass reports UNHEALTHY instead.
                if self._failing_since is None:
                    self._failing_since = time.monotonic()
                self._last_error = f"{type(e).__name__}: {e}"
                logger.error(
                    f"[FUEL_MGMT] Update pass failed "
                    f"(failing for {time.monotonic() - self._failing_since:.1f}s): {e}")
                self._running.wait(self.ERROR_BACKOFF_S)
                continue
            # A pass completed, so the failure streak (if any) is over.
            self._failing_since = None
            self._last_error = ""
            self._running.wait(self.update_interval)

    def start(self):
        """
        Singleton-service style entry point (matches EngineControlUnit /
        PowerManagementSystem / ThrustManagementSystem): spawns run() as a
        background daemon thread, rather than relying on a caller to do it
        (the old __main__ block below does that manually, which is fine for
        a standalone script but not for system_manager.py's boot sequence).
        """
        # Guarded by a dedicated lock against a TOCTOU race in the
        # check-then-create sequence below -- see PowerManagementSystem
        # .start()'s comment (powerManagement/elec/powerManagementSystem.py)
        # for the full writeup.
        with self._start_lock:
            if getattr(self, '_thread', None) is not None and self._thread.is_alive():
                return
            self._running.clear()
            self._thread = threading.Thread(target=self.run, daemon=True, name="FMS_Update")
            self._thread.start()
            logger.info("[FUEL_MGMT] Fuel Management System started.")

    def stop(self):
        self._running.set()
        thread = getattr(self, '_thread', None)
        if thread is not None:
            thread.join(timeout=2)
        logger.info("Fuel Management System stopped.")

    def set_engine_thrust(self, engine_name: str, thrust_percentage: float):
        self.fuel_system.set_engine_thrust(engine_name, thrust_percentage)

    def get_fuel_status(self):
        return self.fuel_system.get_fuel_status()

    def transfer_fuel(self, source_tank: str, destination_tank: str, amount: float):
        self.fuel_system.transfer_fuel(source_tank, destination_tank, amount)

    def get_status(self):
        # 'running' reports the WORKER THREAD, as it does in the other eight
        # subsystems. It used to report self.running, a bool that run() set
        # True and nothing ever reset -- so when the unguarded loop body
        # killed the worker, this kept reporting running=True over a dead
        # thread. Measured directly: worker gone, get_status()['running']
        # still True.
        return {
            'running': (self._thread is not None and self._thread.is_alive()
                        and not self._running.is_set()),
            'fuel_status': self.fuel_system.get_fuel_status(),
            'engines': {name: eng.current_thrust for name, eng in self.fuel_system.engines.items()},
            'monitor': self.monitor.get_status(),
            'transfers': self.transfer_manager.get_status(),
        }

    # ------------------------------------------------------------------ health
    def health_report(self):
        """Report this subsystem's health through the C2.1 contract.

        Story C2.2/C2.3 added this method; it also reports SUSTAINED
        FAILURE, which all nine subsystems now do.

        Guarding the worker loop (see run()) means a transient error no
        longer kills the thread -- but it also means a permanently broken
        subsystem stays alive, and a liveness-only check would call that
        HEALTHY forever. The failure streak run() maintains is passed
        through so that failing continuously past the grace period reports
        UNHEALTHY with the last error attached. All nine subsystems do this
        now; this one was first.

        This class is no longer the polarity exception among the nine:
        self.running was a plain bool, set True inside run() rather than by
        start() and never reset when the worker died, and it is now
        self._running, a stop Event cleared by start() and set by stop(),
        exactly as in the other eight.
        """
        from FMOFP.Utils.common.health import worker_health

        from FMOFP.Utils.common.health import DEFAULT_FAILURE_GRACE_S

        return worker_health(
            "FUEL", getattr(self, '_thread', None),
            stop_requested=self._running.is_set(),
            failing_since=self._failing_since,
            failure_grace_s=(self.FAILURE_GRACE_S
                             if self.FAILURE_GRACE_S is not None
                             else DEFAULT_FAILURE_GRACE_S),
            last_error=self._last_error)


def get_fuel_management_system() -> "FuelManagementSystem":
    global _fms_instance
    if _fms_instance is None:
        _fms_instance = FuelManagementSystem()
        _fms_instance.setup_aircraft()
    return _fms_instance


# Example usage
if __name__ == "__main__":
    fms = FuelManagementSystem()
    fms.setup_aircraft()

    # Start the fuel management system
    fms_thread = threading.Thread(target=fms.run)    # THREAD STARTED IN WRONG PLACE - SHOULD START IN system_manager.py
    fms_thread.start()

    try:
        # Simulate a flight
        for i in range(100):
            # Randomly adjust engine thrust
            fms.set_engine_thrust("Engine 1", random.uniform(50, 100))
            fms.set_engine_thrust("Engine 2", random.uniform(50, 100))

            # logger.info fuel status every 10 iterations
            if i % 10 == 0:
                logger.info(f"Fuel status: {fms.get_fuel_status()}")

            time.sleep(1)

    finally:
        # Stop the fuel management system
        fms.stop()
        fms_thread.join()

    logger.info("Final fuel status:", fms.get_fuel_status())
