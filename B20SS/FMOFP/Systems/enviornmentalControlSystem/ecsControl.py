import time
import threading
from FMOFP.Utils.logger.sys_logger import get_logger
from FMOFP.Systems.enviornmentalControlSystem.climate.climateControl import ClimateControl
from FMOFP.Systems.enviornmentalControlSystem.oxygenGenerationsys.oxygenControl import OxygenControl

logger = get_logger()

_ecs_instance = None

class ECSControl:
    def __init__(self):
        if not hasattr(self, 'running'):
            self.running = False
        self._thread = None
        self._start_lock = threading.Lock()

        # ClimateControl and OxygenControl (climate/climateControl.py,
        # oxygenGenerationsys/oxygenControl.py) were fixed earlier this
        # session (both imported a nonexistent FMOFP.local_messaging
        # .Messaging.Messaging class and called sys_logger(name) as a
        # constructor, raising ModuleNotFoundError on every import) but
        # were never actually wired into anything -- confirmed via a
        # repo-wide dead-code sweep (production readiness re-analysis,
        # August 2026) that neither class was instantiated anywhere
        # outside its own file. They're complementary to, not duplicates
        # of, this class's existing get_temperature()/get_pressure()/
        # get_air_quality() readings: those three remain a documented
        # stub (no real sensor data source exists), while ClimateControl
        # tracks temperature/humidity as genuinely mutable, adjustable
        # state and OxygenControl models cabin oxygen generation -- a
        # concern ECSControl didn't cover at all before. Owned here the
        # same way PowerManagementSystem owns its battery/cooling/HVAC
        # sub-components.
        self.climate = ClimateControl()

        # No sensor source exists; these stay None until set_sensor_readings()
        # is called with real values. See the getters below.
        self._temperature_c = None
        self._pressure_kpa = None
        self._air_quality_pct = None
        self._warned_no_sensors = False
        self.oxygen = OxygenControl()

    def initialize(self):
        logger.info("Initializing Environmental Control System")

    def run(self):
        # NOTE: previously just set self.running = True and returned
        # immediately -- not an actual monitoring loop, so calling run()
        # (e.g. as a thread target) would do nothing beyond that one flag
        # flip. This class isn't currently instantiated anywhere in the
        # codebase (dead code today), so the gap was entirely latent.
        # Turned into a real periodic monitor loop, matching the pattern
        # used by every other subsystem singleton wired into
        # system_manager.py this round.
        self.initialize()
        self.running = True
        while self.running:
            # try/except added round 20: this loop previously had no
            # exception handling at all -- live-reproduced (via the
            # identical pattern in hydrControl.py's run(), same fix
            # applied there) that an uncaught exception here would kill
            # this daemon thread permanently and silently, with no
            # restart and no entry in this app's own log file. Matches
            # the self-healing try/except-per-iteration pattern already
            # established in ThrustManagementSystem/MissionService/
            # NavService's own update loops.
            try:
                self.monitor_ecs()
            except Exception as e:
                logger.error(f"[ECS] Monitor error: {e}")
            time.sleep(2.0)

    def start(self):
        # Guarded by a dedicated lock against a TOCTOU race in the
        # check-then-create sequence below -- see PowerManagementSystem
        # .start()'s comment (powerManagement/elec/powerManagementSystem.py)
        # for the full writeup.
        with self._start_lock:
            if self._thread and self._thread.is_alive():
                return
            self._thread = threading.Thread(target=self.run, daemon=True, name="ECS_Update")
            self._thread.start()
            logger.info("[ECS] Environmental Control System started")

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("Environmental Control System stopped")

    def get_status(self):
        return {
            'running': self._thread is not None and self._thread.is_alive(),
            'temperature_c': self.get_temperature(),
            'pressure_kpa': self.get_pressure(),
            'air_quality_pct': self.get_air_quality(),
            'climate': self.climate.get_climate_status(),
            'oxygen': self.oxygen.get_oxygen_status(),
        }

    def monitor_ecs(self):
        # Monitor temperature, pressure, and air quality
        temperature = self.get_temperature()
        pressure = self.get_pressure()
        air_quality = self.get_air_quality()

        if not self.has_sensor_data():
            # Once, not once per tick: a dead sensor feed should be visible in
            # the log without burying everything else in it.
            if not self._warned_no_sensors:
                logger.warning(
                    "[ECS] No sensor source configured -- temperature, pressure "
                    "and air quality are UNKNOWN. Control loops are idle; they "
                    "are not being driven by placeholder values.")
                self._warned_no_sensors = True
            return

        logger.info(f"ECS Status - Temp: {temperature}°C, Pressure: {pressure} kPa, Air Quality: {air_quality}%")

        # Adjust system based on readings
        self.adjust_temperature(temperature)
        self.adjust_pressure(pressure)
        self.adjust_air_quality(air_quality)

        # Drive the two previously-unwired sub-components each tick:
        # oxygen generation runs continuously in a real ECS, and climate
        # is nudged toward this cycle's temperature reading so
        # get_status()['climate'] reflects genuinely live (if still
        # simulated) state rather than sitting frozen at its 22.0C/50%
        # construction-time defaults forever.
        self.oxygen.generate_oxygen()
        self.climate.adjust_temperature(temperature)

    # These returned the literals 22.5 C / 101.3 kPa / 98.5% -- plausible cabin
    # values indistinguishable from measurements. Worse, the loop was circular:
    # get_temperature() returned 22.5, monitor_ecs() passed it to
    # climate.adjust_temperature(22.5), which stored it, and
    # get_status()['climate']['temperature'] then reported 22.5 back as live
    # state. A constant echoed round and presented as a measurement.
    #
    # There is no sensor source in this project, so the honest answer is None.
    # set_sensor_readings() exists for when one is wired up.

    def set_sensor_readings(self, temperature_c=None, pressure_kpa=None,
                            air_quality_pct=None):
        """Supply real sensor readings. Any value left as None stays unknown."""
        if temperature_c is not None:
            self._temperature_c = float(temperature_c)
        if pressure_kpa is not None:
            self._pressure_kpa = float(pressure_kpa)
        if air_quality_pct is not None:
            self._air_quality_pct = float(air_quality_pct)

    def has_sensor_data(self):
        return None not in (self._temperature_c, self._pressure_kpa,
                            self._air_quality_pct)

    def get_temperature(self):
        """Cabin temperature in C, or None when no sensor has reported."""
        return self._temperature_c

    def get_pressure(self):
        """Cabin pressure in kPa, or None when no sensor has reported."""
        return self._pressure_kpa

    def get_air_quality(self):
        """Clean-air percentage, or None when no sensor has reported."""
        return self._air_quality_pct

    def adjust_temperature(self, current_temp):
        target_temp = 22.0  # Target temperature in °C
        if abs(current_temp - target_temp) > 0.5:
            logger.info(f"Adjusting temperature from {current_temp}°C to {target_temp}°C")
            # Code to adjust temperature

    def adjust_pressure(self, current_pressure):
        target_pressure = 101.3  # Target pressure in kPa
        if abs(current_pressure - target_pressure) > 0.5:
            logger.info(f"Adjusting pressure from {current_pressure} kPa to {target_pressure} kPa")
            # Code to adjust pressure

    def adjust_air_quality(self, current_quality):
        if current_quality < 95:
            logger.info(f"Air quality below threshold. Current: {current_quality}%. Activating air purification.")
            # Code to activate air purification systems

def get_ecs_control() -> "ECSControl":
    global _ecs_instance
    if _ecs_instance is None:
        _ecs_instance = ECSControl()
    return _ecs_instance


if __name__ == "__main__":
    ecs = ECSControl()
    ecs.initialize()
