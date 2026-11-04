import os
import xml.etree.ElementTree as ET
import time
import threading
from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()

_airframe_instance = None

# asmConfig.xml lives alongside this file. The previous relative path,
# "Systems/airframeSystemManagement/asmConfig.xml", was missing the "FMOFP/" segment (only
# B20SS/FMOFP/Systems/... exists, not B20SS/Systems/...), so it raised
# FileNotFoundError on every construction -- and this class isn't
# currently instantiated anywhere in the codebase, so the bug was
# entirely latent (production readiness reanalysis, dead-subsystem
# audit). Resolving relative to this file's own directory, matching
# the pattern already used in bitControl.py / DBM.py / baseStartUp.py,
# fixes this regardless of working directory if this class is ever
# wired up.
_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "asmConfig.xml")

class AirframeSystemManager:
    def __init__(self):

        
        tree = ET.parse(_CONFIG_PATH)
        config = tree.getroot()
        
        self.subsystems = []
        for subsys in config.findall("subsystems/subsystem"):
            name = subsys.find("name").text
            desc = subsys.find("description").text
            self.subsystems.append({"name": name, "description": desc})
            
        self.sensors = []
        for sensor in config.findall("sensors/sensor"):
            sensor_id = int(sensor.find("id").text)
            type = sensor.find("type").text
            location = sensor.find("location").text
            self.sensors.append({"id": sensor_id, "type": type, "location": location})

        self._running = threading.Event()
        self._thread = None
        self._start_lock = threading.Lock()

        # Commanded gear position. UNKNOWN until something commands it -- the
        # class has no way to sense the real position.
        self.gear_state = "UNKNOWN"

        # Real sensor readings keyed by sensor id. Empty means no source.
        self.sensor_readings = {}
        self._warned_no_sensor_source = False

    def monitor_airframe(self):
        """Report airframe sensor state.

        This used to log a hardcoded 25 for every sensor, once per second, in
        the same format a real reading would use -- so the log filled with
        invented measurements that nothing distinguished from live data. There
        is no sensor source, so it now says that once instead.
        """
        if not self.sensor_readings:
            if not self._warned_no_sensor_source:
                logger.warning(
                    f"[AIRFRAME] No sensor source configured -- {len(self.sensors)} "
                    "sensors are UNKNOWN. No readings are being published.")
                self._warned_no_sensor_source = True
            return

        for sensor in self.sensors:
            value = self.sensor_readings.get(sensor["id"])
            if value is None:
                continue
            logger.info(
                f"Sensor {sensor['id']} ({sensor['location']}) - "
                f"{sensor['type']}: {value}")

    def set_sensor_reading(self, sensor_id, value):
        """Supply a real reading for one sensor."""
        self.sensor_readings[sensor_id] = value
        
    def control_landing_gear(self, command):
        """Command the landing gear. Idempotent and state-tracking.

        The deployment/retraction sequences are still unimplemented, but the
        commanded state is now recorded, so a caller can see what was last
        asked for and a repeat command is not logged as if it were new.
        """
        if command not in ("deploy", "retract"):
            logger.warning(f"[AIRFRAME] Ignoring unknown gear command: {command!r}")
            return self.gear_state

        target = "DEPLOYED" if command == "deploy" else "RETRACTED"
        if self.gear_state == target:
            logger.debug(f"[AIRFRAME] Landing gear already {target}; no action")
            return self.gear_state

        logger.info(f"[AIRFRAME] Landing gear: {self.gear_state} -> {target}")
        self.gear_state = target
        # Activate the real deployment/retraction sequence here once it exists.
        return self.gear_state
            
    def run(self):
        # NOTE: previously `while True:` with no sleep at all -- a 100%-CPU
        # busy loop with no way to exit, if this class were ever wired up
        # and its run() called (it currently isn't instantiated anywhere in
        # the codebase). No documented polling interval exists in
        # asmConfig.xml, so using 1s as a reasonable generic monitoring
        # cadence, matching the interval other subsystem singletons default
        # to (e.g. CommsService, MissionService) absent a more specific
        # requirement.
        while not self._running.is_set():
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
                self.monitor_airframe()

                # The gear used to be commanded to "deploy" here, once per
                # second, forever -- unconditionally, from a MONITOR loop. It
                # was a placeholder that survived several rounds of work
                # because the method body is only a log statement, so nothing
                # visibly moved. That is exactly why it is dangerous: the day
                # someone implements the deployment sequence, this loop starts
                # commanding it every second in flight.
                #
                # Monitoring does not command. Gear commands come from
                # control_landing_gear(), called by whatever actually decides
                # to move the gear.
            except Exception as e:
                logger.error(f"[AIRFRAME] Monitor error: {e}")
            time.sleep(1.0)

    def start(self):
        # Guarded by a dedicated lock -- see PowerManagementSystem.start()'s
        # comment for the full TOCTOU race explanation confirmed live via
        # an extended boot test; same fix applied here.
        with self._start_lock:
            if self._thread and self._thread.is_alive():
                return
            self._running.clear()
            self._thread = threading.Thread(target=self.run, daemon=True, name="Airframe_Update")
            self._thread.start()
            logger.info("[AIRFRAME] Airframe System Manager started")

    def stop(self):
        self._running.set()
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("Airframe System Manager stopped")

    def get_status(self):
        return {
            'running': self._thread is not None and self._thread.is_alive(),
            'subsystems': list(self.subsystems),
            'sensor_count': len(self.sensors),
        }


def get_airframe_system_manager() -> "AirframeSystemManager":
    global _airframe_instance
    if _airframe_instance is None:
        _airframe_instance = AirframeSystemManager()
    return _airframe_instance


if __name__ == "__main__":
    manager = AirframeSystemManager()
    manager.run()