"""The ported flight control partition against the Python it replaces.

The plant (Python, in-process) holds the aircraft in a chosen situation; the
real fmofp_fcs process receives it over the bus from the real fmofp_bc; the
monitor port shows what the FCS published. The alerts it published are then
compared with groundCollisionAvoidanceSys.GCAS._evaluate() run on the same
numbers: the original Python is the oracle.

Needs FMOFP_BC and FMOFP_FCS pointing at the binaries (CI sets them).
"""

import os
import subprocess
import sys
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import GcasStatus, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 55000
FRAMES_PER_SCENARIO = 30  # FCS_GCAS_ALERTS is 5 Hz: at least two polls, after the data settled

SCENARIOS = {
    "cruise": dict(altitude=30000.0, vertical_speed=0.0, terrain_clearance=30000.0),
    "pullup": dict(altitude=1500.0, vertical_speed=-4500.0, terrain_clearance=1500.0),
    "sink rate": dict(altitude=12000.0, vertical_speed=-2500.0, terrain_clearance=12000.0),
    "terrain": dict(altitude=3000.0, vertical_speed=0.0, terrain_clearance=400.0),
    "alt low": dict(altitude=150.0, vertical_speed=0.0, terrain_clearance=1000.0),
    "pullup and terrain": dict(altitude=150.0, vertical_speed=-4200.0, terrain_clearance=100.0),
}


def python_oracle(hold):
    """What the original GCAS says for these numbers."""
    from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys import GCAS
    alerts = GCAS()._evaluate({"altitude_ft": hold["altitude"], "vertical_speed_fpm": hold["vertical_speed"]},
                              hold["terrain_clearance"])
    codes = {a["code"] for a in alerts}
    status = ("WARNING" if any(a["severity"] == 1 for a in alerts) else "CAUTION" if alerts else "ARMED")
    return codes, status


def binary_env():
    env = dict(os.environ)
    if sys.platform == "win32" and os.path.isdir(r"C:\msys64\ucrt64\bin"):
        env["PATH"] = r"C:\msys64\ucrt64\bin;" + env.get("PATH", "")
    return env


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@unittest.skipUnless(os.environ.get("FMOFP_BC") and os.environ.get("FMOFP_FCS"),
                     "set FMOFP_BC and FMOFP_FCS to the binaries to run")
class FlightControlEndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = PlantModel(seed=1)
        cls.plant = PlantTerminal(cls.model, PORT_BASE)
        assert cls.plant.init() == Status.OK, "plant port in use?"
        cls.plant_thread = PlantThread(cls.plant)
        cls.plant_thread.start()
        cls.monitor = Monitor(PORT_BASE)
        assert cls.monitor.open() == Status.OK, "monitor port in use?"
        cls.fcs = subprocess.Popen([os.environ["FMOFP_FCS"], "--port-base", str(PORT_BASE), "--quiet"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        time.sleep(0.3)

    @classmethod
    def tearDownClass(cls):
        cls.fcs.kill()
        cls.fcs.wait(timeout=10)
        cls.plant_thread.stop.set()
        cls.plant_thread.join(timeout=2)
        cls.plant.shutdown()
        cls.monitor.close()

    def run_bus(self, frames):
        result = subprocess.run(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", "5", "--port-base", str(PORT_BASE), "--frames",
             str(frames), "--free-run", "--strict"],
            capture_output=True, text=True, timeout=60, env=binary_env())
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        return self.monitor.drain()

    def last_transmit(self, records, cls, rt=None):
        """The newest words a terminal transmitted for message class `cls`.

        `rt` is needed for messages every terminal carries (TERM_STATUS).
        """
        rt = cls.RT if rt is None else rt
        self.assertNotEqual(m.EVERY_TERMINAL, rt, f"{cls.__name__} needs an explicit rt")
        for record in reversed(records):
            cw = record.command
            if (cw.transmit and not cw.is_mode_code and cw.rt == rt and cw.subaddress == cls.SUBADDRESS
                    and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                return msg
        self.fail(f"no {cls.__name__} from RT {rt} on the monitor")

    def test_fcs_matches_the_python_gcas_in_every_scenario(self):
        self.monitor.drain()
        for name, hold in SCENARIOS.items():
            with self.subTest(scenario=name):
                self.model.hold = dict(hold)
                records = self.run_bus(FRAMES_PER_SCENARIO)
                alerts = self.last_transmit(records, m.FcsGcasAlerts)
                term = self.last_transmit(records, m.TermStatus, rt=m.FcsStatus.RT)
                self.assertEqual(TerminalState.NORMAL, term.state)

                codes, status = python_oracle(hold)
                self.assertEqual("PULLUP" in codes, alerts.pullup, name)
                self.assertEqual("SINK RATE" in codes, alerts.sink_rate, name)
                self.assertEqual("TERRAIN" in codes, alerts.terrain, name)
                self.assertEqual("ALT LOW" in codes, alerts.alt_low, name)
                self.assertEqual(GcasStatus[status], alerts.status, name)
                self.assertAlmostEqual(hold["vertical_speed"], alerts.vertical_speed, delta=0.5)
                self.assertAlmostEqual(hold["altitude"], alerts.altitude, delta=0.5)

    def test_attitude_is_what_the_plant_published(self):
        self.model.hold = dict(SCENARIOS["cruise"], heading=270.0, pitch=2.5, roll=-10.0)
        records = self.run_bus(FRAMES_PER_SCENARIO)
        attitude = self.last_transmit(records, m.FcsAttitude)
        self.assertAlmostEqual(-90.0, attitude.yaw, delta=0.01)
        self.assertAlmostEqual(2.5, attitude.pitch, delta=0.01)
        self.assertAlmostEqual(-10.0, attitude.roll, delta=0.01)
        self.assertEqual(m.Ssm.NORMAL, attitude.roll_ssm)


if __name__ == "__main__":
    unittest.main()
