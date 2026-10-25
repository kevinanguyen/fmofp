"""The ported flight control partition against the Python it replaces.

The plant (Python, in-process) holds the aircraft in a chosen situation over
flat terrain at a chosen elevation; the real fmofp_radar, commanded into
terrain following, measures the clearance from the plant's terrain profile;
the real fmofp_fcs receives the aircraft state and that clearance over the bus
from the real fmofp_bc; the monitor port shows what the FCS published. The
alerts are then compared with groundCollisionAvoidanceSys.GCAS._evaluate() run
on the same numbers: the original Python is the oracle.

Needs FMOFP_BC, FMOFP_FCS and FMOFP_RADAR pointing at the binaries (CI sets them).
"""

import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import GcasStatus, Ssm, TerminalState, TfrMode
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal
from fmofp_plant.terrain import FlatTerrain

PORT_BASE = 55000
FRAMES_PER_SCENARIO = 45  # the terrain reaches the FCS by frame ~15; FCS_GCAS_ALERTS is 5 Hz

# altitude and vertical speed are held; the terrain is flat at terrain_ft, so the
# radar's minimum clearance is altitude - terrain_ft.
SCENARIOS = {
    "cruise": dict(altitude=30000.0, vertical_speed=0.0, terrain_ft=0.0),
    "pullup": dict(altitude=1500.0, vertical_speed=-4500.0, terrain_ft=0.0),
    "sink rate": dict(altitude=12000.0, vertical_speed=-2500.0, terrain_ft=0.0),
    "terrain": dict(altitude=3000.0, vertical_speed=0.0, terrain_ft=2600.0),
    "alt low": dict(altitude=150.0, vertical_speed=0.0, terrain_ft=-850.0),
    "pullup and terrain": dict(altitude=150.0, vertical_speed=-4200.0, terrain_ft=50.0),
}


def clearance_ft(scenario):
    return scenario["altitude"] - scenario["terrain_ft"]


def python_oracle(scenario):
    """What the original GCAS says for these numbers."""
    from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys import GCAS
    alerts = GCAS()._evaluate({"altitude_ft": scenario["altitude"], "vertical_speed_fpm": scenario["vertical_speed"]},
                              clearance_ft(scenario))
    codes = {a["code"] for a in alerts}
    status = ("WARNING" if any(a["severity"] == 1 for a in alerts) else "CAUTION" if alerts else "ARMED")
    return codes, status


def write_script(path):
    """The terrain-following radar is switched on in the first frame of every run."""
    cmd = m.RdrTfrModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL, requested_mode=TfrMode.TERRAIN_FOLLOWING,
                          set_clearance=1000.0)
    status, words = cmd.encode()
    assert status == Status.OK, status
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        f.write(f"1 {cmd.RT} {cmd.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_FCS", "FMOFP_RADAR")
class FlightControlEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_fcs_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=1, terrain=FlatTerrain(0.0))
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for binary in ("FMOFP_FCS", "FMOFP_RADAR"):
                cls.processes.append(subprocess.Popen(
                    [os.environ[binary], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, env=binary_env()))
            time.sleep(0.3)
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        for p in cls.processes:
            p.kill()
            p.wait(timeout=10)
            p.stdout.close()
        cls.processes = []
        if cls.plant_thread is not None:
            cls.plant_thread.stop.set()
            cls.plant_thread.join(timeout=2)
        if cls.plant is not None:
            cls.plant.shutdown()
        if cls.monitor is not None:
            cls.monitor.close()

    def run_bus(self, frames):
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", "5", "--rt", "9", "--port-base", str(PORT_BASE), "--frames",
             str(frames), "--free-run", "--strict", "--timeout-ms", "20", "--script", self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)  # read as it runs: no buffer overflow
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records

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
        for name, scenario in SCENARIOS.items():
            with self.subTest(scenario=name):
                self.model.terrain = FlatTerrain(scenario["terrain_ft"])
                self.model.hold = dict(altitude=scenario["altitude"], vertical_speed=scenario["vertical_speed"])
                records = self.run_bus(FRAMES_PER_SCENARIO)
                alerts = self.last_transmit(records, m.FcsGcasAlerts)
                term = self.last_transmit(records, m.TermStatus, rt=m.FcsStatus.RT)
                self.assertEqual(TerminalState.NORMAL, term.state)
                guidance = self.last_transmit(records, m.RdrTfrGuidance)
                self.assertEqual(TfrMode.TERRAIN_FOLLOWING, guidance.mode)
                self.assertEqual(Ssm.NORMAL, guidance.clearance_ssm)

                codes, status = python_oracle(scenario)
                self.assertEqual("PULLUP" in codes, alerts.pullup, name)
                self.assertEqual("SINK RATE" in codes, alerts.sink_rate, name)
                self.assertEqual("TERRAIN" in codes, alerts.terrain, name)
                self.assertEqual("ALT LOW" in codes, alerts.alt_low, name)
                self.assertEqual(GcasStatus[status], alerts.status, name)
                self.assertAlmostEqual(scenario["vertical_speed"], alerts.vertical_speed, delta=0.5)
                self.assertAlmostEqual(scenario["altitude"], alerts.altitude, delta=0.5)
                self.assertEqual(Ssm.NORMAL, alerts.clearance_ssm, "the clearance came from the radar")
                self.assertAlmostEqual(clearance_ft(scenario), alerts.terrain_clearance, delta=1.5)

    def test_attitude_is_what_the_plant_published(self):
        self.model.terrain = FlatTerrain(0.0)
        self.model.hold = dict(altitude=30000.0, vertical_speed=0.0, heading=270.0, pitch=2.5, roll=-10.0)
        records = self.run_bus(FRAMES_PER_SCENARIO)
        attitude = self.last_transmit(records, m.FcsAttitude)
        self.assertAlmostEqual(-90.0, attitude.yaw, delta=0.01)
        self.assertAlmostEqual(2.5, attitude.pitch, delta=0.01)
        self.assertAlmostEqual(-10.0, attitude.roll, delta=0.01)
        self.assertEqual(m.Ssm.NORMAL, attitude.roll_ssm)


if __name__ == "__main__":
    unittest.main()
