"""The ported navigation partition on the bus, fed by the plant.

The plant (Python, in-process) flies east at a held speed and measures GPS
pseudoranges once a second; the real fmofp_nav receives both over the bus from
the real fmofp_bc; the monitor port shows what it published. The solution has
to land on the plant's true position and move with it between fixes.

Needs FMOFP_BC and FMOFP_NAV pointing at the binaries (CI sets them).
"""

import os
import subprocess
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import Ssm, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 56000
FRAMES = 300  # six seconds of simulated flight: six GPS epochs


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_NAV")
class NavigationEndToEndTest(unittest.TestCase):
    nav = None
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        try:
            cls.model = PlantModel(seed=3)
            cls.model.hold = dict(speed=450.0, heading=90.0, altitude=30000.0, vertical_speed=0.0, pitch=0.0, roll=0.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            cls.nav = subprocess.Popen([os.environ["FMOFP_NAV"], "--port-base", str(PORT_BASE), "--quiet"],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
            time.sleep(0.3)
        except BaseException:
            cls.tearDownClass()  # a partition left running would answer the next run
            raise

    @classmethod
    def tearDownClass(cls):
        if cls.nav is not None:
            cls.nav.kill()
            cls.nav.wait(timeout=10)
            cls.nav.stdout.close()
            cls.nav = None
        if cls.plant_thread is not None:
            cls.plant_thread.stop.set()
            cls.plant_thread.join(timeout=2)
        if cls.plant is not None:
            cls.plant.shutdown()
        if cls.monitor is not None:
            cls.monitor.close()

    def run_bus(self, frames):
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", "7", "--port-base", str(PORT_BASE), "--frames",
             str(frames), "--free-run", "--strict"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)  # read as it runs: no buffer overflow
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records

    @staticmethod
    def transmits(records, cls, rt):
        out = []
        for record in records:
            cw = record.command
            if cw.transmit and not cw.is_mode_code and cw.rt == rt and cw.subaddress == cls.SUBADDRESS and record.answered:
                status, msg = cls.decode(record.received)
                if status == Status.OK:
                    out.append(msg)
        return out

    def test_the_solution_lands_on_the_plant_and_moves_with_it(self):
        self.monitor.drain()
        records = self.run_bus(FRAMES)
        solutions = self.transmits(records, m.NavSolution, m.NavSolution.RT)
        self.assertGreater(len(solutions), 10, "NAV_SOLUTION is 10 Hz")
        last = solutions[-1]
        self.assertEqual(Ssm.NORMAL, last.position_ssm)
        self.assertEqual(Ssm.NORMAL, last.altitude_ssm)
        self.assertTrue(last.gps_valid)
        self.assertGreaterEqual(last.sats_used, 4)
        self.assertGreaterEqual(last.fix_count, 4)
        self.assertLessEqual(last.fix_age, 1)
        # Five satellites, 3 m of noise, an estimated clock bias, and the plant
        # moving 230 m/s between the fix and the end: within a hundred metres.
        self.assertAlmostEqual(self.model.latitude, last.latitude, delta=0.001)
        self.assertAlmostEqual(self.model.longitude, last.longitude, delta=0.001)
        self.assertAlmostEqual(30000.0, last.altitude, delta=60.0)
        self.assertAlmostEqual(1.0, last.clock_bias, delta=0.05)  # microseconds

        # Flying east: the longitude the solution reports keeps increasing.
        valid = [s for s in solutions if s.position_ssm == Ssm.NORMAL]
        self.assertGreater(valid[-1].longitude, valid[0].longitude + 0.005)
        self.assertAlmostEqual(valid[0].latitude, valid[-1].latitude, delta=0.001)

        term = self.transmits(records, m.TermStatus, m.NavSolution.RT)
        self.assertTrue(term)
        self.assertEqual(TerminalState.NORMAL, term[-1].state)
        self.assertEqual((m.ICD_MAJOR, m.ICD_MINOR), (term[-1].icd_major, term[-1].icd_minor))


if __name__ == "__main__":
    unittest.main()
