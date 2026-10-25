"""The engine management partition on the bus, end to end.

The plant (Python, in-process, flying level at 450 kt with its two engines),
the real fmofp_fcs, fmofp_fms and fmofp_engine under the real fmofp_bc with a
script of commands: the FMS thrust lever moves, an engine is commanded alone,
the autothrottle is engaged and fuel is transferred. The monitor port shows
what every terminal published: the thrust command reaching the plant's engines
and their measurements coming back, the engine status broadcast to the flight
computer and its fuel efficiency, the autothrottle driving the engines to the
target, the fuel burning along the lines.

Needs FMOFP_BC, FMOFP_FCS, FMOFP_FMS and FMOFP_ENGINE (CI sets them).
"""

import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import CmdStatus, EngineCommand, EngineSelect, FuelTank, Ssm, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 62000
FRAMES = 330
SYNC_MODE_CODE = 17
AIRSPEED_KT = 450.0

# (frame, message) in bus-controller frame numbers; see the windows in the test.
SCRIPT = [
    (10, m.FmsControlCmd(cmd_seq=1, thrust_present=True, thrust=0.8)),
    (100, m.EngCmd(cmd_seq=1, command=EngineCommand.SET_THRUST, engine=EngineSelect.ENGINE_2, thrust=40.0)),
    (160, m.EngCmd(cmd_seq=2, command=EngineCommand.AUTOTHROTTLE_ENGAGE, target_airspeed=500.0)),
    (260, m.EngCmd(cmd_seq=3, command=EngineCommand.FUEL_TRANSFER, source=FuelTank.CENTER,
                   destination=FuelTank.MAIN_LEFT, amount=1000.0)),
]


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_FCS", "FMOFP_FMS", "FMOFP_ENGINE")
class EngineEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_engine_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=9)
            cls.model.hold = dict(speed=AIRSPEED_KT, heading=90.0, altitude=20000.0, vertical_speed=0.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for name in ("FMOFP_FCS", "FMOFP_FMS", "FMOFP_ENGINE"):
                cls.processes.append(subprocess.Popen(
                    [os.environ[name], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
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

    def run_bus(self):
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", str(m.FcsStatus.RT), "--rt", str(m.FmsStatus.RT), "--rt",
             str(m.EngStatus.RT), "--port-base", str(PORT_BASE), "--frames", str(FRAMES), "--free-run", "--strict",
             "--timeout-ms", "20", "--script", self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)  # read as it runs: no buffer overflow
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records, output

    @staticmethod
    def by_frame(records):
        """(frame, record) for every record, frames counted from the sync broadcasts."""
        frame = -1
        out = []
        for record in records:
            cw = record.command
            if cw.is_mode_code and cw.count_or_mode == SYNC_MODE_CODE:
                frame += 1
                continue
            out.append((frame, record))
        return out

    def last_in_window(self, framed, cls, first, last, rt=None):
        """The newest `cls` the terminal transmitted in frames first..last-1."""
        rt = cls.RT if rt is None else rt
        for frame, record in reversed(framed):
            cw = record.command
            if (first <= frame < last and cw.transmit and not cw.is_mode_code and cw.rt == rt
                    and cw.subaddress == cls.SUBADDRESS and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                return msg
        self.fail(f"no {cls.__name__} from RT {rt} in frames {first}..{last - 1}")

    def test_the_engines_follow_the_lever_the_commands_and_the_autothrottle(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # Frame 10: the FMS lever to 80 %. The engine computer commands both engines, the plant's
        # engines follow at their next 2 Hz step, and their measurements come back.
        cmd = self.last_in_window(framed, m.EngThrustCmd, 60, 100)
        self.assertEqual((80.0, 80.0, True, False, False, 0.0),
                         (cmd.thrust_1, cmd.thrust_2, cmd.commands_valid, cmd.autothrottle_engaged,
                          cmd.asymmetric_thrust, cmd.asymmetry))
        plant = self.last_in_window(framed, m.PlantEngine, 70, 100)
        self.assertEqual((80.0, 80.0), (plant.thrust_1, plant.thrust_2), "the plant's engines run the command")
        engines = self.last_in_window(framed, m.EngEngineStatus, 70, 100)
        self.assertEqual((Ssm.NORMAL, Ssm.NORMAL), (engines.engine_1_ssm, engines.engine_2_ssm))
        self.assertAlmostEqual(80.0, engines.thrust_1, delta=0.01)
        self.assertAlmostEqual(0.80 * 80 + 18, engines.n1_1, delta=0.6, msg="N1 with the ECU's 0.1 sigma noise")
        self.assertAlmostEqual(350 + 80 * 4.5, engines.egt_2, delta=5.0)
        self.assertAlmostEqual(80 * 42, engines.fuel_flow_1, delta=25.0)
        self.assertAlmostEqual(2 * 80 * 42, engines.total_fuel_flow, delta=50.0)
        self.assertFalse(engines.overspeed_1 or engines.oil_low_1 or engines.oil_hot_1 or engines.vibration_high_1)
        # The flight computer heard the engine status by broadcast: fuel efficiency = airspeed / (flow / 60).
        perf = self.last_in_window(framed, m.FcsPerfStatus, 70, 120)
        self.assertEqual(Ssm.NORMAL, perf.fuel_ssm)
        self.assertAlmostEqual(AIRSPEED_KT / (2 * 80 * 42 / 60.0), perf.fuel_efficiency, delta=0.05)

        # Frame 100: engine 2 alone to 40 %: asymmetric thrust.
        cmd = self.last_in_window(framed, m.EngThrustCmd, 130, 160)
        self.assertEqual((80.0, 40.0, True, 40.0), (cmd.thrust_1, cmd.thrust_2, cmd.asymmetric_thrust, cmd.asymmetry))
        status = self.last_in_window(framed, m.EngStatus, 105, 160)  # 1 Hz: a window of a second or more
        self.assertEqual((CmdStatus.ACCEPTED, True, False), (status.engine_cmd_status, status.asymmetric_thrust,
                                                              status.autothrottle_engaged))
        self.assertEqual(TerminalState.NORMAL, status.state)

        # Frame 160: the autothrottle to 500 kt at 450 kt: two PI steps take both engines to 100 %.
        cmd = self.last_in_window(framed, m.EngThrustCmd, 230, 260)
        self.assertEqual((100.0, 100.0, True, False), (cmd.thrust_1, cmd.thrust_2, cmd.autothrottle_engaged,
                                                       cmd.asymmetric_thrust))
        status = self.last_in_window(framed, m.EngStatus, 210, 260)
        self.assertEqual((True, 500.0), (status.autothrottle_engaged, status.autothrottle_target))
        engines = self.last_in_window(framed, m.EngEngineStatus, 250, 260)
        self.assertAlmostEqual(100.0, engines.thrust_1, delta=0.01)
        self.assertAlmostEqual(98.0, engines.n1_2, delta=0.6)

        # Frame 260: 1000 kg from the Center to Main Left, which has burned only a few kg: that much moves.
        status = self.last_in_window(framed, m.EngStatus, 265, FRAMES)
        self.assertEqual(CmdStatus.ACCEPTED, status.engine_cmd_status)
        self.assertEqual((False, False, False), (status.low_fuel, status.critical_fuel, status.leak_suspected))
        fuel = self.last_in_window(framed, m.EngFuelStatus, 265, FRAMES)
        self.assertEqual(10000.0, fuel.main_left, "topped up to capacity")
        self.assertLess(fuel.center, 20000.0)
        self.assertGreaterEqual(fuel.transferred, 1.0)
        self.assertLess(fuel.total, 40000.0 - 5.0, "six seconds of burn")
        self.assertGreater(fuel.total, 40000.0 - 60.0)
        self.assertAlmostEqual(2 * 100 * 42, fuel.burn_rate, delta=700.0, msg="the monitor's rate at full thrust")
        self.assertEqual((False, False), (fuel.transfer_left, fuel.transfer_right), "the policy had no cause")

        term = self.last_in_window(framed, m.TermStatus, 280, FRAMES, rt=m.EngStatus.RT)
        self.assertEqual((TerminalState.NORMAL, True, m.ICD_MINOR), (term.state, term.healthy, term.icd_minor))
        self.assertIn("overruns=0", output)
        self.assertGreater(self.plant.thrust_commands, 0, "the plant took the thrust command")


if __name__ == "__main__":
    unittest.main()
