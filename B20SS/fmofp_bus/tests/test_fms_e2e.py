"""The ported flight management partition against the Python it replaces.

Plant (Python, in-process), the real fmofp_fcs, fmofp_nav and fmofp_fms, and
the real fmofp_bc with a command script: flight modes, maneuvers, an attitude
command, autopilot targets, thrust and gear go to the FMS over the bus at
known frames, and what it reported afterwards is read off the monitor port.
The same commands go to the frozen fmsControl.FMSControl in this process,
which is the oracle.

One thing is set up for the oracle: before each command its FMS attitude
dict is written back to the measured attitude (zeros here, the plant being
held level). In the live Python the flight control object overwrote that dict
twenty times a second, so by the time of the next command it held the measured
values; without the overwrite the dict would keep the previous command's
limited values and the "current" attitude an absent field falls back to would
differ from what the port sees.

Needs FMOFP_BC, FMOFP_FCS, FMOFP_NAV and FMOFP_FMS (CI sets them).
"""

import math
import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import CmdStatus, FmsFlightMode, GearPosition, Maneuver, Ssm, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 57000
FRAMES = 200
SYNC_MODE_CODE = 17
LEVEL = dict(roll=0.0, pitch=0.0, yaw=0.0, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0)

# (frame, message) in bus-controller frame numbers; see the windows in the test.
SCRIPT = [
    (10, m.FmsManeuverCmd(cmd_seq=1, maneuver=Maneuver.BREAK_RIGHT)),
    (40, m.FmsFlightModeCmd(cmd_seq=1, flight_mode=FmsFlightMode.COMBAT)),
    (50, m.FmsAttitudeCmd(cmd_seq=1, roll_present=True, pitch_present=True, roll_rate_present=True,
                          roll=45.0, pitch=10.0, roll_rate=50.0)),
    (60, m.FmsNavUpdateCmd(cmd_seq=1, altitude_present=True, airspeed_present=True, heading_present=True,
                           vertical_speed_present=True, altitude=59000.0, airspeed=50.0, heading=5.0,
                           vertical_speed=-9000.0)),
    (70, m.FmsControlCmd(cmd_seq=1, thrust_present=True, thrust=0.95)),
    (90, m.FmsControlCmd(cmd_seq=2, thrust_present=True, thrust=0.87)),
    (110, m.FmsControlCmd(cmd_seq=3, thrust_present=True, gear_present=True, thrust=0.5, gear=GearPosition.DOWN)),
    (130, m.FmsManeuverCmd(cmd_seq=2, maneuver=Maneuver.DIVE, dive_angle=0.0)),
    (140, m.FmsManeuverCmd(cmd_seq=3, maneuver=Maneuver.BARREL_ROLL)),
    (150, m.FmsFlightModeCmd(cmd_seq=2, flight_mode=FmsFlightMode.EMERGENCY)),
    (155, m.FmsManeuverCmd(cmd_seq=4, maneuver=Maneuver.DEFENSIVE_SPLIT_S)),
]

WARNING_BITS = {"BANK_ANGLE": "bank_limited", "PITCH_ANGLE": "pitch_limited", "ROLL_RATE": "roll_rate_limited",
                "PITCH_RATE": "pitch_rate_limited", "YAW_RATE": "yaw_rate_limited"}


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


# The plant answers the bus from a thread of this process, beside the monitor
# reader, so the controller gives it 20 ms to reply (a real terminal answers
# from its buffers); --strict then still fails on any reply that never comes.
class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS")
class FlightManagementEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        # Anything that can fail comes before the processes are spawned, and a
        # failure after that still tears them down: a partition left running
        # keeps its port and answers the next run with its old state.
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_fms_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=5)
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=30000.0, vertical_speed=0.0, pitch=0.0,
                                  roll=0.0, angle_of_attack=2.0, g_force=1.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for name in ("FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS"):
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
        # The monitor is read while the bus runs: a free run of four terminals
        # produces more records than a socket buffer holds.
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", "5", "--rt", "7", "--rt", "12", "--port-base",
             str(PORT_BASE), "--frames", str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script",
             self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        self.assertIn(f"script: {len(SCRIPT)} commands", output)
        return records

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

    @classmethod
    def oracle(cls):
        from FMOFP.Systems.flightManagementSys.fmsControl import FMSControl
        fc = FMSControl()
        fc.fms.set_flight_parameters({"attitude": dict(LEVEL)})
        return fc

    @staticmethod
    def applied_attitude(fc):
        attitude = fc.fms.get_flight_data()["attitude"]
        limited = {bit: name in fc.envelope_warnings for name, bit in WARNING_BITS.items()}
        fc.fms.set_flight_parameters({"attitude": dict(LEVEL)})  # the next command sees the measured attitude
        return attitude, limited

    def assert_attitude(self, status, attitude, limited, label):
        for key in ("roll", "pitch", "yaw"):
            self.assertAlmostEqual(attitude[key], getattr(status, key), delta=0.01, msg=f"{label} {key}")
        for key in ("roll_rate", "pitch_rate", "yaw_rate"):
            self.assertAlmostEqual(attitude[key], getattr(status, key), delta=0.02, msg=f"{label} {key}")
        for bit, value in limited.items():
            self.assertEqual(value, getattr(status, bit), f"{label} {bit}")
        self.assertEqual(Ssm.NORMAL, status.roll_ssm)

    def test_the_fms_answers_every_command_as_fmscontrol_does(self):
        self.monitor.drain()
        framed = self.by_frame(self.run_bus())
        fc = self.oracle()

        # Frame 10: BREAK_RIGHT in NORMAL, limited by the NORMAL profile.
        self.assertTrue(fc.execute_maneuver("BREAK_RIGHT"))
        attitude, limited = self.applied_attitude(fc)
        self.assertTrue(limited["bank_limited"])
        self.assert_attitude(self.last_in_window(framed, m.FmsAttitudeStatus, 12, 50), attitude, limited, "break right")
        maneuver = self.last_in_window(framed, m.FmsManeuverStatus, 12, 130)
        self.assertEqual((Maneuver.BREAK_RIGHT, CmdStatus.COMPLETED, 1),
                         (maneuver.maneuver, maneuver.status, maneuver.last_cmd_seq))

        # Frame 40: COMBAT; frame 50: an attitude command that COMBAT allows in full.
        self.assertTrue(fc.set_mode("COMBAT"))
        status = self.last_in_window(framed, m.FmsStatus, 42, 150)
        self.assertEqual(FmsFlightMode.COMBAT, status.flight_mode)
        self.assertEqual(CmdStatus.COMPLETED, status.flight_mode_status)
        self.assertEqual(fc.tactical_systems["countermeasures"], status.countermeasures.name)
        self.assertEqual(fc.tactical_systems["targeting"], status.targeting.name)
        self.assertEqual(fc.tactical_systems["weapons"], status.weapons.name)
        self.assertEqual(fc.tactical_systems["stealth_mode"] == "ON", status.stealth_mode)
        self.assertEqual(TerminalState.NORMAL, status.state)
        fc.update_attitude(roll=45.0, pitch=10.0, roll_rate=50.0)
        attitude, limited = self.applied_attitude(fc)
        self.assertFalse(any(limited.values()))
        att = self.last_in_window(framed, m.FmsAttitudeStatus, 52, 130)
        self.assert_attitude(att, attitude, limited, "attitude command")
        self.assertEqual((1, CmdStatus.COMPLETED), (att.last_cmd_seq, att.status))

        # Frame 60: autopilot targets; airspeed and vertical speed outside their limits
        # (the altitude field itself ends at the FMS's 60000 ft limit).
        targets = fc.set_autopilot_targets(altitude=59000.0, airspeed=50.0, heading=5.0, vertical_speed=-9000.0)
        nav = self.last_in_window(framed, m.FmsNavUpdateStatus, 62, FRAMES)
        self.assertEqual(CmdStatus.COMPLETED, nav.status)
        self.assertAlmostEqual(targets["altitude"], nav.altitude, delta=0.5)
        self.assertAlmostEqual(targets["airspeed"], nav.airspeed, delta=0.05)
        self.assertAlmostEqual(targets["heading"], nav.heading, delta=0.01)
        self.assertAlmostEqual(targets["vertical_speed"], nav.vertical_speed, delta=0.5)

        # Frames 70, 90, 110: thrust three times, then gear down with the last.
        for frame, level in ((70, 0.95), (90, 0.87), (110, 0.5)):
            fc.set_thrust(level)
            ctl = self.last_in_window(framed, m.FmsControlStatus, frame + 2, frame + 20)
            self.assertEqual(CmdStatus.COMPLETED, ctl.status, frame)
            self.assertAlmostEqual(fc.thrust, ctl.thrust, delta=0.0001, msg=frame)
            self.assertEqual(fc.throttle_position, ctl.throttle_position, frame)
            self.assertEqual(fc.afterburner, ctl.afterburner, frame)
        self.assertEqual("DOWN", fc.set_landing_gear("DOWN"))
        self.assertEqual(GearPosition.DOWN, ctl.gear)

        # Frame 130: DIVE with the default angle; frame 140: BARREL_ROLL is not implemented.
        self.assertTrue(fc.execute_maneuver("DIVE"))
        attitude, limited = self.applied_attitude(fc)
        self.assertAlmostEqual(-30.0, attitude["pitch"])
        self.assert_attitude(self.last_in_window(framed, m.FmsAttitudeStatus, 132, 155), attitude, limited, "dive")
        self.assertFalse(fc.execute_maneuver("BARREL_ROLL"))
        maneuver = self.last_in_window(framed, m.FmsManeuverStatus, 142, 155)
        self.assertEqual((Maneuver.BARREL_ROLL, CmdStatus.NOT_IMPLEMENTED), (maneuver.maneuver, maneuver.status))

        # Frame 150: EMERGENCY; frame 155: a split-S, its 180 degrees of roll held to 90.
        self.assertTrue(fc.set_mode("EMERGENCY"))
        self.assertTrue(fc.execute_maneuver("DEFENSIVE_SPLIT_S"))
        attitude, limited = self.applied_attitude(fc)
        self.assertAlmostEqual(90.0, attitude["roll"])
        self.assertTrue(limited["bank_limited"])
        self.assert_attitude(self.last_in_window(framed, m.FmsAttitudeStatus, 157, FRAMES), attitude, limited, "split-S")
        status = self.last_in_window(framed, m.FmsStatus, 152, FRAMES)
        self.assertEqual(FmsFlightMode.EMERGENCY, status.flight_mode)
        self.assertEqual(fc.tactical_systems["weapons"], status.weapons.name)

        # Flight data and energy at the end, in EMERGENCY, with the plant held level.
        data = self.last_in_window(framed, m.FmsFlightData, 160, FRAMES)
        self.assertEqual(Ssm.NORMAL, data.airspeed_ssm)
        self.assertAlmostEqual(450.0, data.airspeed, delta=0.05)
        self.assertAlmostEqual(450.0 / (661.4788 * 0.98), data.mach, delta=0.001)
        self.assertAlmostEqual(450.0, data.ground_speed, delta=0.05)
        self.assertAlmostEqual(30000.0, data.altitude, delta=0.5)
        self.assertEqual((Ssm.NORMAL, Ssm.NORMAL), (data.roll_ssm, data.pitch_ssm))
        self.assertAlmostEqual(0.0, data.roll, delta=0.01)
        self.assertEqual(Ssm.NORMAL, data.position_ssm)
        self.assertAlmostEqual(self.model.latitude, data.latitude, delta=0.001)
        self.assertEqual(Ssm.NORMAL, data.nav_altitude_ssm)
        self.assertAlmostEqual(30000.0, data.nav_altitude, delta=60.0)
        self.assertEqual((Ssm.NO_COMPUTED_DATA, Ssm.NO_COMPUTED_DATA), (data.track_ssm, data.sideslip_ssm))
        self.assertEqual(FmsFlightMode.EMERGENCY, data.flight_mode)
        # energy_state as flightManagementSystem.update_flight_data computed it, from the fused altitude.
        v = 450.0 * 0.51444
        expected = min(100.0, (0.5 * 15000 * v * v + 15000 * 9.81 * data.nav_altitude * 0.3048) / 1e10 * 100)
        self.assertEqual(Ssm.NORMAL, data.energy_ssm)
        self.assertAlmostEqual(expected, data.energy_state, delta=0.02)

        energy = self.last_in_window(framed, m.FmsEnergy, 152, FRAMES)
        fc.fms.set_flight_parameters({"navigation": {"altitude": data.nav_altitude},
                                      "velocity": {"airspeed": 450.0}, "tactical": {"aoa": 2.0}})
        metrics = fc.calculate_energy_maneuverability()
        self.assertEqual(Ssm.NORMAL, energy.ssm)
        self.assertAlmostEqual(metrics["specific_excess_power"], energy.specific_excess_power, delta=0.05)
        self.assertAlmostEqual(metrics["max_sustained_turn_rate"], energy.max_sustained_turn_rate, delta=0.1)
        self.assertAlmostEqual(metrics["max_instantaneous_turn_rate"], energy.max_instantaneous_turn_rate, delta=0.1)
        self.assertAlmostEqual(metrics["max_climb_rate"], energy.max_climb_rate, delta=8.0)
        self.assertAlmostEqual(metrics["energy_advantage"], energy.energy_advantage, delta=0.5)

        term = self.last_in_window(framed, m.TermStatus, 50, FRAMES, rt=m.FmsStatus.RT)  # 1 Hz
        self.assertEqual(TerminalState.NORMAL, term.state)
        self.assertFalse(math.isnan(energy.specific_excess_power))


if __name__ == "__main__":
    unittest.main()
