"""The mission computer on the bus, end to end.

The plant (Python, in-process, flying north from home at 450 kt), the real
fmofp_fcs, fmofp_nav, fmofp_fms and fmofp_mission under the real fmofp_bc with
a script of mission commands: a route is loaded and its first waypoint reached
as the aircraft passes it, the phase is set and advanced, a target is
designated (and its TGT waypoint inserted before the nearest waypoint ahead),
acquired, engaged and assessed destroyed, an enemy SAM site is added and the
route optimised away from it. The monitor port shows MSN_STATUS and the
MSN_PLAN blocks the mission computer published.

Needs FMOFP_BC, FMOFP_FCS, FMOFP_NAV, FMOFP_FMS and FMOFP_MISSION (CI sets them).
"""

import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.block import blocks_from_records
from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import (Affiliation, CmdStatus, MissionCommand, MissionPhase, TargetKind, TargetStatus,
                                 TerminalState, UnitKind, WaypointType, Weapon)
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_display.systems_feed import name_of
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 65000
FRAMES = 550
SYNC_MODE_CODE = 17
MISSION_RT = m.MsnStatus.RT


def name_words(text):
    data = text.encode("ascii")[:8].ljust(8, b"\0")
    return [(data[2 * i] << 8) | data[2 * i + 1] for i in range(4)]


def waypoint(seq, name, lat, lon, alt_ft, kind=WaypointType.NAV):
    words = name_words(name)
    return m.MsnCmd(cmd_seq=seq, command=MissionCommand.ADD_WAYPOINT, waypoint_type=kind, fly_over=True, latitude=lat,
                    longitude=lon, altitude=alt_ft, name_0=words[0], name_1=words[1], name_2=words[2], name_3=words[3])


# (frame, message) in bus-controller frame numbers. The aircraft flies up the meridian from 0 N at 231 m/s:
# WP1, 1.3 km out, is within 0.5 nm after two seconds; WP2 at 5.6 km stays ahead for the whole run.
SCRIPT = [
    (5, waypoint(1, "WP1", 0.012, 0.0, 10000.0)),
    (6, waypoint(2, "WP2", 0.05, 0.0, 10000.0)),
    (7, waypoint(3, "EGRESS", 0.1, 0.0, 12000.0, WaypointType.EGR)),
    (20, m.MsnCmd(cmd_seq=4, command=MissionCommand.SET_PHASE, phase=MissionPhase.INGRESS)),
    (60, m.MsnCmd(cmd_seq=5, command=MissionCommand.DESIGNATE_TARGET, target_kind=TargetKind.BRIDGE, priority=1,
                  latitude=0.08, longitude=0.0, altitude=0.0)),
    (120, m.MsnCmd(cmd_seq=6, command=MissionCommand.ADD_UNIT, unit_kind=UnitKind.SAM_SITE,
                   affiliation=Affiliation.ENEMY, threat_level=3, latitude=0.05, longitude=0.003, altitude=0.0)),
    (260, m.MsnCmd(cmd_seq=7, command=MissionCommand.OPTIMISE_ROUTE)),
    (300, m.MsnCmd(cmd_seq=8, command=MissionCommand.ACQUIRE_TARGET, id=1)),
    (350, m.MsnCmd(cmd_seq=9, command=MissionCommand.ENGAGE_TARGET, id=1, weapon=Weapon.JDAM)),
    (400, m.MsnCmd(cmd_seq=10, command=MissionCommand.ASSESS_DAMAGE, id=1, destroyed=True)),
    (430, m.MsnCmd(cmd_seq=11, command=MissionCommand.ADVANCE_PHASE)),
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
            self.plant.poll(0.05)


@needs("FMOFP_BC", "FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS", "FMOFP_MISSION")
class MissionEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_mission_"), "commands.txt")
        write_script(cls.script)
        try:
            model = PlantModel(seed=5, traffic=False, ground=False)
            model.hold = dict(speed=450.0, heading=0.0, altitude=10000.0, vertical_speed=0.0)
            cls.plant = PlantTerminal(model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for binary in ("FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS", "FMOFP_MISSION"):
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

    def run_bus(self):
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", str(m.FcsStatus.RT), "--rt", str(m.NavSolution.RT),
             "--rt", str(m.FmsStatus.RT), "--rt", str(MISSION_RT), "--port-base", str(PORT_BASE), "--frames",
             str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script", self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records, output

    @staticmethod
    def by_frame(records):
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
        rt = cls.RT if rt is None else rt
        for frame, record in reversed(framed):
            cw = record.command
            if (first <= frame < last and cw.transmit and not cw.is_mode_code and cw.rt == rt
                    and cw.subaddress == cls.SUBADDRESS and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                return msg
        self.fail(f"no {cls.__name__} from RT {rt} in frames {first}..{last - 1}")

    def test_the_route_is_flown_the_target_engaged_and_the_plan_published(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)
        self.assertIn("script: 11 commands sent of 11", output)

        # The designation was accepted and put a TGT waypoint before EGRESS, the nearest waypoint ahead of it.
        status = self.last_in_window(framed, m.MsnStatus, 70, 120)
        self.assertEqual(CmdStatus.ACCEPTED, status.mission_cmd_status)
        self.assertEqual((MissionPhase.INGRESS, 4, 1), (status.phase, status.waypoint_count, status.target_count))
        self.assertTrue(status.position_valid, "the FMS flight data broadcast reached the mission computer")

        # WP1 was reached as the aircraft passed within 0.5 nm; WP2, 2-3 nm ahead on the meridian, is active.
        status = self.last_in_window(framed, m.MsnStatus, 200, 250)
        self.assertEqual((1, True, 1), (status.active_index, status.route_active, status.enemy_units))
        self.assertTrue(1.0 < status.distance_to_next < 3.2, status.distance_to_next)
        self.assertTrue(status.bearing_to_next < 5.0 or status.bearing_to_next > 355.0, status.bearing_to_next)

        # The optimisation pushed WP2 (0.18 nm from the SAM) some twenty miles west: the next waypoint is far, and
        # the bearing to it westerly.
        status = self.last_in_window(framed, m.MsnStatus, 290, 340)
        self.assertEqual(1, status.active_index)
        self.assertGreater(status.distance_to_next, 15.0)
        self.assertTrue(250.0 < status.bearing_to_next < 300.0, status.bearing_to_next)

        # Acquired, engaged, assessed destroyed: no active target at the end; the phase advanced to OBJECTIVE.
        status = self.last_in_window(framed, m.MsnStatus, 480, FRAMES)
        self.assertEqual((MissionPhase.OBJECTIVE, 0, CmdStatus.ACCEPTED),
                         (status.phase, status.target_count, status.mission_cmd_status))
        term = self.last_in_window(framed, m.TermStatus, 480, FRAMES, rt=MISSION_RT)
        self.assertEqual(TerminalState.NORMAL, term.state)

        # The plan blocks: the route with its names in order, WP1 passed, the unit, and the target on its way
        # through the status machine before it left the list.
        plans = []
        for _, _version, payload in blocks_from_records(records, MISSION_RT, b.MsnPlan.DESCRIPTOR_SUBADDRESS,
                                                        b.MsnPlan.SEGMENT_SUBADDRESS):
            status, plan = b.MsnPlan.decode(payload)
            self.assertEqual(Status.OK, status)
            plans.append(plan)
        self.assertGreaterEqual(len(plans), 3, "the first plan, the loaded route, the later changes")
        self.assertEqual(0, len(plans[0].waypoint), "the plan at start: an empty route")
        last = plans[-1]
        self.assertEqual(["WP1", "WP2", "TGT-0001", "EGRESS"],
                         [name_of((w.name_0, w.name_1, w.name_2, w.name_3)) for w in last.waypoint])
        self.assertEqual([WaypointType.NAV, WaypointType.NAV, WaypointType.TGT, WaypointType.EGR],
                         [w.type for w in last.waypoint])
        self.assertTrue(last.waypoint[0].passed)
        self.assertFalse(last.waypoint[1].passed)
        self.assertLess(last.waypoint[1].longitude, -0.1, "WP2 deviated from the SAM")
        self.assertEqual(1, last.active_index)
        self.assertEqual(MissionPhase.OBJECTIVE, last.phase)
        self.assertEqual(0, len(last.target))
        self.assertEqual(1, len(last.unit))
        self.assertEqual((UnitKind.SAM_SITE, Affiliation.ENEMY, 3),
                         (last.unit[0].kind, last.unit[0].affiliation, last.unit[0].threat_level))
        statuses = {t.status for plan in plans for t in plan.target}
        self.assertIn(TargetStatus.DESIGNATED, statuses)
        self.assertTrue(statuses & {TargetStatus.ACQUIRED, TargetStatus.ENGAGED}, statuses)
        engaged = [t for plan in plans for t in plan.target if t.status == TargetStatus.ENGAGED]
        for t in engaged:
            self.assertEqual((Weapon.JDAM, TargetKind.BRIDGE, 1), (t.weapon, t.kind, t.priority))
        self.assertIn("overruns=0", output)


if __name__ == "__main__":
    unittest.main()
