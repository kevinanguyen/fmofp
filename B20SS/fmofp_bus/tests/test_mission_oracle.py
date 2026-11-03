"""The mission computer's route, targeting and order of battle against the frozen Python, through fmofp_mission_tool.

handleRoute.RouteManager, targeting.Targeting and orderOfBattle.OrderOfBattle run in this process beside the
C++ (the tool prints with 17 significant digits). The Python's targeting decay reads the wall clock, so its
time module is replaced by a clock the test moves a second at a time, which is the bus's fifty frames.

Needs FMOFP_MISSION_TOOL (CI sets it).
"""

import os
import subprocess
import sys
import unittest
from unittest import mock

from fmofp_bus.tests._support import binary_env, needs

B20SS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if B20SS not in sys.path:
    sys.path.insert(0, B20SS)

TARGET_KIND = {"POINT": 1, "SAM": 2, "BRIDGE": 4}
STATUS_NAMES = ("DESIGNATED", "ACQUIRED", "TRACKING", "ENGAGED", "BDA_PENDING", "DESTROYED", "LOST")


class MissionTool:
    """One fmofp_mission_tool run; answers read back in the order asked."""

    def __init__(self):
        self.lines = []
        self.out = []
        self.pos = 0

    def send(self, *commands):
        self.lines += commands

    def run(self):
        result = subprocess.run([os.environ["FMOFP_MISSION_TOOL"]], input="\n".join(self.lines) + "\n",
                                capture_output=True, text=True, timeout=120, env=binary_env())
        assert result.returncode == 0, result.stderr
        self.out = result.stdout.splitlines()
        self.pos = 0
        return self

    def line(self):
        self.pos += 1
        return self.out[self.pos - 1].split()

    def numbers(self):
        return [float(x) for x in self.line()]

    def block(self):
        rows = []
        while True:
            row = self.line()
            if row == ["end"]:
                return rows
            rows.append(row)


@needs("FMOFP_MISSION_TOOL")
class RouteOracleTest(unittest.TestCase):
    def test_progress_etes_insertion_and_deviation_match_handle_route(self):
        from FMOFP.Systems.missionPlanning.routeManagement.handleRoute import RouteManager, WaypointType
        py = RouteManager()
        tool = MissionTool()
        tool.send("route_clear")
        route = [("A", 1.0, 0.0, 10000.0, "NAV"), ("B", 2.0, 0.3, 12000.0, "IP"), ("C", 3.0, 0.5, 8000.0, "EGR")]
        for name, lat, lon, alt, kind in route:
            py.add_waypoint(name, lat, lon, alt, WaypointType[kind])
            tool.send(f"route_add {name} {lat!r} {lon!r} {alt!r} {kind}")
        positions = [(0.0, 0.0, 10000.0, 450.0), (0.5, 0.1, 10000.0, 420.0), (1.0 - 0.3 / 60.0, 0.0, 10000.0, 450.0),
                     (1.5, 0.2, 10000.0, 500.0)]
        for pos in positions:
            tool.send(f"route_pos {pos[0]!r} {pos[1]!r} {pos[2]!r} {pos[3]!r}")
            tool.send("route_dump")
        tool.send("route_insert_target TGT-0001 2.6 0.4 500.0")
        tool.send("route_dump")
        tool.send("route_deviate 2.0 0.4 20.0")
        tool.send("route_dump")
        tool.send("route_pos 1.5 0.2 10000.0 500.0")
        tool.run()
        tool.line()  # route_clear's ok
        for _ in route:
            tool.line()  # the ids
        for pos in positions:
            py.update_position(*pos)
            nav = py.get_nav_data()
            active, distance, bearing, remaining = tool.numbers()
            self.assertEqual(nav["active_index"], int(active))
            self.assertEqual(nav["waypoints_remaining"], int(remaining))
            if nav["active_waypoint"] is not None:
                self.assertAlmostEqual(nav["distance_to_next_nm"], distance, delta=0.051, msg="the Python rounds to 0.1")
                self.assertAlmostEqual(nav["bearing_to_next_deg"], bearing, delta=0.051)
            self.compare_routes(py.get_full_route(), tool.block())
        py.insert_target("TGT-0001", 2.6, 0.4, 500.0)
        tgt_id, index = tool.numbers()
        self.assertEqual([w["name"] for w in py.get_full_route()].index("TGT-0001"), int(index))
        self.compare_routes(py.get_full_route(), tool.block())
        moved = py.deviate_from_threat(2.0, 0.4, 20.0)
        self.assertEqual(moved, int(tool.numbers()[0]))
        self.assertGreater(moved, 0, "the threat sits between B and the target")
        self.compare_routes(py.get_full_route(), tool.block())
        py.update_position(1.5, 0.2, 10000.0, 500.0)
        nav = py.get_nav_data()
        active, distance, bearing, remaining = tool.numbers()
        self.assertEqual(nav["active_index"], int(active))
        self.assertAlmostEqual(nav["distance_to_next_nm"], distance, delta=0.051)

    def compare_routes(self, py_route, rows):
        self.assertEqual(len(py_route), len(rows))
        for wp, row in zip(py_route, rows):
            wp_id, lat, lon, alt, ete = (float(x) for x in row)
            self.assertEqual(wp["id"], int(wp_id))
            self.assertAlmostEqual(wp["lat"], lat, places=9)
            self.assertAlmostEqual(wp["lon"], lon, places=9)
            self.assertEqual(wp["alt_ft"], alt)
            if wp["ete_min"] is None:
                self.assertEqual(-1.0, ete)
            else:
                self.assertAlmostEqual(wp["ete_min"], ete, places=9)


@needs("FMOFP_MISSION_TOOL")
class TargetingOracleTest(unittest.TestCase):
    def test_the_status_machine_and_the_decay_match_targeting_py(self):
        import FMOFP.Systems.missionPlanning.targeting.targeting as targeting_module
        clock = [1000.0]
        fake_time = mock.Mock()
        fake_time.time = lambda: clock[0]
        with mock.patch.object(targeting_module, "time", fake_time):
            py = targeting_module.Targeting()
            tool = MissionTool()
            tool.send("tgt_clear")
            t1 = targeting_module.Target("T1", "SAM", (1.0, 1.0, 0.0), 2)
            t2 = targeting_module.Target("T2", "BRIDGE", (2.0, 2.0, 100.0), 1)
            py.add_target(t1)
            py.add_target(t2)
            tool.send(f"tgt_add {TARGET_KIND['SAM']} 1.0 1.0 0.0 2 0")
            tool.send(f"tgt_add {TARGET_KIND['BRIDGE']} 2.0 2.0 100.0 1 0")
            steps = []  # (python call result, tool command)
            steps.append((py.engage_target("T1", "JDAM"), "tgt_engage 1 4"))
            steps.append((py.acquire("T1"), "tgt_acquire 1 0"))
            steps.append((py.acquire("T1"), "tgt_acquire 1 0"))
            steps.append((py.engage_target("T1", "JDAM"), "tgt_engage 1 4"))
            steps.append((py.disengage_target("T1"), "tgt_disengage 1"))
            steps.append((py.engage_target("T1", "MK-82"), "tgt_engage 1 6"))
            steps.append((py.assess_damage("T1", True), "tgt_assess 1 1"))
            steps.append((py.mark_bda_pending("T1"), "tgt_bda 1"))
            steps.append((py.assess_damage("T1", False), "tgt_assess 1 0"))
            steps.append((py.update_from_radar("T2", 2.1, 2.1, 120.0, 30.0, 90.0, 1.5), "tgt_update 2 2.1 2.1 120.0 30.0 90.0 1.5 0"))
            steps.append((py.update_from_radar("T2", 2.1, 2.1, 120.0, 30.0, 90.0, 0.8), "tgt_update 2 2.1 2.1 120.0 30.0 90.0 0.8 0"))
            steps.append((py.update_from_radar("T9", 0, 0, 0), "tgt_update 9 0 0 0 0 0 0.5 0"))
            for _, command in steps:
                tool.send(command)
            tool.send("tgt_dump")
            # The decay, a second a tick: the Python's clock moves, the C++ counts fifty frames.
            ticks = 32
            for n in range(1, ticks + 1):
                tool.send(f"tgt_tick {50 * n}")
                tool.send("tgt_dump")
            tool.run()
            tool.line()  # tgt_clear's ok
            tool.line()  # the two ids
            tool.line()
            for result, command in steps:
                self.assertEqual(result, bool(int(tool.numbers()[0])), command)
            self.compare_dump(py.get_prioritised(), tool.block())
            for n in range(1, ticks + 1):
                clock[0] = 1000.0 + n
                py.simulate_tick()
                tool.line()  # changed flag
                self.compare_dump(py.get_prioritised(), tool.block())
            statuses = {t["id"]: t["status"] for t in py.get_all()}
            self.assertEqual("LOST", statuses["T2"], "a tracked target decays below 0.1 within 32 seconds")
            self.assertEqual("ACQUIRED", statuses["T1"])

    def compare_dump(self, py_targets, rows):
        self.assertEqual(len(py_targets), len(rows))
        for target, row in zip(py_targets, rows):
            self.assertEqual(int(target["id"][1:]), int(row[0]))
            self.assertEqual(target["status"], row[1])
            self.assertAlmostEqual(target["track_quality"], round(float(row[2]), 2), places=9)


@needs("FMOFP_MISSION_TOOL")
class OrderOfBattleOracleTest(unittest.TestCase):
    def test_threats_and_summary_match_order_of_battle_py(self):
        from FMOFP.Systems.missionPlanning.orderOfBattle.orderOfBattle import OrderOfBattle, Unit
        py = OrderOfBattle()
        tool = MissionTool()
        tool.send("oob_clear")
        units = [("F001", "FIGHTER", "Friendly", 1, (1.0, 1.0, 20000.0), 0),
                 ("E001", "SAM_SITE", "Enemy", 2, (1.0, 1.1, 0.0), 4),
                 ("E002", "ARMOR", "Enemy", 2, (1.0, 1.2, 0.0), 1),
                 ("N001", "SHIP", "Neutral", 3, (1.0, 1.3, 0.0), 0),
                 ("U001", "SHIP", "Martian", 0, (1.0, 1.4, 0.0), 5)]
        for uid, kind, affiliation, affiliation_code, pos, threat in units:
            py.add_unit(Unit(uid, kind, affiliation, pos, [], threat_level=threat))
            tool.send(f"oob_add 6 {affiliation_code} {pos[0]!r} {pos[1]!r} {pos[2]!r} {threat}")
        tool.send("oob_threats 2")
        tool.send("oob_threats 1")
        tool.send("oob_summary")
        tool.run()
        tool.line()  # oob_clear's ok
        ids = [int(tool.numbers()[0]) for _ in units]
        self.assertEqual([1, 2, 3, 4, 5], ids)
        by_id = {i + 1: unit[0] for i, unit in enumerate(units)}
        for min_level in (2, 1):
            expected = [u["id"] for u in py.get_threats(min_level)]
            self.assertEqual(expected, [by_id[int(row[0])] for row in tool.block()])
        summary = py.summary()
        friendly, enemy, neutral = tool.numbers()
        self.assertEqual((summary["FRIENDLY"], summary["ENEMY"], summary["NEUTRAL"]),
                         (int(friendly), int(enemy), int(neutral)), "an unknown affiliation is neutral")


if __name__ == "__main__":
    unittest.main()
