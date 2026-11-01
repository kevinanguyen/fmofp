"""The one-command demo: its scripted flight is valid bus traffic, its dashboard reads the monitor, and with the
binaries it runs the whole aircraft headless for a few seconds."""

import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest

from fmofp_bus.command_word import CommandWord, decode_status_word
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import FmsFlightMode, GcasStatus, Ssm, TerminalState, WeatherMode
from fmofp_bus.monitor import MonitorRecord
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_demo.demo import (B20SS, DISPLAYS, PARTITIONS, PLANT_RT, SYNC_HZ, SYNC_MODE_CODE, Dashboard, binaries,
                             default_events, event_frame, missing_binaries, script_lines)

PORT_BASE = 63000


class ScriptedFlightTest(unittest.TestCase):
    def test_every_event_is_a_valid_command_in_time_order(self):
        events = default_events()
        lines = script_lines(events)
        self.assertTrue(lines[0].startswith("#"))
        self.assertEqual(len(events), len(lines) - 1)
        for event, line in zip(events, lines[1:]):
            frame, rt, sa, *words = line.split()
            self.assertEqual(event_frame(event), int(frame))
            self.assertEqual((event.message.RT, event.message.SUBADDRESS), (int(rt), int(sa)))
            status, back = type(event.message).decode([int(w, 16) for w in words])
            self.assertEqual(Status.OK, status, event.what)
            self.assertEqual(event.message.cmd_seq, back.cmd_seq)
            self.assertGreaterEqual(event.message.cmd_seq, 201, "clear of the radar display window's own numbering")
        self.assertEqual([e.seconds for e in events], sorted(e.seconds for e in events))
        first = events[0].message
        self.assertEqual((WeatherMode.SURVEILLANCE, 200.0), (first.requested_mode, first.range_km))

    def test_the_binaries_are_named_per_partition(self):
        paths = binaries(os.path.join("build", "x"))
        self.assertEqual({"bc", "fcs", "nav", "fms", "radar", "engine"}, set(paths))
        self.assertEqual(len(paths), len(missing_binaries(paths)), "a build directory that does not exist")
        self.assertEqual([5, 7, 12, 9, 3], [rt for _, rt in PARTITIONS])
        self.assertEqual([15, 11, 13, 14], [rt for _, rt in DISPLAYS])
        self.assertEqual(27, PLANT_RT)


def transmit(rt, subaddress, message):
    status, words = message.encode()
    assert status == Status.OK, status
    _, status_word = decode_status_word(rt << 11)
    return MonitorRecord(1, CommandWord(rt=rt, transmit=True, subaddress=subaddress, count_or_mode=len(words)),
                         status=status_word, received=words)


class DashboardTest(unittest.TestCase):
    def test_frames_messages_and_silent_terminals(self):
        dash = Dashboard([PLANT_RT, m.FmsStatus.RT, m.FcsStatus.RT])
        sync = MonitorRecord(1, CommandWord(rt=31, transmit=False, subaddress=0, count_or_mode=SYNC_MODE_CODE))
        for _ in range(3):
            dash.take(sync, 0.5)
        self.assertEqual(2, dash.frame, "frames count from zero")
        flight = m.FmsFlightData(altitude_ssm=Ssm.NORMAL, airspeed_ssm=Ssm.NORMAL, heading_ssm=Ssm.NORMAL,
                                 altitude=10000.0, airspeed=497.0, heading=2.0, flight_mode=FmsFlightMode.COMBAT)
        dash.take(transmit(m.FmsStatus.RT, m.FmsFlightData.SUBADDRESS, flight), 1.0)
        dash.take(transmit(m.FcsStatus.RT, m.FcsGcasAlerts.SUBADDRESS, m.FcsGcasAlerts(status=GcasStatus.WARNING,
                                                                                        terrain=True)), 1.0)
        unanswered = MonitorRecord(2, CommandWord(rt=PLANT_RT, transmit=True, subaddress=1, count_or_mode=11))
        dash.take(unanswered, 1.0)
        line = dash.line(1.0, 1.0)
        self.assertIn("alt 10000 ft  ias 497 kt  hdg 002  COMBAT", line)
        self.assertIn("GCAS WARNING TERRAIN", line)
        self.assertIn("2/3 answering (silent: plant)", line)
        self.assertEqual([m.FmsStatus.RT, m.FcsStatus.RT], dash.answering(1.0))
        self.assertEqual([], dash.answering(10.0), "an answer ages out")
        term = m.TermStatus(state=TerminalState.NORMAL, healthy=True)
        dash.take(transmit(m.FcsStatus.RT, m.TermStatus.SUBADDRESS, term), 1.0)
        self.assertEqual(TerminalState.NORMAL, dash.latest[(m.TermStatus, m.FcsStatus.RT)].state)


@needs("FMOFP_BC", "FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS", "FMOFP_RADAR", "FMOFP_ENGINE")
@unittest.skipUnless(importlib.util.find_spec("PyQt6"), "PyQt6 is needed for the display programs")
class DemoRunTest(unittest.TestCase):
    def test_the_whole_aircraft_runs_headless_for_eight_seconds(self):
        build = os.path.dirname(os.path.dirname(os.environ["FMOFP_BC"]))
        log_dir = tempfile.mkdtemp(prefix="fmofp_demo_")
        env = binary_env()
        env["PYTHONIOENCODING"] = "utf-8"
        result = subprocess.run(
            [sys.executable, "-m", "fmofp_demo", "--port-base", str(PORT_BASE), "--seconds", "8", "--headless",
             "--build", build, "--log-dir", log_dir, "--seed", "4"],
            capture_output=True, text=True, timeout=240, env=env, cwd=B20SS)
        output = result.stdout + result.stderr
        self.assertEqual(0, result.returncode, output)
        self.assertIn(">> t=2 s: weather radar on", output)
        self.assertIn(">> t=3 s: terrain-following radar on", output)
        self.assertIn("10/10 answering", output)
        self.assertIn("GCAS ", output)
        self.assertIn("N1 ", output)
        self.assertIn("WX SURVEILLANCE", output)
        self.assertIn("TFR TERRAIN_FOLLOWING", output)
        self.assertIn(f"bc: frames={8 * SYNC_HZ}", output)
        self.assertIn("bc exit 0", output)
        self.assertTrue(os.path.isfile(os.path.join(log_dir, "events.txt")))


if __name__ == "__main__":
    unittest.main()
