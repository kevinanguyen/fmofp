"""The terrain-following radar's arithmetic against the Python it was ported from.

fmofp_radar_tool puts the C++ on stdin/stdout. Each test gives the same terrain
profile to the frozen Python (tfr_processor's ClearanceManager and
PathOptimiser, tfr_radar's terrain warnings) and compares: the original Python
is the oracle. The profile is the plant's hills north of home, quantised as the
PLANT_TERRAIN block carries them.

Two things are set up for the oracle, each recorded in the ICD README: the
path optimiser's fixed 300 m clearance is patched to the set clearance the port
flies, and its aircraft speed is given through update_speed(), which nothing in
the Python ever called.

Needs FMOFP_RADAR_TOOL pointing at the binary (CI sets it).
"""

import sys
import unittest
from unittest import mock

import numpy as np

from fmofp_bus.icd.blocks import PlantTerrain
from fmofp_bus.tests._support import RadarTool, needs
from fmofp_plant.terrain import PROFILE_RANGE_M, PROFILE_SAMPLES, HillTerrain, profile_message, sample_distance

M_PER_FT = 0.3048


def quantised_profile():
    """The hills north of home as the radar receives them: through PLANT_TERRAIN's quarter-metre field."""
    status, words = profile_message(HillTerrain(), 0.0, 0.0, 0.0, seq=1).encode()
    assert status.name == "OK", status
    status, block = PlantTerrain.decode(words)
    assert status.name == "OK", status
    return [record.elevation for record in block.elevation]


def python_points(heights):
    return [(sample_distance(i), h) for i, h in enumerate(heights)]


def tool_with_profile(heights):
    tool = RadarTool()
    tool.send("tfr_profile %r %d %s" % (PROFILE_RANGE_M, len(heights), " ".join(repr(h) for h in heights)))
    return tool


@needs("FMOFP_RADAR_TOOL")
class TerrainFollowingOracleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if "." not in sys.path:
            sys.path.insert(0, ".")
        from FMOFP.Systems.radarManagement.terrainFollowing import tfr_processor
        from FMOFP.Systems.radarManagement.terrainFollowing.tfr_radar import tfr_radar
        cls.tfr_processor = tfr_processor
        cls.tfr_radar = tfr_radar
        cls.heights = quantised_profile()

    def test_the_profile_is_the_pythons_geometry(self):
        linspace = np.linspace(0.0, PROFILE_RANGE_M, PROFILE_SAMPLES)
        for i in range(PROFILE_SAMPLES):
            self.assertEqual(linspace[i], sample_distance(i))
        # The tool reports the same distances for every sample.
        tool = tool_with_profile(self.heights)
        tool.send("tfr_warnings")
        tool.run()
        self.assertEqual(["ok"], tool.line())
        rows = tool.block()
        self.assertEqual(PROFILE_SAMPLES, len(rows))
        for i, row in enumerate(rows):
            self.assertEqual(sample_distance(i), float(row[1]), i)

    def test_clearance_assessment_matches_the_clearance_manager(self):
        altitudes_m = [1000.0, 1300.0, 1700.0, 2500.0]
        tool = tool_with_profile(self.heights)
        for alt in altitudes_m:
            tool.send("tfr_reset", "tfr_assess %r" % alt)
        tool.run()
        self.assertEqual(["ok"], tool.line())
        points = python_points(self.heights)
        for alt in altitudes_m:
            with self.subTest(altitude_m=alt):
                manager = self.tfr_processor.ClearanceManager()
                warnings = manager.assess_profile(points, alt)
                self.assertEqual(["ok"], tool.line())
                master, min_clearance, min_index, advisories = tool.line()
                rows = tool.block()
                self.assertEqual(manager.master_level, master)
                self.assertEqual(manager.min_clearance_m, float(min_clearance))
                clearances = [alt - h for h in self.heights]
                self.assertEqual(clearances.index(min(clearances)), int(min_index))
                self.assertEqual(len(warnings), int(advisories))
                levels = {w.distance_m: w.level for w in warnings}
                for i, row in enumerate(rows):
                    expected = levels.get(sample_distance(i), "CLEAR")
                    self.assertEqual(expected, row[2], f"sample {i} at {alt} m")

    def test_the_master_level_latches_as_the_clearance_manager_does(self):
        # PULL_UP low over the hills, then a profile with only LOW advisories keeps PULL_UP,
        # then a clear profile resets to CLEAR, exactly as the Python's latch behaves.
        sequence = [1000.0, 1700.0, 1900.0, 2500.0]
        tool = tool_with_profile(self.heights)
        tool.send("tfr_reset")
        for alt in sequence:
            tool.send("tfr_assess %r" % alt)
        tool.run()
        self.assertEqual(["ok"], tool.line())
        self.assertEqual(["ok"], tool.line())
        manager = self.tfr_processor.ClearanceManager()
        points = python_points(self.heights)
        seen = []
        for alt in sequence:
            manager.assess_profile(points, alt)
            master = tool.line()[0]
            tool.block()
            self.assertEqual(manager.master_level, master, alt)
            seen.append(master)
        self.assertEqual("PULL_UP", seen[0])
        self.assertIn("CLEAR", seen[-1:], "the highest altitude clears everything")

    def test_path_commands_match_the_path_optimiser(self):
        cases = [(1000.0, 250.0, 300.0), (1300.0, 128.6, 300.0), (2500.0, 250.0, 150.0), (900.0, 300.0, 1524.0),
                 (1700.0, 50.0, 299.9232)]
        tool = tool_with_profile(self.heights)
        for alt, speed, clearance in cases:
            tool.send("tfr_path %r %r %r" % (alt, speed, clearance))
        tool.run()
        self.assertEqual(["ok"], tool.line())
        points = python_points(self.heights)
        for alt, speed, clearance in cases:
            with self.subTest(altitude_m=alt, speed_mps=speed, clearance_m=clearance):
                optimiser = self.tfr_processor.PathOptimiser()
                optimiser.update_speed(speed)
                with mock.patch.object(self.tfr_processor, "MIN_CLEARANCE_M", clearance):
                    commands = optimiser.compute_path(points, alt)
                rows = tool.block()
                self.assertEqual(len(commands), len(rows))
                for command, row in zip(commands, rows):
                    self.assertEqual(command.distance_m, float(row[0]))
                    self.assertAlmostEqual(command.altitude_m, float(row[1]), places=9)
                    self.assertEqual(command.reason, row[2])
                self.assertEqual("CLEARANCE", rows[-1][2] if rows else "CLEARANCE")

    def test_terrain_warnings_match_the_radar(self):
        radar = self.tfr_radar("tfr_radar", None, None)
        radar.terrain_data = python_points(self.heights)
        radar.last_warning_time = 0
        expected = {w["distance"]: w["type"] for w in radar._check_terrain_warnings()}
        self.assertTrue(expected, "the hills carry warnings")
        tool = tool_with_profile(self.heights)
        tool.send("tfr_warnings")
        tool.run()
        self.assertEqual(["ok"], tool.line())
        rows = tool.block()
        self.assertEqual(PROFILE_SAMPLES, len(rows))
        for i, row in enumerate(rows):
            self.assertEqual(expected.get(sample_distance(i), "NONE"), row[2], i)


if __name__ == "__main__":
    unittest.main()
