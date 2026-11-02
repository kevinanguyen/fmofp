"""The ported synthetic aperture radar against the frozen Python, through fmofp_radar_tool.

sar_processor.ChangeDetector is run on the same image pairs as the C++ detector: a
compact synthetic change, a uniform block (which the CFAR masks, and whose
neighbourhood the filter's rounding residue reports instead, in both), random
noise, and two pictures the tool renders of a ship that moved. The radar's own
picture has no Python counterpart (sar_radar.py drew random bars), so its geometry
is checked against the plant's frame and its levels against the ICD's rules.

Needs FMOFP_RADAR_TOOL (CI sets it), numpy and scipy.
"""

import math
import os
import sys
import unittest

from fmofp_bus.tests._support import RadarTool, needs
from fmofp_plant.ground import to_latlon

try:
    import numpy as np
    from scipy import ndimage  # noqa: F401  (the Python detector needs it)
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover - depends on the machine
    HAVE_SCIPY = False

B20SS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIZE = 64


def image_lines(image):
    return [" ".join(str(int(v)) for v in row) for row in image]


def python_events(a, b):
    """sar_processor.ChangeDetector on the pair: the first image is the reference."""
    if B20SS not in sys.path:
        sys.path.insert(0, B20SS)
    from FMOFP.Systems.radarManagement.syntheticAperture.sar_processor import ChangeDetector
    detector = ChangeDetector()
    detector.process(a)
    return detector.process(b)


def road_scene(tool):
    """The road beside a northbound track at 0 N 0 E, 2.5 km east, as the tool's scene."""
    lat_1, lon_1 = to_latlon(2500.0, -10000.0)
    lat_2, lon_2 = to_latlon(2500.0, 10000.0)
    tool.send("sar_scene_clear", f"sar_line ROAD {lat_1!r} {lon_1!r} {lat_2!r} {lon_2!r} 12 20")


@needs("FMOFP_RADAR_TOOL")
@unittest.skipUnless(HAVE_SCIPY, "numpy and scipy are needed for the Python detector")
class SarChangeDetectorOracleTest(unittest.TestCase):
    def tool_events(self, tool):
        """After a sar_detect: the header and the events as the tool prints them."""
        header = tool.line()
        self.assertEqual("n", header[0])
        count = int(header[1])
        events = [tool.line() for _ in range(count)]
        self.assertEqual(["end"], tool.line())
        return int(header[1]), int(header[2]), int(header[3]), events

    def compare(self, a, b, label):
        tool = RadarTool()
        tool.send("sar_image", *image_lines(a), "sar_image", *image_lines(b), "sar_detect")
        tool.run()
        self.assertEqual(["ok"], tool.line())
        self.assertEqual(["ok"], tool.line())
        count, significant, _, cpp = self.tool_events(tool)
        py = python_events(a, b)
        self.assertEqual(len(py), count, f"{label}: {cpp} vs {py}")
        self.assertEqual(len([e for e in py if e.is_significant]), significant, label)
        for c, p in zip(cpp, py):
            self.assertEqual((p.event_id, p.centroid_px[0], p.centroid_px[1], p.area_px),
                             (int(c[0]), int(c[1]), int(c[2]), int(c[3])), label)
            self.assertAlmostEqual(p.log_ratio_mean, float(c[4]), delta=1e-9, msg=label)
            self.assertEqual(p.change_type, c[5], label)
            self.assertAlmostEqual(p.confidence, float(c[6]), delta=1e-9, msg=label)
            self.assertEqual(p.is_significant, c[7] == "1", label)
        return py

    def test_a_compact_change_is_reported_alike(self):
        a = np.full((SIZE, SIZE), 60, dtype=np.uint8)
        b = a.copy()
        for r, c in ((30, 30), (29, 30), (31, 30), (30, 29), (30, 31), (30, 33), (29, 33), (31, 33), (30, 32), (30, 34)):
            b[r, c] = 255
        events = self.compare(a, b, "double cross")
        self.assertEqual(1, len(events))
        self.assertEqual(((30, 31), 10, "ADDITION", False), (events[0].centroid_px, events[0].area_px,
                                                              events[0].change_type, events[0].is_significant))
        self.compare(b, a, "double cross removed")

    def test_a_uniform_block_masks_itself_and_the_residue_is_the_same(self):
        a = np.full((SIZE, SIZE), 60, dtype=np.uint8)
        b = a.copy()
        b[20:30, 20:30] = 255
        events = self.compare(a, b, "block")
        for e in events:
            self.assertAlmostEqual(0.0, e.log_ratio_mean, delta=1e-9,
                                   msg="not the block: unchanged ground where the running sum left a residue")

    def test_noise_and_rendered_pictures_agree(self):
        rng = np.random.RandomState(5)
        for i in range(3):
            n1 = rng.randint(0, 256, (SIZE, SIZE)).astype(np.uint8)
            n2 = rng.randint(0, 256, (SIZE, SIZE)).astype(np.uint8)
            self.compare(n1, n2, f"noise {i}")
        # A ship on a lake, spotlighted at 0.3 m, between two pictures 15 m apart (as 3 s at 5 m/s).
        tool = RadarTool()
        lat, lon = to_latlon(1900.0, 1850.0)
        tool.send("sar_scene_clear", f"sar_area LAKE {lat!r} {lon!r} 500 6")
        spot_lat, spot_lon = to_latlon(0.0, 1855.0)  # the aircraft when it commanded the spotlight
        for seed, north in enumerate((1815.0, 1830.0), start=1):
            ship_lat, ship_lon = to_latlon(2154.0, north)
            tool.send(f"sar_point SHIP {ship_lat!r} {ship_lon!r} 90 190",
                      f"sar_render SPOTLIGHT 0.3 {spot_lat!r} {spot_lon!r} 0 {seed}",
                      "sar_scene_clear", f"sar_area LAKE {lat!r} {lon!r} 500 6")
        tool.send("sar_detect")
        tool.run()
        self.assertEqual(["ok"], tool.line())  # the scene cleared
        self.assertEqual(["ok"], tool.line())  # the lake
        images = []
        for _ in range(2):
            self.assertEqual(["ok"], tool.line())  # the ship
            rows = tool.block()
            self.assertEqual(SIZE, len(rows))
            images.append(np.array([[int(v) for v in row] for row in rows], dtype=np.uint8))
            self.assertEqual(["ok"], tool.line())  # cleared again
            self.assertEqual(["ok"], tool.line())  # the lake again
        count, _, _, cpp = self.tool_events(tool)
        py = python_events(images[0], images[1])
        self.assertEqual(len(py), count, (cpp, py))
        for c, p in zip(cpp, py):
            self.assertEqual((p.centroid_px[0], p.centroid_px[1], p.area_px, p.change_type),
                             (int(c[1]), int(c[2]), int(c[3]), c[5]))
        bright = [(int(np.argwhere(img > 120)[:, 0].mean())) for img in images]
        self.assertGreater(np.sum(images[0] > 120), 200, "the ship, 19 pixels square, against the lake")
        self.assertEqual(3, bright[0] - bright[1], "15 m north is three 4.8 m pixels up")


@needs("FMOFP_RADAR_TOOL")
class SarPictureTest(unittest.TestCase):
    def test_the_strip_lies_where_the_plant_would_put_it(self):
        tool = RadarTool()
        tool.send("sar_geometry STRIPMAP 1.0 0 0 0", "sar_geometry STRIPMAP 1.0 0 0 90", "sar_geometry SCANSAR 10 0 0 0")
        tool.run()
        north = [float(v) for v in tool.line()]
        self.assertEqual([16.0, 0.0, 2512.0], north[:3])
        self.assertAlmostEqual(0.0, north[3], places=12)
        self.assertAlmostEqual(to_latlon(2512.0, 0.0)[1], north[4], places=12, msg="east of a northbound aircraft")
        east = [float(v) for v in tool.line()]
        self.assertAlmostEqual(to_latlon(0.0, -2512.0)[0], east[3], places=12, msg="south of an eastbound one")
        self.assertAlmostEqual(0.0, east[4], places=12)
        wide = [float(v) for v in tool.line()]
        self.assertEqual([640.0, 0.0, 2000.0 + 32.0 * 640.0], wide[:3])

    def test_a_road_darkens_its_column_and_the_picture_is_repeatable(self):
        tool = RadarTool()
        road_scene(tool)
        tool.send("sar_level STRIPMAP 1.0 0 0 0 20 31", "sar_level STRIPMAP 1.0 0 0 0 20 10",
                  "sar_render STRIPMAP 1.0 0 0 0 7", "sar_render STRIPMAP 1.0 0 0 0 7")
        tool.run()
        self.assertEqual(["ok"], tool.line())
        self.assertEqual(["ok"], tool.line())
        self.assertAlmostEqual(35.0, float(tool.line()[0]), places=6, msg="10 of the pixel's 16 m under the road")
        self.assertAlmostEqual(60.0, float(tool.line()[0]), places=6)
        first = [[int(v) for v in row] for row in tool.block()]
        second = [[int(v) for v in row] for row in tool.block()]
        self.assertEqual(first, second, "the same seed, the same speckle")
        self.assertEqual((SIZE, SIZE), (len(first), len(first[0])))
        road = sum(row[31] for row in first) / SIZE
        open_ground = sum(row[10] for row in first) / SIZE
        self.assertLess(road + 10.0, open_ground)
        self.assertLess(abs(open_ground - 60.0), 8.0, "four-look speckle averages to the level")
        self.assertTrue(all(0 <= v <= 255 for row in first for v in row))


if __name__ == "__main__":
    unittest.main()
