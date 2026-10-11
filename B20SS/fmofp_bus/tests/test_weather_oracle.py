"""The weather radar's arithmetic against the Python it was ported from.

fmofp_radar_tool puts the C++ on stdin/stdout. Each test gives the same weather
to the frozen Python (reflectivity_simulator's renderers, the VIL and
precipitation generators, the storm cell tracker, the precipitation analyzer,
the wind shear and turbulence processors) and compares: the original Python is
the oracle.

What is set up for the oracle, each a decision recorded in the ICD README:
the Python's random draws are patched to their midpoints (the port's texture
is off), its wind to calm (the port's selection has no drift), numpy's random
500-point subset to the port's even spread, the VIL point jitter to none, and
the processors are given the volume's true 200 km range. The Python's
stormCellTracking measured its vertical development rate over the wall-clock
gap between two of its own statements; here that gap is made the scan interval.

Needs FMOFP_RADAR_TOOL pointing at the binary (CI sets it) and scipy.
"""

import math
import os
import random
import subprocess
import time
import unittest
from unittest import mock

import numpy as np

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd.enums import PrecipType, WeatherMode
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs

SCAN_INTERVAL_S = 1.2
AZIMUTHS, RANGES = 360, 200
VCP = {"SURVEILLANCE": (0.5, 1.5, 2.4, 3.4), "MAPPING": (0.5, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0),
       "WINDSHEAR": (0.5,), "TURBULENCE": (0.5,)}

# Three cells (the middle one small and sharp enough to be wind shear) and a stratiform area.
CELLS = [(45.0, 60.0, 12.0, 58.0, 0), (200.0, 100.0, 4.0, 65.0, 1), (300.0, 150.0, 20.0, 50.0, 0)]
AREA = (120.0, 70.0, 80.0, 50.0, 26.0)
TYPE_NAMES = {"rain": PrecipType.RAIN, "snow": PrecipType.SNOW, "mixed": PrecipType.MIXED, "hail": PrecipType.HAIL,
              "heavy_rain": PrecipType.HEAVY_RAIN}


def truth_command(cells, area):
    words = [str(len(cells))]
    for az, rng, size, dbz, profile in cells:
        words += [repr(az), repr(rng), repr(size), repr(dbz), str(profile)]
    if area is None:
        words.append("0")
    else:
        words += ["1"] + [repr(v) for v in area]
    return "truth " + " ".join(words)


def profile(entry):
    if entry:
        return {i: max(0.2, 1.0 - (i * 0.15)) for i in range(10)}
    return {i: (1.0 - (i * 0.1)) if i < 4 else max(0.2, 1.0 - (i * 0.2)) for i in range(10)}


class Tool:
    """One fmofp_radar_tool run; answers are read back command by command."""

    def __init__(self):
        self.lines = []
        self.out = []
        self.pos = 0

    def send(self, *commands):
        self.lines += commands

    def run(self):
        result = subprocess.run([os.environ["FMOFP_RADAR_TOOL"]], input="\n".join(self.lines) + "\n",
                                capture_output=True, text=True, timeout=600, env=binary_env())
        assert result.returncode == 0, result.stderr
        self.out = result.stdout.splitlines()
        self.pos = 0
        return self

    def line(self):
        self.pos += 1
        return self.out[self.pos - 1].split()

    def block(self):
        rows = []
        while True:
            row = self.line()
            if row == ["end"]:
                return rows
            rows.append(row)


def python_volume(cells, area, elevations, azimuths=AZIMUTHS, ranges=RANGES):
    """reflectivity_simulator's renderers on a zero volume, with its random draws at their midpoints."""
    from FMOFP.Systems.radarManagement.weather.reflectivity_simulator import ReflectivitySimulator
    from FMOFP.Systems.radarManagement.weather.weather_state_manager import StormCell, StratiformArea

    sim = ReflectivitySimulator({})
    state = sim.weather_state
    state.storm_cells = [StormCell((az, rng), size, dbz, profile(entry)) for az, rng, size, dbz, entry in cells]
    state.stratiform_areas = [] if area is None else [StratiformArea((area[0], area[1]), area[2], area[3], area[4])]
    state.initialized = True
    reflectivity = np.zeros((azimuths, len(elevations), ranges))
    with mock.patch.object(random, "uniform", lambda a, c: 0.0), mock.patch.object(random, "random", lambda: 0.0):
        sim._render_storm_cells(reflectivity, len(elevations))
        sim._render_stratiform_areas(reflectivity, len(elevations))
        sim._add_ground_clutter(reflectivity, azimuths, len(elevations))
    return np.clip(reflectivity, -20, 65)


def neutral_uniform(low, high, size=None):
    """np.random.uniform with the VIL jitter turned off: no range perturbation, no azimuth shift."""
    return np.full(size, 1.0 if low == 0.8 else 0.0)


def even_choice(n, k, replace=False):
    """np.random.choice as the port selects its 500: every (n / k)th significant bin."""
    assert not replace
    return np.array([i * n // k for i in range(k)])


def calm():
    from FMOFP.Systems.radarManagement.weather.weather_state_manager import WeatherStateManager
    state = WeatherStateManager.get_instance()
    state.wind["surface"]["speed"] = 0
    state.wind["altitude"]["speed"] = 0
    state.tracked_vil_points = {}
    state.tracked_precip_points = {}
    return state


@needs("FMOFP_RADAR_TOOL")
class WeatherOracleTest(unittest.TestCase):
    _cache = {}

    @classmethod
    def volume(cls, mode):
        key = (mode, tuple(CELLS), AREA)
        if key not in cls._cache:
            cls._cache[key] = python_volume(CELLS, AREA, VCP[mode])
        return cls._cache[key]

    def test_round_to_sigfigs(self):
        from FMOFP.Systems.radarManagement.weather.vil_data_generator_sync import round_to_sigfigs
        values = [1234.5678, 0.0012345, 2.5, 1.005, 65.0, 99.95, 123456.0, 4.4445, 1e-7, 0.1, 3.0, 12500.0, 0.00344,
                  7.77777, 999.5, 0.0005]
        tool = Tool()
        tool.send(*[f"sigfigs {v!r}" for v in values])
        tool.run()
        for v in values:
            self.assertEqual(round_to_sigfigs(v), float(tool.line()[0]), v)

    def test_rendered_volume_is_bin_for_bin_the_pythons(self):
        for mode in ("SURVEILLANCE", "MAPPING"):
            tool = Tool()
            tool.send(truth_command(CELLS, AREA), f"mode {mode}", "texture 0 0", "volume 72 50")
            tool.run()
            tool.line()
            tool.line()
            tool.line()
            rows = tool.block()
            nel = len(VCP[mode])
            mine = np.zeros((72, nel, 50))
            for row in rows:
                az, el = int(row[0]), int(row[1])
                mine[az, el, :] = [float(v) for v in row[2:]]
            theirs = python_volume(CELLS, AREA, VCP[mode], azimuths=72, ranges=50)
            np.testing.assert_array_equal(theirs, mine, err_msg=mode)
            self.assertGreater(mine.max(), 60.0)

    def scan(self, mode, frame=60, extra=()):
        tool = Tool()
        tool.send(truth_command(CELLS, AREA), f"mode {mode}", "texture 0 0", f"scan {frame}", *extra)
        tool.run()
        tool.line()
        tool.line()
        tool.line()
        scan = tool.line()
        self.assertEqual("scan", scan[0])
        return tool

    def field(self, tool):
        rows = tool.block()
        self.assertEqual(AZIMUTHS, len(rows))
        return np.array([[float(v) for v in row] for row in rows])

    def test_vil_and_rain_rate_fields_match(self):
        from FMOFP.Systems.radarManagement.weather.precipitation_data_generator_sync import PrecipitationDataGenerator
        from FMOFP.Systems.radarManagement.weather.vil_data_generator_sync import VILDataGenerator
        tool = self.scan("SURVEILLANCE", extra=("field vil", "field precip", "field composite"))
        vil, precip, composite = self.field(tool), self.field(tool), self.field(tool)
        reflectivity = self.volume("SURVEILLANCE")
        np.testing.assert_array_equal(reflectivity.max(axis=1), composite)
        np.testing.assert_allclose(VILDataGenerator({}).calculate_vil(reflectivity, VCP["SURVEILLANCE"]), vil,
                                   rtol=1e-12, atol=0)
        np.testing.assert_allclose(
            PrecipitationDataGenerator({}).calculate_precipitation(reflectivity, VCP["SURVEILLANCE"]), precip,
            rtol=1e-12, atol=0)
        self.assertGreater(vil.max(), 100.0)

    def points_of(self, tool):
        header = tool.line()
        self.assertEqual("n", header[0])
        count, significant, maximum = int(header[1]), int(header[2]), float(header[3])
        rows = tool.block()
        self.assertEqual(count, len(rows))
        return significant, maximum, rows

    def assert_points(self, rows, objects, kind):
        self.assertEqual(len(objects), len(rows), kind)
        for row, obj in zip(rows, objects):
            az_index, r_index = int(row[0]), int(row[1])
            value, intensity, layers, show, type_name = float(row[2]), float(row[3]), int(row[4]), row[5] == "1", row[6]
            x, y = float(row[7]), float(row[8])
            label = f"{kind} point ({az_index}, {r_index})"
            self.assertAlmostEqual(obj.position[0], x, delta=1e-9, msg=label)
            self.assertAlmostEqual(obj.position[1], y, delta=1e-9, msg=label)
            if kind == "vil":
                self.assertAlmostEqual(obj.value, value, delta=1e-9, msg=label)
                self.assertEqual(obj.layer_count, layers, label)
            else:
                self.assertAlmostEqual(obj.rate, value, delta=1e-9, msg=label)
                theirs = TYPE_NAMES[obj.type]
                mine = PrecipType[type_name]
                light = value <= 10.0
                self.assertTrue(theirs == mine or (light and {theirs, mine} <= {PrecipType.RAIN, PrecipType.SNOW}),
                                f"{label}: {obj.type} vs {type_name}")
            self.assertAlmostEqual(obj.intensity, intensity, delta=1e-9, msg=label)
            self.assertEqual(obj.show_values, show, label)

    def test_vil_and_precipitation_points_match_over_two_scans(self):
        from FMOFP.Systems.radarManagement.weather.precipitation_data_generator_sync import PrecipitationDataGenerator
        from FMOFP.Systems.radarManagement.weather.vil_data_generator_sync import VILDataGenerator
        moved = [(az + 1.0, rng, size, dbz, entry) for az, rng, size, dbz, entry in CELLS]
        tool = Tool()
        tool.send(truth_command(CELLS, AREA), "mode SURVEILLANCE", "texture 0 0", "scan 60", "points vil",
                  "points precip", truth_command(moved, AREA), "scan 120", "points vil", "points precip")
        tool.run()
        for _ in range(4):
            tool.line()
        state = calm()
        vil_gen, precip_gen = VILDataGenerator({}), PrecipitationDataGenerator({})
        for cells in (CELLS, moved):
            reflectivity = python_volume(cells, AREA, VCP["SURVEILLANCE"])
            vil_array = vil_gen.calculate_vil(reflectivity, VCP["SURVEILLANCE"])
            precip_array = precip_gen.calculate_precipitation(reflectivity, VCP["SURVEILLANCE"])
            with mock.patch.object(np.random, "uniform", neutral_uniform), \
                    mock.patch.object(np.random, "choice", even_choice):
                vil_objects = vil_gen.generate_vil_data_objects(vil_array, "oracle")
                precip_objects = precip_gen.generate_precipitation_data_objects(precip_array, "oracle")
            significant, maximum, rows = self.points_of(tool)
            self.assertEqual(int(np.count_nonzero(vil_array[::8, ::8] > 0.1)), significant)
            self.assertAlmostEqual(float(vil_array.max()), maximum, delta=1e-9)
            self.assert_points(rows, vil_objects, "vil")
            self.assertTrue(10 < len(rows) <= 20, "the selection ran: many significant columns, at most twenty kept")
            _, _, rows = self.points_of(tool)
            self.assert_points(rows, precip_objects, "precip")
            self.assertTrue(8 < len(rows) <= 15)
            if cells is CELLS:
                tool.line()  # the second truth's "ok"
                tool.line()  # the second scan
        self.assertEqual(20, len(state.tracked_vil_points))

    def test_storm_cells_match_the_pythons_detection_over_two_scans(self):
        """The Python's StormCellTracker detects cells correctly but then deletes
        every new cell in the same call that created it (its "remove unmatched"
        pass never marks the cells it just added), so its get_active_cells() is
        always empty. The port keeps them; this compares the detection, the
        intensity and the vertical development estimate with the Python's own
        functions, and the tracking with the rules the Python wrote down."""
        from FMOFP.Systems.radarManagement.radar_messaging.stormCellTracking import StormCellTracker
        moved = [(az, rng + 3.0, size, dbz, entry) for az, rng, size, dbz, entry in CELLS]
        tool = Tool()
        tool.send(truth_command(CELLS, AREA), "mode SURVEILLANCE", "texture 0 0", "scan 60", "cells",
                  truth_command(moved, AREA), "scan 120", "cells")
        tool.run()
        for _ in range(4):
            tool.line()
        tracker = StormCellTracker()
        previous = []
        for cells, now in ((CELLS, 60 * 0.02), (moved, 120 * 0.02)):
            composite = python_volume(cells, AREA, VCP["SURVEILLANCE"]).max(axis=1)
            detected = tracker._detect_cells(composite)
            tracker.process_radar_data(composite, scan_elevation=0.5, timestamp=now)
            self.assertEqual([], tracker.get_active_cells(), "the Python keeps nothing (see the docstring)")
            rows = tool.block()
            self.assertEqual(len(detected), len(rows))
            self.assertGreaterEqual(len(rows), 3)
            current = []
            for row, cell in zip(rows, detected):
                label = f"cell {row[0]} at {now} s"
                center_r, center_az = cell["position"]
                self.assertAlmostEqual(center_az, float(row[1]), delta=1e-9, msg=label)
                self.assertAlmostEqual(center_r, float(row[2]), delta=1e-9, msg=label)
                self.assertAlmostEqual(cell["reflectivity"], float(row[3]), delta=1e-9, msg=label)
                self.assertAlmostEqual(cell["size"], float(row[4]), delta=1e-12, msg=label)
                self.assertAlmostEqual(tracker._calculate_intensity(cell["reflectivity"]), float(row[5]), delta=1e-12,
                                       msg=label)
                top = cell["size"] * 1852 * math.tan(math.radians(0.5))
                self.assertAlmostEqual(top, float(row[8]), delta=1e-9, msg=label)
                # Tracking: the first earlier cell within five bins keeps its id and gets a velocity.
                match = next((p for p in previous if math.hypot(center_r - p["r"], center_az - p["az"]) <= 5.0), None)
                if match is None:
                    self.assertEqual(0.0, float(row[6]), label)
                    self.assertEqual(0.0, float(row[7]), label)
                    self.assertAlmostEqual(top / SCAN_INTERVAL_S, float(row[9]), delta=1e-6, msg=label)
                else:
                    self.assertEqual(match["id"], int(row[0]), label)
                    self.assertAlmostEqual((center_az - match["az"]) / SCAN_INTERVAL_S, float(row[6]), delta=1e-9, msg=label)
                    self.assertAlmostEqual((center_r - match["r"]) / SCAN_INTERVAL_S, float(row[7]), delta=1e-9, msg=label)
                    self.assertAlmostEqual((top - match["top"]) / SCAN_INTERVAL_S, float(row[9]), delta=1e-6, msg=label)
                    previous.remove(match)
                current.append(dict(id=int(row[0]), az=center_az, r=center_r, top=top))
            previous = current
            if cells is CELLS:
                tool.line()
                tool.line()
        self.assertTrue(any(float(row[7]) > 0 for row in rows), "the cells moved out in range")

    def test_precipitation_areas_match(self):
        from FMOFP.Systems.radarManagement.radar_messaging.precipitation_analysis import PrecipitationAnalyzer
        tool = self.scan("MAPPING", extra=("areas",))
        rows = tool.block()
        analyzer = PrecipitationAnalyzer()
        composite = self.volume("MAPPING").max(axis=1)
        self.assertTrue(analyzer.process_radar_data(composite, scan_elevation=0.5, timestamp=1.2))
        theirs = analyzer.get_precipitation_data()
        self.assertEqual(len(theirs), len(rows))
        self.assertGreaterEqual(len(rows), 3)
        for row, area in zip(rows, theirs):
            self.assertAlmostEqual(area.position[1], float(row[0]), delta=1e-9)
            self.assertAlmostEqual(area.position[0], float(row[1]), delta=1e-9)
            self.assertAlmostEqual(area.reflectivity, float(row[2]), delta=1e-9)
            self.assertAlmostEqual(area.rate, float(row[3]), delta=1e-9 * max(1.0, area.rate))
            self.assertEqual(TYPE_NAMES[area.type], PrecipType[row[4]])
            self.assertAlmostEqual(area.intensity, float(row[5]), delta=1e-12)

    def test_wind_shear_events_match(self):
        from FMOFP.Systems.radarManagement.weather.weather_processor import WindShearProcessor
        tool = self.scan("WINDSHEAR", extra=("windshear",))
        rows = tool.block()
        events = WindShearProcessor().process(self.volume("WINDSHEAR"), VCP["WINDSHEAR"], scan_range_km=200.0)
        self.assertEqual(len(events), len(rows))
        self.assertGreater(len(rows), 0)
        range_step_nm = (200.0 * 0.539957) / 199
        severities = {"LOW": 1, "MODERATE": 2, "SEVERE": 3}
        for row, e in zip(rows, events):
            az, r = int(row[0]), int(row[1])
            self.assertAlmostEqual(e.position_nm[0], float(az), delta=1e-9)
            self.assertAlmostEqual(e.position_nm[1], r * range_step_nm, delta=1e-9)
            self.assertAlmostEqual(e.shear_knots, float(row[2]), delta=1e-9)
            self.assertAlmostEqual(e.divergence, float(row[3]), delta=1e-12)
            self.assertEqual(e.is_microburst, row[4] == "1")
            self.assertEqual(severities[e.severity], severities[row[5]])
            self.assertAlmostEqual(e.altitude_ft, float(row[6]), delta=1e-9)

    def test_turbulence_cells_match(self):
        """The EDR proxy falls with the cube root of the range in metres, so only
        returns within a few kilometres ever reach 0.1: a sharp cell close in."""
        from FMOFP.Systems.radarManagement.weather.weather_processor import TurbulenceProcessor
        near = CELLS + [(10.0, 3.0, 3.0, 65.0, 0)]
        tool = Tool()
        tool.send(truth_command(near, AREA), "mode TURBULENCE", "texture 0 0", "scan 60", "turbulence")
        tool.run()
        for _ in range(4):
            tool.line()
        rows = tool.block()
        cells = TurbulenceProcessor().process(python_volume(near, AREA, VCP["TURBULENCE"]), VCP["TURBULENCE"],
                                              scan_range_km=200.0)
        self.assertEqual(len(cells), len(rows))
        self.assertGreater(len(rows), 0)
        range_step_nm = (200.0 * 0.539957) / 199
        for row, c in zip(rows, cells):
            az, r = int(row[0]), int(row[1])
            self.assertAlmostEqual(c.position_nm[0], float(az), delta=1e-9)
            self.assertAlmostEqual(c.position_nm[1], r * range_step_nm, delta=1e-9)
            self.assertAlmostEqual(c.intensity, float(row[2]), delta=1e-9)
            self.assertAlmostEqual(c.spectrum_width, float(row[3]), delta=1e-9)
            self.assertEqual(c.category, row[4])
            self.assertAlmostEqual(c.altitude_ft, float(row[5]), delta=1e-9)

    def test_the_products_block_carries_what_the_scan_found(self):
        tool = self.scan("SURVEILLANCE", extra=("points vil", "points precip", "summary", "payload"))
        _, _, vil_rows = self.points_of(tool)
        _, _, precip_rows = self.points_of(tool)
        summary = tool.line()
        payload = tool.line()
        self.assertEqual("payload", payload[0])
        words = [int(w, 16) for w in payload[2:]]
        self.assertEqual(int(payload[1]), len(words))
        status, products = b.RdrWxProducts.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual(WeatherMode.SURVEILLANCE, products.mode)
        self.assertEqual(1, products.scan_id)
        self.assertEqual(len(vil_rows), len(products.vil))
        self.assertEqual(len(precip_rows), len(products.precip))
        self.assertEqual(int(summary[3]), len(products.cells) if int(summary[3]) <= 12 else 12)
        for row, rec in zip(vil_rows, products.vil):
            self.assertAlmostEqual(int(row[0]) * 8.0, rec.azimuth, delta=0.01)
            self.assertAlmostEqual(int(row[1]) * 8.0, rec.range, delta=0.02)
            self.assertAlmostEqual(float(row[2]), rec.value, delta=0.13)
            self.assertEqual(int(row[4]), rec.layer_count)
        self.assertEqual(65.0, products.max_reflectivity)
        sizes = [c.size for c in products.cells]
        self.assertEqual(sorted(sizes, reverse=True), sizes, "the largest cells are reported first")


if __name__ == "__main__":
    unittest.main()
