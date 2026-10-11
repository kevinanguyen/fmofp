"""The navigation partition's arithmetic against the Python it was ported from.

fmofp_nav_tool puts the C++ functions on stdin/stdout. Each test feeds the same
inputs to Systems/nav/gps/globalPositionSystem.py (the receiver's solver and
coordinate conversion) or Systems/nav/dataFusion/fuseData.py (the Kalman
fusion) and compares: the original Python is the oracle.

Needs FMOFP_NAV_TOOL pointing at the binary (CI sets it).
"""

import math
import os
import subprocess
import unittest

from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import GPS_MIN_ELEVATION_DEG, PlantModel, wgs84_to_ecef


def run_tool(lines):
    """Each command's answer, as a list of tokens."""
    result = subprocess.run([os.environ["FMOFP_NAV_TOOL"]], input="\n".join(lines) + "\n",
                            capture_output=True, text=True, timeout=60, env=binary_env())
    assert result.returncode == 0, result.stderr
    return [line.split() for line in result.stdout.splitlines()]


def floats(tokens):
    assert tokens and tokens[0] != "error", tokens
    return [float(t) for t in tokens]


@needs("FMOFP_NAV_TOOL")
class NavigationOracleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from FMOFP.Systems.nav.gps.globalPositionSystem import GPSReceiver, _build_constellation
        from FMOFP.Systems.nav.dataFusion.fuseData import NavDataFusion
        cls.GPSReceiver = GPSReceiver
        cls.NavDataFusion = NavDataFusion
        cls.sats = _build_constellation()

    def test_constellation_is_the_pythons(self):
        rows = run_tool([f"sat {sat.id}" for sat in self.sats])
        for sat, row in zip(self.sats, rows):
            x, y, z, bias = floats(row)
            for mine, theirs in zip((x, y, z), sat.position):
                self.assertAlmostEqual(theirs, mine, delta=1e-6, msg=f"satellite {sat.id}")
            self.assertAlmostEqual(sat.clock_bias, bias, delta=1e-21)

    def test_visibility_decision_matches(self):
        receiver = self.GPSReceiver()
        truth = wgs84_to_ecef(35.4147, -97.3866, 30000.0)
        receiver._true_position_ecef = truth
        rows = run_tool([f"elev {truth[0]!r} {truth[1]!r} {truth[2]!r} {sat.position[0]!r} {sat.position[1]!r} "
                         f"{sat.position[2]!r}" for sat in self.sats])
        for sat, row in zip(self.sats, rows):
            self.assertEqual(receiver._is_visible(sat), floats(row)[0] >= GPS_MIN_ELEVATION_DEG, f"satellite {sat.id}")

    def solve_in_python(self, ranges, start, clock_bias):
        """GPSReceiver._wls_solve on given pseudoranges, from a given start.

        The Python started its first solve from the simulation's true
        position; the C++ starts from a closed-form acquisition, and the test
        gives the Python the same start. (From the Earth's centre both raise
        or return "singular": the four satellites seen from home are coplanar.)
        """
        receiver = self.GPSReceiver()
        receiver._true_position_ecef = (0.0, 0.0, 0.0)
        receiver._last_known_ecef = tuple(start)
        receiver._clock_bias = clock_bias
        receiver._pseudorange = lambda sat: ranges[sat.id]
        visible = [sat for sat in self.sats if sat.id in ranges]
        (x, y, z), bias = receiver._wls_solve(visible)
        return (x, y, z), bias

    def test_wls_solve_matches_the_python_receiver(self):
        model = PlantModel(seed=7)
        model.altitude = 30000.0
        gps = model.gps()  # five satellites from home, 3 m noise, 1 us of receiver clock bias
        ranges = {sid: getattr(gps, f"pseudorange_{sid}") for sid in range(1, 13) if gps.visible_mask & (1 << (sid - 1))}
        self.assertGreaterEqual(len(ranges), 4)
        pairs = " ".join(f"{sid} {r!r}" for sid, r in ranges.items())

        # First epoch: the C++ acquires a start in closed form; both iterate from it.
        sx, sy, sz, sbias = floats(run_tool([f"bancroft {len(ranges)} {pairs}"])[0])
        (px, py, pz), pbias = self.solve_in_python(ranges, (sx, sy, sz), sbias)
        cx, cy, cz, cbias, iterations, converged = floats(
            run_tool([f"wls {sx!r} {sy!r} {sz!r} {sbias!r} {len(ranges)} {pairs}"])[0])
        for mine, theirs in ((cx, px), (cy, py), (cz, pz)):
            self.assertAlmostEqual(theirs, mine, delta=0.02)  # metres; the stop rule is 0.01 m
        self.assertAlmostEqual(pbias, cbias, delta=1e-10)
        self.assertEqual(1.0, converged)
        self.assertLessEqual(iterations, 3)
        self.assertAlmostEqual(1e-6, cbias, delta=2e-8)  # the plant's true bias, within the noise
        # The acquisition itself is already within the noise of the truth.
        truth = wgs84_to_ecef(model.latitude, model.longitude, model.altitude)
        self.assertLess(math.dist(truth, (sx, sy, sz)), 60.0)

        # Second epoch: both start from the first fix.
        gps = model.gps()
        ranges = {sid: getattr(gps, f"pseudorange_{sid}") for sid in range(1, 13) if gps.visible_mask & (1 << (sid - 1))}
        pairs = " ".join(f"{sid} {r!r}" for sid, r in ranges.items())
        (qx, qy, qz), qbias = self.solve_in_python(ranges, (px, py, pz), pbias)
        dx, dy, dz, dbias, iterations, _ = floats(
            run_tool([f"wls {px!r} {py!r} {pz!r} {pbias!r} {len(ranges)} {pairs}"])[0])
        for mine, theirs in ((dx, qx), (dy, qy), (dz, qz)):
            self.assertAlmostEqual(theirs, mine, delta=0.02)
        self.assertAlmostEqual(qbias, dbias, delta=1e-10)
        self.assertLessEqual(iterations, 3)

    def test_the_coplanar_four_are_singular_from_the_centre_for_both(self):
        """Why the acquisition exists: from the Python INS's starting point the
        constellation shows four coplanar satellites, and from the Earth's
        centre the Python's own iteration raises on them, as the C++ reports."""
        model = PlantModel(seed=1, latitude=35.4147, longitude=-97.3866, gps_noise_m=0.0)
        model.altitude = 30000.0
        gps = model.gps()
        ranges = {sid: getattr(gps, f"pseudorange_{sid}") for sid in range(1, 13) if gps.visible_mask & (1 << (sid - 1))}
        self.assertEqual(4, len(ranges))
        pairs = " ".join(f"{sid} {r!r}" for sid, r in ranges.items())
        with self.assertRaises(ValueError):
            self.solve_in_python(ranges, (0.0, 0.0, 0.0), 0.0)
        self.assertEqual(["error", "malformed"], run_tool([f"wls 0 0 0 0 {len(ranges)} {pairs}"])[0])
        # The closed form has no such trouble, and seeds the iteration to convergence.
        sx, sy, sz, sbias = floats(run_tool([f"bancroft {len(ranges)} {pairs}"])[0])
        truth = wgs84_to_ecef(35.4147, -97.3866, 30000.0)
        self.assertLess(math.dist(truth, (sx, sy, sz)), 0.05)
        (px, py, pz), pbias = self.solve_in_python(ranges, (sx, sy, sz), sbias)
        self.assertLess(math.dist(truth, (px, py, pz)), 0.02)

    def test_ecef_to_wgs84_matches(self):
        points = [wgs84_to_ecef(35.4147, -97.3866, 30000.0), wgs84_to_ecef(0.0, 0.0, 0.0),
                  wgs84_to_ecef(-33.9, 151.2, 1200.0), wgs84_to_ecef(89.5, 10.0, 500.0)]
        rows = run_tool([f"wgs84 {x!r} {y!r} {z!r}" for x, y, z in points])
        for (x, y, z), row in zip(points, rows):
            lat, lon, alt = self.GPSReceiver._ecef_to_wgs84(x, y, z)
            clat, clon, calt = floats(row)
            self.assertAlmostEqual(lat, clat, delta=1e-9)
            self.assertAlmostEqual(lon, clon, delta=1e-9)
            self.assertAlmostEqual(alt, calt, delta=1e-5)

    def test_fusion_matches_the_python_filter(self):
        fusion = self.NavDataFusion()
        script = ["fusion"]
        for _ in range(10):
            fusion.predict(0.02, 1.0e-5, 2.0e-5, 3.0, 0.0)
            script.append("predict 0.02 1e-05 2e-05 3.0")
        fusion.update_gps(35.4147, -97.3866, 29990.0)
        script.append("update 35.4147 -97.3866 29990.0")
        for _ in range(50):
            fusion.predict(0.02, -4.0e-6, 2.5e-5, -1.5, 0.0)
            script.append("predict 0.02 -4e-06 2.5e-05 -1.5")
        fusion.update_gps(35.4149, -97.3860, 30010.0)
        script.append("update 35.4149 -97.386 30010.0")
        script.append("get")
        lat, lon, alt, count = floats(run_tool(script)[-1])
        plat, plon, palt, _ = fusion.get_fused_position()
        self.assertAlmostEqual(plat, lat, delta=1e-12)
        self.assertAlmostEqual(plon, lon, delta=1e-12)
        self.assertAlmostEqual(palt, alt, delta=1e-8)
        self.assertEqual(fusion.get_gps_fix_count(), int(count))


if __name__ == "__main__":
    unittest.main()
