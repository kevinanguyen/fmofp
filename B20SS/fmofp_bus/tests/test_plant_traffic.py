"""The plant's air traffic: the Python radars' generators and kinematics, flown by the plant and measured for the
radars (fmofp_plant.traffic)."""

import math
import random
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.status import Status
from fmofp_plant.traffic import AEWC, MAX_TARGETS, TARGETING, Target, TrafficWorld, ownship_enu, ownship_velocity


class TrafficWorldTest(unittest.TestCase):
    def setUp(self):
        self.world = TrafficWorld(random.Random(7))
        self.ownship = (1000.0, 2000.0, 3000.0)

    def test_the_populations_are_the_two_generators(self):
        targets = self.world.populate(self.ownship)
        self.assertEqual(MAX_TARGETS, len(targets))
        self.assertEqual([TARGETING] * 5 + [AEWC] * 7, [t.population for t in targets])
        self.assertEqual(list(range(1, 13)), [t.target_id for t in targets])
        for t in targets:
            p = t.population
            self.assertTrue(p.min_range_m <= self.world.range_m(t, self.ownship) <= p.max_range_m, p.name)
            speed = math.sqrt(sum(v * v for v in t.velocity))
            self.assertTrue(p.min_speed - 1e-9 <= speed <= p.max_speed + 1e-9, p.name)
            dx, dy, dz = self.world.relative(t, self.ownship)
            self.assertLessEqual(abs(math.atan2(dz, math.hypot(dx, dy))), p.elevation_rad + 1e-9)
            accel = math.sqrt(sum(a * a for a in t.acceleration))
            self.assertLess(accel, p.max_accel * 10.0, "a_mag times three gaussians")
            if t.stealth:
                self.assertIs(AEWC, p, "only the AEWC's generator made stealth aircraft")
                self.assertTrue(0.01 <= t.rcs <= 0.1)
            else:
                self.assertTrue(p.rcs_range[0] <= t.rcs <= p.rcs_range[1])

    def test_the_draws_follow_the_pythons_distributions(self):
        stealth = 0
        for _ in range(300):
            t = self.world.spawn(AEWC, self.ownship)
            stealth += t.stealth
            self.assertTrue(5000.0 <= self.world.range_m(t, self.ownship) <= 400000.0)
        self.assertTrue(60 <= stealth <= 120, f"three in ten are stealth: {stealth} of 300")
        for _ in range(100):
            t = self.world.spawn(TARGETING, self.ownship)
            self.assertFalse(t.stealth)
            self.assertTrue(1000.0 <= self.world.range_m(t, self.ownship) <= 100000.0)
            self.assertTrue(1.0 <= t.rcs <= 10.0)

    def test_the_kinematics_are_the_pythons(self):
        # _update_targets: v' = v + a dt; p' = p + (v + v') / 2 dt.
        self.world.targets = [Target(1, TARGETING, [10000.0, 0.0, 0.0], [100.0, 0.0, 0.0], [2.0, -4.0, 0.0], 2.0, False)]
        self.world.update(0.5, (0.0, 0.0, 0.0))
        t = self.world.targets[0]
        self.assertEqual([101.0, -2.0, 0.0], t.velocity)
        self.assertEqual([10000.0 + 0.5 * (100.0 + 101.0) * 0.5, 0.5 * (0.0 - 2.0) * 0.5, 0.0], t.position)
        self.assertEqual(1, t.target_id, "within range: the same aircraft")

    def test_an_aircraft_beyond_its_range_is_replaced(self):
        self.world.targets = [Target(1, TARGETING, [99000.0, 0.0, 0.0], [300.0, 0.0, 0.0], [0.0, 0.0, 0.0], 2.0, False)]
        self.world.next_id = 2
        self.world.update(10.0, (0.0, 0.0, 0.0))  # 3 km further: beyond the targeting radar's 100 km
        t = self.world.targets[0]
        self.assertEqual(2, t.target_id)
        self.assertIs(TARGETING, t.population)
        self.assertTrue(1000.0 <= self.world.range_m(t, (0.0, 0.0, 0.0)) <= 100000.0)

    def test_the_message_measures_from_the_aircraft(self):
        self.world.targets = [Target(9, TARGETING, [1000.0 + 3000.0, 2000.0 + 4000.0, 3000.0 + 1000.0],
                                     [50.0, 60.0, 7.0], [0.0, 0.0, 0.0], 2.5, False)]
        block = self.world.message(self.ownship, (10.0, 20.0, 0.0))
        self.assertEqual(1, block.traffic_seq)
        self.assertEqual(1, len(block.target))
        r = block.target[0]
        self.assertEqual(9, r.target_id)
        self.assertAlmostEqual(math.degrees(math.atan2(3000.0, 4000.0)), r.azimuth, places=9)
        self.assertAlmostEqual(math.sqrt(3000.0 ** 2 + 4000.0 ** 2 + 1000.0 ** 2) / 1000.0, r.range, places=9)
        self.assertAlmostEqual(math.degrees(math.atan2(1000.0, 5000.0)), r.elevation, places=9)
        self.assertEqual((40.0, 40.0, 7.0), (r.velocity_east, r.velocity_north, r.velocity_up), "relative to the aircraft")
        self.assertEqual(2.5, r.rcs)
        status, words = block.encode()
        self.assertEqual(Status.OK, status)
        status, back = b.PlantTraffic.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual(1, len(back.target))
        back_r = back.target[0]
        self.assertAlmostEqual(r.azimuth, back_r.azimuth, delta=360.0 / 65536)
        self.assertAlmostEqual(r.range, back_r.range, delta=1.0 / 64)
        self.assertAlmostEqual(r.elevation, back_r.elevation, delta=180.0 / 32768)
        self.assertAlmostEqual(r.rcs, back_r.rcs, delta=0.01)
        # The block carries at most twelve, and the sequence counts round.
        self.world.populate(self.ownship)
        self.world.targets.append(self.world.spawn(AEWC, self.ownship))
        self.assertEqual(MAX_TARGETS, len(self.world.message(self.ownship, (0.0, 0.0, 0.0)).target))
        self.world.seq = 255
        self.assertEqual(0, self.world.message(self.ownship, (0.0, 0.0, 0.0)).traffic_seq)

    def test_the_aircrafts_own_frame(self):
        east, north, up = ownship_enu(0.001, 0.002, 10000.0)
        self.assertAlmostEqual(math.radians(0.002) * 6371000.0, east, places=6)
        self.assertAlmostEqual(math.radians(0.001) * 6371000.0, north, places=6)
        self.assertAlmostEqual(3048.0, up)
        east, north, up = ownship_enu(10.001, 20.0, 0.0, home_latitude=10.0, home_longitude=20.0)
        self.assertAlmostEqual(0.0, east)
        self.assertAlmostEqual(math.radians(0.001) * 6371000.0, north, places=6)
        ve, vn, vu = ownship_velocity(450.0, 90.0, 600.0)
        self.assertAlmostEqual(450.0 * 0.514444, ve, places=9)
        self.assertAlmostEqual(0.0, vn, places=9)
        self.assertAlmostEqual(3.048, vu, places=9)


if __name__ == "__main__":
    unittest.main()
