"""The plant's RF environment: CommsService._simulate's draws as the world, on PLANT_RF. No sockets."""

import random
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.status import Status
from fmofp_plant.plant import PlantModel
from fmofp_plant.rf import RfWorld


class RfWorldTest(unittest.TestCase):
    def test_the_field_walks_from_zero_within_its_bounds(self):
        rf = RfWorld(random.Random(3))
        self.assertEqual(0.0, rf.radio_field)
        for _ in range(500):
            before = rf.radio_field
            rf.step()
            self.assertLessEqual(abs(rf.radio_field - before), 3.0 + 1e-9)
            self.assertGreaterEqual(rf.radio_field, 0.0)
            self.assertLessEqual(rf.radio_field, 100.0)
            self.assertIn(rf.packets_in, (0, 1, 2))
            self.assertIn(rf.packets_out, (0, 1, 2))
            self.assertGreaterEqual(rf.link_error_rate, 0.0)

    def test_a_satellite_comes_into_view_with_the_pythons_numbers_and_stays(self):
        rf = RfWorld(random.Random(7), satellite_in_view=True)
        self.assertTrue(rf.satellite_available)
        self.assertTrue(40.0 <= rf.satellite_signal <= 80.0)
        self.assertTrue(50.0 <= rf.satellite_data_rate <= 200.0)
        self.assertTrue(200.0 <= rf.satellite_latency <= 600.0)
        self.assertTrue(1 <= rf.satellite_id <= 12)
        for _ in range(200):
            rf.step()
            self.assertTrue(rf.satellite_available, "the Python's connection never dropped")
            self.assertTrue(0.0 <= rf.satellite_signal <= 100.0)
            self.assertGreaterEqual(rf.satellite_data_rate, 0.0)
            self.assertGreaterEqual(rf.satellite_latency, 200.0)
        # By chance: a 1 % draw a second, so over 2000 seconds it is all but certain.
        chance = RfWorld(random.Random(11))
        seconds = 0
        while not chance.satellite_available and seconds < 2000:
            chance.step()
            seconds += 1
        self.assertTrue(chance.satellite_available)
        self.assertGreater(seconds, 0)

    def test_a_scripted_satellite_appears_on_time(self):
        rf = RfWorld(random.Random(1), satellite_after_s=3)
        rf.step()
        rf.step()
        self.assertFalse(rf.satellite_available)
        status, words = rf.message().encode()
        self.assertEqual(Status.OK, status)
        status, back = m.PlantRf.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual((False, 0, 0.0, 0.0, 0.0), (back.satellite_available, back.satellite_id, back.satellite_signal,
                                                     back.satellite_data_rate, back.satellite_latency))
        rf.step()
        self.assertTrue(rf.satellite_available)
        status, back = m.PlantRf.decode(rf.message().encode()[1])
        self.assertEqual(Status.OK, status)
        self.assertTrue(back.satellite_available)
        self.assertEqual(rf.satellite_id, back.satellite_id)
        self.assertAlmostEqual(rf.satellite_latency, back.satellite_latency, delta=0.5)
        self.assertAlmostEqual(rf.radio_field, back.radio_field, delta=0.005)


class PlantRfTest(unittest.TestCase):
    def test_the_model_carries_an_rf_world_of_its_own_seed(self):
        a = PlantModel(seed=5)
        b = PlantModel(seed=5)
        for _ in range(20):
            a.rf.step()
            b.rf.step()
        self.assertEqual(a.rf.radio_field, b.rf.radio_field, "a seeded run repeats")
        status, words = a.rf_message().encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual(m.PlantRf.WORDS, len(words))
        silent = PlantModel(seed=5, rf=False)
        self.assertIsNone(silent.rf)
        self.assertEqual(m.PlantRf(), silent.rf_message())
        scripted = PlantModel(seed=5, satellite_after_s=1)
        scripted.rf.step()
        self.assertTrue(scripted.rf.satellite_available)


if __name__ == "__main__":
    unittest.main()
