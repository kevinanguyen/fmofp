"""The plant's weather world: repeatable, within the ICD's limits, and aged as the Python aged it."""

import random
import unittest

from fmofp_bus.icd.messages import PlantWeather
from fmofp_bus.status import Status
from fmofp_plant.plant import PlantModel
from fmofp_plant.weather import WIND, StormCell, WeatherWorld


class WeatherWorldTest(unittest.TestCase):
    def test_initial_world_and_message(self):
        world = WeatherWorld(random.Random(3))
        self.assertTrue(3 <= len(world.cells) <= 8)
        self.assertEqual(1, len(world.areas))
        for cell in world.cells:
            self.assertTrue(0 <= cell.position[0] < 360)
            self.assertTrue(20 <= cell.position[1] <= 180)
            self.assertTrue(5 <= cell.size <= 20)
            self.assertTrue(35 <= cell.intensity <= 65)
            self.assertFalse(cell.entry_profile)
        m = world.message()
        status, words = m.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual(32, len(words))
        self.assertEqual(len(world.cells), m.cell_count)
        self.assertEqual(1, m.area_count)
        again = PlantWeather.decode(words)[1]
        self.assertAlmostEqual(world.cells[0].position[0], again.cell_1_azimuth, delta=0.01)
        self.assertAlmostEqual(world.cells[0].size, again.cell_1_size, delta=0.25)
        self.assertEqual(2, world.message().weather_seq)

    def test_same_seed_same_world(self):
        a, b = WeatherWorld(random.Random(11)), WeatherWorld(random.Random(11))
        for _ in range(5):
            a.update(10.0)
            b.update(10.0)
        self.assertEqual(a.message().encode(), b.message().encode())

    def test_cells_drift_and_age_like_the_python(self):
        """StormCell.update, step for step: growth, then the mature plateau, then decay to nothing."""
        rng = random.Random(1)
        cell = StormCell(rng, (90.0, 100.0), 10.0, 40.0, False)
        cell.lifetime, cell.growth_phase, cell.mature_phase = 1000.0, 0.2, 0.5
        self.assertTrue(cell.update(rng, 100.0, WIND["altitude"]))
        self.assertAlmostEqual(50.0, cell.intensity)  # half way through the growth phase: 35 + 30 * 0.5
        # The wind at 290 degrees, 25 knots, for 100 s moves it 1.286 km outward.
        self.assertAlmostEqual(100.0 + 25 * 1.852 / 3600 * 100 * 0.3420201433, cell.position[1], places=6)
        self.assertTrue(cell.update(rng, 300.0, WIND["altitude"]))
        self.assertTrue(60 <= cell.intensity <= 70)
        self.assertFalse(cell.update(rng, 700.0, WIND["altitude"]), "past its lifetime")

    def test_the_sky_is_never_empty(self):
        world = WeatherWorld(random.Random(2), populate=False)
        world.update(1.0)
        self.assertEqual(1, len(world.cells))
        self.assertEqual(1, len(world.areas))

    def test_plant_publishes_it(self):
        model = PlantModel(seed=5)
        m = model.weather_message()
        self.assertGreaterEqual(m.cell_count, 3)
        model.move(0.1)
        self.assertEqual(2, model.weather_message().weather_seq)
        quiet = PlantModel(seed=5, weather=False)
        self.assertEqual(0, quiet.weather_message().cell_count)


if __name__ == "__main__":
    unittest.main()
