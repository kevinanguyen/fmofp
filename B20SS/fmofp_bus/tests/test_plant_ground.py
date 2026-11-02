"""The plant's ground scene for the synthetic aperture radar (fmofp_plant.ground): the training range, its movers,
its block, and the plant's hold on it."""

import math
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd.enums import GroundFeature
from fmofp_bus.status import Status
from fmofp_plant.ground import MAX_AREAS, MAX_LINES, MAX_POINTS, Area, GroundScene, Line, Point, to_latlon
from fmofp_plant.plant import PlantModel
from fmofp_plant.traffic import ownship_enu


def metres(latitude, longitude):
    """Back from the block's degrees to the plant's east and north of home."""
    east, north, _ = ownship_enu(latitude, longitude, 0.0)
    return east, north


class GroundSceneTest(unittest.TestCase):
    def test_the_training_range_fits_the_block(self):
        scene = GroundScene()
        block = scene.message()
        status, words = block.encode()
        self.assertEqual(Status.OK, status)
        self.assertLessEqual(len(words), b.PlantGround.MAX_PAYLOAD_WORDS)
        self.assertEqual((8, 3, 4), (len(block.point), len(block.line), len(block.area)))
        self.assertTrue(len(block.point) <= MAX_POINTS and len(block.line) <= MAX_LINES and len(block.area) <= MAX_AREAS)
        kinds = [p.kind for p in block.point]
        self.assertEqual(1, kinds.count(GroundFeature.SHIP))
        self.assertEqual(2, kinds.count(GroundFeature.VEHICLE))
        self.assertEqual(GroundFeature.LAKE, block.area[-1].kind, "the lake paints last, over the forest's edge")
        status, back = b.PlantGround.decode(words)
        self.assertEqual(Status.OK, status)
        # The road beside the northbound track: 2.5 km east of home, from 2 km behind to the lake's shore.
        road = back.line[0]
        self.assertEqual(GroundFeature.ROAD, road.kind)
        east_1, north_1 = metres(road.latitude_1, road.longitude_1)
        east_2, north_2 = metres(road.latitude_2, road.longitude_2)
        self.assertAlmostEqual(2500.0, east_1, delta=0.01)
        self.assertAlmostEqual(-2000.0, north_1, delta=0.01)
        self.assertAlmostEqual(2500.0, east_2, delta=0.01)
        self.assertAlmostEqual(13500.0, north_2, delta=0.01)
        self.assertAlmostEqual(12.0, road.width)
        self.assertEqual(20, road.brightness)
        lake = back.area[-1]
        self.assertAlmostEqual(1200.0, lake.radius)
        self.assertEqual(6, lake.brightness)

    def test_degrees_and_metres_round_trip(self):
        for east, north in ((2500.0, 15500.0), (-3000.0, 5000.0), (0.0, 0.0)):
            lat, lon = to_latlon(east, north)
            back_east, back_north = metres(lat, lon)
            self.assertAlmostEqual(east, back_east, delta=1e-6)
            self.assertAlmostEqual(north, back_north, delta=1e-6)
        lat, lon = to_latlon(1000.0, 1000.0, 45.0, 10.0)
        self.assertAlmostEqual(45.0 + math.degrees(1000.0 / 6_371_000.0), lat)
        self.assertAlmostEqual(10.0 + math.degrees(1000.0 / (6_371_000.0 * math.cos(math.radians(45.0)))), lon)

    def test_the_movers_shuttle_along_their_paths(self):
        scene = GroundScene()
        ship = scene.points[-1]
        truck = scene.points[-3]
        self.assertEqual(GroundFeature.SHIP, ship.kind)
        self.assertEqual((2200.0, 14800.0), ship.position(0.0))
        scene.update(100.0)
        self.assertEqual((2200.0, 15300.0), ship.position(scene.t), "5 m/s north")
        self.assertEqual((2500.0, 4500.0), truck.position(scene.t), "15 m/s south along the road")
        scene.update(180.0)
        self.assertAlmostEqual(16200.0, ship.position(scene.t)[1], "the far end, 1400 m in 280 s")
        scene.update(20.0)
        self.assertAlmostEqual(16100.0, ship.position(scene.t)[1], "and back")
        positions = scene.positions()
        self.assertEqual(8, len(positions))
        self.assertEqual((2300.0, 4700.0), positions[0], "a building stays put")
        block = scene.message()
        east, north = metres(block.point[-1].latitude, block.point[-1].longitude)
        self.assertAlmostEqual(2200.0, east, delta=0.01)
        self.assertAlmostEqual(16100.0, north, delta=0.01)

    def test_a_custom_scene_bare_ground_and_the_plants_hold_on_it(self):
        custom = GroundScene(points=[Point(GroundFeature.SHIP, 190, 90.0, (2154.0, 1760.0), (2154.0, 2200.0), 5.0)],
                             areas=[Area(GroundFeature.LAKE, 6, 500.0, (1900.0, 1850.0))],
                             lines=[Line(GroundFeature.ROAD, 20, 12.0, (2500.0, -2000.0), (2500.0, 30000.0))])
        block = custom.message()
        self.assertEqual((1, 1, 1), (len(block.point), len(block.line), len(block.area)))
        self.assertEqual(Status.OK, block.encode()[0])
        self.assertEqual(1, block.scene_seq)
        self.assertEqual(2, custom.message().scene_seq)
        model = PlantModel(seed=3, ground=custom)
        self.assertIs(custom, model.ground)
        model.move(1.0)
        self.assertEqual(1.0, custom.t, "the plant's walk moves the scene")
        bare = PlantModel(seed=3, ground=False)
        empty = bare.ground_message()
        self.assertEqual(([], [], []), (empty.point, empty.line, empty.area))
        self.assertEqual(1, empty.scene_seq)
        default = PlantModel(seed=3)
        self.assertEqual(8, len(default.ground_message().point))
        self.assertEqual((0.0, 0.0), default.ground.home)


if __name__ == "__main__":
    unittest.main()
