"""The ground the synthetic aperture radar images: the plant's training range (PLANT_GROUND, ICD v1.13).

sar_radar.py drew its images from nothing: random bars, a disc, swaths, noise.
The port draws the line between world and sensor as it did for the weather, the
terrain and the traffic: the plant holds the ground (point scatterers, lines and
areas with the backscatter the radar would measure of them) and the radar makes
its picture of whatever lies in its strip. The range is laid out along the track
north from home that the demo flies: a road beside the track, a town with its
buildings and a crossroad, a river, forests, and a lake with a ship under way,
with vehicles on the roads. Positions go out as WGS-84 so the radar can place
them against the navigation it is given. The movers move with the plant's walk.
"""

import math

from fmofp_bus.icd.blocks import PlantGround, PlantGroundArea, PlantGroundLine, PlantGroundPoint
from fmofp_bus.icd.enums import GroundFeature

EARTH_RADIUS_M = 6_371_000.0
OPEN_GROUND = 60  # the radar's level for ground that is nothing in particular (kSarOpenGround)
MAX_POINTS = PlantGround.MAX_POINT
MAX_LINES = PlantGround.MAX_LINE
MAX_AREAS = PlantGround.MAX_AREA


def to_latlon(east_m, north_m, home_latitude=0.0, home_longitude=0.0):
    """Degrees of a point east and north of home, the inverse of fmofp_plant.traffic.ownship_enu."""
    latitude = home_latitude + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = home_longitude + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(home_latitude))))
    return latitude, longitude


class Mover:
    """A point that shuttles between two places at a speed, back and forth."""

    __slots__ = ("start", "end", "speed")

    def __init__(self, start, end, speed):
        self.start = start
        self.end = end
        self.speed = float(speed)

    def position(self, t):
        length = math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])
        if length <= 0.0 or self.speed <= 0.0:
            return self.start
        s = math.fmod(t * self.speed, 2.0 * length)
        if s > length:
            s = 2.0 * length - s
        f = s / length
        return (self.start[0] + (self.end[0] - self.start[0]) * f, self.start[1] + (self.end[1] - self.start[1]) * f)


class Point:
    """A point scatterer: a square `size` metres on a side at `brightness`; moving when given an end and a speed."""

    __slots__ = ("kind", "brightness", "size", "mover")

    def __init__(self, kind, brightness, size, position, end=None, speed=0.0):
        self.kind = kind
        self.brightness = int(brightness)
        self.size = float(size)
        self.mover = Mover(position, position if end is None else end, speed)

    def position(self, t):
        return self.mover.position(t)

    def record(self, t, home):
        lat, lon = to_latlon(*self.position(t), *home)
        return PlantGroundPoint(kind=self.kind, brightness=self.brightness, latitude=lat, longitude=lon, size=self.size)


class Line:
    """A road or a river: a line `width` metres wide between two points."""

    __slots__ = ("kind", "brightness", "width", "start", "end")

    def __init__(self, kind, brightness, width, start, end):
        self.kind = kind
        self.brightness = int(brightness)
        self.width = float(width)
        self.start = start
        self.end = end

    def record(self, home):
        lat1, lon1 = to_latlon(*self.start, *home)
        lat2, lon2 = to_latlon(*self.end, *home)
        return PlantGroundLine(kind=self.kind, brightness=self.brightness, latitude_1=lat1, longitude_1=lon1,
                               latitude_2=lat2, longitude_2=lon2, width=self.width)


class Area:
    """A lake, a forest or a town: a disc."""

    __slots__ = ("kind", "brightness", "radius", "centre")

    def __init__(self, kind, brightness, radius, centre):
        self.kind = kind
        self.brightness = int(brightness)
        self.radius = float(radius)
        self.centre = centre

    def record(self, home):
        lat, lon = to_latlon(*self.centre, *home)
        return PlantGroundArea(kind=self.kind, brightness=self.brightness, latitude=lat, longitude=lon,
                               radius=self.radius)


def default_areas():
    """Forests, the town and the lake, in metres east and north of home. The lake comes last: areas paint in
    order, and the lake's shore wins where the forest reaches it."""
    return [Area(GroundFeature.FOREST, 110, 1800.0, (2000.0, 12500.0)),
            Area(GroundFeature.URBAN, 150, 1500.0, (2600.0, 5000.0)),
            Area(GroundFeature.FOREST, 100, 1800.0, (6500.0, 8000.0)),
            Area(GroundFeature.LAKE, 6, 1200.0, (2150.0, 15500.0))]


def default_lines():
    """The road beside the track (2.5 km east of it, up to the lake's shore), the crossroad through the town, and
    the river."""
    return [Line(GroundFeature.ROAD, 20, 12.0, (2500.0, -2000.0), (2500.0, 13500.0)),
            Line(GroundFeature.ROAD, 20, 12.0, (-3000.0, 5000.0), (9000.0, 5000.0)),
            Line(GroundFeature.RIVER, 8, 60.0, (-1000.0, 9000.0), (9000.0, 11000.0))]


def default_points():
    """The town's buildings, a truck on each road and the ship on the lake."""
    return [Point(GroundFeature.BUILDING, 230, 40.0, (2300.0, 4700.0)),
            Point(GroundFeature.BUILDING, 220, 50.0, (2800.0, 5200.0)),
            Point(GroundFeature.BUILDING, 240, 30.0, (2250.0, 5300.0)),
            Point(GroundFeature.BUILDING, 225, 35.0, (2900.0, 4600.0)),
            Point(GroundFeature.BUILDING, 235, 60.0, (2600.0, 5000.0)),
            Point(GroundFeature.VEHICLE, 200, 6.0, (2500.0, 6000.0), (2500.0, -2000.0), 15.0),
            Point(GroundFeature.VEHICLE, 200, 8.0, (-3000.0, 5000.0), (9000.0, 5000.0), 20.0),
            Point(GroundFeature.SHIP, 190, 90.0, (2200.0, 14800.0), (2200.0, 16200.0), 5.0)]


class GroundScene:
    """The scene: update() every walk step moves the movers, message() is PLANT_GROUND."""

    def __init__(self, points=None, lines=None, areas=None, home_latitude=0.0, home_longitude=0.0):
        default = points is None and lines is None and areas is None
        self.points = list(default_points() if default else (points or []))[:MAX_POINTS]
        self.lines = list(default_lines() if default else (lines or []))[:MAX_LINES]
        self.areas = list(default_areas() if default else (areas or []))[:MAX_AREAS]
        self.home = (float(home_latitude), float(home_longitude))
        self.t = 0.0
        self.seq = 0

    def update(self, dt):
        self.t += dt

    def positions(self):
        """Every point's metres east and north of home now, for the tests."""
        return [p.position(self.t) for p in self.points]

    def message(self):
        self.seq = (self.seq + 1) % 256
        block = PlantGround(scene_seq=self.seq)
        block.point = [p.record(self.t, self.home) for p in self.points]
        block.line = [l.record(self.home) for l in self.lines]
        block.area = [a.record(self.home) for a in self.areas]
        return block
