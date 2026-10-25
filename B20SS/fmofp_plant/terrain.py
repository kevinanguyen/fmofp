"""The terrain the terrain-following radar flies over, and the profile the plant publishes.

tfr_radar.py invented its terrain: 1000 + 500 sin(d / 1000) + 200 sin(d / 300) metres
plus Gaussian noise of 50 m at each of 100 samples over 10 km, regenerated whenever
the radar entered search or track, and never moving with the aircraft. The port
makes the terrain a property of the world: a fixed function of position that the
plant samples ahead of the aircraft along its heading, 100 samples over 10 km as
before, and publishes as the PLANT_TERRAIN block, which the bus controller relays
to the radar. Flying north from home the profile is the Python's; elsewhere the
hills roll with the easting, and the per-sample noise becomes two fixed ripples of
35 m, so the same place always has the same height.
"""

import math

from fmofp_bus.icd.blocks import PlantTerrain, PlantTerrainElevation

PROFILE_SAMPLES = 100      # tfr_radar.elevation_points
PROFILE_RANGE_M = 10000.0  # tfr_radar.scan_range, metres
M_PER_DEG = 111_000.0      # the plant's dead-reckoning constant (flightManagementSystem's metres per degree)
M_PER_FT = 0.3048


class FlatTerrain:
    """Level ground at one elevation (feet), for scenarios that need a known clearance."""

    def __init__(self, elevation_ft=0.0):
        self.elevation_m = float(elevation_ft) * M_PER_FT

    def elevation(self, east_m, north_m):
        return self.elevation_m


class HillTerrain:
    """The Python's hills, made a place: metres above sea level at (east, north) metres from home."""

    def elevation(self, east_m, north_m):
        return (1000.0 + 500.0 * math.sin(north_m / 1000.0) * math.cos(east_m / 2500.0)
                + 200.0 * math.sin(north_m / 300.0 + east_m / 700.0)
                + 35.0 * math.sin(north_m / 137.0 + east_m / 89.0)
                + 35.0 * math.sin(east_m / 113.0 - north_m / 71.0))


def local_metres(latitude_deg, longitude_deg):
    """(east, north) metres from home (0 N 0 E), with the plant's flat-earth scale."""
    north = latitude_deg * M_PER_DEG
    east = longitude_deg * M_PER_DEG * math.cos(math.radians(latitude_deg))
    return east, north


def sample_distance(i, samples=PROFILE_SAMPLES, range_m=PROFILE_RANGE_M):
    """numpy.linspace(0, range_m, samples)[i], as tfr_radar laid its samples out; the radar computes the same."""
    if samples < 2:
        return 0.0
    if i >= samples - 1:
        return range_m
    return i * (range_m / (samples - 1))


def profile(terrain, latitude_deg, longitude_deg, heading_deg, samples=PROFILE_SAMPLES, range_m=PROFILE_RANGE_M):
    """The terrain elevations (metres) at the sample distances ahead along the heading."""
    east, north = local_metres(latitude_deg, longitude_deg)
    heading = math.radians(heading_deg)
    sin_h, cos_h = math.sin(heading), math.cos(heading)
    return [terrain.elevation(east + d * sin_h, north + d * cos_h)
            for d in (sample_distance(i, samples, range_m) for i in range(samples))]


def profile_message(terrain, latitude_deg, longitude_deg, heading_deg, seq):
    """PLANT_TERRAIN for the aircraft's position and heading."""
    m = PlantTerrain()
    m.terrain_seq = seq % 256
    m.range_m = PROFILE_RANGE_M
    m.elevation = [PlantTerrainElevation(elevation=e) for e in profile(terrain, latitude_deg, longitude_deg, heading_deg)]
    return m
