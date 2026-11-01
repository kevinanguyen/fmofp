"""The simulated air traffic: the aircraft targeting_radar.py and aewc_radar.py invented, flown by the plant.

Each Python radar generated its own targets around the aircraft (_generate_target:
a position within its range, a velocity, a constant acceleration, a radar cross-
section) and moved them itself on every track request, in a frame that moved with
the aircraft and never knew it. The port draws the line between world and sensor
as it did for the weather and the terrain: the plant owns the traffic, moves it
with the Python's kinematics at its own 10 Hz in a frame fixed to the plant's
home, replaces an aircraft that leaves its generator's range (the Python deleted
it and drew another), and publishes what the antennas would measure of each:
slant range, true azimuth and elevation from the aircraft, the velocity relative
to the aircraft and the cross-section (PLANT_TRAFFIC, five times a second). The
radars classify from that; nothing in the block says what an aircraft is.

The generators' elevation angles put aircraft many kilometres above and below the
aircraft (the AEWC's 45 degrees at 400 km reaches 280 km); they are kept as
written, the radars do not care, and the pictures draw east and north only.
"""

import math

from fmofp_bus.icd.blocks import PlantTraffic, PlantTrafficTarget

EARTH_RADIUS_M = 6_371_000.0
M_PER_FT = 0.3048
MPS_PER_KT = 0.514444
MPS_PER_FPM = 0.3048 / 60.0
MAX_TARGETS = PlantTraffic.MAX_TARGET  # 12


class Population:
    """One radar's generator: how many aircraft it kept and the distributions it drew them from."""

    def __init__(self, name, count, min_range_m, max_range_m, elevation_rad, min_speed, max_speed, max_accel,
                 stealth_fraction, rcs_range, stealth_rcs_range):
        self.name = name
        self.count = count
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.elevation_rad = elevation_rad
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.stealth_fraction = stealth_fraction
        self.rcs_range = rcs_range
        self.stealth_rcs_range = stealth_rcs_range


# targeting_radar: at most five targets, 1-100 km, elevation within 30 degrees, 100-300 m/s, accelerations up to
# 30 m/s2, RCS 1-10 m2, never stealth.
TARGETING = Population("targeting", 5, 1000.0, 100000.0, math.pi / 6, 100.0, 300.0, 30.0, 0.0, (1.0, 10.0), None)
# aewc_radar: at most ten (seven here, the block's room beside the five), 5-400 km, within 45 degrees, 150-500 m/s,
# up to 50 m/s2, three in ten stealth with RCS 0.01-0.1 m2, the rest 1-100 m2.
AEWC = Population("aewc", 7, 5000.0, 400000.0, math.pi / 4, 150.0, 500.0, 50.0, 0.3, (1.0, 100.0), (0.01, 0.1))


class Target:
    """One aircraft: metres east, north and up of the plant's home, m/s, m/s2."""

    __slots__ = ("target_id", "population", "position", "velocity", "acceleration", "rcs", "stealth")

    def __init__(self, target_id, population, position, velocity, acceleration, rcs, stealth):
        self.target_id = target_id
        self.population = population
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.rcs = rcs
        self.stealth = stealth


def ownship_enu(latitude, longitude, altitude_ft, home_latitude=0.0, home_longitude=0.0):
    """Metres east, north and up of the aircraft from the plant's home (a flat frame; fine over a few hundred km)."""
    east = math.radians(longitude - home_longitude) * math.cos(math.radians(home_latitude)) * EARTH_RADIUS_M
    north = math.radians(latitude - home_latitude) * EARTH_RADIUS_M
    return (east, north, altitude_ft * M_PER_FT)


def ownship_velocity(speed_kt, heading_deg, vertical_speed_fpm):
    """The aircraft's velocity east, north and up in m/s (ground speed is the airspeed: no wind in the plant)."""
    speed = speed_kt * MPS_PER_KT
    heading = math.radians(heading_deg)
    return (speed * math.sin(heading), speed * math.cos(heading), vertical_speed_fpm * MPS_PER_FPM)


class TrafficWorld:
    """The traffic: populate() around the aircraft, update() every walk step, message() for the radars."""

    def __init__(self, rng, populations=(TARGETING, AEWC)):
        self.rng = rng
        self.populations = tuple(populations)
        self.targets = []
        self.next_id = 1
        self.seq = 0

    def spawn(self, population, ownship):
        """_generate_target: the Python's draws, in its order, around the aircraft's present position; the
        velocity it drew in the aircraft's frame is taken as the aircraft's velocity over the ground."""
        u = self.rng.uniform
        r = u(population.min_range_m, population.max_range_m)
        theta = u(0.0, 2.0 * math.pi)
        phi = u(-population.elevation_rad, population.elevation_rad)
        offset = (r * math.cos(phi) * math.cos(theta), r * math.cos(phi) * math.sin(theta), r * math.sin(phi))
        v_mag = u(population.min_speed, population.max_speed)
        v_theta = u(0.0, 2.0 * math.pi)
        v_phi = u(-math.pi / 6, math.pi / 6)
        velocity = [v_mag * math.cos(v_phi) * math.cos(v_theta), v_mag * math.cos(v_phi) * math.sin(v_theta),
                    v_mag * math.sin(v_phi)]
        a_mag = u(0.0, population.max_accel)
        acceleration = [a_mag * self.rng.gauss(0.0, 1.0) for _ in range(3)]
        stealth = population.stealth_fraction > 0.0 and self.rng.random() < population.stealth_fraction
        rcs = u(*population.stealth_rcs_range) if stealth else u(*population.rcs_range)
        target = Target(self.next_id, population, [o + d for o, d in zip(ownship, offset)], velocity, acceleration,
                        rcs, stealth)
        self.next_id = self.next_id % 255 + 1  # PLANT_TRAFFIC's target_id is a byte
        return target

    def populate(self, ownship):
        """Every population's count of aircraft, around the aircraft."""
        self.targets = []
        for population in self.populations:
            for _ in range(population.count):
                self.targets.append(self.spawn(population, ownship))
        return self.targets

    @staticmethod
    def relative(target, ownship):
        return tuple(p - o for p, o in zip(target.position, ownship))

    def range_m(self, target, ownship):
        dx, dy, dz = self.relative(target, ownship)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def update(self, dt, ownship):
        """_update_targets: the velocity takes the acceleration, the position the mean of the old and new
        velocities; an aircraft beyond its generator's range is replaced."""
        for i, target in enumerate(self.targets):
            new_velocity = [v + a * dt for v, a in zip(target.velocity, target.acceleration)]
            target.position = [p + 0.5 * (v + nv) * dt for p, v, nv in zip(target.position, target.velocity,
                                                                           new_velocity)]
            target.velocity = new_velocity
            if self.range_m(target, ownship) > target.population.max_range_m:
                self.targets[i] = self.spawn(target.population, ownship)

    def message(self, ownship, ownship_velocity):
        """PLANT_TRAFFIC: each aircraft as the antennas would measure it from the aircraft."""
        self.seq = (self.seq + 1) % 256
        block = PlantTraffic(traffic_seq=self.seq)
        records = []
        for target in self.targets[:MAX_TARGETS]:
            dx, dy, dz = self.relative(target, ownship)
            horizontal = math.hypot(dx, dy)
            records.append(PlantTrafficTarget(
                target_id=target.target_id,
                azimuth=math.degrees(math.atan2(dx, dy)) % 360.0,
                range=math.sqrt(horizontal * horizontal + dz * dz) / 1000.0,
                elevation=math.degrees(math.atan2(dz, horizontal)),
                velocity_east=target.velocity[0] - ownship_velocity[0],
                velocity_north=target.velocity[1] - ownship_velocity[1],
                velocity_up=target.velocity[2] - ownship_velocity[2],
                rcs=target.rcs))
        block.target = records
        return block
