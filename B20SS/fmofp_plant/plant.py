"""The plant terminal.

    py -m fmofp_plant [--port-base 51000] [--syncs 0] [--seed N] [--terrain-ft ELEVATION]
                      [--lat 0.0] [--lon 0.0]
                      [--gps-noise-m 3.0] [--gps-clock-bias-us 1.0] [--gps-outage]
                      [--no-weather]
                      [--hold altitude=1500,vertical_speed=-4500,...] [--quiet]

PlantModel is Systems/flightControlSys/flightControlComputer.FlightControlComputer's
random walk, step for step, at the same 10 Hz, plus what the ported navigation needs
from the simulated world: a position that moves with the (held) speed and heading,
and a GPS receiver front end that measures pseudoranges to the 12-satellite
constellation of Systems/nav/gps/globalPositionSystem.py: a new epoch once a second, its ranges following the aircraft in between. The
position solver itself lives in the navigation terminal; the plant only produces
what the receiver would measure, noise included. It also owns the weather
(fmofp_plant.weather): the storm cells and stratiform area the radar's sensor
model renders, published as PLANT_WEATHER once a second; and the terrain
(fmofp_plant.terrain): the elevation profile ahead of the aircraft that the
terrain-following radar assesses, published as the PLANT_TERRAIN block ten times
a second. --terrain-ft flattens the world to one elevation. And it runs the two
engines (fmofp_plant.engine): engineControlUnit.py's engine model at its 2 Hz,
at the thrust the engine computer commands (PLANT_THRUST_CMD, relayed from the
engine terminal's ENG_THRUST_CMD), published as PLANT_ENGINE ten times a second.

--hold freezes named quantities at fixed values, which is how the tests put the
aircraft into a known situation.
"""

import argparse
import math
import random
import sys

from fmofp_bus.block import BlockPublisher
from fmofp_bus.icd.blocks import PlantTerrain
from fmofp_bus.icd.enums import GearPosition, Ssm
from fmofp_bus.icd.messages import PlantGps, PlantSensors, PlantThrustCmd, PlantWeather
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status
from fmofp_plant.engine import ECU_DT, ECU_HZ, EngineWorld
from fmofp_plant.terrain import FlatTerrain, HillTerrain, profile_message
from fmofp_plant.weather import WeatherWorld

PLANT_RT = PlantSensors.RT
SYNC_HZ = 50
WALK_HZ = 10  # the FCC's update rate
SYNCS_PER_WALK = SYNC_HZ // WALK_HZ
GPS_HZ = 1  # GPSSystem.UPDATE_INTERVAL_S
SYNCS_PER_GPS = SYNC_HZ // GPS_HZ
WEATHER_HZ = 1
SYNCS_PER_WEATHER = SYNC_HZ // WEATHER_HZ
TERRAIN_HZ = 10  # a new profile each time the aircraft has moved
SYNCS_PER_TERRAIN = SYNC_HZ // TERRAIN_HZ
SYNCS_PER_ECU_STEP = SYNC_HZ // ECU_HZ  # the engine model's own rate
ENGINE_HZ = 10  # PLANT_ENGINE's rate; a reading stays current for three periods at its consumer
SYNCS_PER_ENGINE = SYNC_HZ // ENGINE_HZ

HOLDABLE = ("altitude", "speed", "heading", "pitch", "roll", "vertical_speed", "angle_of_attack", "g_force",
            "flaps", "latitude", "longitude")

# Where the Python aircraft lived: flightManagementSystem's navigation dict starts
# at latitude 0, longitude 0, and the GPS truth followed it, so every fix in the
# frozen Python was made here. It is also a spot the 12-satellite constellation
# serves well (five satellites, a fix good to a few metres). The INS's starting
# point, 35.4147 N 97.3866 W, was never used by anything, and from there the
# constellation shows four coplanar satellites whose fixes are off by kilometres.
HOME_LATITUDE = 0.0
HOME_LONGITUDE = 0.0

# WGS-84 and the constellation, as globalPositionSystem.py has them.
WGS84_A = 6_378_137.0
WGS84_B = 6_356_752.314_245
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2
SPEED_OF_LIGHT = 299_792_458.0
GPS_ORBIT_RADIUS_M = 26_560_000.0
GPS_INCLINATION = math.radians(55.0)
GPS_MIN_ELEVATION_DEG = 5.0
GPS_SATELLITES = 12


def build_constellation():
    """globalPositionSystem._build_constellation(): [(id, (x, y, z) m, clock bias s)]."""
    sats = []
    sat_id = 1
    for plane in range(3):
        raan = math.radians(plane * 120)
        for slot in range(4):
            u = math.radians(slot * 90)
            x_p = GPS_ORBIT_RADIUS_M * math.cos(u)
            y_p = GPS_ORBIT_RADIUS_M * math.sin(u)
            x_i = x_p
            y_i = y_p * math.cos(GPS_INCLINATION)
            z_i = y_p * math.sin(GPS_INCLINATION)
            x = x_i * math.cos(raan) - y_i * math.sin(raan)
            y = x_i * math.sin(raan) + y_i * math.cos(raan)
            z = z_i
            clock_bias = (sat_id * 1.3e-9) % 1e-6
            sats.append((sat_id, (x, y, z), clock_bias))
            sat_id += 1
    return sats


def wgs84_to_ecef(lat_deg, lon_deg, alt_ft):
    """GPSReceiver.update_true_position(): geodetic degrees and feet to ECEF metres."""
    alt_m = alt_ft * 0.3048
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1 - WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def elevation_deg(receiver, satellite):
    """GPSReceiver._is_visible()'s elevation angle; 90 at the Earth's centre, as that code behaves."""
    rx, ry, rz = receiver
    sx, sy, sz = satellite
    dx, dy, dz = sx - rx, sy - ry, sz - rz
    r_mag = math.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
    if r_mag == 0:
        return 90.0
    nx, ny, nz = rx / r_mag, ry / r_mag, rz / r_mag
    d_mag = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
    if d_mag == 0:
        return -90.0
    cos_el = (dx * nx + dy * ny + dz * nz) / d_mag
    return math.degrees(math.asin(max(-1.0, min(1.0, cos_el))))


class PlantModel:
    """FlightControlComputer._adjust(), including its clamps and starting state, plus position and GPS."""

    def __init__(self, seed=None, terrain=None, latitude=HOME_LATITUDE, longitude=HOME_LONGITUDE,
                 gps_noise_m=3.0, gps_clock_bias_us=1.0, gps_outage=False, weather=True):
        self.rng = random.Random(seed)
        # The weather draws from its own generator so adding it changed nothing else about a seeded run.
        self.weather = WeatherWorld(random.Random(None if seed is None else seed ^ 0x5745)) if weather else None
        self.weather_seq = 0
        self.terrain = HillTerrain() if terrain is None else terrain
        self.terrain_seq = 0
        # The engines draw from their own generator too, for the same reason.
        self.engines = EngineWorld(random.Random(None if seed is None else seed ^ 0x454E47))
        self.altitude = 10000.0       # feet
        self.speed = 500.0            # knots
        self.heading = 0.0            # degrees true
        self.pitch = 0.0              # degrees
        self.roll = 0.0               # degrees
        self.vertical_speed = 0.0     # feet/min
        self.angle_of_attack = 2.0    # degrees
        self.g_force = 1.0
        self.flaps = 0.0              # degrees
        self.landing_gear = "up"
        self.latitude = float(latitude)    # degrees, WGS-84
        self.longitude = float(longitude)
        self.gps_noise_m = float(gps_noise_m)        # GPSReceiver._measurement_noise_m
        self.gps_clock_bias_s = float(gps_clock_bias_us) * 1e-6
        self.gps_outage = bool(gps_outage)
        self.satellites = build_constellation()
        self.gps_epoch = 0
        self.gps_noise = None  # per satellite, drawn by new_gps_epoch()
        self.hold = {}

    def adjust(self):
        """One 10 Hz step of the FCC's random walk."""
        u = self.rng.uniform
        self.altitude += u(-50, 50)
        self.speed += u(-5, 5)
        self.heading = (self.heading + u(-1, 1)) % 360
        self.pitch += u(-0.5, 0.5)
        self.roll += u(-0.5, 0.5)
        self.vertical_speed = u(-500, 500)
        self.angle_of_attack = max(0, self.angle_of_attack + u(-0.2, 0.2))
        self.g_force = max(0.5, min(9.0, self.g_force + u(-0.05, 0.05)))

        self.altitude = max(0, min(60000, self.altitude))
        self.speed = max(100, min(1000, self.speed))
        self.pitch = max(-45, min(45, self.pitch))
        self.roll = max(-60, min(60, self.roll))
        self.angle_of_attack = min(25, self.angle_of_attack)

    def move(self, dt):
        """Advance the position by the (held) speed and heading for dt seconds.

        The same dead-reckoning rates flightManagementSystem.update_flight_data
        derived (knots to feet per second to degrees per second), so the
        navigation terminal's prediction between fixes matches the motion.
        """
        gs_kts = self.value("speed")
        heading_rad = math.radians(self.value("heading"))
        gs_dps_lat = (gs_kts * 1.68781) / (111_000 / 0.3048)
        gs_dps_lon = gs_dps_lat / max(0.001, math.cos(math.radians(self.latitude)))
        self.latitude += gs_dps_lat * math.cos(heading_rad) * dt
        self.longitude += gs_dps_lon * math.sin(heading_rad) * dt
        self.latitude = max(-90.0, min(90.0, self.latitude))
        self.longitude = (self.longitude + 180.0) % 360.0 - 180.0
        if self.weather is not None:
            self.weather.update(dt)

    def value(self, name):
        return self.hold.get(name, getattr(self, name))

    def sensors(self):
        """PLANT_SENSORS for the current state, every quantity NORMAL."""
        s = PlantSensors()
        for ssm in ("altitude_ssm", "airspeed_ssm", "pitch_ssm", "roll_ssm", "heading_ssm", "vertical_speed_ssm",
                    "aoa_ssm", "g_ssm", "flaps_ssm"):
            setattr(s, ssm, Ssm.NORMAL)
        s.gear = GearPosition.DOWN if self.landing_gear == "down" else GearPosition.UP
        s.altitude = self.value("altitude")
        s.airspeed = self.value("speed")
        s.vertical_speed = self.value("vertical_speed")
        s.pitch = self.value("pitch")
        s.roll = self.value("roll")
        s.heading = self.value("heading")
        s.angle_of_attack = self.value("angle_of_attack")
        s.g_force = self.value("g_force")
        s.flaps = self.value("flaps")
        return s

    def terrain_message(self):
        """PLANT_TERRAIN for the current position and heading: the profile the radar's antenna would measure."""
        self.terrain_seq = (self.terrain_seq + 1) % 256
        return profile_message(self.terrain, self.value("latitude"), self.value("longitude"), self.value("heading"),
                               self.terrain_seq)

    def weather_message(self):
        """PLANT_WEATHER for the current weather; an empty sky when the weather is off."""
        if self.weather is not None:
            return self.weather.message()
        m = PlantWeather()
        self.weather_seq = (self.weather_seq + 1) % 256
        m.weather_seq = self.weather_seq
        return m

    def engine_message(self):
        """PLANT_ENGINE: what the engines' sensors read now."""
        return self.engines.message()

    def thrust_command(self, words):
        """A PLANT_THRUST_CMD received: the engines take the engine computer's command. True when applied."""
        status, cmd = PlantThrustCmd.decode(words)
        if status != Status.OK:
            return False
        return self.engines.command(cmd.payload)

    def true_position_ecef(self):
        return wgs84_to_ecef(self.value("latitude"), self.value("longitude"), self.value("altitude"))

    def new_gps_epoch(self):
        """Starts a GPS epoch: the epoch count moves on and each satellite's measurement noise is drawn once
        (GPSReceiver._pseudorange drew it per measurement; the solver sees one consistent set per epoch)."""
        self.gps_epoch = (self.gps_epoch + 1) % 16
        self.gps_noise = {sat_id: self.rng.gauss(0.0, self.gps_noise_m) for sat_id, _, _ in self.satellites}

    def gps_message(self):
        """PLANT_GPS as the receiver's output register reads now: this epoch's noise on the geometric ranges from
        the present true position, so a fix is current whenever the bus controller polls it, in whatever frame
        the schedule gives that poll (the navigation solves an epoch once, when its count changes).

        A satellite counts as visible at 5 degrees of elevation or more, seen from
        the true position (GPSReceiver._is_visible). The pseudorange is the
        geometric range plus c times (receiver clock bias minus satellite clock
        bias) plus the epoch's Gaussian noise (GPSReceiver._pseudorange). During
        an outage no satellite is visible.
        """
        if self.gps_noise is None:
            self.new_gps_epoch()
        g = PlantGps()
        g.epoch = self.gps_epoch
        if self.gps_outage:
            return g
        receiver = self.true_position_ecef()
        mask = 0
        for sat_id, position, sat_bias in self.satellites:
            if elevation_deg(receiver, position) < GPS_MIN_ELEVATION_DEG:
                continue
            geometric = math.dist(receiver, position)
            pseudorange = geometric + SPEED_OF_LIGHT * (self.gps_clock_bias_s - sat_bias) + self.gps_noise[sat_id]
            setattr(g, f"pseudorange_{sat_id}", pseudorange)
            mask |= 1 << (sat_id - 1)
        g.visible_mask = mask
        return g

    def gps(self):
        """PLANT_GPS for a new epoch, measured now."""
        self.new_gps_epoch()
        return self.gps_message()


class PlantTerminal:
    """RT 27 on the bus: walks the model at 10 Hz, publishes PLANT_SENSORS every frame, PLANT_WEATHER once a
    second, PLANT_GPS with a new epoch once a second and its ranges refreshed at the walk rate, the
    PLANT_TERRAIN block ten times a second and PLANT_ENGINE ten times a second (the engine model steps at 2 Hz),
    and takes the engine computer's thrust command from PLANT_THRUST_CMD."""

    def __init__(self, model, port_base=51000):
        self.model = model
        self.process = RtProcess()
        self.port_base = port_base
        self.syncs = 0
        self.publish_failures = 0
        self.thrust_commands = 0
        self.terrain_block = BlockPublisher(PlantTerrain.MAX_PAYLOAD_WORDS)

    def init(self):
        status = self.process.init(PLANT_RT, self.port_base)
        if status == Status.OK:
            self.publish()
            self.publish_gps()
            self.publish_weather()
            self.publish_terrain()
            self.serve_terrain()
            self.publish_engine()
        return status

    def _load(self, message):
        status, words = message.encode()
        if status not in (Status.OK, Status.CLAMPED):
            self.publish_failures += 1
            return
        self.process.terminal.set_transmit(message.SUBADDRESS, words)

    def publish(self):
        self._load(self.model.sensors())

    def publish_gps(self):
        self._load(self.model.gps_message())

    def publish_weather(self):
        self._load(self.model.weather_message())

    def publish_engine(self):
        self._load(self.model.engine_message())

    def take_thrust_command(self):
        """Applies a PLANT_THRUST_CMD that arrived since the last frame."""
        status, words, fresh = self.process.terminal.get_received(PlantThrustCmd.SUBADDRESS)
        if status == Status.OK and fresh and self.model.thrust_command(words):
            self.thrust_commands += 1

    def publish_terrain(self):
        status, words = self.model.terrain_message().encode()
        if status not in (Status.OK, Status.CLAMPED):
            self.publish_failures += 1
            return
        if self.terrain_block.publish(self.process.terminal, PlantTerrain.SEGMENT_SUBADDRESS, words) != Status.OK:
            self.publish_failures += 1

    def serve_terrain(self):
        self.terrain_block.serve(self.process.terminal, PlantTerrain.DESCRIPTOR_SUBADDRESS,
                                 PlantTerrain.SEGMENT_SUBADDRESS)

    def poll(self, timeout_s=0.1):
        """One iteration of the loop: handle a datagram, step on sync. Returns True on a sync."""
        self.process.poll(timeout_s)
        got, _ = self.process.terminal.take_sync()
        if not got:
            if self.process.terminal.take_reset_request():
                self.terrain_block = BlockPublisher(PlantTerrain.MAX_PAYLOAD_WORDS)
                self.publish()
                self.publish_gps()
                self.publish_weather()
                self.publish_terrain()
                self.serve_terrain()
                self.publish_engine()
            return False
        self.syncs += 1
        self.take_thrust_command()
        if self.syncs % SYNCS_PER_ECU_STEP == 0:
            self.model.engines.step(ECU_DT)
        if self.syncs % SYNCS_PER_ENGINE == 0:
            self.publish_engine()
        if self.syncs % SYNCS_PER_GPS == 0:
            self.model.new_gps_epoch()
        if self.syncs % SYNCS_PER_WALK == 0:
            self.model.adjust()
            self.model.move(1.0 / WALK_HZ)
            self.publish_gps()  # the receiver's output follows the aircraft; the epoch count says when a fix is new
        if self.syncs % SYNCS_PER_WEATHER == 0:
            self.publish_weather()
        if self.syncs % SYNCS_PER_TERRAIN == 0:
            self.publish_terrain()
        self.serve_terrain()
        self.publish()
        return True

    def shutdown(self):
        self.process.shutdown()


def parse_hold(text):
    hold = {}
    for item in filter(None, (text or "").split(",")):
        name, _, value = item.partition("=")
        if name not in HOLDABLE:
            raise argparse.ArgumentTypeError(f"cannot hold '{name}'; one of {', '.join(HOLDABLE)}")
        hold[name] = float(value)
    return hold


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port-base", type=int, default=51000)
    parser.add_argument("--syncs", type=int, default=0, help="exit after this many frames (0 = run until killed)")
    parser.add_argument("--seed", type=int, default=None, help="random seed, for repeatable runs")
    parser.add_argument("--terrain-ft", type=float, default=None,
                        help="flat terrain at this elevation everywhere (default: the hills of fmofp_plant.terrain)")
    parser.add_argument("--lat", type=float, default=HOME_LATITUDE, help="starting latitude, degrees")
    parser.add_argument("--lon", type=float, default=HOME_LONGITUDE, help="starting longitude, degrees")
    parser.add_argument("--gps-noise-m", type=float, default=3.0, help="pseudorange noise, 1 sigma")
    parser.add_argument("--gps-clock-bias-us", type=float, default=1.0, help="true receiver clock bias")
    parser.add_argument("--gps-outage", action="store_true", help="no satellite is ever visible")
    parser.add_argument("--no-weather", action="store_true", help="a clear sky: PLANT_WEATHER carries no cells or area")
    parser.add_argument("--hold", type=parse_hold, default={}, help="name=value,... held constant")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    terrain = None if args.terrain_ft is None else FlatTerrain(args.terrain_ft)
    model = PlantModel(seed=args.seed, terrain=terrain, latitude=args.lat, longitude=args.lon,
                       gps_noise_m=args.gps_noise_m, gps_clock_bias_us=args.gps_clock_bias_us,
                       gps_outage=args.gps_outage, weather=not args.no_weather)
    model.hold = args.hold
    plant = PlantTerminal(model, args.port_base)
    status = plant.init()
    if status != Status.OK:
        print(f"plant: init failed: {status.name} (port {args.port_base + PLANT_RT} in use?)", file=sys.stderr)
        return 1
    if not args.quiet:
        print(f"plant: RT {PLANT_RT} on 127.0.0.1:{args.port_base + PLANT_RT}")
    try:
        while args.syncs == 0 or plant.syncs < args.syncs:
            if plant.poll() and not args.quiet and plant.syncs % 50 == 1:
                print(f"plant: frame {plant.syncs}: alt {model.value('altitude'):.0f} ft, "
                      f"{model.value('speed'):.0f} kt, hdg {model.value('heading'):.0f}, "
                      f"{model.value('latitude'):.4f} {model.value('longitude'):.4f}, "
                      f"thrust {model.engines.engines[0].thrust:.0f}/{model.engines.engines[1].thrust:.0f}%")
    except KeyboardInterrupt:
        pass
    finally:
        plant.shutdown()
    c = plant.process.terminal.counters
    print(f"plant: syncs={plant.syncs} transmits={c.transmits} message_errors={c.message_errors} "
          f"publish_failures={plant.publish_failures} thrust_commands={plant.thrust_commands}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
