"""The plant terminal.

    py -m fmofp_plant [--port-base 51000] [--syncs 0] [--seed N] [--terrain-ft 0]
                      [--hold altitude=1500,vertical_speed=-4500,...] [--quiet]

PlantModel is Systems/flightControlSys/flightControlComputer.FlightControlComputer's
random walk, step for step, at the same 10 Hz. --hold freezes named quantities
at fixed values, which is how the tests put the aircraft into a known situation.
"""

import argparse
import random
import sys

from fmofp_bus.icd.enums import GearPosition, Ssm
from fmofp_bus.icd.messages import PlantSensors
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status

PLANT_RT = PlantSensors.RT
SYNC_HZ = 50
WALK_HZ = 10  # the FCC's update rate
SYNCS_PER_WALK = SYNC_HZ // WALK_HZ

HOLDABLE = ("altitude", "speed", "heading", "pitch", "roll", "vertical_speed", "angle_of_attack", "g_force",
            "flaps", "terrain_clearance")


class PlantModel:
    """FlightControlComputer._adjust(), including its clamps and starting state."""

    def __init__(self, seed=None, terrain_ft=0.0):
        self.rng = random.Random(seed)
        self.terrain_ft = float(terrain_ft)
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
        self.hold = {}

    def adjust(self):
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

    @property
    def terrain_clearance(self):
        return max(0.0, self.altitude - self.terrain_ft)

    def value(self, name):
        return self.hold.get(name, getattr(self, name))

    def sensors(self):
        """PLANT_SENSORS for the current state, every quantity NORMAL."""
        s = PlantSensors()
        for ssm in ("altitude_ssm", "airspeed_ssm", "pitch_ssm", "roll_ssm", "heading_ssm", "vertical_speed_ssm",
                    "aoa_ssm", "g_ssm", "clearance_ssm", "flaps_ssm"):
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
        s.terrain_clearance = self.value("terrain_clearance")
        return s


class PlantTerminal:
    """RT 27 on the bus: walks the model at 10 Hz and publishes PLANT_SENSORS every frame."""

    def __init__(self, model, port_base=51000):
        self.model = model
        self.process = RtProcess()
        self.port_base = port_base
        self.syncs = 0
        self.publish_failures = 0

    def init(self):
        status = self.process.init(PLANT_RT, self.port_base)
        if status == Status.OK:
            self.publish()
        return status

    def publish(self):
        status, words = self.model.sensors().encode()
        if status not in (Status.OK, Status.CLAMPED):
            self.publish_failures += 1
            return
        self.process.terminal.set_transmit(PlantSensors.SUBADDRESS, words)

    def poll(self, timeout_s=0.1):
        """One iteration of the loop: handle a datagram, step on sync. Returns True on a sync."""
        self.process.poll(timeout_s)
        got, _ = self.process.terminal.take_sync()
        if not got:
            if self.process.terminal.take_reset_request():
                self.publish()
            return False
        self.syncs += 1
        if self.syncs % SYNCS_PER_WALK == 0:
            self.model.adjust()
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
    parser.add_argument("--terrain-ft", type=float, default=0.0, help="terrain elevation under the aircraft")
    parser.add_argument("--hold", type=parse_hold, default={}, help="name=value,... held constant")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    model = PlantModel(seed=args.seed, terrain_ft=args.terrain_ft)
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
                      f"{model.value('speed'):.0f} kt, hdg {model.value('heading'):.0f}")
    except KeyboardInterrupt:
        pass
    finally:
        plant.shutdown()
    c = plant.process.terminal.counters
    print(f"plant: syncs={plant.syncs} transmits={c.transmits} message_errors={c.message_errors} "
          f"publish_failures={plant.publish_failures}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
