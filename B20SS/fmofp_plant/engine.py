"""The engines, in the plant.

Systems/engineManagement/ecu/engineControlUnit.py simulated one turbofan: at 2 Hz
its _adjust() derived N1, N2, EGT and fuel flow from the thrust, oil pressure, oil
temperature and vibration from sine waves of the time, and random-walked the
compressor efficiency, each with Gaussian noise. That is a model of an engine, not
engine software, so it runs here for each of the two engines the fuel system and
the thrust management system name (Engine 1, Engine 2), and the engine computer
(the C++ partition at RT 3) commands it and reads it over the bus.

Two differences from the Python, both recorded in the ICD README (section 2):
the thrust is the engine computer's command (PLANT_THRUST_CMD) instead of a
random walk, because the walk was what an engine nothing commanded did; and t is
simulated seconds from the start instead of the wall clock, so the sine waves
start at a known phase.
"""

import math

from fmofp_bus.icd.enums import Ssm
from fmofp_bus.icd.messages import PlantEngine

ENGINES = 2
ECU_HZ = 2  # EngineControlUnit._update_loop: time.sleep(0.5)
ECU_DT = 1.0 / ECU_HZ

# EngineControlUnit.__init__'s starting state.
INITIAL = dict(thrust=70.0, n1=78.5, n2=84.2, egt=665.0, fuel_flow=2400.0, oil_pressure=60.0,
               oil_temperature=92.0, vibration=0.3, compressor_efficiency=88.0)

# The PLANT_ENGINE field suffixes, in the order the message carries them.
FIELDS = ("thrust", "n1", "n2", "egt", "fuel_flow", "oil_pressure", "oil_temperature", "vibration",
          "compressor_efficiency")

# EngineControlUnit._snapshot()'s keys and rounding, for anything that wants the ECU's dictionary.
SNAPSHOT = (("thrust_pct", "thrust", 2), ("n1_pct", "n1", 2), ("n2_pct", "n2", 2), ("egt_c", "egt", 1),
            ("ff_kgh", "fuel_flow", 1), ("oil_psi", "oil_pressure", 2), ("oil_temp_c", "oil_temperature", 2),
            ("vibration", "vibration", 3), ("compressor_eff", "compressor_efficiency", 2))


class EngineState:
    """One engine's measurements."""

    def __init__(self):
        for name, value in INITIAL.items():
            setattr(self, name, value)

    def snapshot(self):
        """EngineControlUnit._snapshot() without the timestamp."""
        return {key: round(getattr(self, name), digits) for key, name, digits in SNAPSHOT}


def ecu_step(engine, thrust_pct, t, rng):
    """EngineControlUnit._adjust() for one engine, the thrust commanded instead of walked.

    The draws come in the Python's order: gauss for N1, N2, EGT, fuel flow, oil
    pressure, oil temperature and vibration, then uniform for the compressor
    efficiency.
    """
    engine.thrust = max(0.0, min(100.0, thrust_pct))
    engine.n1 = 0.80 * engine.thrust + 18 + rng.gauss(0, 0.1)
    engine.n2 = 0.75 * engine.thrust + 22 + rng.gauss(0, 0.1)
    engine.egt = 350 + engine.thrust * 4.5 + rng.gauss(0, 1)
    engine.fuel_flow = engine.thrust * 42 + rng.gauss(0, 5)
    engine.oil_pressure = 60 + 4 * math.sin(t * 0.1) + rng.gauss(0, 0.2)
    engine.oil_temperature = 92 + 8 * math.sin(t * 0.05 + 1) + rng.gauss(0, 0.1)
    engine.vibration = 0.2 + 0.15 * abs(math.sin(t * 0.3)) + rng.gauss(0, 0.01)
    engine.compressor_efficiency = max(75, min(98, engine.compressor_efficiency + rng.uniform(-0.1, 0.1)))


class EngineWorld:
    """Both engines, stepped at the ECU's rate, at the thrust the engine computer commands."""

    def __init__(self, rng):
        self.rng = rng
        self.time_s = 0.0
        self.engines = [EngineState() for _ in range(ENGINES)]
        self.commanded = None  # [pct per engine] once a command with commands_valid arrived
        self.commands = 0

    def command(self, thrust_cmd):
        """An ENG_THRUST_CMD (the payload of PLANT_THRUST_CMD): taken while commands_valid is set."""
        if not thrust_cmd.commands_valid:
            return False
        self.commanded = [float(thrust_cmd.thrust_1), float(thrust_cmd.thrust_2)]
        self.commands += 1
        return True

    def step(self, dt=ECU_DT):
        """One ECU pass for both engines."""
        self.time_s += dt
        for i, engine in enumerate(self.engines):
            thrust = engine.thrust if self.commanded is None else self.commanded[i]
            ecu_step(engine, thrust, self.time_s, self.rng)

    def message(self):
        """PLANT_ENGINE for the present measurements, both engines NORMAL.

        A reading the noise took below zero (a fuel flow or a vibration at idle)
        is reported as zero: the sensors read no negative flow.
        """
        m = PlantEngine()
        m.engine_1_ssm = Ssm.NORMAL
        m.engine_2_ssm = Ssm.NORMAL
        for i, engine in enumerate(self.engines, start=1):
            for name in FIELDS:
                value = getattr(engine, name)
                if name != "oil_temperature":
                    value = max(0.0, value)
                setattr(m, f"{name}_{i}", value)
        return m
