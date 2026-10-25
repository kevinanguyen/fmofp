"""The plant's engines: engineControlUnit.py's model, commanded over the bus. No sockets."""

import math
import random
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import Ssm
from fmofp_bus.status import Status
from fmofp_plant.engine import ECU_DT, FIELDS, INITIAL, EngineState, EngineWorld, ecu_step
from fmofp_plant.plant import PlantModel


class NoNoise:
    """A generator whose draws are the centre of every distribution: the formulas alone."""

    @staticmethod
    def gauss(mu, sigma):
        return mu

    @staticmethod
    def uniform(a, b):
        return (a + b) / 2.0


def thrust_command(thrust_1, thrust_2, valid=True):
    cmd = m.PlantThrustCmd(payload=m.EngThrustCmd(thrust_1=thrust_1, thrust_2=thrust_2, commands_valid=valid))
    status, words = cmd.encode()
    assert status == Status.OK, status
    return words


class EcuStepTest(unittest.TestCase):
    def test_the_formulas_of_engine_control_unit_adjust(self):
        engine = EngineState()
        self.assertEqual(INITIAL, {name: getattr(engine, name) for name in INITIAL})
        ecu_step(engine, 80.0, 0.5, NoNoise())
        self.assertEqual(80.0, engine.thrust)
        self.assertAlmostEqual(0.80 * 80 + 18, engine.n1)
        self.assertAlmostEqual(0.75 * 80 + 22, engine.n2)
        self.assertAlmostEqual(350 + 80 * 4.5, engine.egt)
        self.assertAlmostEqual(80 * 42, engine.fuel_flow)
        self.assertAlmostEqual(60 + 4 * math.sin(0.05), engine.oil_pressure)
        self.assertAlmostEqual(92 + 8 * math.sin(0.025 + 1), engine.oil_temperature)
        self.assertAlmostEqual(0.2 + 0.15 * abs(math.sin(0.15)), engine.vibration)
        self.assertEqual(88.0, engine.compressor_efficiency)
        ecu_step(engine, 150.0, 1.0, NoNoise())
        self.assertEqual(100.0, engine.thrust, "clamped to 0..100 as the ECU clamped its walk")
        ecu_step(engine, -3.0, 1.5, NoNoise())
        self.assertEqual(0.0, engine.thrust)
        self.assertAlmostEqual(18.0, engine.n1)
        self.assertEqual({"thrust_pct": 0.0, "n1_pct": 18.0, "n2_pct": 22.0, "egt_c": 350.0, "ff_kgh": 0.0,
                          "oil_psi": round(60 + 4 * math.sin(0.15), 2), "oil_temp_c": round(92 + 8 * math.sin(1.075), 2),
                          "vibration": round(0.2 + 0.15 * abs(math.sin(0.45)), 3), "compressor_eff": 88.0},
                         engine.snapshot(), "EngineControlUnit._snapshot's keys and rounding")

    def test_the_noise_comes_from_the_given_generator_in_the_pythons_order(self):
        rng = random.Random(11)
        engine = EngineState()
        ecu_step(engine, 70.0, 0.5, rng)
        again = random.Random(11)
        expected = {"n1": 0.80 * 70 + 18 + again.gauss(0, 0.1), "n2": 0.75 * 70 + 22 + again.gauss(0, 0.1),
                    "egt": 350 + 70 * 4.5 + again.gauss(0, 1), "fuel_flow": 70 * 42 + again.gauss(0, 5),
                    "oil_pressure": 60 + 4 * math.sin(0.05) + again.gauss(0, 0.2),
                    "oil_temperature": 92 + 8 * math.sin(1.025) + again.gauss(0, 0.1),
                    "vibration": 0.2 + 0.15 * abs(math.sin(0.15)) + again.gauss(0, 0.01),
                    "compressor_efficiency": 88 + again.uniform(-0.1, 0.1)}
        for name, value in expected.items():
            self.assertAlmostEqual(value, getattr(engine, name), places=12, msg=name)


class EngineWorldTest(unittest.TestCase):
    def test_uncommanded_engines_sit_at_the_ecus_starting_thrust(self):
        world = EngineWorld(NoNoise())
        message = world.message()
        self.assertEqual((Ssm.NORMAL, Ssm.NORMAL), (message.engine_1_ssm, message.engine_2_ssm))
        self.assertEqual((70.0, 78.5, 2400.0), (message.thrust_1, message.n1_1, message.fuel_flow_1))
        self.assertEqual((70.0, 78.5, 2400.0), (message.thrust_2, message.n1_2, message.fuel_flow_2))
        world.step()
        world.step()
        self.assertEqual(1.0, world.time_s)
        self.assertEqual(70.0, world.engines[0].thrust, "no command: the thrust holds (the Python's walked)")
        self.assertAlmostEqual(74.0, world.engines[0].n1, msg="the formulas took over from the starting numbers")
        self.assertAlmostEqual(74.0, world.engines[1].n1)

    def test_a_valid_command_sets_the_thrust_at_the_next_step(self):
        world = EngineWorld(NoNoise())
        cmd = m.EngThrustCmd(thrust_1=80.0, thrust_2=60.0, commands_valid=False)
        self.assertFalse(world.command(cmd), "commands_valid clear: the engines keep their own setting")
        world.step()
        self.assertEqual((70.0, 70.0), (world.engines[0].thrust, world.engines[1].thrust))
        cmd.commands_valid = True
        self.assertTrue(world.command(cmd))
        self.assertEqual((70.0, 70.0), (world.engines[0].thrust, world.engines[1].thrust), "applied at the next step")
        world.step()
        self.assertEqual((80.0, 60.0), (world.engines[0].thrust, world.engines[1].thrust))
        self.assertAlmostEqual(82.0, world.engines[0].n1)
        self.assertAlmostEqual(66.0, world.engines[1].n1)
        self.assertEqual(1, world.commands)
        message = world.message()
        self.assertEqual((80.0, 60.0), (message.thrust_1, message.thrust_2))
        self.assertAlmostEqual(60 * 42, message.fuel_flow_2)

    def test_the_message_round_trips_and_never_reports_a_negative_reading(self):
        world = EngineWorld(random.Random(2))
        world.command(m.EngThrustCmd(thrust_1=0.0, thrust_2=90.0, commands_valid=True))
        world.step()
        world.engines[0].fuel_flow = -3.5  # the noise at idle can take the flow below zero
        world.engines[0].vibration = -0.001
        world.engines[0].oil_temperature = -1.0  # a temperature may be negative
        message = world.message()
        self.assertEqual((0.0, 0.0, -1.0), (message.fuel_flow_1, message.vibration_1, message.oil_temperature_1))
        status, words = message.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual(m.PlantEngine.WORDS, len(words))
        status, back = m.PlantEngine.decode(words)
        self.assertEqual(Status.OK, status)
        # Half an LSB each way: PLANT_ENGINE's resolutions per field.
        lsb = dict(thrust=0.01, n1=0.01, n2=0.01, egt=0.1, fuel_flow=1.0, oil_pressure=0.01, oil_temperature=0.01,
                   vibration=0.001, compressor_efficiency=0.01)
        for i in (1, 2):
            for name in FIELDS:
                field = f"{name}_{i}"
                self.assertAlmostEqual(getattr(message, field), getattr(back, field), delta=lsb[name] / 2 + 1e-9,
                                       msg=field)
        self.assertEqual(90.0, back.thrust_2)


class PlantEngineTest(unittest.TestCase):
    def test_the_plant_applies_a_thrust_command_from_its_words(self):
        model = PlantModel(seed=1)
        self.assertFalse(model.thrust_command(thrust_command(80.0, 60.0, valid=False)))
        self.assertTrue(model.thrust_command(thrust_command(80.0, 60.0)))
        model.engines.step(ECU_DT)
        self.assertEqual((80.0, 60.0), (model.engines.engines[0].thrust, model.engines.engines[1].thrust))
        self.assertFalse(model.thrust_command([0x1234]), "words that do not decode are no command")
        message = model.engine_message()
        self.assertEqual(80.0, message.thrust_1)
        self.assertAlmostEqual(82.0, message.n1_1, delta=0.6)

    def test_the_engines_do_not_disturb_a_seeded_flight(self):
        """The engines draw from their own generator: a seeded run's aircraft is what it was before them."""
        model = PlantModel(seed=3)
        for _ in range(5):
            model.engines.step(ECU_DT)
        self.assertEqual(random.Random(3).random(), model.rng.random())
        twin = PlantModel(seed=3)
        for _ in range(5):
            twin.engines.step(ECU_DT)
        self.assertEqual(model.engines.engines[0].n1, twin.engines.engines[0].n1, "and they are repeatable")
        other = PlantModel(seed=4)
        other.engines.step(ECU_DT)
        self.assertNotEqual(model.engines.engines[0].oil_pressure, other.engines.engines[0].oil_pressure)


if __name__ == "__main__":
    unittest.main()
