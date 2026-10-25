"""The engine partition's thrust management against the Python it was ported from.

fmofp_engine_tool puts the C++ ThrustManager on stdin/stdout. The test drives the
same sequence of lever inputs, autothrottle engagements and PI updates through
Systems/engineManagement/thrustManagement.py's ThrustManagementSystem and
compares: the original Python is the oracle.

Needs FMOFP_ENGINE_TOOL pointing at the binary (CI sets it).
"""

import os
import random
import subprocess
import unittest

from fmofp_bus.tests._support import binary_env, needs


def run_tool(lines):
    """Each command's answer, as a list of tokens."""
    result = subprocess.run([os.environ["FMOFP_ENGINE_TOOL"]], input="\n".join(lines) + "\n",
                            capture_output=True, text=True, timeout=60, env=binary_env())
    assert result.returncode == 0, result.stderr
    return [line.split() for line in result.stdout.splitlines()]


def floats(tokens):
    assert tokens and tokens[0] != "error", tokens
    return [float(t) for t in tokens]


@needs("FMOFP_ENGINE_TOOL")
class EngineOracleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from FMOFP.Systems.engineManagement.thrustManagement.thrustManagement import (MAX_SAFE_THRUST_ASYMMETRY_PCT,
                                                                                      ThrustManagementSystem)
        cls.ThrustManagementSystem = ThrustManagementSystem
        cls.limit = MAX_SAFE_THRUST_ASYMMETRY_PCT

    def python(self):
        tms = self.ThrustManagementSystem()
        tms.register_engine("Engine 1", 0.0)
        tms.register_engine("Engine 2", 0.0)
        return tms

    @staticmethod
    def thrusts(tms):
        return [tms.get_thrust("Engine 1"), tms.get_thrust("Engine 2")]

    def test_the_thrust_manager_matches_thrust_management_system_step_for_step(self):
        rng = random.Random(21)
        tms = self.python()
        lines = []
        expected = []  # per line: the Python's answer
        # Lever inputs, per-engine inputs, an engagement, PI updates at the Python's 2 Hz around the target,
        # a disengagement and inputs beyond the range.
        script = [("set_all", 60.0), ("set", 2, 40.0), ("asym",), ("engage", 500.0)]
        script += [("update", 500.0 + rng.uniform(-80.0, 80.0), 0.5) for _ in range(12)]
        script += [("update", 450.0, 0.0), ("asym",), ("disengage",), ("set_all", 150.0), ("set", 1, -5.0),
                   ("asym",), ("engage", 300.0), ("update", 330.0, 0.47), ("update", 290.0, 0.53)]
        for step in script:
            lines.append(" ".join(repr(x) if isinstance(x, float) else str(x) for x in step))
            kind = step[0]
            if kind == "set_all":
                tms.set_all_thrust(step[1])
                expected.append(self.thrusts(tms))
            elif kind == "set":
                self.assertTrue(tms.set_thrust(f"Engine {step[1]}", step[2]))
                expected.append(self.thrusts(tms))
            elif kind == "engage":
                tms.engage_autothrottle(step[1])
                expected.append(self.thrusts(tms))
            elif kind == "disengage":
                tms.disengage_autothrottle()
                expected.append(self.thrusts(tms))
            elif kind == "update":
                tms.update_autothrottle(step[1], step[2])
                expected.append(self.thrusts(tms) + [tms._at_integral, tms._at_last_error])
            elif kind == "asym":
                asymmetry = tms.check_asymmetry()
                expected.append([asymmetry, 1.0 if asymmetry > self.limit else 0.0])
        rows = run_tool(lines)
        self.assertEqual(len(script), len(rows))
        for step, want, row in zip(script, expected, rows):
            got = floats(row)
            self.assertEqual(len(want), len(got), step)
            for a, b in zip(want, got):
                self.assertAlmostEqual(a, b, delta=1e-9, msg=step)

    def test_the_fuel_tanks_start_as_setup_aircraft_made_them(self):
        from FMOFP.Systems.engineManagement.fuelManagement.fuelControl import FuelTank
        tank = FuelTank("Main Left", 10000.0)
        self.assertEqual(10000.0, tank.current_level, "add_tank left every tank full")
        self.assertEqual(300.0, tank.consume(300.0))
        self.assertEqual(9700.0, tank.consume(20000.0), "consume gives what is there")
        self.assertEqual(0.0, tank.current_level)
        rows = run_tool(["fuel_init", "fuel_set 0 300", "fuel_transfer 0 2 1000", "fuel_set 1 9000",
                         "fuel_transfer 2 1 5000"])
        self.assertEqual([10000.0, 10000.0, 20000.0, 40000.0], floats(rows[0]))
        self.assertEqual([300.0, 10000.0, 20000.0], floats(rows[1]))
        self.assertEqual([0.0, 300.0, 10000.0, 20000.0], floats(rows[2]), "the Center is full: nothing moves")
        self.assertEqual([1000.0, 300.0, 10000.0, 19000.0], floats(rows[4]),
                         "a transfer moves what the destination can take, where the Python's refill lost the overflow")


if __name__ == "__main__":
    unittest.main()
