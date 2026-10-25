"""
Test suite: a subsystem's worker loop survives a transient error, and says so
honestly when it does not.

Two findings in FuelManagementSystem, both measured.

1. ONE TRANSIENT ERROR KILLED THE WORKER PERMANENTLY. run()'s loop called
   update_fuel_system(), check_fuel_levels() and perform_fuel_transfer()
   OUTSIDE its try, which covered only the monitor/transfer pair below them.
   Anything the first three raised propagated out of the daemon thread and
   fuel management stopped for the rest of the process's life. Reproduced by
   injecting a single RuntimeError on the second pass: worker gone.

   An AST sweep of all nine safety-critical subsystems found this was the
   ONLY one with unguarded calls in its worker loop. The other eight wrap
   the whole body, log, back off on the stop Event and continue. That shape
   is adopted here, and the sweep is kept as a regression guard.

2. get_status() REPORTED running=True OVER A DEAD THREAD. It returned
   self.running, a plain bool that run() set True and nothing ever reset --
   so the subsystem's own status said it was running while its worker was
   gone. Measured in the same reproduction. The other eight report thread
   liveness, which cannot lie this way, and this one now does too.

AND THE CONSEQUENCE OF FIXING THE FIRST. Guarding the loop converts "the
worker died" into "the worker is alive and achieving nothing", and a
liveness-only health check calls that HEALTHY forever. Measured on the real
FlightControlComputer with every pass raising: thread alive, health HEALTHY,
reason empty -- and readiness granted, because FCC is in CRITICAL_COMPONENTS.

So every one of the nine now records the START of its current failure streak
and health_report() passes it to worker_health(), which reports UNHEALTHY
once a streak outlives the grace period, with the last error attached.

WHY A DURATION AND NOT A COUNT. This was first built as a count of
consecutive failures with a threshold of 3, and that does not survive being
applied to the other eight: their loop cadences run from 0.02 s
(FlightSurfaceService, 50 Hz) to 1.0 s (hydraulics), and four of the nine
back off 5 s on error while four fall straight through to their normal
cadence. Three failures therefore means 15 s on the FCC and 0.3 s on the nav
service -- the same threshold calling a momentary glitch a failure on one
subsystem and taking a quarter of a minute to notice a dead one on another.
A wall-clock grace period means the same thing everywhere.

Run:  python3 -m FMOFP.Tests.test_worker_loop_resilience
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ast
import pathlib
import textwrap
import threading
import time
import traceback

from FMOFP.Utils.common.health import (
    DEFAULT_FAILURE_GRACE_S, HealthState, worker_health)


class _Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self._failures = []

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            msg = f"  FAIL  {name}" + (f"  [{detail}]" if detail else "")
            print(msg)
            self._failures.append(msg)

    def section(self, title):
        print(f"\n{title}")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n  {self.passed}/{total} passed")
        if self._failures:
            print("\n  Failures:")
            for f in self._failures:
                print(f"    {f}")
        return self.failed == 0


R = _Results()

SUBSYSTEMS = {
    "FCC": "FMOFP/Systems/flightControlSys/flightControlComputer/flightControlComputer.py",
    "nav": "FMOFP/Systems/nav/navService.py",
    "ECU": "FMOFP/Systems/engineManagement/ecu/engineControlUnit.py",
    "surfaces": "FMOFP/Systems/flightSurfaces/flightSurfaceService.py",
    "power": "FMOFP/Systems/powerManagement/elec/powerManagementSystem.py",
    "fuel": "FMOFP/Systems/engineManagement/fuelManagement/fuelControl.py",
    "thrust": "FMOFP/Systems/engineManagement/thrustManagement/thrustManagement.py",
    "hydraulics": "FMOFP/Systems/hydraulics/hydrControl.py",
    "GCAS": ("FMOFP/Systems/flightControlSys/groundCollisionAvoidanceSys/"
             "groundCollisionAvoidanceSys.py"),
}

# Calls that are part of the loop's own plumbing rather than its work.
_PLUMBING = {"sleep", "wait", "is_set", "monotonic", "time", "clear", "set"}


def unguarded_calls(source):
    """Calls made directly in a while-loop body, outside any try.

    A substring search for 'try:' near the loop is not enough and gave the
    wrong answer when this was first checked by eye: fuel's loop DID contain
    a try, several lines below three unguarded calls. This walks the loop
    body's top-level statements instead.
    """
    found = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.While):
            continue
        for stmt in node.body:
            if isinstance(stmt, ast.Try):
                continue
            for sub in ast.walk(stmt):
                if not isinstance(sub, ast.Call):
                    continue
                fn = sub.func
                name = (fn.attr if isinstance(fn, ast.Attribute)
                        else getattr(fn, "id", "?"))
                if name not in _PLUMBING:
                    found.append((node.lineno, name, sub.lineno))
    return found


def test_every_worker_loop_guards_its_body():
    R.section("all nine worker loops guard their whole body")

    for label, rel in SUBSYSTEMS.items():
        src = pathlib.Path(os.path.join(_B20SS, rel)).read_text(encoding="utf-8")
        found = unguarded_calls(src)
        R.check(f"{label}: no unguarded call in a worker loop",
                not found,
                ", ".join(f"{n}() L{ln}" for _, n, ln in found[:5]))


def test_the_scanner_can_see_an_unguarded_loop():
    R.section("NON-TAUTOLOGICAL — the scanner is not simply blind")

    # The exact pre-fix shape of fuel's loop: a try that covers only the
    # tail, with the real work above it.
    planted = textwrap.dedent('''
        def run(self):
            while self.running:
                self.update_fuel_system()
                self.check_fuel_levels()
                try:
                    self.monitor.poll(1)
                except Exception as e:
                    pass
                time.sleep(1)
    ''')
    found = unguarded_calls(planted)
    names = {n for _, n, _ in found}
    R.check("it flags the two calls above the try",
            {"update_fuel_system", "check_fuel_levels"} <= names, str(sorted(names)))
    R.check("...and does not flag the one inside it",
            "poll" not in names, str(sorted(names)))
    R.check("...nor the loop's own sleep", "sleep" not in names, str(sorted(names)))

    guarded = textwrap.dedent('''
        def run(self):
            while not self._running.is_set():
                try:
                    self.work()
                    self.more_work()
                except Exception as e:
                    self._running.wait(5)
                    continue
                self._running.wait(1)
    ''')
    R.check("and it passes a loop whose body is wholly inside the try",
            unguarded_calls(guarded) == [], str(unguarded_calls(guarded)))


def _fuel():
    from FMOFP.Systems.engineManagement.fuelManagement.fuelControl import (
        get_fuel_management_system)
    f = get_fuel_management_system()
    f.update_interval = 0.05
    f.ERROR_BACKOFF_S = 0.1
    f._failing_since = None
    f._last_error = ""
    return f


def test_one_transient_error_is_not_fatal():
    R.section("a single transient error no longer kills fuel management")

    f = _fuel()
    calls = {"n": 0}
    real = f.update_fuel_system

    def flaky():
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("transient sensor glitch")
        return real()

    f.update_fuel_system = flaky
    try:
        f.start()
        time.sleep(1.0)
        alive = f._thread.is_alive()
        report = f.health_report()
        passes = calls["n"]
    finally:
        f.update_fuel_system = real
        f.stop()

    # Pre-fix this was: alive False, passes 2, health UNHEALTHY.
    R.check("the worker is still alive after the error", alive, str(alive))
    R.check("...and kept working past it (more than the 2 passes it used to "
            "manage)", passes > 2, f"passes={passes}")
    R.check("...and reports HEALTHY, because one glitch is not a failure",
            report.state is HealthState.HEALTHY, str(report))


SINGLETONS = {
    "FCC": ("FMOFP.Systems.flightControlSys.flightControlComputer."
            "flightControlComputer", "get_flight_control_computer", "_adjust"),
    "nav": ("FMOFP.Systems.nav.navService", "get_nav_service", "_tick"),
    "ECU": ("FMOFP.Systems.engineManagement.ecu.engineControlUnit",
            "get_engine_control_unit", "_persist"),
    "surfaces": ("FMOFP.Systems.flightSurfaces.flightSurfaceService",
                 "get_flight_surface_service", "_persist"),
    "thrust": ("FMOFP.Systems.engineManagement.thrustManagement."
               "thrustManagement", "get_thrust_management_system",
               "check_asymmetry"),
    "hydraulics": ("FMOFP.Systems.hydraulics.hydrControl",
                   "get_hydraulic_system_controller", "monitor_pressure"),
    "fuel": ("FMOFP.Systems.engineManagement.fuelManagement.fuelControl",
             "get_fuel_management_system", "update_fuel_system"),
}


def test_every_subsystem_records_its_failure_streak():
    R.section("all nine record a streak and hand it to worker_health")

    import importlib

    for label, rel in SUBSYSTEMS.items():
        src = pathlib.Path(os.path.join(_B20SS, rel)).read_text(encoding="utf-8")
        R.check(f"{label}: stamps _failing_since in its except branch",
                "self._failing_since = time.monotonic()" in src)
        R.check(f"{label}: clears it after a successful pass",
                "self._failing_since = None" in src)
        R.check(f"{label}: passes it to worker_health",
                "failing_since=self._failing_since" in src)
        R.check(f"{label}: and passes the last error with it",
                "last_error=self._last_error" in src)


def test_sustained_failure_reports_unhealthy_everywhere():
    R.section("a worker alive but failing every pass reports UNHEALTHY")

    import importlib

    for label, (mod, accessor, victim) in SINGLETONS.items():
        module = importlib.import_module(mod)
        obj = getattr(module, accessor)()
        if not hasattr(obj, victim):
            R.check(f"{label}: has the method this test breaks ({victim})",
                    False, "the loop body changed; pick a new victim")
            continue
        real = getattr(obj, victim)
        if hasattr(obj, "ERROR_BACKOFF_S"):
            obj.ERROR_BACKOFF_S = 0.05
        setattr(obj, victim,
                lambda *a, **k: (_ for _ in ()).throw(RuntimeError(f"{label} offline")))
        try:
            obj.start()
            time.sleep(0.6)
            fresh = obj.health_report()
            started = obj._failing_since
            # Backdate the streak rather than waiting out the grace period:
            # nine subsystems x 15s is not a test, and the arithmetic under
            # test is now - failing_since.
            if obj._failing_since is not None:
                obj._failing_since -= 20.0
            aged = obj.health_report()
        finally:
            setattr(obj, victim, real)
            obj.stop()

        R.check(f"{label}: a failure streak was recorded",
                started is not None, str(started))
        R.check(f"{label}: a fresh streak is still HEALTHY, so a glitch "
                f"costs nothing",
                fresh.state is HealthState.HEALTHY, str(fresh))
        R.check(f"{label}: a streak past the grace period is UNHEALTHY",
                aged.state is HealthState.UNHEALTHY, str(aged))
        R.check(f"{label}: ...and the reason names the error",
                f"{label} offline" in aged.reason, aged.reason)


def test_recovery_clears_the_streak():
    R.section("a subsystem that starts working again goes back to HEALTHY")

    import importlib

    # FCC is the interesting case: it backs off 5s on error, so its streak
    # clears on the next successful pass up to five seconds later, not
    # immediately. Measured: UNHEALTHY at 1s and 3s after the fault is
    # removed, HEALTHY by 6s. That is cadence, not latching, and this check
    # is what distinguishes the two.
    mod, accessor, victim = SINGLETONS["FCC"]
    obj = getattr(importlib.import_module(mod), accessor)()
    real = getattr(obj, victim)
    setattr(obj, victim, lambda *a, **k: (_ for _ in ()).throw(RuntimeError("AHRS offline")))
    try:
        obj.start()
        time.sleep(0.5)
        obj._failing_since -= 20.0
        while_failing = obj.health_report()
        setattr(obj, victim, real)
        deadline = time.monotonic() + 12.0
        recovered = None
        while time.monotonic() < deadline:
            report = obj.health_report()
            if report.state is HealthState.HEALTHY:
                recovered = report
                break
            time.sleep(0.25)
    finally:
        setattr(obj, victim, real)
        obj.stop()

    R.check("FCC was UNHEALTHY while failing",
            while_failing.state is HealthState.UNHEALTHY, str(while_failing))
    R.check("...and returns to HEALTHY once a pass succeeds, not latched",
            recovered is not None, "still UNHEALTHY after 12s")
    R.check("...and the streak is cleared, not merely ignored",
            obj._failing_since is None, str(obj._failing_since))


def test_get_status_cannot_claim_a_dead_worker_is_running():
    R.section("get_status() reports the thread, not an unresettable flag")

    f = _fuel()
    f.start()
    time.sleep(0.2)
    running_while_up = f.get_status()["running"]
    f.stop()
    time.sleep(0.2)
    running_after_stop = f.get_status()["running"]

    R.check("running is True while the worker is up", running_while_up is True)
    # Pre-fix this was True: run() set self.running = True and nothing reset
    # it, so a dead worker still reported running.
    R.check("running is False once the worker has gone",
            running_after_stop is False, str(running_after_stop))
    R.check("...and it is derived from the thread, so it cannot disagree "
            "with health_report()",
            (f.health_report().state is not HealthState.HEALTHY)
            and running_after_stop is False)


def test_stop_interrupts_the_error_backoff():
    R.section("the backoff waits on the stop Event, it does not sleep")

    f = _fuel()
    f.ERROR_BACKOFF_S = 30.0
    real = f.update_fuel_system
    f.update_fuel_system = lambda: (_ for _ in ()).throw(RuntimeError("x"))
    try:
        f.start()
        time.sleep(0.3)
        t0 = time.monotonic()
        f.stop()
        elapsed = time.monotonic() - t0
        alive = f._thread.is_alive()
    finally:
        f.update_fuel_system = real

    # A time.sleep(30) backoff would have made stop()'s join(timeout=2)
    # return after ~2s with the thread still alive -- the exact failure the
    # note in FlightControlComputer._update_loop records.
    R.check("stop() returns promptly even mid-backoff, not after the join "
            "timeout", elapsed < 1.5, f"{elapsed:.2f}s")
    R.check("...and the worker is actually gone, not merely reported stopped",
            not alive, str(alive))


def test_worker_health_grace_period():
    R.section("worker_health's duration arguments")

    class _T:
        def __init__(self, alive=True, name="W"):
            self._alive = alive
            self.name = name

        def is_alive(self):
            return self._alive

    live = _T()
    now = time.monotonic()

    R.check("a streak inside the grace period stays HEALTHY",
            worker_health("x", live, stop_requested=False,
                          failing_since=now - 1.0,
                          failure_grace_s=15.0).state is HealthState.HEALTHY)
    R.check("a streak past it becomes UNHEALTHY",
            worker_health("x", live, stop_requested=False,
                          failing_since=now - 16.0,
                          failure_grace_s=15.0).state is HealthState.UNHEALTHY)
    R.check("the reason states how long it has been failing",
            "16.0s" in worker_health("x", live, stop_requested=False,
                                     failing_since=now - 16.0,
                                     failure_grace_s=15.0).reason)
    R.check("the last error is carried into the reason",
            "boom" in worker_health("x", live, stop_requested=False,
                                    failing_since=now - 16.0,
                                    failure_grace_s=15.0,
                                    last_error="boom").reason)
    R.check("failing_since=None is HEALTHY however long the process has run",
            worker_health("x", live, stop_requested=False,
                          failing_since=None).state is HealthState.HEALTHY)
    R.check("a grace of 0 disables the check",
            worker_health("x", live, stop_requested=False,
                          failing_since=now - 999.0,
                          failure_grace_s=0).state is HealthState.HEALTHY)
    R.check("a dead worker is UNHEALTHY regardless of the streak",
            worker_health("x", _T(alive=False), stop_requested=False,
                          failing_since=None).state is HealthState.UNHEALTHY)
    R.check("stop requested still wins over a long streak",
            worker_health("x", live, stop_requested=True,
                          failing_since=now - 999.0).state is HealthState.UNKNOWN)

    # NON-TAUTOLOGICAL: this is the pre-fix answer. A caller that does not
    # pass a streak gets HEALTHY for a worker that is failing every pass,
    # which is exactly what all nine did before this change.
    R.check("a caller passing no streak still gets HEALTHY, which is the "
            "blind spot this closed",
            worker_health("x", live, stop_requested=False).state
            is HealthState.HEALTHY)

    R.check("the shared default grace is used when none is given",
            DEFAULT_FAILURE_GRACE_S > 0 and
            worker_health("x", live, stop_requested=False,
                          failing_since=now - (DEFAULT_FAILURE_GRACE_S + 1)
                          ).state is HealthState.UNHEALTHY,
            str(DEFAULT_FAILURE_GRACE_S))


def main():
    print("=" * 60)
    print("  Worker loops: survive a glitch, admit a real failure")
    print("=" * 60)

    for test in (test_every_worker_loop_guards_its_body,
                 test_the_scanner_can_see_an_unguarded_loop,
                 test_one_transient_error_is_not_fatal,
                 test_every_subsystem_records_its_failure_streak,
                 test_sustained_failure_reports_unhealthy_everywhere,
                 test_recovery_clears_the_streak,
                 test_get_status_cannot_claim_a_dead_worker_is_running,
                 test_stop_interrupts_the_error_backoff,
                 test_worker_health_grace_period):
        try:
            test()
        except Exception:
            R.failed += 1
            R._failures.append(f"  FAIL  {test.__name__} raised")
            print(f"  FAIL  {test.__name__} raised:")
            traceback.print_exc()

    print("\n" + "=" * 60)
    ok = R.summary()
    print("=" * 60)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
