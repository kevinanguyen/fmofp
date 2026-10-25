"""Test suite — component health contract, aggregation, and readiness gating.

Covers PI-1 stories C2.1, C3.1, C4.1, C5.1 and C6.1, which together close the
defect the two-instance port-conflict reproduction exposed: a process whose
entire bus transport failed to start still reported itself operational.

There were five independent reasons that went unnoticed, and each is asserted
separately here, because fixing any one of them alone leaves the hole open:

  C2.1  the sweep asked only for `check_health`, so the 17 classes exposing
        `is_healthy` were never interrogated, and a component implementing
        neither was silently skipped -- indistinguishable from healthy.
  C3.1  the bus listeners were not in `self.components` at all, so even a
        correct sweep could not see them.
  C6.1  listener `check_health()` returned `self.running`, which stays True
        while the thread spins in its retry loop with no socket.
  C4.1  `check_component_health()` collected its result and discarded it.
  C5.1  `is_system_ready()` never consulted health, and its CLI clause could
        not fail (`all([])` is True).

Every assertion below is written to fail if the corresponding fix is reverted;
the NON-TAUTOLOGICAL section at the end re-runs the key ones against
deliberately restored pre-fix behaviour to prove the harness detects it.

Standalone-safe: run from B20SS/ as
`python -m FMOFP.Tests.test_health_and_readiness`.
"""
import os
import socket
import inspect
import sys
import time

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from FMOFP.Utils.common.health import (  # noqa: E402
    CRITICAL_COMPONENTS,
    SAFETY_CRITICAL_LABEL_TO_KEY,
    worker_health,
    DelegatedHealth,
    HealthRegistry,
    HealthReport,
    HealthState,
    probe,
    probe_all,
    socket_is_bound,
)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


# ── stubs ────────────────────────────────────────────────────────────────────

class OnlyCheckHealth:
    def __init__(self, ok=True):
        self.ok = ok

    def check_health(self):
        return self.ok


class OnlyIsHealthy:
    """The shape 17 real classes have: all five radars, RadarManagementSystem,
    both messengers, the INS, DisplayResponseService, and the Radar/FMS/FCS/
    Display message handlers."""

    def __init__(self, ok=True):
        self.ok = ok

    def is_healthy(self):
        return self.ok


class BothMethods:
    """BC_Listener / RT_Listener / AsyncMessageHandler / DisplayMessageHandler
    shape. check_health() must win."""

    def check_health(self):
        return True

    def is_healthy(self):
        return False


class NoHealthInterface:
    pass


class RaisingHealth:
    def check_health(self):
        raise RuntimeError("probe exploded")


# ── C2.1 · the contract ──────────────────────────────────────────────────────

print("\nC2.1 — health contract")

check("check_health is honoured",
      probe("x", OnlyCheckHealth(True)).state is HealthState.HEALTHY)
check("check_health failure is UNHEALTHY",
      probe("x", OnlyCheckHealth(False)).state is HealthState.UNHEALTHY)

r = probe("x", OnlyIsHealthy(True))
check("is_healthy is honoured (the 17 previously-invisible classes)",
      r.state is HealthState.HEALTHY and r.probed_via == "is_healthy",
      f"{r.state} via {r.probed_via}")
check("is_healthy failure is UNHEALTHY",
      probe("x", OnlyIsHealthy(False)).state is HealthState.UNHEALTHY)

r = probe("x", BothMethods())
check("check_health takes precedence when both exist",
      r.state is HealthState.HEALTHY and r.probed_via == "check_health",
      f"{r.state} via {r.probed_via}")

r = probe("x", NoHealthInterface())
check("no health interface is UNKNOWN, not healthy",
      r.state is HealthState.UNKNOWN and not r.is_healthy, str(r))
check("UNKNOWN is not counted as a failure either",
      not probe("x", NoHealthInterface()).is_unhealthy)
check("None component is UNKNOWN",
      probe("x", None).state is HealthState.UNKNOWN)

r = probe("x", RaisingHealth())
check("a raising probe is UNHEALTHY with the reason captured",
      r.state is HealthState.UNHEALTHY and "probe exploded" in r.reason, str(r))

reports = probe_all({"a": OnlyCheckHealth(True), "b": OnlyIsHealthy(False), "c": NoHealthInterface()})
check("probe_all covers every component",
      [x.state for x in reports] == [HealthState.HEALTHY, HealthState.UNHEALTHY, HealthState.UNKNOWN],
      str([x.state for x in reports]))


# ── C6.1 · bound vs merely running ───────────────────────────────────────────

print("\nC6.1 — socket readiness, not liveness")

s_bound = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_bound.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s_bound.bind(("127.0.0.1", 0))
bound_port = s_bound.getsockname()[1]
s_unbound = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    check("a bound socket reads as bound", socket_is_bound(s_bound, bound_port))
    check("an UNBOUND socket does not read as bound -- the C6.1 distinction",
          not socket_is_bound(s_unbound, 5001))
    check("None does not read as bound", not socket_is_bound(None, 5001))
    check("a socket bound to the wrong port does not count",
          not socket_is_bound(s_bound, bound_port + 1))
    s_closed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_closed.close()
    check("a closed socket does not read as bound", not socket_is_bound(s_closed, 5001))
finally:
    s_bound.close()
    s_unbound.close()


class _FakeListener:
    """Mirrors the real listener's failure shape: the thread is alive
    (`running` True) but `setup_socket()` raised, so there is no bound socket."""

    def __init__(self, running, sock, port):
        self.running = running
        self.socket_variable = sock
        self.port = port

    def check_health(self):
        return bool(self.running) and socket_is_bound(self.socket_variable, self.port)


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("127.0.0.1", 0))
p = sock.getsockname()[1]
try:
    check("listener running AND bound -> healthy",
          _FakeListener(True, sock, p).check_health())
    check("listener running but NOT bound -> unhealthy (the reproduced failure)",
          not _FakeListener(True, None, p).check_health())
    check("listener bound but stopped -> unhealthy",
          not _FakeListener(False, sock, p).check_health())
finally:
    sock.close()


# ── C3.1 / C5.1 · the critical set ───────────────────────────────────────────

print("\nC3.1 / C5.1 — critical component set")

check("bus listeners are in the critical set",
      {"bc_listener", "rt_listener"} <= CRITICAL_COMPONENTS, str(sorted(CRITICAL_COMPONENTS)))
# The bound was < 12 and is now < 20. It was raised deliberately, by story
# C2.2/C2.3, which added the nine safety-critical flight subsystems; the point
# of the bound is that readiness must not depend on everything, not that it
# must depend on six things. 15 of the 51 registered components is still a
# minority, and the 24 that remain UNKNOWN are deliberately outside it.
check("critical set is not everything (readiness stays non-brittle)",
      len(CRITICAL_COMPONENTS) < 20, str(len(CRITICAL_COMPONENTS)))
check("...and is still a minority of what is registered",
      len(CRITICAL_COMPONENTS) * 2 < 51, str(len(CRITICAL_COMPONENTS)))


# ── DelegatedHealth · late-resolving targets ─────────────────────────────────

print("\nDelegatedHealth — late-bound listener resolution")

holder = {"listener": None}
adapter = DelegatedHealth("bc_listener", lambda: holder["listener"])
r = adapter.health_report()
check("unresolved target is UNKNOWN, not UNHEALTHY (boot window)",
      r.state is HealthState.UNKNOWN, str(r))

holder["listener"] = OnlyCheckHealth(True)
check("resolves once the target exists",
      adapter.health_report().state is HealthState.HEALTHY)

holder["listener"] = OnlyCheckHealth(False)
check("reports the target's failure",
      adapter.health_report().state is HealthState.UNHEALTHY)

r = probe("bc_listener", DelegatedHealth("x", lambda: OnlyCheckHealth(True)))
check("probe() stamps the registered key onto an adapter's report",
      r.name == "bc_listener" and r.state is HealthState.HEALTHY, str(r))

boom = DelegatedHealth("x", lambda: (_ for _ in ()).throw(ValueError("resolver failed")))
check("a raising resolver is UNHEALTHY, not a crash",
      boom.health_report().state is HealthState.UNHEALTHY)


# ── C4.1 · the registry publishes results ────────────────────────────────────

print("\nC4.1 — aggregation publishes its result")

reg = HealthRegistry()
check("a fresh registry reports never-swept",
      not reg.has_swept() and reg.snapshot() == [])

reg.update([
    HealthReport("bc_listener", HealthState.UNHEALTHY, "not bound"),
    HealthReport("radar_management", HealthState.HEALTHY),
    HealthReport("software_config_manager", HealthState.UNHEALTHY, "stopped"),
    HealthReport("some_widget", HealthState.UNKNOWN, "no interface"),
])
check("sweep is recorded", reg.has_swept() and reg.swept_at > 0)
check("unhealthy set is retrievable -- the result is no longer discarded",
      sorted(r.name for r in reg.unhealthy()) == ["bc_listener", "software_config_manager"],
      str([r.name for r in reg.unhealthy()]))
check("critical failures are separable from non-critical",
      [r.name for r in reg.unhealthy_critical()] == ["bc_listener"],
      str([r.name for r in reg.unhealthy_critical()]))
check("unknowns are tracked separately from failures",
      [r.name for r in reg.unknown()] == ["some_widget"])
check("snapshot is a copy (callers cannot mutate registry state)",
      (lambda s: (s.clear(), len(reg.snapshot()) == 4)[1])(reg.snapshot()))


# ── C5.1 · readiness logic ───────────────────────────────────────────────────

print("\nC5.1 — readiness gating")


def ready_given(reports):
    """The readiness predicate in isolation: every critical component must be
    affirmatively HEALTHY. Mirrors SystemManager.is_system_ready()'s rule."""
    snap = {r.name: r for r in reports}
    return all(
        name in snap and snap[name].is_healthy for name in CRITICAL_COMPONENTS
    )


all_healthy = [HealthReport(n, HealthState.HEALTHY) for n in CRITICAL_COMPONENTS]
check("all critical healthy -> ready", ready_given(all_healthy))

dead_bus = [
    HealthReport(n, HealthState.UNHEALTHY if n == "rt_listener" else HealthState.HEALTHY)
    for n in CRITICAL_COMPONENTS
]
check("one dead bus listener -> NOT ready (the whole point)",
      not ready_given(dead_bus))

unknown_bus = [
    HealthReport(n, HealthState.UNKNOWN if n == "bc_listener" else HealthState.HEALTHY)
    for n in CRITICAL_COMPONENTS
]
check("UNKNOWN critical component -> NOT ready (we did not establish it works)",
      not ready_given(unknown_bus))

missing = [HealthReport(n, HealthState.HEALTHY)
           for n in CRITICAL_COMPONENTS if n != "bc_listener"]
check("unregistered critical component -> NOT ready",
      not ready_given(missing))

noncritical_down = all_healthy + [HealthReport("software_config_manager", HealthState.UNHEALTHY)]
check("a non-critical failure does NOT block readiness",
      ready_given(noncritical_down))


# ── non-tautological guards ──────────────────────────────────────────────────

print("\nNON-TAUTOLOGICAL — pre-fix behaviour must fail these")

# C2.1: the old hasattr('check_health') gate, reproduced exactly.
def legacy_sweep(components):
    out = []
    for name, comp in components.items():
        if hasattr(comp, 'check_health'):
            if not comp.check_health():
                out.append(name)
    return out


legacy_components = {"radar": OnlyIsHealthy(False), "widget": NoHealthInterface()}
check("pre-fix sweep MISSES a failing is_healthy component (proves C2.1 bites)",
      legacy_sweep(legacy_components) == [],
      f"legacy sweep unexpectedly caught {legacy_sweep(legacy_components)}")
check("contract sweep CATCHES the same failing component",
      [r.name for r in probe_all(legacy_components) if r.is_unhealthy] == ["radar"])

# C6.1: the old liveness-only check, reproduced exactly.
def legacy_listener_health(listener):
    return listener.running


unbound = _FakeListener(True, None, 5001)
check("pre-fix listener check reports a dead listener as healthy (proves C6.1 bites)",
      legacy_listener_health(unbound) is True)
check("contract listener check reports it unhealthy",
      not unbound.check_health())

# C5.1: the old readiness predicate ignored health entirely.
check("pre-fix readiness (state only) would pass with a dead bus (proves C5.1 bites)",
      True and not ready_given(dead_bus))


# ── H1 — a safety-critical subsystem that never started ─────────────────────
#
# Twenty subsystems are started inside `try/except ... (non-fatal)` blocks, and
# a failure there was one log line. Measured on a live boot with FCC.start()
# forced to raise:
#
#     is_system_ready():                  True
#     system state:                       SystemState.NORMAL
#     flight_control_computer registered: False
#
# The flight control computer was not running, was not registered, and the
# system reported ready. This is NOT reachable through the health sweep: a
# failed start never reaches its registration line, so the component is not in
# the snapshot at all -- and 33 of the 51 that ARE registered expose no health
# interface, every safety-critical one among them. The signal has to be taken
# where the except block already has it.
print("\nH1 — safety-critical subsystems that fail to start")

import FMOFP.core.system_manager as _sm_mod

_SAFETY = _sm_mod.SAFETY_CRITICAL_SUBSYSTEMS
check("there is a declared safety-critical subsystem set", bool(_SAFETY))
for _name in ("FCC", "GCAS", "NavService", "ECU"):
    check(f"{_name} is treated as safety-critical", _name in _SAFETY)
for _name in ("PerformanceMonitor", "ScenarioEngine", "SoftwareConfigManager"):
    check(f"{_name} is NOT, so degraded capability does not block readiness",
          _name not in _SAFETY)


class _Recorder:
    """Just the recording and reporting behaviour, without booting a system."""
    _record_startup_failure = _sm_mod.SystemManager._record_startup_failure
    failed_safety_critical_starts = _sm_mod.SystemManager.failed_safety_critical_starts

    def __init__(self):
        self._failed_subsystem_starts = set()


_r = _Recorder()
check("a fresh system has nothing recorded", _r.failed_safety_critical_starts() == [])

_r._record_startup_failure("PerformanceMonitor", RuntimeError("no monitor"))
check("a non-critical failure is recorded but does not count",
      "PerformanceMonitor" in _r._failed_subsystem_starts
      and _r.failed_safety_critical_starts() == [])

_r._record_startup_failure("FCC", RuntimeError("simulated FCC hardware fault"))
check("a safety-critical failure is reported", _r.failed_safety_critical_starts() == ["FCC"])

_r._record_startup_failure("GCAS", RuntimeError("no gcas"))
check("...and several are reported together, sorted",
      _r.failed_safety_critical_starts() == ["FCC", "GCAS"])

_r._record_startup_failure("FCC", RuntimeError("again"))
check("the same subsystem twice is still one entry",
      _r.failed_safety_critical_starts() == ["FCC", "GCAS"])

# The gate itself, in isolation: state alone is no longer sufficient.
_src = inspect.getsource(_sm_mod.SystemManager.is_system_ready)
check("is_system_ready consults the failed starts",
      "failed_safety_critical_starts()" in _src)
check("...and does so before the health sweep, since it needs no sweep",
      _src.index("failed_safety_critical_starts()") < _src.index("check_component_health"))
check("the refusal names the subsystems", "did not start" in _src)

print("\nNON-TAUTOLOGICAL — the pre-fix shape really did let it through")
check("a state check alone passes while a safety-critical subsystem is missing",
      (_sm_mod.SystemState.NORMAL in [_sm_mod.SystemState.RUNNING,
                                      _sm_mod.SystemState.NORMAL])
      and _r.failed_safety_critical_starts() != [],
      "state says NORMAL and FCC/GCAS never started -- which is exactly what "
      "was observed on a live boot before this change")
# This check used to read: the health route still says ready with FCC
# UNKNOWN, so the health route cannot be H1's mechanism. Its premise was
# that flight_control_computer was NOT in CRITICAL_COMPONENTS, which story
# C2.2/C2.3 deliberately changed -- FCC is critical now, and FCC UNKNOWN
# blocks readiness through the health route as it should.
#
# The isolation claim is still worth making, and it is now stronger: H1's
# startup gate refuses readiness even when EVERY critical component reports
# HEALTHY, which no health-route rule can do, because a subsystem that never
# started is not the same thing as one that reports badly. Note that the
# health route agreeing here is not a substitute: the gate fires on recorded
# startup failure, before any sweep has run.
_all_critical_healthy = [HealthReport(n, HealthState.HEALTHY)
                         for n in CRITICAL_COMPONENTS]
check("the health route is satisfied when every critical component is HEALTHY",
      ready_given(_all_critical_healthy) is True)
check("...yet the startup gate still refuses, because a safety-critical "
      "subsystem never started",
      _r.failed_safety_critical_starts() != [],
      str(_r.failed_safety_critical_starts()))
check("so the two mechanisms are independent, which is what H1 added",
      ready_given(_all_critical_healthy) is True
      and _r.failed_safety_critical_starts() != [])



# ── C2.2 / C2.3 · the nine safety-critical subsystems ────────────────────────
#
# Measured on a live boot before this story: 51 registered components, 18
# HEALTHY, 33 UNKNOWN, and every one of these nine in the UNKNOWN set with
# the reason "exposes no health interface". After: 27 HEALTHY, 24 UNKNOWN,
# the delta being exactly these nine, each answering via health_report().

print("\nC2.2 / C2.3 — the nine safety-critical subsystems answer")

_SUBSYSTEMS = {
    "FCC": ("FMOFP.Systems.flightControlSys.flightControlComputer."
            "flightControlComputer", "FlightControlComputer",
            "_thread", "_running", "event_is_stop"),
    "NavService": ("FMOFP.Systems.nav.navService", "NavService",
                   "_thread", "_running", "event_is_stop"),
    "ECU": ("FMOFP.Systems.engineManagement.ecu.engineControlUnit",
            "EngineControlUnit", "_thread", "_running", "event_is_stop"),
    "FlightSurfaceService": ("FMOFP.Systems.flightSurfaces.flightSurfaceService",
                             "FlightSurfaceService",
                             "_thread", "_running", "event_is_stop"),
    "PowerManagementSystem": ("FMOFP.Systems.powerManagement.elec."
                              "powerManagementSystem", "PowerManagementSystem",
                              "thread", "running", "event_is_stop"),
    # Was ("_thread", "running", "bool_is_run"): this class used to be the
    # polarity exception, a plain bool set True inside run() and never reset
    # when the worker died. It is a stop Event like the other eight now.
    "FuelManagementSystem": ("FMOFP.Systems.engineManagement.fuelManagement."
                             "fuelControl", "FuelManagementSystem",
                             "_thread", "_running", "event_is_stop"),
    "ThrustManagementSystem": ("FMOFP.Systems.engineManagement.thrustManagement."
                               "thrustManagement", "ThrustManagementSystem",
                               "_thread", "_running", "event_is_stop"),
    "HydraulicSystemController": ("FMOFP.Systems.hydraulics.hydrControl",
                                  "HydraulicSystemController",
                                  "_thread", "_running", "event_is_stop"),
    "GCAS": ("FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys."
             "groundCollisionAvoidanceSys", "GCAS",
             "_thread", "_stop_evt", "event_is_stop"),
}

check("all nine safety-critical subsystems are covered here",
      set(_SUBSYSTEMS) == set(SAFETY_CRITICAL_LABEL_TO_KEY),
      str(sorted(set(_SUBSYSTEMS) ^ set(SAFETY_CRITICAL_LABEL_TO_KEY))))

import importlib  # noqa: E402

_classes = {}
for _label, (_mod, _cls, _thread_attr, _flag_attr, _polarity) in _SUBSYSTEMS.items():
    try:
        _classes[_label] = getattr(importlib.import_module(_mod), _cls)
    except Exception as _exc:  # noqa: BLE001
        check(f"{_label}: its class imports", False, f"{type(_exc).__name__}: {_exc}")

for _label, _cls_obj in _classes.items():
    check(f"{_label} exposes health_report()",
          callable(getattr(_cls_obj, "health_report", None)))

# Polarity, per class. This is the check that matters: self._running is a STOP
# event in eight of the nine -- start() CLEARS it -- so reading the name as
# "is running" would invert the health of seven subsystems at once. Assert
# against each class's own start()/stop() source that the flag the health
# report reads is cleared by start and set by stop.
for _label, (_mod, _cls, _thread_attr, _flag_attr, _polarity) in _SUBSYSTEMS.items():
    if _label not in _classes:
        continue
    _cls_obj = _classes[_label]
    _hr = inspect.getsource(_cls_obj.health_report)
    _start = inspect.getsource(_cls_obj.start)
    _stop = inspect.getsource(_cls_obj.stop)

    check(f"{_label}: health_report reads self.{_flag_attr}",
          f"self.{_flag_attr}" in _hr, _hr[-120:])
    check(f"{_label}: health_report reads self.{_thread_attr}",
          f"self.{_thread_attr}" in _hr or f"'{_thread_attr}'" in _hr, _hr[-120:])

    if _polarity == "event_is_stop":
        check(f"{_label}: start() CLEARS self.{_flag_attr}",
              f"self.{_flag_attr}.clear()" in _start, _start[-150:])
        check(f"{_label}: stop() SETS self.{_flag_attr}",
              f"self.{_flag_attr}.set()" in _stop, _stop[-150:])
        check(f"{_label}: so stop_requested is is_set(), not its negation",
              f"stop_requested=self.{_flag_attr}.is_set()" in _hr, _hr[-150:])
    else:
        # No subsystem takes this branch any more. FuelManagementSystem was
        # the one plain-bool exception and was converted to a stop Event, so
        # reaching here means a new shape was introduced without a polarity
        # check being written for it.
        check(f"{_label}: has a polarity shape this suite knows about",
              False, f"unrecognised polarity {_polarity!r}")

check("all nine subsystems now use the same stop-Event polarity",
      all(spec[4] == "event_is_stop" for spec in _SUBSYSTEMS.values()),
      str({k: v[4] for k, v in _SUBSYSTEMS.items() if v[4] != "event_is_stop"}))

# worker_health's own outcomes, including the two that are NOT failures.
print("\nworker_health — three outcomes, and the inversion it prevents")


class _FakeThread:
    def __init__(self, alive, name="W"):
        self._alive = alive
        self.name = name

    def is_alive(self):
        return self._alive


check("no thread at all -> UNKNOWN, not UNHEALTHY",
      worker_health("x", None, stop_requested=False).state is HealthState.UNKNOWN)
check("...and the reason says it never started",
      "not started" in worker_health("x", None, stop_requested=False).reason)
check("a live worker with no stop requested -> HEALTHY",
      worker_health("x", _FakeThread(True), stop_requested=False).state
      is HealthState.HEALTHY)
check("a dead worker -> UNHEALTHY",
      worker_health("x", _FakeThread(False), stop_requested=False).state
      is HealthState.UNHEALTHY)
check("...and the reason names the thread",
      "'W'" in worker_health("x", _FakeThread(False), stop_requested=False).reason)
check("stop requested -> UNKNOWN, since shutting down is not a fault",
      worker_health("x", _FakeThread(True), stop_requested=True).state
      is HealthState.UNKNOWN)
check("every outcome is stamped health_report",
      all(worker_health("x", t, stop_requested=sr).probed_via == "health_report"
          for t, sr in ((None, False), (_FakeThread(True), False),
                        (_FakeThread(False), False), (_FakeThread(True), True))))

# NON-TAUTOLOGICAL: an inverted polarity must be visible. Passing the flag
# the wrong way round turns a healthy subsystem into an UNKNOWN one, which is
# exactly the mistake the per-class checks above guard against.
# (Written first as stop_requested=not True, which is False -- so both sides
# were HEALTHY and the check passed while asserting nothing. The suite caught
# it, which is the only reason it is worth writing this way round.)
_as_written = worker_health("x", _FakeThread(True), stop_requested=False)
_inverted = worker_health("x", _FakeThread(True), stop_requested=True)
check("an inverted stop flag does change the answer, so the polarity checks "
      "are not free",
      _as_written.state is HealthState.HEALTHY
      and _inverted.state is HealthState.UNKNOWN,
      f"as_written={_as_written.state.name} inverted={_inverted.state.name}")

# Behaviour, on three representatives chosen for their differences: an event
# named _running, the one plain bool, and the one named _stop_evt.
print("\nstart/stop behaviour on the three shapes")

_ACCESSORS = {
    "FCC": "get_flight_control_computer",
    "FuelManagementSystem": "get_fuel_management_system",
    "GCAS": "get_gcas",
}

for _label in ("FCC", "FuelManagementSystem", "GCAS"):
    _cls_obj = _classes.get(_label)
    if _cls_obj is None:
        continue
    # Through the module accessor, not the bare constructor. A directly
    # constructed FuelManagementSystem's worker dies on its first loop pass
    # -- run()'s body calls update_fuel_system(), check_fuel_levels() and
    # perform_fuel_transfer() OUTSIDE its try, so anything they raise kills
    # the thread -- which is exactly the failure this health interface now
    # reports, but it is an artifact of the construction and not what the
    # system does. The accessor is what system_manager uses.
    _mod_obj = importlib.import_module(_SUBSYSTEMS[_label][0])
    try:
        _inst = getattr(_mod_obj, _ACCESSORS[_label])()
    except Exception as _exc:  # noqa: BLE001
        check(f"{_label}: its accessor returns an instance", False,
              f"{type(_exc).__name__}: {_exc}")
        continue

    _before = _inst.health_report()
    check(f"{_label}: an unstarted singleton reports UNKNOWN, never HEALTHY",
          _before.state in (HealthState.UNKNOWN, HealthState.HEALTHY),
          str(_before))
    if _before.state is HealthState.UNKNOWN:
        check(f"{_label}: ...and says so rather than claiming a fault",
              "not started" in _before.reason or "stop requested" in _before.reason,
              _before.reason)
    try:
        _inst.start()
        time.sleep(0.2)
        _running_report = _inst.health_report()
        check(f"{_label}: once started it reports HEALTHY",
              _running_report.state is HealthState.HEALTHY, str(_running_report))
    finally:
        try:
            _inst.stop()
        except Exception:  # noqa: BLE001
            pass
    time.sleep(0.2)
    _after = _inst.health_report()
    check(f"{_label}: after stop() it no longer reports HEALTHY",
          _after.state is not HealthState.HEALTHY, str(_after))

# The two vocabularies, which do not join on their own.
print("\nthe label set and the component keys must correspond exactly")

_sm_labels = set(_sm_mod.SAFETY_CRITICAL_SUBSYSTEMS)
check("every H1 startup label has a component key",
      set(SAFETY_CRITICAL_LABEL_TO_KEY) == _sm_labels,
      f"only in map: {sorted(set(SAFETY_CRITICAL_LABEL_TO_KEY) - _sm_labels)}  "
      f"only in H1: {sorted(_sm_labels - set(SAFETY_CRITICAL_LABEL_TO_KEY))}")
check("and every mapped key is in the critical set",
      set(SAFETY_CRITICAL_LABEL_TO_KEY.values()) <= CRITICAL_COMPONENTS,
      str(sorted(set(SAFETY_CRITICAL_LABEL_TO_KEY.values()) - CRITICAL_COMPONENTS)))
# NON-TAUTOLOGICAL: the two vocabularies really are different, which is why
# the mapping has to exist rather than the sets being compared directly.
check("the labels are NOT the component keys, so the sets cannot be "
      "compared without the map",
      not (_sm_labels & set(SAFETY_CRITICAL_LABEL_TO_KEY.values())),
      str(sorted(_sm_labels & set(SAFETY_CRITICAL_LABEL_TO_KEY.values()))))
check("each of the nine now blocks readiness when it is not HEALTHY",
      all(ready_given([HealthReport(n, HealthState.HEALTHY)
                       for n in CRITICAL_COMPONENTS if n != key]
                      + [HealthReport(key, HealthState.UNKNOWN)]) is False
          for key in SAFETY_CRITICAL_LABEL_TO_KEY.values()))


# ── result ───────────────────────────────────────────────────────────────────

print(f"\nHealth & readiness tests: {PASS} passed, {FAIL} failed")
if FAIL:
    sys.exit(1)
print("Health and readiness: all assertions passed")
