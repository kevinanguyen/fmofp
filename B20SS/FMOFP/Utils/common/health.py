"""
Component health contract (PI-1 story C2.1).

WHY THIS EXISTS
---------------
Before this module there were two competing health interfaces in the codebase
and no definition of which one was authoritative:

    check_health()   9 definitions across 7 files
    is_healthy()    22 definitions across 21 files

`SystemManager.check_component_health()` interrogated components with

    if hasattr(component, 'check_health'):

so the 17 classes that expose ONLY `is_healthy()` -- all five radars,
`RadarManagementSystem`, both messengers, the INS, `DisplayResponseService`,
and the Radar/FMS/FCS/Display message handlers -- were never asked at all.
Of the registered components, at most ~8 could be interrogated, and a
component implementing neither method was silently treated as healthy
because the `hasattr` guard simply skipped it.

That last part is the dangerous half. "We never asked" and "we asked and it
said yes" were indistinguishable, so the health sweep could report a clean
sweep over a system it had barely inspected.

THE CONTRACT
------------
A component is health-interrogable if it exposes `health_report()`,
`check_health()` or `is_healthy()`, resolved in that order. `health_report()`
returns a HealthReport directly and exists for adapters that must resolve their
real target at probe time (see DelegatedHealth); the other two return a bool.
The bool forms are supported permanently -- `is_healthy()` has eight
live internal call sites (RadarMessageHandler, DisplayMessageHandler,
FMSMessageHandler) that are not going away, and renaming 22 definitions to
unify the spelling would be churn with no behavioural benefit.

Where a class defines both `check_health()` and `is_healthy()` -- BC_Listener,
RT_Listener, AsyncMessageHandler, DisplayMessageHandler -- `check_health()`
wins, which is the historical behaviour of the sweep.

Three outcomes, not two:

    HEALTHY    the component was asked and said yes
    UNHEALTHY  the component was asked and said no, or raised
    UNKNOWN    the component exposes no health interface -- we did not ask

UNKNOWN is deliberately NOT an alias for HEALTHY. A caller deciding whether
the system is ready must be able to distinguish "nothing is wrong" from
"we have no idea", and the previous code could not.

Deliberately dependency-free (stdlib + the project logger) so it can be
imported from the socket layer, the system manager, and the tests without
creating an import cycle.
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()


class HealthState(Enum):
    """Outcome of interrogating one component."""

    HEALTHY = "healthy"
    UNHEALTHY = "unhealthy"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class HealthReport:
    """One component's health at one moment.

    `probed_via` records which method answered ('health_report', 'check_health',
    'is_healthy', or None when the component exposes none of them), so a
    confusing result can be traced back to the interface that produced it
    without re-deriving it.
    """

    name: str
    state: HealthState
    reason: str = ""
    probed_via: Optional[str] = None

    @property
    def is_healthy(self) -> bool:
        """True only for HEALTHY. UNKNOWN is not healthy -- see module docstring."""
        return self.state is HealthState.HEALTHY

    @property
    def is_unhealthy(self) -> bool:
        """True only for UNHEALTHY. UNKNOWN is not a failure either."""
        return self.state is HealthState.UNHEALTHY


#: Components whose failure means the system cannot do useful work, and which
#: therefore block readiness (story C5.1). The criterion is "if this is down,
#: the simulation cannot move data end to end" -- not "this is important".
#:
#: The bus listeners are here because a dead transport is exactly the failure
#: the two-instance port-conflict reproduction exposes, and the whole point of
#: this work is that such a system must not report itself operational.
#:
#: Deliberately NOT critical: the software config manager, flight-data
#: monitoring, fitness monitoring, the CLI threads, and the various airframe
#: monitors. Any of those being down degrades the simulation without making it
#: dishonest, and grounding boot on them would make readiness brittle.
#: The nine safety-critical flight subsystems (story C2.2/C2.3). These are
#: keyed by COMPONENT REGISTRY key, which is not how story H1 spells them:
#: SystemManager.SAFETY_CRITICAL_SUBSYSTEMS holds startup LABELS ("FCC",
#: "ECU", "NavService") because that is what _record_startup_failure() is
#: given, while sm.components is keyed in snake_case. The two vocabularies
#: do not join, and anyone adding one set to the other without noticing gets
#: a silently empty intersection. SAFETY_CRITICAL_LABEL_TO_KEY below is the
#: mapping, and test_health_and_readiness asserts the two sets correspond
#: exactly, so they cannot drift apart.
SAFETY_CRITICAL_LABEL_TO_KEY = {
    "FCC": "flight_control_computer",
    "FlightSurfaceService": "flight_surface_service",
    "HydraulicSystemController": "hydraulic_system_controller",
    "ECU": "engine_control_unit",
    "ThrustManagementSystem": "thrust_management_system",
    "FuelManagementSystem": "fuel_management_system",
    "PowerManagementSystem": "power_management_system",
    "GCAS": "gcas",
    "NavService": "nav_service",
}

CRITICAL_COMPONENTS = frozenset({
    "bc_listener",
    "rt_listener",
    "async_message_handler",
    "message_routing_service",
    "radar_management",
    "flightManagementSystem",
    # Story C2.2/C2.3. These nine could not be here before: they exposed no
    # health interface, so the sweep answered UNKNOWN for every one of them,
    # and UNKNOWN blocks readiness -- putting them here would have wedged
    # the boot permanently rather than gating it. That is also why story H1
    # had to gate on a safety-critical subsystem FAILING TO START rather
    # than on its health. Each now reports through worker_health(), and a
    # live boot answers HEALTHY for all nine (measured: the sweep went from
    # 18 HEALTHY / 33 UNKNOWN to 27 HEALTHY / 24 UNKNOWN, the delta being
    # exactly these).
    #
    # They meet the criterion stated above -- "if this is down, the
    # simulation cannot move data end to end" -- in the sense that matters
    # for a flight trainer: an aircraft with a dead flight control computer,
    # dead surfaces, dead hydraulics or no navigation is not a system that
    # should report itself ready to train on.
    *SAFETY_CRITICAL_LABEL_TO_KEY.values(),
})


def probe(name: str, component: Any) -> HealthReport:
    """Interrogate one component against the contract above.

    Never raises: a component whose health check itself blows up is reported
    UNHEALTHY with the exception text as the reason, which is strictly more
    informative than the previous behaviour of logging and appending the name
    with no detail.
    """
    if component is None:
        return HealthReport(name, HealthState.UNKNOWN, "component is None")

    reporter = getattr(component, "health_report", None)
    if callable(reporter):
        try:
            report = reporter()
        except Exception as exc:  # noqa: BLE001
            return HealthReport(
                name,
                HealthState.UNHEALTHY,
                f"health_report() raised {type(exc).__name__}: {exc}",
                "health_report",
            )
        if isinstance(report, HealthReport):
            # Adapters don't know the key they were registered under; stamp it.
            return HealthReport(name, report.state, report.reason, report.probed_via)

    for method_name in ("check_health", "is_healthy"):
        method = getattr(component, method_name, None)
        if not callable(method):
            continue
        try:
            result = method()
        except Exception as exc:  # noqa: BLE001 - contract: a raising probe is a failure
            return HealthReport(
                name,
                HealthState.UNHEALTHY,
                f"{method_name}() raised {type(exc).__name__}: {exc}",
                method_name,
            )
        if result:
            return HealthReport(name, HealthState.HEALTHY, "", method_name)
        return HealthReport(
            name, HealthState.UNHEALTHY, f"{method_name}() returned {result!r}", method_name
        )

    return HealthReport(
        name,
        HealthState.UNKNOWN,
        "exposes no health interface "
        "(health_report(), check_health() or is_healthy())",
        None,
    )


#: How long a worker must have been failing CONTINUOUSLY before its
#: subsystem is reported UNHEALTHY rather than HEALTHY. Fifteen seconds is
#: long enough that a transient error costs nothing and short enough that an
#: operator is not looking at a stale green.
DEFAULT_FAILURE_GRACE_S = 15.0


def worker_health(name: str, thread: Optional[threading.Thread], *,
                  stop_requested: bool,
                  failing_since: Optional[float] = None,
                  failure_grace_s: float = DEFAULT_FAILURE_GRACE_S,
                  last_error: str = "") -> HealthReport:
    """Health of a subsystem whose work is done by one background thread.

    Story C2.2/C2.3. Nine safety-critical subsystems exposed no health
    interface at all, so the sweep reported UNKNOWN for every one of them --
    measured on a live boot: 51 components, 18 HEALTHY, 33 UNKNOWN, with
    the flight control computer, the flight surface service, the hydraulic
    system controller, the ECU, thrust and fuel management, power
    management, GCAS and the nav service all in the UNKNOWN set. That is
    also why story H1 could only gate readiness on a safety-critical
    subsystem FAILING TO START: UNKNOWN blocks readiness, so putting these
    in CRITICAL_COMPONENTS while they answered UNKNOWN would have wedged
    the boot permanently.

    The signal is not invented for the occasion. Every one of these
    subsystems runs exactly one daemon worker thread and already publishes
    `'running': self._thread is not None and self._thread.is_alive()` from
    its own get_status(). This surfaces that same fact through the health
    contract, with a reason attached.

    The polarity of the run flag is NOT uniform across them and is
    therefore the caller's business, passed in as `stop_requested`:

        FCC, nav, ECU, flight surfaces, power, thrust, hydraulics
                        self._running / self.running is a STOP event,
                        cleared by start() and set by stop(), so
                        stop_requested=self._running.is_set()
        GCAS            the same, spelled self._stop_evt
        fuel management self.running is a plain bool set True by start(),
                        so stop_requested=not self.running

    Reading any of those as "running" because of its name would inverted
    seven of the nine.

    Three outcomes, matching the module contract:

        UNKNOWN    no thread yet (never started), or stop already
                   requested. Neither is a fault, and neither is a claim
                   that the subsystem is working.
        UNHEALTHY  a thread was created and is not alive. The subsystem was
                   started and its worker has died, which is the failure
                   this exists to catch. OR: the worker is alive but has
                   failed `failure_threshold` consecutive passes.
        HEALTHY    the worker is alive, no stop has been requested, and it
                   is not failing every pass.

    ALIVE IS NOT THE SAME AS WORKING, which is why failing_since exists.
    Guarding a worker loop so a transient error cannot kill the thread --
    the right thing to do, and what all nine subsystems now do -- converts
    "the worker died" into "the worker is alive and achieving nothing", and
    a liveness-only check calls that HEALTHY. Measured on the real
    FlightControlComputer with every pass raising: thread alive, health
    HEALTHY, reason empty, and readiness granted, because FCC is in
    CRITICAL_COMPONENTS.

    WHY A DURATION AND NOT A COUNT. This argument was first written as
    consecutive_failures with a threshold of 3, which is wrong across these
    nine because a pass is not a unit of time here. Their loop cadences run
    from 0.02 s (FlightSurfaceService, 50 Hz) to 1.0 s (hydraulics), and
    four of the nine back off 5 s on error while four fall straight through
    to their normal cadence. Three consecutive failures therefore means
    15 s on the FCC and 0.3 s on the nav service -- the same threshold
    reporting a momentary glitch as a failure on one subsystem and taking
    a quarter of a minute to notice a dead one on another. A wall-clock
    grace period means the same thing everywhere regardless of rate.

    `failing_since` is the monotonic timestamp of the first failure in the
    current unbroken streak, or None when the last pass succeeded. The
    subsystem sets it in its except branch and clears it as the last
    statement of its try body, so a single success resets it. A caller that
    does not track failures passes nothing and keeps the liveness-only
    behaviour.
    """
    if thread is None:
        return HealthReport(name, HealthState.UNKNOWN,
                            "not started: no worker thread", "health_report")
    if stop_requested:
        return HealthReport(name, HealthState.UNKNOWN,
                            f"stop requested; worker {thread.name!r} "
                            f"{'still running' if thread.is_alive() else 'stopped'}",
                            "health_report")
    if not thread.is_alive():
        return HealthReport(name, HealthState.UNHEALTHY,
                            f"worker thread {thread.name!r} is not alive",
                            "health_report")
    if failing_since is not None and failure_grace_s > 0:
        failing_for = time.monotonic() - failing_since
        if failing_for >= failure_grace_s:
            detail = f": {last_error}" if last_error else ""
            return HealthReport(
                name, HealthState.UNHEALTHY,
                f"worker {thread.name!r} is alive but every pass has failed "
                f"for {failing_for:.1f}s{detail}",
                "health_report")
    return HealthReport(name, HealthState.HEALTHY, "", "health_report")


class DelegatedHealth:
    """Health adapter that resolves its real target at probe time.

    Needed because the two bus listeners cannot simply be registered directly
    (story C3.1):

      * `Bus_Controller` and `Remote_Terminal` -- the objects `SystemManager`
        holds -- are thin wrappers that expose no health interface at all.
        Registering them would report UNKNOWN forever, which is the same blind
        spot in a new costume.

      * `BC_Listener` is NOT a singleton. `Bus_Controller.start_listener()`
        constructs a fresh `BC_Listener()` and keeps it as `self._listener`,
        so the module-level instance returned by `get_bc_listener()` is a
        DIFFERENT object that never binds a port and would report unhealthy
        forever -- a false negative that would block boot outright. (RT does
        use the global via `get_rt_listener()`; the two sides differ.)

      * The BC listener is created inside the listener THREAD, so it does not
        exist at registration time. Resolving late is the only correct option.

    Before the target exists the adapter reports UNKNOWN rather than UNHEALTHY,
    so a component that simply has not started yet is never mistaken for one
    that has failed.
    """

    def __init__(self, description: str, resolver: Any) -> None:
        self._description = description
        self._resolver = resolver

    def health_report(self) -> HealthReport:
        try:
            target = self._resolver()
        except Exception as exc:  # noqa: BLE001
            return HealthReport(
                self._description,
                HealthState.UNHEALTHY,
                f"resolving {self._description} raised {type(exc).__name__}: {exc}",
                "health_report",
            )
        if target is None:
            return HealthReport(
                self._description,
                HealthState.UNKNOWN,
                f"{self._description} not created yet",
                "health_report",
            )
        return probe(self._description, target)


def probe_all(components: Dict[str, Any]) -> List[HealthReport]:
    """Probe every component, preserving iteration order."""
    return [probe(name, component) for name, component in components.items()]


def socket_is_bound(sock: Any, expected_port: Optional[int] = None) -> bool:
    """True only if `sock` is a live socket actually bound to `expected_port`.

    This is the distinction story C6.1 turns on. A listener whose `setup_socket()`
    failed still holds a socket OBJECT -- construction succeeded, only `bind()`
    raised -- and its thread is still alive, spinning in the retry loop. Testing
    `self.running`, or even `socket_variable is not None`, reports such a listener
    as healthy while it can neither accept nor receive anything.

    An unbound socket answers `getsockname()` with port 0, so comparing the bound
    port against the port we intended is what actually separates "listening" from
    "holding a useless file descriptor".
    """
    if sock is None:
        return False
    try:
        if sock.fileno() < 0:  # closed
            return False
        bound_host, bound_port = sock.getsockname()[:2]
    except Exception:
        # Closed, detached, or not a socket -- all mean "not listening".
        return False
    if not bound_port:
        return False
    if expected_port is not None and bound_port != expected_port:
        return False
    return True


class HealthRegistry:
    """Thread-safe holder for the most recent sweep.

    The health monitor runs on its own thread while readiness is evaluated from
    a Qt timer on the main thread, so the last result crosses threads and needs
    a lock. Reads are cheap and frequent (every 100 ms during boot); writes
    happen once per health-check interval.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._reports: List[HealthReport] = []
        self._swept_at: float = 0.0

    def update(self, reports: List[HealthReport]) -> None:
        with self._lock:
            self._reports = list(reports)
            self._swept_at = time.time()

    def snapshot(self) -> List[HealthReport]:
        with self._lock:
            return list(self._reports)

    @property
    def swept_at(self) -> float:
        """Wall-clock time of the last sweep, or 0.0 if none has run."""
        with self._lock:
            return self._swept_at

    def unhealthy(self) -> List[HealthReport]:
        return [r for r in self.snapshot() if r.is_unhealthy]

    def unknown(self) -> List[HealthReport]:
        return [r for r in self.snapshot() if r.state is HealthState.UNKNOWN]

    def unhealthy_critical(self) -> List[HealthReport]:
        return [r for r in self.unhealthy() if r.name in CRITICAL_COMPONENTS]

    def has_swept(self) -> bool:
        return self.swept_at > 0.0
