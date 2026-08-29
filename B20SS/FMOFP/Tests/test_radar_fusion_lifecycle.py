"""Test suite — cross-radar data fusion runs on every start, cold or warm.

Guards a determinism defect in the radar-management lifecycle: the fusion
worker started on a machine's FIRST EVER run and never again.

`RadarDataFusion` is started by `RadarManagementSystem`. That start call used
to live in `initialize_radars()`, on the branch taken only when the persistent
`radar_init` marker was absent. The marker is written to `FMOFP/tracking/` and
nothing ever clears it, so the branch ran once per working copy. Every later
run took the "already completed" path, rebuilt the radar objects, and returned
without starting the worker — leaving TSD and EICAS with no fused tracks.

Nothing raised, and `get_fused_tracks()` stayed callable throughout; it simply
returned an empty list forever. A check that only asserts "initialization
returned without throwing" therefore passes in both the working and broken
states, which is why the assertions below require the worker thread to be
alive AND its update loop to have actually executed.

Two runs cannot share a process: the marker, the `RadarManagementSystem`
singleton and the `RadarDataFusion` singleton are all process-scoped, so a
second in-process start would observe state left by the first. Each scenario
therefore runs in a subprocess, the same approach ci_test_boot_smoke.py uses.

Standalone-safe: run from B20SS/ as
`python -m FMOFP.Tests.test_radar_fusion_lifecycle`.
"""
import os
import subprocess
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

TIMEOUT_S = 180
MARKER = os.path.join(_B20SS, 'FMOFP', 'tracking',
                      'radar_init_radar_management.lock')

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


# Child program: bring the radar management system up through its real
# lifecycle and report whether the fusion worker is genuinely running.
#
# `_update` is wrapped with a counter BEFORE start() so the loop body executing
# is observable. Thread liveness alone would not distinguish a running worker
# from one that started and immediately died.
CHILD = r'''
import os, sys, threading, time
sys.path.insert(0, os.getcwd())
from FMOFP.Utils.dual_path_compat import install as _i; _i()
from PyQt6.QtWidgets import QApplication
_app = QApplication([])

marker = os.path.join('FMOFP', 'tracking', 'radar_init_radar_management.lock')
marker_before = os.path.exists(marker)

from FMOFP.Systems.radarManagement.radar_data_fusion import get_radar_data_fusion
fusion = get_radar_data_fusion()

updates = {'n': 0}
_real_update = fusion._update
def _counting_update():
    updates['n'] += 1
    return _real_update()
fusion._update = _counting_update

from FMOFP.Systems.radarManagement.radarControl import get_radar_management_system
rms = get_radar_management_system()
rms.initialize()
rms.start()

deadline = time.time() + 8.0
while time.time() < deadline and updates['n'] < 2:
    time.sleep(0.05)

worker = next((t for t in threading.enumerate() if t.name == 'RadarDataFusion'), None)
alive = bool(worker and worker.is_alive())
stop_set = fusion._stop_event.is_set()
radars = len(rms.radars)

print("RESULT "
      f"marker_before={marker_before} alive={alive} stop_set={stop_set} "
      f"updates={updates['n']} radars={radars}", flush=True)
sys.stdout.flush()
sys.stderr.flush()
# os._exit: Qt/qasync and the subsystem threads keep a normal interpreter exit
# from completing. Output is flushed above because os._exit skips that.
os._exit(0)
'''


def run_child(label):
    """Run one lifecycle scenario; return the parsed RESULT fields."""
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in (_B20SS, os.path.join(_B20SS, 'FMOFP'),
                    env.get("PYTHONPATH", "")) if p
    )
    try:
        proc = subprocess.run(
            [sys.executable, "-c", CHILD], timeout=TIMEOUT_S,
            capture_output=True, text=True, env=env, cwd=_B20SS,
        )
    except subprocess.TimeoutExpired:
        print(f"  ({label}: TIMED OUT after {TIMEOUT_S}s)")
        return None
    line = next((ln for ln in (proc.stdout or "").splitlines()
                 if ln.startswith("RESULT ")), None)
    if line is None:
        tail = "\n".join((proc.stderr or proc.stdout or "").splitlines()[-15:])
        print(f"  ({label}: child produced no RESULT line, exit "
              f"{proc.returncode})\n{tail}")
        return None
    out = {}
    for field in line[len("RESULT "):].split():
        k, _, v = field.partition("=")
        out[k] = v
    return out


print("Radar fusion lifecycle (each scenario subprocess-isolated):\n")

# ── scenario 1: cold start — no radar_init marker ────────────────────────────
if os.path.exists(MARKER):
    os.remove(MARKER)

cold = run_child("cold")
check("cold: scenario produced a result", cold is not None)
if cold:
    check("cold: took the first-run path", cold["marker_before"] == "False",
          f"marker_before={cold.get('marker_before')}")
    check("cold: all five radars constructed", cold["radars"] == "5",
          f"radars={cold.get('radars')}")
    check("cold: fusion worker thread alive", cold["alive"] == "True")
    check("cold: fusion not in stopped state", cold["stop_set"] == "False")
    check("cold: fusion update loop actually executed",
          int(cold["updates"]) >= 2, f"updates={cold.get('updates')}")

# ── scenario 2: warm start — marker left behind by the cold run ──────────────
# This is the regression guard. Before the fix this scenario reported
# alive=False / updates=0 while every other field matched the cold run.
check("warm: cold run left the radar_init marker behind",
      os.path.exists(MARKER),
      "marker absent — scenario 2 would not exercise the warm path")

warm = run_child("warm")
check("warm: scenario produced a result", warm is not None)
if warm:
    check("warm: took the already-initialized path",
          warm["marker_before"] == "True",
          f"marker_before={warm.get('marker_before')} — not actually a warm start")
    check("warm: all five radars constructed", warm["radars"] == "5",
          f"radars={warm.get('radars')}")
    check("warm: fusion worker thread alive", warm["alive"] == "True",
          "fusion did not start on a warm boot")
    check("warm: fusion not in stopped state", warm["stop_set"] == "False")
    check("warm: fusion update loop actually executed",
          int(warm["updates"]) >= 2,
          f"updates={warm.get('updates')} — worker present but loop not running")

# ── scenario 3: cold and warm are equivalent ─────────────────────────────────
if cold and warm:
    check("cold and warm produce the same fusion state",
          (cold["alive"], cold["stop_set"]) == (warm["alive"], warm["stop_set"]),
          f"cold={cold} warm={warm}")

# ── scenario 4: start() is idempotent and survives a stop/start cycle ────────
# In-process; no marker interaction needed. Protects the "safe to call
# repeatedly" contract that starting from the lifecycle method now relies on.
from FMOFP.Systems.radarManagement.radar_data_fusion import get_radar_data_fusion

_f = get_radar_data_fusion()
try:
    _f.start()
    _f.start()
    import threading as _t
    workers = [t for t in _t.enumerate() if t.name == "RadarDataFusion"]
    check("repeated start() yields exactly one worker", len(workers) == 1,
          f"found {len(workers)}")

    _f.stop()
    check("stop() halts the worker",
          not any(t.name == "RadarDataFusion" and t.is_alive()
                  for t in _t.enumerate()))

    _f.start()
    restarted = [t for t in _t.enumerate()
                 if t.name == "RadarDataFusion" and t.is_alive()]
    check("start() after stop() brings the worker back", len(restarted) == 1,
          f"found {len(restarted)}")
finally:
    _f.stop()

# ── result ───────────────────────────────────────────────────────────────────

print(f"\nRadar fusion lifecycle tests: {PASS} passed, {FAIL} failed")
if FAIL:
    sys.exit(1)
print("Radar fusion lifecycle: all assertions passed")
