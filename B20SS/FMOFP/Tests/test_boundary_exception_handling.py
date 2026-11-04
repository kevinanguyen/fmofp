"""
Test suite: a swallowed exception at a system boundary keeps its traceback.

Story C15 / punch-list P2-3. The punch list proposed narrowing 1,934
`except Exception` handlers, scoped to the boundaries that matter -- "bus
send/receive, routing dispatch, the P0 health checks" -- and said of the rest:
"Elsewhere add exc_info=True."

MEASURED FIRST, and the finding is smaller and more precise than the estimate.
Across the four boundary files there are 34 `except Exception` handlers, and
most already preserve the traceback through traceback.format_exc() or by
re-raising. Exactly EIGHT lost it: they logged f"...{e}" and returned a benign
value, so the log said that something failed and nothing about where.

That is the mechanism behind a good deal of what this codebase's audits have
turned up. H6's `await self._wait_for_response(...)` would have raised
AttributeError into exactly such a handler and been reported as a failed send;
BLOCKER B7's `.get()` on a bool was; and the punch list cites a scenario
injector that called a nonexistent method, swallowed by its own handler, for
months.

The eight now pass exc_info=True, which is verified end to end below rather
than assumed -- SysLogger wraps logging, so a kwarg it dropped would have made
all eight no-ops.

NOT NARROWED, deliberately. Changing `except Exception` to a specific tuple at
these sites would let a programming error propagate out of a bus worker or a
routing dispatch, and the commit immediately before this one went the other
way on purpose: FuelManagementSystem's loop was guarded so a transient error
could not kill the worker. Those are consistent only if the failure becomes
VISIBLE rather than silent, which is what a traceback plus the sustained-
failure health reporting achieves. Control flow is unchanged here; what
changed is that the evidence survives.

TWO HANDLERS ARE CORRECT AS THEY STAND and are exempted by name below rather
than quietly skipped, because an unexplained exemption is how the next sweep
gets it wrong:

  * bus_adapter.reset_bus_adapters -- `except Exception: pass` around
    adapter.close() in test-only teardown that is discarding the adapters
    anyway.
  * health.socket_is_bound -- `except Exception: return False` around
    getsockname(); closed, detached and not-a-socket all genuinely mean
    "not listening", which its own comment says.

Run:  python3 -m FMOFP.Tests.test_boundary_exception_handling
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
import subprocess
import tempfile
import textwrap
import traceback


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

_FMOFP = os.path.join(_B20SS, 'FMOFP')

#: The boundaries the punch list named: bus send/receive, routing dispatch,
#: and the health checks.
BOUNDARIES = (
    'MIL_STD_1553B/Messaging.py',
    'MIL_STD_1553B/bus_adapter.py',
    'local_messaging/routing/MessageRoutingService.py',
    'Utils/common/health.py',
)

#: Handlers that lose the exception on purpose, keyed by the function that
#: contains them. Each entry is a decision, not an oversight.
EXEMPT = {
    'reset_bus_adapters':
        'test-only teardown, closing adapters it is about to discard',
    'socket_is_bound':
        'a closed, detached or non-socket genuinely means "not listening"',
}


def _enclosing_functions(tree):
    """Map each node to the name of the function enclosing it."""
    owner = {}
    def walk(node, current):
        for child in ast.iter_child_nodes(node):
            name = (child.name
                    if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
                    else current)
            owner[child] = name
            walk(child, name)
    walk(tree, '<module>')
    return owner


def audit(source):
    """Every `except Exception` handler, classified.

    Returns (total, lossy, exempted) where lossy and exempted are lists of
    (function_name, lineno, first_body_line).
    """
    tree = ast.parse(source)
    lines = source.splitlines()
    owner = _enclosing_functions(tree)
    total, lossy, exempted = 0, [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Try):
            continue
        for h in node.handlers:
            t = h.type
            name = (t.id if isinstance(t, ast.Name)
                    else ast.unparse(t) if t is not None else 'BARE')
            if 'Exception' not in name:
                continue
            total += 1
            text = "\n".join(lines[l - 1] for l in range(h.lineno, h.end_lineno + 1))
            fn = owner.get(h) or owner.get(node) or '<module>'
            body = [lines[l - 1].strip()
                    for l in range(h.body[0].lineno, h.body[-1].end_lineno + 1)]
            body = [b for b in body if b and not b.startswith('#')]
            first = body[0] if body else ''

            # Keeps the evidence: re-raises, logs a traceback, or carries the
            # exception text into a value the caller receives.
            if any(isinstance(n, ast.Raise) for n in ast.walk(h)):
                continue
            if 'exc_info' in text or 'format_exc' in text:
                continue
            if 'HealthReport' in text:
                continue
            if fn in EXEMPT:
                exempted.append((fn, h.lineno, first))
                continue
            lossy.append((fn, h.lineno, first))
    return total, lossy, exempted


def test_no_boundary_handler_loses_its_traceback():
    R.section("every boundary handler keeps the evidence")

    grand_total = 0
    for rel in BOUNDARIES:
        path = os.path.join(_FMOFP, rel)
        R.check(f"{rel} exists", os.path.exists(path), path)
        if not os.path.exists(path):
            continue
        total, lossy, exempted = audit(
            pathlib.Path(path).read_text(encoding='utf-8'))
        grand_total += total
        R.check(f"{rel}: {total} except-Exception handlers, none losing its "
                f"traceback",
                not lossy,
                "; ".join(f"{fn}:{ln} {first[:40]}" for fn, ln, first in lossy))
        for fn, ln, first in exempted:
            R.check(f"{rel}: {fn}() is exempt on purpose -- {EXEMPT[fn]}",
                    True)

    # The count is asserted so that a boundary file losing its handlers
    # entirely -- or gaining a pile of new ones -- is visible rather than
    # silently passing.
    R.check(f"the four boundaries carry the expected number of handlers "
            f"({grand_total})",
            25 <= grand_total <= 45, str(grand_total))


def test_the_auditor_can_see_a_lossy_handler():
    R.section("NON-TAUTOLOGICAL — the auditor is not simply blind")

    # The exact pre-fix shape of the eight.
    lossy_src = textwrap.dedent('''
        def send(self):
            try:
                self._write()
            except Exception as e:
                logger.error(f"Error sending message: {str(e)}")
                return False
    ''')
    total, lossy, exempt = audit(lossy_src)
    R.check("it flags a handler that logs only the message",
            [l[0] for l in lossy] == ['send'], str(lossy))

    fixed_src = lossy_src.replace(
        'logger.error(f"Error sending message: {str(e)}")',
        'logger.error(f"Error sending message: {str(e)}", exc_info=True)')
    total, lossy, exempt = audit(fixed_src)
    R.check("...and spares the same handler once exc_info is added",
            not lossy, str(lossy))

    reraise_src = textwrap.dedent('''
        def send(self):
            try:
                self._write()
            except Exception as e:
                logger.error(f"boom: {e}")
                raise
    ''')
    R.check("...and spares one that re-raises, since the traceback survives",
            not audit(reraise_src)[1])

    tb_src = textwrap.dedent('''
        def send(self):
            try:
                self._write()
            except Exception as e:
                logger.error(f"boom: {e}")
                logger.error(traceback.format_exc())
                return False
    ''')
    R.check("...and spares one that logs format_exc()",
            not audit(tb_src)[1])

    # An exemption must be earned by name, not by shape.
    exempt_src = textwrap.dedent('''
        def reset_bus_adapters():
            try:
                adapter.close()
            except Exception:
                pass
    ''')
    total, lossy, exempted = audit(exempt_src)
    R.check("a named exemption is recorded as exempt, not as clean",
            not lossy and [e[0] for e in exempted] == ['reset_bus_adapters'],
            f"lossy={lossy} exempt={exempted}")

    unnamed_src = exempt_src.replace('reset_bus_adapters', 'some_other_teardown')
    R.check("...and the same shape under another name is NOT exempt",
            [l[0] for l in audit(unnamed_src)[1]] == ['some_other_teardown'],
            str(audit(unnamed_src)[1]))


def test_exc_info_actually_reaches_the_log():
    R.section("exc_info=True survives SysLogger's wrapper")

    # SysLogger.error() forwards to root_logger.error(msg, *args, **kwargs).
    # If it dropped kwargs, all eight changes would be no-ops, so this is
    # checked end to end against a real log file rather than assumed.
    child = textwrap.dedent(f'''
        import glob, logging, os, sys
        sys.path.insert(0, {_B20SS!r})
        sys.path.insert(0, {os.path.join(_B20SS, 'FMOFP')!r})
        from FMOFP.Utils.logger.sys_logger import get_logger
        from FMOFP.Utils.common.fetching import resolve_data_dir
        log = get_logger()

        def raiser():
            raise RuntimeError("BOUNDARY_PROBE")

        try:
            raiser()
        except Exception as exc:
            log.error(f"probe: {{exc}}", exc_info=True)
        logging.shutdown()

        files = sorted(glob.glob(os.path.join(resolve_data_dir("logs"), "*.log")),
                       key=os.path.getmtime)
        text = open(files[-1], encoding="utf-8", errors="replace").read() if files else ""
        print("MSG=" + str("probe: BOUNDARY_PROBE" in text))
        print("TB=" + str("Traceback (most recent call last)" in text))
        print("FN=" + str("in raiser" in text))
    ''')
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ)
        env["FMOFP_DATA_DIR"] = tmp
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["PYTHONUTF8"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        proc = subprocess.run([sys.executable, "-c", child], capture_output=True,
                              text=True, timeout=180, cwd=_B20SS, env=env)
    out = proc.stdout or ""
    R.check("the child ran", "MSG=" in out, (proc.stderr or "")[-300:])
    R.check("the message reaches the log file", "MSG=True" in out, out[-200:])
    R.check("...and so does the traceback", "TB=True" in out, out[-200:])
    R.check("...naming the function that raised, which the message alone "
            "never did", "FN=True" in out, out[-200:])


def test_the_eight_sites_carry_it():
    R.section("the eight that were losing it now pass exc_info")

    expected = {
        'MIL_STD_1553B/Messaging.py': 2,
        'MIL_STD_1553B/bus_adapter.py': 4,
        'local_messaging/routing/MessageRoutingService.py': 2,
    }
    for rel, count in expected.items():
        src = pathlib.Path(os.path.join(_FMOFP, rel)).read_text(encoding='utf-8')
        found = src.count('exc_info=True')
        R.check(f"{rel} passes exc_info at least {count} time(s)",
                found >= count, f"found {found}")

    # health.py deliberately has none: its handlers put the exception text
    # into the HealthReport they return, which is a caller-visible value and
    # strictly more useful than a log line.
    health = pathlib.Path(os.path.join(_FMOFP, 'Utils/common/health.py')
                          ).read_text(encoding='utf-8')
    R.check("health.py carries the exception in its HealthReport instead",
            'raised {type(exc).__name__}' in health)


def main():
    print("=" * 60)
    print("  Boundary handlers keep their tracebacks")
    print("=" * 60)

    for test in (test_no_boundary_handler_loses_its_traceback,
                 test_the_auditor_can_see_a_lossy_handler,
                 test_exc_info_actually_reaches_the_log,
                 test_the_eight_sites_carry_it):
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
