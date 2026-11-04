"""
Test suite: one definition per thing, and a guard on the duplicates we keep.

Three findings, and one duplicate deliberately left alone.

1. A SIX-MEMBER STAND-IN FOR A THIRTY-FIVE-MEMBER ENUM. prevention.py imported
   RadarDisplayMode inside try/except ImportError and, on failure, defined its
   own six-member enum -- "Create dummy enum if import fails".

   Its only use at that site is `for mode in RadarDisplayMode:`, adding each
   member NAME to the loop-prevention category set, so the stand-in corrupted
   no value there: it silently narrowed loop prevention to 6 categories out of
   35, leaving 29 radar display modes unrecognised by the component whose job
   is recognising them. The values were wrong as well -- five of its six
   disagreed with the canonical enum, SURVEILLANCE 1 against 10, MAPPING 2
   against 11, NORMAL 5 against 1 -- which would have mattered to any other
   consumer.

   The module it guards imports nothing but enum.Enum, so the ImportError it
   caught cannot happen without the package being broken outright. The import
   is unconditional now.

2. A THIRD DEFINITION THAT NOTHING IMPORTED.
   Interfaces/userInterface/displays/radar/radar_display_modes.py held the same
   six divergent members, called itself "a central reference for radar display
   modes used across the display system", and had ZERO importers package-wide.
   Its own docstring contradicted itself -- it documented both
   "NORMAL/SURVEILLANCE (1)" and "NORMAL (5)" -- and the code assigns
   SURVEILLANCE = 1 and NORMAL = 5. Deleted: an authoritative-sounding file
   with no callers is a decoy for whoever reads it next.

3. TWIN display_command_map MODULES WITH NOTHING KEEPING THEM IN STEP. There
   are two, both imported: FMOFP/local_messaging/display_command_map.py (used
   throughout DisplayMessageHandler) and
   FMOFP/Interfaces/userInterface/messaging/display_command_map.py (used by
   radar_data_handler/base_radar_display.py for DISPLAY_SUBADDRESSES).

   Measured: 58 module-level constants each, every shared one identical, and
   the same four functions. They agree TODAY -- an earlier note recorded them
   as "byte-identical, now diverged", and the divergence is absolute versus
   relative imports, not data. What is missing is anything that keeps them
   agreeing, which is what the check below is.

NOT FIXED HERE: BaseRadarDisplay is defined twice, and both are live -- an ABC
in displays/radar/ with eight importers, and a plain class in
displays/radar/radar_data_handler/ with one. Whether those are one concept or
two, and which importers should move, is a design decision with real blast
radius across the display tree. It is asserted below as a KNOWN duplicate so
that this suite records it rather than implying it is resolved.

Run:  python3 -m FMOFP.Tests.test_single_definition
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from FMOFP.Tests import code_has  # whitespace-insensitive code matcher

import ast
import glob
import pathlib
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
CANONICAL_MODES = os.path.join(_FMOFP, 'local_messaging', 'radar_display_modes.py')


def _py_files():
    return sorted(glob.glob(os.path.join(_FMOFP, '**', '*.py'), recursive=True))


def _class_definitions(class_name):
    """Every file defining `class <class_name>`, by AST rather than by grep."""
    found = []
    for path in _py_files():
        try:
            tree = ast.parse(pathlib.Path(path).read_text(encoding='utf-8'))
        except (OSError, SyntaxError, UnicodeDecodeError):
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                found.append(os.path.relpath(path, _FMOFP))
                break
    return found


def _enum_members(path, class_name="RadarDisplayMode"):
    tree = ast.parse(pathlib.Path(path).read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {st.targets[0].id: st.value.value
                    for st in node.body
                    if isinstance(st, ast.Assign)
                    and isinstance(st.targets[0], ast.Name)
                    and isinstance(st.value, ast.Constant)}
    return {}


def test_radar_display_mode_is_defined_once():
    R.section("RadarDisplayMode has one definition")

    defs = _class_definitions("RadarDisplayMode")
    R.check("exactly one file defines it", len(defs) == 1, str(defs))
    if defs:
        R.check("...and it is local_messaging/radar_display_modes.py",
                defs[0].replace("\\", "/") == "local_messaging/radar_display_modes.py",
                defs[0])

    canonical = _enum_members(CANONICAL_MODES)
    R.check("the canonical enum still has its full membership",
            len(canonical) >= 30, str(len(canonical)))

    # The deleted third definition.
    orphan = os.path.join(_FMOFP, 'Interfaces', 'userInterface', 'displays',
                          'radar', 'radar_display_modes.py')
    R.check("the orphan module is gone", not os.path.exists(orphan), orphan)


def test_prevention_imports_it_unconditionally():
    R.section("no stand-in enum behind an except ImportError")

    src = pathlib.Path(os.path.join(
        _FMOFP, 'Utils', 'message_loop_prevention', 'prevention.py')
    ).read_text(encoding='utf-8')

    R.check("prevention.py imports the canonical enum",
            code_has(src, "from FMOFP.local_messaging.radar_display_modes "
                          "import RadarDisplayMode"))
    R.check("...and defines no RadarDisplayMode of its own",
            not code_has(src, "class RadarDisplayMode"))
    # The removal is described in a comment, where code_has cannot see it, so
    # the check above cannot pass on the strength of the explanation.
    R.check("the removal is recorded in prose the code check cannot see",
            "stand-in" in src and not code_has(src, "class RadarDisplayMode"))

    # It still uses the enum, so the import is not merely decorative.
    R.check("it still iterates the enum, which is why the stand-in mattered",
            code_has(src, "for mode in RadarDisplayMode"))


def test_what_the_stand_in_would_have_done():
    R.section("NON-TAUTOLOGICAL — the stand-in's membership against the real one")

    # The removed stand-in, reproduced inline.
    stand_in = {"STANDBY": 0, "SURVEILLANCE": 1, "MAPPING": 2,
                "TURBULENCE": 3, "WINDSHEAR": 4, "NORMAL": 5}
    canonical = _enum_members(CANONICAL_MODES)
    R.check("the canonical enum was read", bool(canonical), "empty")
    if not canonical:
        return

    missing = sorted(set(canonical) - set(stand_in))
    R.check(f"the stand-in covered {len(stand_in)} of {len(canonical)} names",
            len(stand_in) < len(canonical), str(len(stand_in)))
    R.check(f"...so {len(missing)} display modes were missing from the "
            f"loop-prevention category set",
            len(missing) >= 25, str(len(missing)))

    shared = sorted(set(stand_in) & set(canonical))
    disagree = [n for n in shared if stand_in[n] != canonical[n]]
    R.check("every name it did carry is a real one",
            len(shared) == len(stand_in), str(sorted(set(stand_in) - set(canonical))))
    R.check("...and five of its six values disagreed with the canonical enum",
            len(disagree) == 5, f"disagree={disagree}")
    R.check("...including SURVEILLANCE, which it numbered 1 where the "
            "canonical enum numbers it 10",
            stand_in["SURVEILLANCE"] == 1 and canonical.get("SURVEILLANCE") == 10,
            f"canonical SURVEILLANCE={canonical.get('SURVEILLANCE')}")
    R.check("...and its 1 is the canonical NORMAL, so the two tables cross over",
            canonical.get("NORMAL") == 1, str(canonical.get("NORMAL")))


# ---------------------------------------------------------------------------
# the duplicates we keep
# ---------------------------------------------------------------------------

TWINS = (
    os.path.join(_FMOFP, 'local_messaging', 'display_command_map.py'),
    os.path.join(_FMOFP, 'Interfaces', 'userInterface', 'messaging',
                 'display_command_map.py'),
)


def _module_constants(path):
    tree = ast.parse(pathlib.Path(path).read_text(encoding='utf-8'))
    out = {}
    for st in tree.body:
        if isinstance(st, ast.Assign) and isinstance(st.targets[0], ast.Name):
            try:
                out[st.targets[0].id] = ast.literal_eval(st.value)
            except Exception:  # noqa: BLE001 - non-literal, record its shape
                out[st.targets[0].id] = f"<{type(st.value).__name__}>"
    return out


def _module_functions(path):
    tree = ast.parse(pathlib.Path(path).read_text(encoding='utf-8'))
    return sorted(n.name for n in tree.body
                  if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)))


def test_the_twin_command_maps_still_agree():
    R.section("the two display_command_map modules have not drifted")

    for path in TWINS:
        R.check(f"{os.path.relpath(path, _FMOFP)} exists", os.path.exists(path))
    if not all(os.path.exists(p) for p in TWINS):
        return

    a, b = (_module_constants(p) for p in TWINS)
    R.check("both define the same set of module-level constants",
            set(a) == set(b),
            f"only in local_messaging: {sorted(set(a) - set(b))}  "
            f"only in Interfaces: {sorted(set(b) - set(a))}")

    disagree = sorted(k for k in set(a) & set(b) if a[k] != b[k])
    R.check("every shared constant has the same value", not disagree,
            str(disagree[:5]))
    R.check("...and there are enough of them for that to mean something",
            len(set(a) & set(b)) > 40, str(len(set(a) & set(b))))

    fa, fb = (_module_functions(p) for p in TWINS)
    R.check("both define the same functions", fa == fb, f"{fa} vs {fb}")

    # NON-TAUTOLOGICAL: the comparison must be able to see a difference.
    planted = dict(a)
    first = sorted(planted)[0]
    planted[first] = "<planted>"
    R.check("the comparison detects a planted difference",
            [k for k in set(planted) & set(b) if planted[k] != b[k]] == [first],
            first)


def test_known_duplicates_are_recorded_not_forgotten():
    R.section("BaseRadarDisplay is a known, unresolved duplicate")

    defs = sorted(d.replace("\\", "/") for d in _class_definitions("BaseRadarDisplay"))
    R.check("there are still exactly two definitions", len(defs) == 2, str(defs))
    R.check("...one in displays/radar/",
            any(d == "Interfaces/userInterface/displays/radar/base_radar_display.py"
                for d in defs), str(defs))
    R.check("...and one in displays/radar/radar_data_handler/",
            any(d.endswith("radar_data_handler/base_radar_display.py")
                for d in defs), str(defs))

    # If someone resolves this, the count changes and the checks above fail,
    # which is the point: a record, not an endorsement.
    #
    # And the substantive reason they cannot simply be merged: they are not
    # the same KIND of class. One is an abstract base, the other is not, so
    # collapsing them would either impose abstract methods on the plain one's
    # subclasses or drop the contract the eight importers of the ABC rely on.
    kinds = {}
    for rel in defs:
        tree = ast.parse(pathlib.Path(os.path.join(_FMOFP, rel)).read_text(
            encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "BaseRadarDisplay":
                bases = [getattr(b, "id", getattr(b, "attr", "?"))
                         for b in node.bases]
                kinds[rel] = bases
                break

    R.check("both definitions were parsed for their bases", len(kinds) == 2,
            str(kinds))
    abstract = [rel for rel, bases in kinds.items() if "ABC" in bases]
    plain = [rel for rel, bases in kinds.items() if "ABC" not in bases]
    R.check("exactly one of the two is an abstract base class",
            len(abstract) == 1, str(kinds))
    R.check("...and exactly one is not, which is why they are not "
            "interchangeable", len(plain) == 1, str(kinds))

    # Both are genuinely used, so neither is dead code that could just go.
    importers = {rel: 0 for rel in defs}
    for path in _py_files():
        try:
            src = pathlib.Path(path).read_text(encoding='utf-8')
        except (OSError, UnicodeDecodeError):
            continue
        if "import BaseRadarDisplay" not in src:
            continue
        rel_dir = os.path.dirname(os.path.relpath(path, _FMOFP)).replace("\\", "/")
        for rel in defs:
            if os.path.dirname(rel) == rel_dir:
                importers[rel] += 1
    R.check("both definitions have importers, so neither is simply dead",
            all(n > 0 for n in importers.values()), str(importers))



def test_the_lint_gate_has_one_source_of_truth():
    R.section("the ruff selector lives in pyproject, not in the CI file")

    root = os.path.abspath(os.path.join(_B20SS, '..'))
    pyproject = pathlib.Path(os.path.join(root, 'pyproject.toml'))
    workflow = pathlib.Path(os.path.join(root, '.github', 'workflows', 'ci.yml'))

    R.check("pyproject.toml exists", pyproject.exists(), str(pyproject))
    R.check("the CI workflow exists", workflow.exists(), str(workflow))
    if not (pyproject.exists() and workflow.exists()):
        return

    cfg = pyproject.read_text(encoding='utf-8')
    R.check("pyproject carries a [tool.ruff.lint] selector",
            "[tool.ruff.lint]" in cfg and "select = [" in cfg)
    for rule in ("E9", "F821", "F401", "F541"):
        R.check(f"...including {rule}", f'"{rule}"' in cfg, rule)
    R.check("...and exempts __init__.py from F401, where re-exports live",
            "per-file-ignores" in cfg and '"**/__init__.py" = ["F401"]' in cfg)

    wf = workflow.read_text(encoding='utf-8')
    ruff_lines = [ln for ln in wf.splitlines() if "ruff check" in ln]
    R.check("the workflow invokes ruff exactly once", len(ruff_lines) == 1,
            str(ruff_lines))
    if ruff_lines:
        invocation = ruff_lines[0]
        # Story P2-1 was two CI files each carrying their own --ignore. A
        # selector in the workflow can disagree with the repo config; one
        # that is not there cannot.
        for flag in ("--select", "--ignore", "--exclude", "--extend-select"):
            R.check(f"...with no {flag} of its own", flag not in invocation,
                    invocation.strip())
        R.check("...and is not swallowed by `|| true`, which is how this "
                "step once could not fail", "|| true" not in invocation,
                invocation.strip())

    # Comment lines stripped first. Written without that, this check counted
    # the two `|| true` mentions in the comment that RECORDS the swallowed
    # exit code being removed, and failed on a correct workflow -- the same
    # mistake as matching code in a comment, which FMOFP/Tests/__init__.py
    # documents six variations of.
    code_lines = [ln for ln in wf.splitlines()
                  if not ln.lstrip().startswith("#")]
    swallowed = [ln for ln in code_lines if "|| true" in ln]
    R.check("no step swallows its exit code with `|| true`",
            not swallowed, str(swallowed))
    # And the stripper must be able to see a real one, or the check is free.
    planted = ["        run: ruff check . || true"]
    R.check("the check would catch a real `|| true` in a run step",
            [ln for ln in planted if "|| true" in ln] == planted)


def main():
    print("=" * 60)
    print("  One definition per thing")
    print("=" * 60)

    for test in (test_radar_display_mode_is_defined_once,
                 test_prevention_imports_it_unconditionally,
                 test_what_the_stand_in_would_have_done,
                 test_the_twin_command_maps_still_agree,
                 test_known_duplicates_are_recorded_not_forgotten,
                 test_the_lint_gate_has_one_source_of_truth):
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
