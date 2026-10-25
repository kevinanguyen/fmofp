"""
Test suite: which Python versions can actually install this, online and offline.

The claim under test is not "CI is green" -- it is "the interpreters CI covers
and the interpreters a shipped offline install supports are the same set". They
are not, and nothing noticed, because CI only ever runs on ubuntu-latest where
WHEEL_DIRS is empty and the bundled-wheel path is never exercised.

Measured with pip's own resolver against B20SS/PyQt6:

    py310 win_amd64: OK
    py311 win_amd64: FAIL   py312: FAIL   py313: FAIL   py314: FAIL

PyQt6 (cp39-abi3) and PyQt6_Qt6 (py3-none) are forward compatible. PyQt6_sip
is not abi3, so PyQt6_sip-13.10.0-cp310-cp310-win_amd64.whl is locked to
CPython 3.10 exactly. install.py --offline has no PyPI fallback, so on any
other Windows Python the documented air-gapped install cannot complete.

It reported that badly, too. _find_wheel() matched on distribution name alone,
so it handed pip a wheel pip would refuse, and --offline then said "no bundled
wheel was found" -- false, and silent about the tag the operator needed.

A SEPARATE correction recorded here: an earlier audit called the 3.14 CI leg
permanently broken, reproducing "Could not find a version that satisfies the
requirement PyQt6-sip==13.10.0". That reproduction used --only-binary, which
is not what CI runs. Measured on 3.14: pip builds sip from its sdist and all
35 suites pass. The leg works -- it just compiles, so it depends on the runner
having a C toolchain.

Run:  python3 -m FMOFP.Tests.test_offline_install_matrix
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import FMOFP.Tests  # noqa: F401  -- UTF-8 stdio for piped output

import importlib.util
import traceback
from pathlib import Path


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

_INSTALL_PY = Path(_B20SS) / "install.py"


def _load_install():
    spec = importlib.util.spec_from_file_location("_fmofp_install", _INSTALL_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Pretend:
    """Run _wheel_is_compatible as if we were another interpreter/arch.

    install.py reads sys.version_info, platform.machine() and
    platform.system() at call time, so they can be swapped on the loaded
    module without touching this process.
    """

    def __init__(self, mod, minor, machine="AMD64", system="Windows",
                 impl="CPython", major=3):
        self.mod = mod
        self.vals = (major, minor, machine, system, impl)

    def __enter__(self):
        mod = self.mod
        major, minor, machine, system, impl = self.vals
        self._saved = (mod.sys.version_info, mod.platform.machine,
                       mod.platform.system, mod.platform.python_implementation)
        mod.sys.version_info = (major, minor, 0, 'final', 0)
        mod.platform.machine = lambda: machine
        mod.platform.system = lambda: system
        mod.platform.python_implementation = lambda: impl
        return mod

    def __exit__(self, *exc):
        mod = self.mod
        (mod.sys.version_info, mod.platform.machine, mod.platform.system,
         mod.platform.python_implementation) = self._saved
        return False


def test_the_tag_checker_is_correct():
    R.section("The wheel tag check agrees with pip on the real bundled wheels")

    mod = _load_install()
    sip = "PyQt6_sip-13.10.0-cp310-cp310-win_amd64.whl"
    qt6 = "PyQt6_Qt6-6.8.2-py3-none-win_amd64.whl"
    pyqt = "PyQt6-6.8.1-cp39-abi3-win_amd64.whl"

    verdicts = {}
    for minor in (10, 11, 12, 13, 14):
        with _Pretend(mod, minor) as m:
            verdicts[minor] = all(
                m._wheel_is_compatible(w)[0] for w in (sip, qt6, pyqt))

    R.check("Windows CPython 3.10 can install the bundled set",
            verdicts[10] is True)
    R.check("3.11, 3.12, 3.13 and 3.14 cannot",
            not any(verdicts[v] for v in (11, 12, 13, 14)),
            f"verdicts={verdicts}")

    with _Pretend(mod, 12) as m:
        okc, why = m._wheel_is_compatible(sip)
        R.check("and the reason names the version, for the error message",
                not okc and "3.10" in why and "3.12" in why, f"why={why!r}")

    R.section("NON-TAUTOLOGICAL — the checker is not simply always False")
    with _Pretend(mod, 12) as m:
        cases = {
            "pure py3":                 ("foo-1.0-py3-none-any.whl", True),
            "abi3 floor below us":      ("foo-1.0-cp39-abi3-win_amd64.whl", True),
            "abi3 floor above us":      ("foo-1.0-cp313-abi3-win_amd64.whl", False),
            "version-locked, matching": ("foo-1.0-cp312-cp312-win_amd64.whl", True),
            "version-locked, mismatch": ("foo-1.0-cp310-cp310-win_amd64.whl", False),
            "multi-tag including ours": ("foo-1.0-cp311.cp312-cp311.cp312-win_amd64.whl", True),
            "wrong architecture":       ("foo-1.0-py3-none-win_arm64.whl", False),
            "unparseable, defer to pip": ("not-a-wheel-name.whl", True),
        }
        wrong = {label: (m._wheel_is_compatible(n)[0], expect)
                 for label, (n, expect) in cases.items()
                 if m._wheel_is_compatible(n)[0] is not expect}
        R.check(f"all {len(cases)} tag forms classify as intended",
                not wrong, f"wrong={wrong}")


def test_bundled_wheels_support_exactly_what_is_claimed():
    R.section("The bundled wheel set's real interpreter coverage")

    pyqt6_dir = Path(_B20SS) / "PyQt6"
    R.check("the bundled wheel directory exists", pyqt6_dir.is_dir())
    wheels = sorted(w.name for w in pyqt6_dir.glob("*.whl"))
    R.check("it holds the three PyQt6-stack wheels", len(wheels) == 3,
            f"wheels={wheels}")

    mod = _load_install()
    supported = []
    for minor in range(9, 16):
        with _Pretend(mod, minor) as m:
            if wheels and all(m._wheel_is_compatible(w)[0] for w in wheels):
                supported.append(f"3.{minor}")

    # Pinning this is the point of the suite: if someone adds or swaps a
    # bundled wheel, the supported set changes here and has to be acknowledged
    # rather than drifting quietly away from what CI and the README claim.
    R.check("offline install supports exactly ['3.10'] on Windows x64",
            supported == ["3.10"], f"supported={supported}")

    with _Pretend(mod, 10, machine="ARM64") as m:
        arm_ok = [w for w in wheels if m._wheel_is_compatible(w)[0]]
    R.check("and nothing installs offline on win_arm64, despite a wheel for it",
            not arm_ok,
            f"usable on arm64: {arm_ok} -- PyQt6 ships arm64 but Qt6 and sip "
            f"do not, so the arm64 wheel cannot be installed alone")


def test_offline_failure_explains_itself():
    R.section("--offline reports the tag mismatch instead of 'not found'")

    src = _INSTALL_PY.read_text(encoding='utf-8')
    # inspect, not a slice of the file: the first version of this check looked
    # at the 600 characters after "def _find_wheel" and missed the call,
    # because the docstring is longer than that. The test was wrong, not the
    # code -- the same mistake as the comment-stripping check in
    # test_request_expiry.
    import inspect
    find_wheel_src = inspect.getsource(_load_install()._find_wheel)
    R.check("_find_wheel filters on compatibility, not just the name",
            "_wheel_is_compatible" in find_wheel_src,
            "it matched on distribution name alone before this")
    R.check("there is a report of why a bundled wheel was unusable",
            "_unusable_wheel_report" in src)
    R.check("and the offline failure path uses it",
            "_unusable_wheel_report(pip_name)" in src)
    R.check("the message names the running interpreter",
            "_current_interpreter_tag()" in src)
    R.check("...and says PyPI would have worked without --offline",
            "Without --offline" in src)

    mod = _load_install()
    R.check("the interpreter tag matches this process",
            mod._current_interpreter_tag() ==
            f"cp{sys.version_info.major}{sys.version_info.minor}",
            mod._current_interpreter_tag())


def test_ci_matrix_matches_what_was_measured():
    R.section("The CI matrix records how 3.14 actually installs")

    ci = Path(_B20SS).parent / ".github" / "workflows" / "ci.yml"
    R.check("the workflow exists", ci.is_file(), str(ci))
    if not ci.is_file():
        return
    text = ci.read_text(encoding='utf-8')

    R.check("3.14 is still covered rather than dropped", '"3.14"' in text)
    R.check("and the matrix records that sip is built from source there",
            "sdist" in text and "PyQt6-sip" in text)
    R.check("including that it depends on a C toolchain on the runner",
            "toolchain" in text)


def main():
    print("=" * 60)
    print("  Install matrix: CI coverage vs shipped offline support")
    print("=" * 60)

    for test in (test_the_tag_checker_is_correct,
                 test_bundled_wheels_support_exactly_what_is_claimed,
                 test_offline_failure_explains_itself,
                 test_ci_matrix_matches_what_was_measured):
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
