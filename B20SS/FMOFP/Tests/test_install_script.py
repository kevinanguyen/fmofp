"""
Test suite: install.py — unit tests for each of the six installer steps

These tests run the installer functions against controlled temporary
directories so they never touch the real repo layout.

Tests
-----
1.  check_python          — passes on this interpreter
2.  check_python          — rejects a too-old version tuple
3.  check_directories     — passes when all required dirs exist
4.  check_directories     — fails (SystemExit) when a dir is missing
5.  check_configs         — passes with valid XML files
6.  check_configs         — fails when an XML file is missing
7.  check_configs         — fails when an XML file is malformed
8.  initialise_databases  — creates expected database files
9.  initialise_databases  — creates tables from schema
10. verify_installation   — module-import check passes for present module
11. MIN_PYTHON constant    — value is (3, 10), matching pyproject.toml
12. REQUIRED_PACKAGES      — includes PyQt6 and numpy
13. _find_wheel            — matches the distribution name, not a prefix (B10)
14. _find_links_args       — one --find-links per existing wheel directory (B10)
15. install_dependencies   — an offline bundled-wheel install passes
                             --find-links so pip can resolve the wheel's own
                             dependencies without an index (B10)
"""

import os
import sys
import sqlite3
import tempfile
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from FMOFP.Utils.logger.sys_logger import get_logger
logger = get_logger()

# Import install.py from the B20SS root
_install_path = os.path.join(_B20SS, 'install.py')
import importlib.util as _ilu
_spec   = _ilu.spec_from_file_location("install", _install_path)
_install = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_install)


# ──────────────────────────────────────── framework ───────────────────────

class _Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self._failures = []

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print(f"  ✓  {name}")
        else:
            self.failed += 1
            msg = f"  ✗  {name}" + (f"  [{detail}]" if detail else "")
            print(msg)
            self._failures.append(msg)

    def summary(self):
        total = self.passed + self.failed
        print(f"\n  {self.passed}/{total} passed")
        if self._failures:
            print("\n  Failures:")
            for f in self._failures:
                print(f"    {f}")
        return self.failed == 0


# ───────────────────────────────── Step 1: Python check ──────────────────

def test_check_python_passes(r: _Results) -> None:
    print("\n  ── Step 1: check_python passes ──")
    try:
        _install.check_python()
        r.check("check_python() does not raise on current interpreter", True)
    except SystemExit as exc:
        r.check("check_python() does not raise on current interpreter",
                False, f"SystemExit: {exc}")


def test_check_python_rejects_old(r: _Results) -> None:
    print("\n  ── Step 1: check_python rejects old version ──")
    original = _install.MIN_PYTHON
    _install.MIN_PYTHON = (99, 0)
    raised = False
    try:
        _install.check_python()
    except SystemExit:
        raised = True
    finally:
        _install.MIN_PYTHON = original
    r.check("check_python() raises SystemExit for old Python", raised)


# ────────────────────────────── Step 2: Directory check ──────────────────

def test_check_directories_all_present(r: _Results) -> None:
    print("\n  ── Step 2: check_directories all present ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        # Create all required directories
        required = [tmp_path / "FMOFP" / sub for sub in
                    ("Systems", "Interfaces", "MIL_STD_1553B",
                     "storage", "Utils", "core", "local_messaging")]
        fmofp_dir = tmp_path / "FMOFP"
        fmofp_dir.mkdir()
        for d in required:
            d.mkdir(parents=True, exist_ok=True)

        # Monkey-patch the constants
        orig_dirs = _install.REQUIRED_DIRS
        _install.REQUIRED_DIRS = [fmofp_dir] + required
        try:
            _install.check_directories()
            r.check("check_directories() passes when all dirs present", True)
        except SystemExit as exc:
            r.check("check_directories() passes when all dirs present",
                    False, str(exc))
        finally:
            _install.REQUIRED_DIRS = orig_dirs


def test_check_directories_missing(r: _Results) -> None:
    print("\n  ── Step 2: check_directories missing dir ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        orig_dirs = _install.REQUIRED_DIRS
        _install.REQUIRED_DIRS = [tmp_path / "does_not_exist"]
        raised = False
        try:
            _install.check_directories()
        except SystemExit:
            raised = True
        finally:
            _install.REQUIRED_DIRS = orig_dirs
        r.check("check_directories() raises SystemExit for missing dir", raised)


# ────────────────────────────── Step 4: Config check ─────────────────────

def _write_xml(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def test_check_configs_valid(r: _Results) -> None:
    print("\n  ── Step 4: check_configs with valid XML ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        configs = [tmp_path / f"cfg{i}.xml" for i in range(4)]
        for cfg in configs:
            _write_xml(cfg, "<root><item>test</item></root>")

        orig = _install.REQUIRED_CONFIGS
        _install.REQUIRED_CONFIGS = configs
        try:
            _install.check_configs()
            r.check("check_configs() passes for valid XML files", True)
        except SystemExit as exc:
            r.check("check_configs() passes for valid XML files",
                    False, str(exc))
        finally:
            _install.REQUIRED_CONFIGS = orig


def test_check_configs_missing(r: _Results) -> None:
    print("\n  ── Step 4: check_configs missing file ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        missing = tmp_path / "missing.xml"
        orig = _install.REQUIRED_CONFIGS
        orig_script = _install.SCRIPT_DIR
        # Point SCRIPT_DIR into tmp so relative_to() works
        _install.SCRIPT_DIR = tmp_path
        _install.REQUIRED_CONFIGS = [missing]
        raised = False
        try:
            _install.check_configs()
        except SystemExit:
            raised = True
        finally:
            _install.REQUIRED_CONFIGS = orig
            _install.SCRIPT_DIR = orig_script
        r.check("check_configs() raises SystemExit for missing config", raised)


def test_check_configs_malformed(r: _Results) -> None:
    print("\n  ── Step 4: check_configs malformed XML ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        bad = tmp_path / "bad.xml"
        _write_xml(bad, "<root><unclosed>")
        orig = _install.REQUIRED_CONFIGS
        _install.REQUIRED_CONFIGS = [bad]
        raised = False
        try:
            _install.check_configs()
        except SystemExit:
            raised = True
        finally:
            _install.REQUIRED_CONFIGS = orig
        r.check("check_configs() raises SystemExit for malformed XML", raised)


# ──────────────────────── Step 5: Database initialisation ─────────────────

_MINIMAL_SCHEMA = """
<databases>
  <system name="test_sys">
    <table name="test_table">
      <field name="id"   type="INTEGER PRIMARY KEY AUTOINCREMENT"/>
      <field name="name" type="TEXT NOT NULL"/>
      <field name="value" type="REAL"/>
    </table>
  </system>
</databases>
"""

_MINIMAL_DBCONFIG = """
<databases>
  <system name="test_sys" db_name="test_sys.db"/>
</databases>
"""


def test_initialise_databases_creates_files(r: _Results) -> None:
    print("\n  ── Step 5: initialise_databases creates files ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path  = Path(tmp)
        fmofp_dir = tmp_path / "FMOFP"
        db_dir    = fmofp_dir / "storage" / "databases"
        db_dir.mkdir(parents=True)

        schema_path = db_dir / "schema.xml"
        db_config   = fmofp_dir / "dbConfig.xml"
        schema_path.write_text(_MINIMAL_SCHEMA)
        db_config.write_text(_MINIMAL_DBCONFIG)

        orig_fmofp  = _install.FMOFP_DIR
        _install.FMOFP_DIR = fmofp_dir
        try:
            _install.initialise_databases()
        except SystemExit as exc:
            r.check("initialise_databases() runs without error",
                    False, str(exc))
            return
        finally:
            _install.FMOFP_DIR = orig_fmofp

        db_path = db_dir / "test_sys.db"
        r.check("database file created", db_path.exists())


def test_initialise_databases_creates_tables(r: _Results) -> None:
    print("\n  ── Step 5: initialise_databases creates tables ──")
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path  = Path(tmp)
        fmofp_dir = tmp_path / "FMOFP"
        db_dir    = fmofp_dir / "storage" / "databases"
        db_dir.mkdir(parents=True)

        (db_dir / "schema.xml").write_text(_MINIMAL_SCHEMA)
        (fmofp_dir / "dbConfig.xml").write_text(_MINIMAL_DBCONFIG)

        orig_fmofp = _install.FMOFP_DIR
        _install.FMOFP_DIR = fmofp_dir
        try:
            _install.initialise_databases()
        finally:
            _install.FMOFP_DIR = orig_fmofp

        db_path = db_dir / "test_sys.db"
        if not db_path.exists():
            r.check("test_table created in database", False, "db not found")
            return

        conn = sqlite3.connect(str(db_path))
        cursor = conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='test_table'"
        )
        rows = cursor.fetchall()
        conn.close()
        r.check("test_table created in database",
                len(rows) == 1, f"got {rows}")


# ──────────────────────────── Constants checks ────────────────────────────

def test_min_python_constant(r: _Results) -> None:
    print("\n  ── Constants: MIN_PYTHON ──")
    # B10: was asserted as (3, 9), which locked in a contradiction --
    # pyproject.toml requires >=3.10, CI tests 3.10+, and install.py's own
    # `Path | None` annotation is a TypeError on 3.9 at module import, before
    # check_python() can print its friendly message. The guard has to name a
    # version the file can actually run on.
    r.check("MIN_PYTHON is (3, 10), matching pyproject.toml requires-python",
            _install.MIN_PYTHON == (3, 10), f"got {_install.MIN_PYTHON}")


def test_required_packages(r: _Results) -> None:
    print("\n  ── Constants: REQUIRED_PACKAGES ──")
    pkg_names = [p[1] for p in _install.REQUIRED_PACKAGES]
    r.check("PyQt6 in REQUIRED_PACKAGES",  "PyQt6"  in pkg_names)
    r.check("numpy in REQUIRED_PACKAGES",  "numpy"  in pkg_names)
    r.check("qasync in REQUIRED_PACKAGES", "qasync" in pkg_names)


# ──────────────────────────────────────────── runner ─────────────────────

# ────────────────────── B10: offline bundled-wheel install ─────────────────
#
# The bundled-wheel path ran `pip install --no-index <wheel>` with no
# --find-links. The bundled PyQt6 wheel declares Requires-Dist on PyQt6-sip and
# PyQt6-Qt6, so with the index disabled and nothing local to resolve them from,
# pip failed on the first package -- and --offline has no PyPI fallback, so the
# documented primary Windows/air-gapped install could not work. CI runs only on
# ubuntu-latest, where WHEEL_DIRS is empty, so the path was never exercised.


# The distribution names are the repo's real ones; the TAGS are deliberately
# py3-none-any. _find_wheel() now also checks tag compatibility, and the real
# bundled wheels are win_amd64 (one of them cp310-only), so reusing their exact
# filenames would make these tests exercise the tag logic on whatever
# interpreter happens to be running. That is test_offline_install_matrix's job.
# These two tests are about name matching and the pip argv.
_FAKE_WHEELS = (
    "PyQt6-6.8.1-py3-none-any.whl",
    "PyQt6_Qt6-6.8.2-py3-none-any.whl",
    "PyQt6_sip-13.10.0-py3-none-any.whl",
)

# One wheel with the real, interpreter-locked tag, for asserting that
# _find_wheel declines it rather than handing pip something pip will refuse.
_LOCKED_SIP_WHEEL = "PyQt6_sip-13.10.0-cp310-cp310-win_amd64.whl"


def _fake_wheel_dir(tmp: Path) -> Path:
    """A directory of wheels with the repo's distribution names."""
    wheel_dir = tmp / "PyQt6"
    wheel_dir.mkdir(parents=True, exist_ok=True)
    for name in _FAKE_WHEELS:
        (wheel_dir / name).write_bytes(b"")
    return wheel_dir


def test_find_wheel_matches_distribution_name(r):
    with tempfile.TemporaryDirectory() as td:
        wheel_dir = _fake_wheel_dir(Path(td))
        original = _install.WHEEL_DIRS
        _install.WHEEL_DIRS = [wheel_dir]
        try:
            # A prefix match makes "PyQt6" match all three of these, and
            # Path.glob order is filesystem-dependent, so the wrong wheel could
            # be returned and the install would then fail the import check.
            for pip_name, expected_prefix in (
                ("PyQt6", "PyQt6-6.8.1"),
                ("PyQt6-Qt6", "PyQt6_Qt6-6.8.2"),
                ("PyQt6-sip", "PyQt6_sip-13.10.0"),
            ):
                found = _install._find_wheel(pip_name)
                r.check(f"_find_wheel({pip_name!r}) -> {expected_prefix}",
                        found is not None and found.name.startswith(expected_prefix),
                        f"got {found.name if found else None}")
            r.check("_find_wheel returns None for a package with no bundled wheel",
                    _install._find_wheel("numpy") is None)
        finally:
            _install.WHEEL_DIRS = original

    # Name matching is necessary but not sufficient: a wheel this interpreter
    # cannot install must not be offered to pip at all.
    with tempfile.TemporaryDirectory() as td:
        wheel_dir = Path(td) / "PyQt6"
        wheel_dir.mkdir(parents=True, exist_ok=True)
        (wheel_dir / _LOCKED_SIP_WHEEL).write_bytes(b"")
        original = _install.WHEEL_DIRS
        _install.WHEEL_DIRS = [wheel_dir]
        try:
            r.check("the distribution name still matches",
                    [w.name for w in _install._bundled_wheels("PyQt6-sip")]
                    == [_LOCKED_SIP_WHEEL])
            native = _install._wheel_is_compatible(_LOCKED_SIP_WHEEL)[0]
            r.check("a wheel for another interpreter is not returned",
                    (_install._find_wheel("PyQt6-sip") is not None) == native,
                    f"compatible here={native}")
            if not native:
                r.check("and the report says why",
                        "3.10" in _install._unusable_wheel_report("PyQt6-sip"),
                        _install._unusable_wheel_report("PyQt6-sip"))
        finally:
            _install.WHEEL_DIRS = original


def test_find_links_args(r):
    with tempfile.TemporaryDirectory() as td:
        wheel_dir = _fake_wheel_dir(Path(td))
        missing = Path(td) / "does not exist"
        original = _install.WHEEL_DIRS
        _install.WHEEL_DIRS = [wheel_dir, missing]
        try:
            args = _install._find_links_args()
            r.check("--find-links is emitted for the existing wheel directory",
                    args.count("--find-links") == 1 and str(wheel_dir) in args,
                    str(args))
            r.check("a non-existent wheel directory is skipped",
                    str(missing) not in args, str(args))
        finally:
            _install.WHEEL_DIRS = original


def test_offline_install_passes_find_links(r):
    """The actual pip argv for a bundled-wheel install."""
    with tempfile.TemporaryDirectory() as td:
        wheel_dir = _fake_wheel_dir(Path(td))
        original_dirs = _install.WHEEL_DIRS
        original_pip = _install._pip_install
        original_installed = _install._is_installed
        calls = []

        # Start with nothing installed; record each pip invocation and mark
        # that package installed, so install_dependencies walks the whole
        # REQUIRED_PACKAGES list exactly as it would on a clean machine.
        installed = set()
        import_to_pip = {imp: pip for imp, pip in _install.REQUIRED_PACKAGES}

        def _record(args):
            calls.append(list(args))
            joined = " ".join(args).lower()
            for pip_name in import_to_pip.values():
                if pip_name.lower().replace("-", "_") in joined:
                    installed.add(pip_name)
            return 0, ""

        _install.WHEEL_DIRS = [wheel_dir]
        _install._pip_install = _record
        _install._is_installed = lambda imp: import_to_pip.get(imp, imp) in installed
        try:
            _install.install_dependencies(offline=True, force_reinstall=False)
        except SystemExit:
            # fail() raises SystemExit for a package with no bundled wheel in
            # offline mode; the PyQt6 calls we care about have already happened.
            pass
        finally:
            _install.WHEEL_DIRS = original_dirs
            _install._pip_install = original_pip
            _install._is_installed = original_installed

        wheel_calls = [c for c in calls if "--no-index" in c]
        r.check("the offline path ran at least one bundled-wheel install",
                wheel_calls != [], str(calls))
        for call in wheel_calls:
            r.check("every --no-index install also passes --find-links",
                    "--find-links" in call,
                    "pip cannot resolve the wheel's own dependencies (B10): "
                    + " ".join(call))
            r.check("--find-links points at the bundled wheel directory",
                    str(wheel_dir) in call, " ".join(call))

        # sip and Qt6 are PyQt6's own dependencies; an offline install has to
        # install them explicitly, because there is no index to pull them from.
        installed_names = " ".join(" ".join(c) for c in calls).lower()
        r.check("the offline path installs PyQt6-sip",
                "pyqt6_sip" in installed_names, installed_names)
        r.check("the offline path installs PyQt6-Qt6",
                "pyqt6_qt6" in installed_names, installed_names)


def test_required_packages_covers_pyqt_dependencies(r):
    names = [pip_name for _, pip_name in _install.REQUIRED_PACKAGES]
    for needed in ("PyQt6", "PyQt6-sip", "PyQt6-Qt6"):
        r.check(f"REQUIRED_PACKAGES lists {needed}", needed in names, str(names))
    r.check("PyQt6-sip is installed before PyQt6",
            names.index("PyQt6-sip") < names.index("PyQt6"), str(names))
    r.check("PyQt6-Qt6 is installed before PyQt6",
            names.index("PyQt6-Qt6") < names.index("PyQt6"), str(names))


def run_all() -> bool:
    print("=" * 60)
    print(" Install Script Test Suite")
    print("=" * 60)

    r = _Results()

    tests = [
        test_check_python_passes,
        test_check_python_rejects_old,
        test_check_directories_all_present,
        test_check_directories_missing,
        test_check_configs_valid,
        test_check_configs_missing,
        test_check_configs_malformed,
        test_initialise_databases_creates_files,
        test_initialise_databases_creates_tables,
        test_min_python_constant,
        test_required_packages,
        test_find_wheel_matches_distribution_name,
        test_find_links_args,
        test_offline_install_passes_find_links,
        test_required_packages_covers_pyqt_dependencies,
    ]

    for test_fn in tests:
        try:
            test_fn(r)
        except Exception as exc:
            r.failed += 1
            print(f"  ✗  {test_fn.__name__} raised: {exc}")
            traceback.print_exc()

    print("\n" + "=" * 60)
    passed = r.summary()
    print("=" * 60)
    return passed


if __name__ == "__main__":
    ok = run_all()
    sys.exit(0 if ok else 1)
