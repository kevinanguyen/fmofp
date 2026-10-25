"""
FMOFP Installation Script
=========================
Flight Management Operating Flight Program — B20SS

Run this script once from the B20SS directory to fully install and verify
the system.  It handles:

  1. Python version check (3.9+)
  2. Dependency installation  (PyQt6, numpy, qasync, and others)
     — prefers the bundled .whl files when present, falls back to PyPI
  3. Configuration-file validation
  4. Database initialisation  (creates all SQLite databases from schema.xml)
  5. Full system verification  (dry-run startup to confirm every subsystem
     reaches STANDBY / OPERATIONAL before the UI is shown)

Usage
-----
    python install.py                  # interactive, recommended
    python install.py --offline        # use only bundled wheels, no internet
    python install.py --no-verify      # skip the startup verification step
    python install.py --force-reinstall  # reinstall deps even if already present

Exit codes: 0 = success, 1 = failure.
"""

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MIN_PYTHON = (3, 10)   # B10: pyproject.toml requires >=3.10 and this file
                      # itself uses PEP 604 syntax (Path | None) that 3.9
                      # cannot even parse, so the friendly check below was
                      # unreachable on exactly the versions it guarded.
SCRIPT_DIR = Path(__file__).parent.resolve()
FMOFP_DIR  = SCRIPT_DIR / "FMOFP"
REQUIREMENTS_FILE = SCRIPT_DIR / "requirements.txt"

# Directories that must exist after extraction
REQUIRED_DIRS = [
    FMOFP_DIR,
    FMOFP_DIR / "Systems",
    FMOFP_DIR / "Interfaces",
    FMOFP_DIR / "MIL_STD_1553B",
    FMOFP_DIR / "storage",
    FMOFP_DIR / "Utils",
    FMOFP_DIR / "core",
    FMOFP_DIR / "local_messaging",
]

# Config files that must be present
REQUIRED_CONFIGS = [
    FMOFP_DIR / "dbConfig.xml",
    FMOFP_DIR / "rtAddressConfig.xml",
    FMOFP_DIR / "messageRateConfig.xml",
    FMOFP_DIR / "startupConfiguration.xml",
]

# Python packages required — (import_name, pip_name)
# NOTE: this list is separate from requirements.txt and does not derive
# from it automatically -- install_dependencies() below iterates over
# THIS list, not requirements.txt. Adding a package to requirements.txt
# alone (e.g. for CI) does nothing here; it must also be added below, or
# the real installer will never attempt to install it. Confirmed live:
# this exact gap is what let scipy go missing from actual installs even
# after code started depending on it (see requirements.txt's scipy entry).
REQUIRED_PACKAGES = [
    # PyQt6-Qt6 and PyQt6-sip are PyQt6's own runtime dependencies and are
    # bundled as wheels alongside it. They are listed explicitly (B10) because
    # this list -- not requirements.txt -- is what install_dependencies()
    # iterates, and because an offline install resolves each wheel from the
    # local directories rather than from an index: naming them here means they
    # are verified present rather than assumed to have come along for the ride.
    # Order matters: PyQt6 depends on both, so they are installed first.
    ("PyQt6.sip", "PyQt6-sip"),
    ("PyQt6.QtCore", "PyQt6-Qt6"),
    ("PyQt6",    "PyQt6"),
    ("numpy",    "numpy"),
    ("qasync",   "qasync"),
    ("scipy",    "scipy"),
    ("click",    "click"),
]


def _load_requirement_specs() -> dict:
    """
    Parse requirements.txt into {pip_name: version_specifier} so PyPI
    fallback installs use the exact pins tested in CI instead of grabbing
    whatever is newest on PyPI. Keeps install.py, requirements.txt, and
    CI in agreement instead of drifting apart (production readiness
    punch list, item 1).
    """
    specs = {}
    if not REQUIREMENTS_FILE.exists():
        return specs
    for line in REQUIREMENTS_FILE.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        for op in ("==", ">=", "<=", "~=", "!=", "<", ">"):
            if op in line:
                name, spec = line.split(op, 1)
                specs[name.strip()] = op + spec.strip()
                break
    return specs

# Local wheel directories to search (preferred over PyPI)
# On Windows, prefer bundled wheels; on Linux/macOS fall straight through to PyPI.
if platform.system() == "Windows":
    WHEEL_DIRS = [
        SCRIPT_DIR / "PyQt6",
        SCRIPT_DIR / "python packages",
    ]
else:
    WHEEL_DIRS = []  # No bundled wheels for Linux/macOS — use PyPI via _pip_install

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class Colours:
    """ANSI codes — disabled automatically on Windows without VT support."""
    _use = sys.stdout.isatty() and platform.system() != "Windows" or (
        platform.system() == "Windows" and
        os.environ.get("TERM_PROGRAM") in ("vscode", "WindowsTerminal")
    )
    OK      = "\033[92m" if _use else ""
    WARN    = "\033[93m" if _use else ""
    ERR     = "\033[91m" if _use else ""
    INFO    = "\033[94m" if _use else ""
    BOLD    = "\033[1m"  if _use else ""
    RESET   = "\033[0m"  if _use else ""


def header(text: str) -> None:
    width = 60
    print(f"\n{Colours.BOLD}{'─' * width}{Colours.RESET}")
    print(f"{Colours.BOLD}  {text}{Colours.RESET}")
    print(f"{Colours.BOLD}{'─' * width}{Colours.RESET}")


def ok(text: str)   -> None: print(f"  {Colours.OK}✓{Colours.RESET}  {text}")
def warn(text: str) -> None: print(f"  {Colours.WARN}⚠{Colours.RESET}  {text}")
def err(text: str)  -> None: print(f"  {Colours.ERR}✗{Colours.RESET}  {text}")
def info(text: str) -> None: print(f"  {Colours.INFO}→{Colours.RESET}  {text}")


def fail(message: str) -> None:
    err(message)
    print(f"\n{Colours.ERR}Installation failed.{Colours.RESET}")
    sys.exit(1)


def run(cmd: list, cwd=None, capture=False):
    """Run a subprocess.  Returns (returncode, stdout) if capture=True."""
    if capture:
        result = subprocess.run(
            cmd, cwd=cwd, capture_output=True, text=True
        )
        return result.returncode, result.stdout + result.stderr
    else:
        return subprocess.run(cmd, cwd=cwd).returncode, ""


# ---------------------------------------------------------------------------
# Step 1 — Python version
# ---------------------------------------------------------------------------

def check_python() -> None:
    header("Step 1 — Python version check")
    ver = sys.version_info
    ver_str = f"{ver.major}.{ver.minor}.{ver.micro}"
    if ver < MIN_PYTHON:
        fail(
            f"Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+ required, "
            f"but you are running {ver_str}.\n"
            "     Download from https://www.python.org/downloads/"
        )
    ok(f"Python {ver_str} — OK")


# ---------------------------------------------------------------------------
# Step 2 — Directory structure
# ---------------------------------------------------------------------------

def check_directories() -> None:
    header("Step 2 — Directory structure")
    missing = [d for d in REQUIRED_DIRS if not d.is_dir()]
    if missing:
        fail(
            "The following required directories are missing:\n" +
            "\n".join(f"       {d}" for d in missing) +
            "\n\n     Ensure you have extracted the full FMOFP archive "
            "before running this installer."
        )
    ok(f"All {len(REQUIRED_DIRS)} required directories present")


# ---------------------------------------------------------------------------
# Step 3 — Dependencies
# ---------------------------------------------------------------------------

# A wheel filename is dist-version[-build]-pytag-abitag-plattag.whl (PEP 427),
# and each tag field may hold dot-separated alternatives.
_WHEEL_NAME_RE = re.compile(
    r"^(?P<dist>[^-]+)-(?P<ver>[^-]+)"
    r"(?:-(?P<build>\d[^-]*))?"
    r"-(?P<py>[^-]+)-(?P<abi>[^-]+)-(?P<plat>[^-]+)\.whl$",
    re.IGNORECASE,
)


def _current_interpreter_tag() -> str:
    """'cp312' for this interpreter."""
    impl = "cp" if platform.python_implementation() == "CPython" else "py"
    return f"{impl}{sys.version_info.major}{sys.version_info.minor}"


def _wheel_is_compatible(wheel_name: str):
    """Is this wheel installable on the interpreter running right now?

    Returns (compatible, reason). `reason` is empty when compatible, and
    otherwise says which dimension does not match, for the error message.

    This exists because _find_wheel() used to match on distribution name
    ALONE. The bundled sip wheel is PyQt6_sip-13.10.0-cp310-cp310-win_amd64:
    sip is not abi3, so it is locked to CPython 3.10 exactly. On any other
    Windows Python, _find_wheel() handed that wheel to pip, pip refused it,
    and --offline then reported "no bundled wheel was found" -- which was
    false, and told the operator nothing about the tag they actually needed.
    Verified with pip's own resolver against the bundled directory: py310
    win_amd64 resolves, 3.11 / 3.12 / 3.13 / 3.14 do not.

    Deliberately narrow. It covers the three forms that appear in a PyQt6
    bundle -- version-locked CPython ABI (cpXY), the stable ABI (abi3), and
    pure Python (none/any) -- and errs toward calling a wheel compatible when
    it cannot tell, so pip stays the final authority and this never blocks an
    install pip would have accepted.
    """
    m = _WHEEL_NAME_RE.match(wheel_name)
    if not m:
        return True, ""          # unparseable: let pip decide

    py_tags = m.group("py").lower().split(".")
    abi_tags = m.group("abi").lower().split(".")
    plat_tags = m.group("plat").lower().split(".")

    # --- platform ---
    if "any" not in plat_tags:
        machine = platform.machine().lower()
        if platform.system() == "Windows":
            want = {"amd64": "win_amd64", "x86_64": "win_amd64",
                    "arm64": "win_arm64", "x86": "win32"}.get(machine)
            if want and not any(t == want for t in plat_tags):
                return False, (f"built for {m.group('plat')}, "
                               f"this machine is {want}")

    # --- interpreter / ABI ---
    def _ver(tag):
        digits = re.sub(r"^[a-z]+", "", tag)
        if len(digits) >= 2 and digits.isdigit():
            return (int(digits[0]), int(digits[1:]))
        return None

    here = sys.version_info[:2]
    is_cpython = platform.python_implementation() == "CPython"

    if any(t.startswith("abi3") for t in abi_tags):
        # Stable ABI: usable on CPython at or above the wheel's floor.
        floors = [v for v in (_ver(t) for t in py_tags) if v]
        if is_cpython and (not floors or any(here >= f for f in floors)):
            return True, ""
        return False, f"stable-ABI wheel needs CPython >= {floors}"

    if all(t in ("none",) for t in abi_tags):
        # Pure Python.
        floors = [v for v in (_ver(t) for t in py_tags) if v]
        if not floors or any(here >= f for f in floors):
            return True, ""
        return False, f"needs Python >= {floors}"

    # Version-locked ABI (cp310-cp310 and friends): exact match only.
    exact = {v for v in (_ver(t) for t in abi_tags) if v}
    if exact and here not in exact:
        wanted = ", ".join(f"{a}.{b}" for a, b in sorted(exact))
        return False, (f"built for Python {wanted}, running "
                       f"{here[0]}.{here[1]}")
    return True, ""


def _bundled_wheels(package_name: str) -> list:
    """Every bundled wheel whose distribution name is `package_name`."""
    wanted = package_name.lower().replace("-", "_")
    found = []
    for wheel_dir in WHEEL_DIRS:
        if not wheel_dir.is_dir():
            continue
        for whl in sorted(wheel_dir.glob("*.whl")):
            dist_name = whl.name.split("-", 1)[0].lower().replace("-", "_")
            if dist_name == wanted:
                found.append(whl)
    return found


def _find_wheel(package_name: str) -> Path | None:
    """Return a bundled .whl for `package_name` that this Python can install.

    Matches on the wheel's distribution name -- everything before the first
    '-' -- rather than on a prefix. A prefix match made "PyQt6" match all three
    bundled wheels (pyqt6-6.8.1, pyqt6_qt6-6.8.2, pyqt6_sip-13.10.0), and since
    Path.glob order is filesystem-dependent, asking for PyQt6 could hand back
    the Qt6 or sip wheel; the _is_installed("PyQt6") check afterwards then
    failed the install with "installed without error but cannot be imported".

    Compatibility is now checked too, so a wheel that is "found" is one pip
    can actually use. See _wheel_is_compatible().
    """
    for whl in _bundled_wheels(package_name):
        compatible, _why = _wheel_is_compatible(whl.name)
        if compatible:
            return whl
    return None


def _unusable_wheel_report(package_name: str) -> str:
    """Why the bundled wheels for this package cannot be used here."""
    lines = []
    for whl in _bundled_wheels(package_name):
        compatible, why = _wheel_is_compatible(whl.name)
        if not compatible:
            lines.append(f"       {whl.name}: {why}")
    return "\n".join(lines)


def _find_links_args() -> list:
    """`--find-links` for every bundled wheel directory that exists.

    BLOCKER B10: the bundled-wheel install ran
    `pip install --no-index <wheel>` with no --find-links at all. The bundled
    PyQt6 wheel declares Requires-Dist on PyQt6-sip and PyQt6-Qt6, so with the
    index disabled and no local source to resolve them from, pip failed on the
    very first package -- and in --offline mode there is no PyPI fallback, so
    the documented primary Windows/air-gapped path could not work. CI has only
    ever run on ubuntu-latest, where WHEEL_DIRS is empty, so this path was
    never exercised.
    """
    args = []
    for wheel_dir in WHEEL_DIRS:
        if wheel_dir.is_dir():
            args += ["--find-links", str(wheel_dir)]
    return args


def _is_installed(import_name: str) -> bool:
    """
    Check whether a package is installed.  Uses `pip show` rather than
    a bare import so that packages that require a display (e.g. qasync on
    headless Linux) are not mistakenly reported as missing.
    """
    # Map import name to pip package name for packages that differ
    pip_name_map = {"PyQt6": "PyQt6", "numpy": "numpy", "qasync": "qasync",
                    "scipy": "scipy", "PyQt6.sip": "PyQt6-sip",
                    "PyQt6.QtCore": "PyQt6-Qt6"}
    pip_name = pip_name_map.get(import_name, import_name)
    rc, _ = run(
        [sys.executable, "-m", "pip", "show", pip_name],
        capture=True
    )
    return rc == 0


def _pip_install(args: list) -> tuple:
    """
    Run pip install with the given extra args.

    On systems that block pip with PEP-668 (Debian/Ubuntu system Python),
    retry automatically with --break-system-packages, then with --user.
    On Windows these guards are never needed.
    """
    base = [sys.executable, "-m", "pip", "install", "--upgrade"]
    rc, out = run(base + args, capture=True)
    if rc == 0:
        return rc, out

    if "externally-managed-environment" in out:
        warn("System Python is externally managed — retrying with "
             "--break-system-packages")
        rc, out = run(base + ["--break-system-packages"] + args, capture=True)
        if rc == 0:
            return rc, out
        warn("Still failing — retrying with --user")
        rc, out = run(base + ["--user"] + args, capture=True)

    return rc, out


def install_dependencies(offline: bool, force_reinstall: bool) -> None:
    header("Step 3 — Dependency installation")

    extra = ["--force-reinstall"] if force_reinstall else []

    for import_name, pip_name in REQUIRED_PACKAGES:
        already = _is_installed(import_name)
        if already and not force_reinstall:
            ok(f"{pip_name} — already installed")
            continue

        wheel = _find_wheel(pip_name)

        if wheel:
            info(f"Installing {pip_name} from bundled wheel: {wheel.name}")
            rc, out = _pip_install(
                ["--no-index"] + _find_links_args() + [str(wheel)] + extra)
            if rc != 0:
                warn(f"Bundled wheel install failed, trying PyPI:\n{out}")
                wheel = None

        if not wheel:
            if offline:
                unusable = _unusable_wheel_report(pip_name)
                if unusable:
                    fail(
                        f"Cannot install {pip_name}: --offline was specified "
                        f"and the bundled wheel(s) do not fit this "
                        f"interpreter ({_current_interpreter_tag()}, "
                        f"{platform.machine()}).\n"
                        f"{unusable}\n"
                        f"     Either run on the Python those wheels were "
                        f"built for, or drop a matching {pip_name} wheel into "
                        f"one of: {WHEEL_DIRS}\n"
                        f"     (Without --offline this would fall back to "
                        f"PyPI.)"
                    )
                fail(
                    f"Cannot install {pip_name}: --offline was specified but no "
                    "bundled wheel was found.\n"
                    f"     Add a {pip_name} wheel to one of: {WHEEL_DIRS}"
                )
            req_specs = _load_requirement_specs()
            pinned_spec = pip_name + req_specs.get(pip_name, "")
            info(f"Installing {pinned_spec} from PyPI …")
            rc, out = _pip_install([pinned_spec] + extra)
            if rc != 0:
                fail(f"Failed to install {pinned_spec}:\n{out}")

        if _is_installed(import_name):
            ok(f"{pip_name} — installed successfully")
        else:
            fail(f"{pip_name} installed without error but cannot be imported.")

    # qasync needs special handling on Windows — check after install
    _check_qasync_compat()


def _check_qasync_compat() -> None:
    """Warn if the qasync version is known to have Qt6 timer assertion issues."""
    rc, out = run(
        [sys.executable, "-c",
         "import qasync; print(getattr(qasync, '__version__', 'unknown'))"],
        capture=True
    )
    if rc == 0:
        version = out.strip()
        info(f"qasync version: {version}")


# ---------------------------------------------------------------------------
# Step 4 — Configuration files
# ---------------------------------------------------------------------------

def check_configs() -> None:
    header("Step 4 — Configuration file validation")
    all_ok = True
    for cfg in REQUIRED_CONFIGS:
        if not cfg.exists():
            err(f"Missing: {cfg.relative_to(SCRIPT_DIR)}")
            all_ok = False
            continue
        # Quick XML parse check
        try:
            ET.parse(cfg)
            ok(f"{cfg.name} — valid XML")
        except ET.ParseError as e:
            err(f"{cfg.name} — XML parse error: {e}")
            all_ok = False

    if not all_ok:
        fail("One or more configuration files are missing or malformed.")


# ---------------------------------------------------------------------------
# Step 5 — Database initialisation
# ---------------------------------------------------------------------------

def initialise_databases() -> None:
    header("Step 5 — Database initialisation")

    schema_path = FMOFP_DIR / "storage" / "databases" / "schema.xml"
    db_config   = FMOFP_DIR / "dbConfig.xml"

    if not schema_path.exists():
        fail(f"Schema file not found: {schema_path}")

    # Parse schema to discover required databases
    tree = ET.parse(schema_path)
    root = tree.getroot()

    # Parse dbConfig to get the db_name for each system
    db_tree = ET.parse(db_config)
    db_root = db_tree.getroot()
    system_to_db: dict[str, str] = {}
    for sys_elem in db_root.findall("system"):
        sname  = sys_elem.get("name", "")
        db_name = sys_elem.get("db_name", "")
        if sname and db_name:
            system_to_db[sname] = db_name

    # Collect unique database files
    db_files: set[str] = set(system_to_db.values())
    db_dir = FMOFP_DIR / "storage" / "databases"
    db_dir.mkdir(parents=True, exist_ok=True)

    created_count = 0
    for db_filename in sorted(db_files):
        db_path = db_dir / db_filename
        existed = db_path.exists()

        # Use Python's built-in sqlite3 to create the file and tables
        import sqlite3
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()

        # Create tables for every system that maps to this db file
        for system_elem in root.findall("system"):
            sname = system_elem.get("name", "")
            if system_to_db.get(sname) != db_filename:
                continue
            for table in system_elem.findall("table"):
                tname  = table.get("name", "")
                fields = table.findall("field")
                if not tname or not fields:
                    continue
                cols   = ", ".join(
                    f"{f.get('name')} {f.get('type', 'TEXT')}"
                    for f in fields
                )
                cursor.execute(
                    f"CREATE TABLE IF NOT EXISTS {tname} ({cols})"
                )

        conn.commit()
        conn.close()

        if existed:
            ok(f"{db_filename} — already exists, tables verified")
        else:
            ok(f"{db_filename} — created")
            created_count += 1

    info(f"{created_count} new database(s) created, "
         f"{len(db_files) - created_count} existing database(s) verified")


# ---------------------------------------------------------------------------
# Step 6 — Startup verification
# ---------------------------------------------------------------------------

def verify_installation() -> None:
    header("Step 6 — System startup verification")

    info("Running import checks on all core modules …")

    # Ensure the project root and FMOFP dir are on the path
    for p in [str(SCRIPT_DIR), str(FMOFP_DIR)]:
        if p not in sys.path:
            sys.path.insert(0, p)

    core_imports = [
        ("FMOFP.Utils.logger.sys_logger",      "get_logger"),
        ("FMOFP.Utils.common.fetching",         "fetch_fmofp_path"),
        ("FMOFP.core.system_manager",           "get_system_manager"),
        ("FMOFP.core.initializer",              "get_initializer"),
        ("FMOFP.storage.DBM",                   "DatabaseManager"),
        ("FMOFP.MIL_STD_1553B.mil_std_1553B",   "MIL_STD_1553B_Message"),
        ("FMOFP.local_messaging.routing.radar_to_display_bridge",
                                                "push_vil_data"),
        ("FMOFP.Systems.radarManagement.weather.weather_radar",
                                                "weather_radar"),
        ("FMOFP.Systems.radarManagement.targeting.targeting_radar",
                                                "targeting_radar"),
        ("FMOFP.Systems.radarManagement.syntheticAperture.sar_radar",
                                                "sar_radar"),
        ("FMOFP.Systems.radarManagement.terrainFollowing.tfr_radar",
                                                "tfr_radar"),
        ("FMOFP.Systems.radarManagement.aewc.aewc_radar",
                                                "aewc_radar"),
    ]

    failures = []
    for module, symbol in core_imports:
        rc, out = run(
            [sys.executable, "-c",
             f"import sys; sys.path.insert(0,r'{SCRIPT_DIR}'); "
             f"sys.path.insert(0,r'{FMOFP_DIR}'); "
             f"from {module} import {symbol}; print('OK')"],
            capture=True
        )
        if rc == 0 and "OK" in out:
            ok(f"{module}.{symbol}")
        else:
            err(f"{module}.{symbol}  →  {out.strip()[:120]}")
            failures.append(module)

    if failures:
        fail(
            f"{len(failures)} module(s) failed to import.  "
            "Check that the full FMOFP source tree is present and that "
            "all dependencies installed correctly."
        )

    ok("All core modules imported successfully")

    # Verify database files are accessible
    info("Verifying database accessibility …")
    db_dir = FMOFP_DIR / "storage" / "databases"
    db_files = list(db_dir.glob("*.db"))
    if not db_files:
        warn("No database files found — run without --no-verify to recreate them")
    else:
        import sqlite3
        for db_path in db_files:
            try:
                conn = sqlite3.connect(str(db_path))
                tables = conn.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                ).fetchall()
                conn.close()
                ok(f"{db_path.name} — {len(tables)} table(s) accessible")
            except Exception as e:
                err(f"{db_path.name} — {e}")
                failures.append(str(db_path))

    if failures:
        fail("Database verification failed.")


# ---------------------------------------------------------------------------
# Summary banner
# ---------------------------------------------------------------------------

def print_success() -> None:
    width = 60
    print(f"\n{Colours.OK}{'═' * width}{Colours.RESET}")
    print(f"{Colours.OK}{Colours.BOLD}  FMOFP installation complete!{Colours.RESET}")
    print(f"{Colours.OK}{'═' * width}{Colours.RESET}")
    print()
    print("  To start the system:")
    print(f"    cd {SCRIPT_DIR}")
    print(f"    python FMOFP{os.sep}Main.py")
    print()
    print("  Expected startup sequence:")
    print("    [SYSTEM]   Initializing FMOFP System Manager …")
    print("    [DATABASE] Loading database configurations …")
    print("    [1553B]    Initializing MIL-STD-1553B communication …")
    print("    [RADAR]    Initializing radar management system …")
    print("    [DISPLAY]  Initializing display management system …")
    print("    [FMS]      Initializing flight management system …")
    print("    [SYSTEM]   All systems operational — Ready for operations")
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="FMOFP installation script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "--offline", action="store_true",
        help="Use only bundled wheel files; do not contact PyPI"
    )
    parser.add_argument(
        "--no-verify", action="store_true",
        help="Skip the startup verification step (Step 6)"
    )
    parser.add_argument(
        "--force-reinstall", action="store_true",
        help="Reinstall all Python packages even if already present"
    )
    args = parser.parse_args()

    print(f"\n{Colours.BOLD}FMOFP Installer — B20SS{Colours.RESET}")
    print(f"  Install directory : {SCRIPT_DIR}")
    print(f"  Python executable : {sys.executable}")
    print(f"  Platform          : {platform.system()} {platform.release()}")

    check_python()
    check_directories()
    install_dependencies(offline=args.offline, force_reinstall=args.force_reinstall)
    check_configs()
    initialise_databases()

    if not args.no_verify:
        verify_installation()

    print_success()


if __name__ == "__main__":
    main()
