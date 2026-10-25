"""
Test suite: the configured log level governs the file, not just the console (H21).

One finding.

<level> IN startupConfiguration.xml REACHED THE CONSOLE ONLY. The console
handler was built with setLevel(self.log_level); the file handler was built
with setLevel(logging.DEBUG), hardcoded. So the single knob the configuration
offers for how much to log governed the terminal and not the file -- and the
file is the sink with the 250MB rotation ceiling.

The asymmetry was not uniform, which is why it reads as deliberate until it is
measured. self.logger and self.command_logger ARE set to log_level, and a
LOGGER level filters before any handler sees the record, so records sent
through those two were filtered correctly. Records sent through the ROOT
logger were not: the root logger is pinned to DEBUG as the floor that lets
handlers decide, and every SysLogger.debug/info/warning/error/critical call
delegates to the root logger -- which is how essentially all application
logging in this codebase is written.

Measured with <level>error</level>, one record at each of five levels through
each of the three loggers (15 records):

    console: 6 of 15   error and critical only, correct
    file:    9 of 15   error and critical from system and command, and ALL
                       FIVE levels from the root logger

The log file is named after the configured level, so the result was a file
called ERROR_<timestamp>.log full of root-logger DEBUG records. An operator
who set 'error' to get a quiet, small log got neither, and the filename told
them they had.

Invisible on a shipped system: startupConfiguration.xml ships
<level>Debug</level>, where correct and incorrect behaviour are identical. It
bites only the operator who changes it -- which is the one person who would
not then go looking.

Run:  python3 -m FMOFP.Tests.test_logging_levels
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from FMOFP.Tests import code_has  # whitespace-insensitive code matcher

import json
import re
import shutil
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

LEVELS = ("debug", "info", "warning", "error", "critical")
LOGGERS = ("root", "system", "command")

# SysLogger is a singleton that reads its configuration from a file at import
# time, so each configured level needs its own process and its own config.
_CHILD = r'''
import glob, json, logging, os, re, sys
sys.path.insert(0, %(b20ss)r)
sys.path.insert(0, %(fmofp)r)

import FMOFP.Utils.logger.sys_logger as sl
# The config path is derived from the project root and is not overridable by
# environment, so point the module's own resolver at the crafted config.
sl.fetch_fmofp_path = lambda: %(cfgdir)r

from FMOFP.Utils.common.fetching import resolve_data_dir

log = sl.get_logger()
targets = {"root": log.root_logger, "system": log.logger, "command": log.command_logger}
for name, lg in targets.items():
    for lvl in %(levels)r:
        getattr(lg, lvl)("PROBEMARK %%s %%s" %% (name, lvl))
logging.shutdown()

in_file = set()
logs_dir = resolve_data_dir("logs")
for path in glob.glob(os.path.join(logs_dir, "*.log")):
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = re.search(r"PROBEMARK (\w+) (\w+)", line)
            if m:
                in_file.add(m.group(1) + "/" + m.group(2))
print("RESULT" + json.dumps({
    "configured": log.level,
    "log_level": logging.getLevelName(log.log_level),
    "in_file": sorted(in_file),
    "log_files": sorted(os.path.basename(p) for p in glob.glob(os.path.join(logs_dir, "*.log"))),
}))
'''


def _run_with_level(level):
    """Boot SysLogger in a child process under <level>, return its result dict.

    Returns (result, console_marks). console_marks is what reached stdout,
    which is where the console handler writes.
    """
    with tempfile.TemporaryDirectory() as tmp:
        cfgdir = os.path.join(tmp, "FMOFP")
        os.makedirs(os.path.join(cfgdir, "Utils", "logger"))
        # Craft the config. Everything except <level> matches what ships.
        with open(os.path.join(cfgdir, "startupConfiguration.xml"), "w",
                  encoding="utf-8") as fh:
            fh.write(
                "<startupConfiguration>\n"
                "    <logging>\n"
                "        <commandInterface>False</commandInterface>\n"
                "        <logging_enabled>True</logging_enabled>\n"
                "        <debugging>True</debugging>\n"
                f"        <level>{level}</level>\n"
                "        <console_output>True</console_output>\n"
                "    </logging>\n"
                "</startupConfiguration>\n"
            )
        shutil.copy(
            os.path.join(_B20SS, "FMOFP", "Utils", "logger",
                         "filtersConfiguration.xml"),
            os.path.join(cfgdir, "Utils", "logger", "filtersConfiguration.xml"),
        )
        script = _CHILD % {
            "b20ss": _B20SS,
            "fmofp": os.path.join(_B20SS, "FMOFP"),
            "cfgdir": cfgdir,
            "levels": LEVELS,
        }
        env = dict(os.environ)
        env["FMOFP_DATA_DIR"] = os.path.join(tmp, "data")
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["PYTHONUTF8"] = "1"
        env["PYTHONIOENCODING"] = "utf-8"
        proc = subprocess.run([sys.executable, "-c", script],
                              capture_output=True, text=True, timeout=120,
                              cwd=_B20SS, env=env)

    result = None
    for line in (proc.stdout or "").splitlines():
        if line.startswith("RESULT"):
            result = json.loads(line[len("RESULT"):])
    console = set(f"{m.group(1)}/{m.group(2)}" for m in
                  re.finditer(r"PROBEMARK (\w+) (\w+)", proc.stdout or ""))
    # The child's own RESULT line echoes the marks; don't count those.
    console = {c for c in console}
    return result, console, proc


def _expected(level):
    """Which marks should survive a handler set to `level`."""
    order = {lvl: i for i, lvl in enumerate(LEVELS)}
    floor = order[level]
    return {f"{lg}/{lvl}" for lg in LOGGERS for lvl in LEVELS
            if order[lvl] >= floor}


def test_the_file_honours_the_configured_level():
    R.section("the file records what <level> asks for, at every level")

    for level in ("error", "warning", "info", "debug"):
        result, console, proc = _run_with_level(level)
        if result is None:
            R.check(f"<level>{level}</level>: the child reported a result",
                    False, (proc.stderr or "")[-300:])
            continue

        R.check(f"<level>{level}</level> is read as {level.upper()}",
                result["log_level"] == level.upper(), str(result["log_level"]))

        got = set(result["in_file"])
        want = _expected(level)
        R.check(f"<level>{level}</level>: the file holds exactly the records "
                f"at or above it ({len(want)} of {len(LOGGERS) * len(LEVELS)})",
                got == want, f"extra={sorted(got - want)} missing={sorted(want - got)}")


def test_the_root_logger_was_the_leak():
    R.section("NON-TAUTOLOGICAL — where the records used to come through")

    result, console, proc = _run_with_level("error")
    R.check("the child ran", result is not None, (proc.stderr or "")[-200:])
    if result is None:
        return
    got = set(result["in_file"])

    # The pre-fix behaviour, stated exactly: all five root levels present,
    # and nine marks in total. If this suite ever has to be changed because
    # those numbers come back, the defect came back.
    leaked = {f"root/{lvl}" for lvl in ("debug", "info", "warning")}
    R.check("no root-logger record below ERROR reaches the file now",
            not (got & leaked), str(sorted(got & leaked)))
    R.check("...and the file holds 6 marks, not the 9 it used to",
            len(got) == 6, f"{len(got)}: {sorted(got)}")

    # The system/command loggers were never the problem -- they filter at the
    # logger, before any handler. Their records at and above the level must
    # still be there, or the fix has over-filtered.
    for name in ("system", "command"):
        R.check(f"{name}/error and {name}/critical still reach the file",
                {f"{name}/error", f"{name}/critical"} <= got,
                str(sorted(m for m in got if m.startswith(name))))


def test_the_console_is_unchanged():
    R.section("the console still behaves as it always did")

    result, console, proc = _run_with_level("error")
    R.check("the child ran", result is not None)
    if result is None:
        return

    want = _expected("error")
    R.check("the console shows exactly error and critical, from all three "
            "loggers (6 of 15)",
            console == want, f"extra={sorted(console - want)} "
                             f"missing={sorted(want - console)}")

    # And the file and console now agree, which is the point of the change.
    R.check("file and console now carry the same set",
            set(result["in_file"]) == console,
            f"file={sorted(result['in_file'])} console={sorted(console)}")


def test_the_filename_no_longer_misdescribes_its_contents():
    R.section("a file named for a level holds records of that level")

    result, console, proc = _run_with_level("error")
    R.check("the child ran", result is not None)
    if result is None:
        return

    named = [f for f in result["log_files"] if f.startswith("ERROR_")]
    R.check("the log file is named after the configured level",
            named, str(result["log_files"]))
    R.check("...and contains no record below that level",
            not any(m.endswith(("/debug", "/info", "/warning"))
                    for m in result["in_file"]),
            str(sorted(result["in_file"])))


def test_the_shipped_config_is_unaffected():
    R.section("the shipped configuration behaves identically either way")

    # This is why the defect was invisible: at Debug, log_level IS DEBUG, so
    # the hardcoded value and the configured one are the same number.
    result, console, proc = _run_with_level("debug")
    R.check("the child ran", result is not None)
    if result is None:
        return
    R.check("at <level>debug</level> the file holds all 15 marks, as before",
            len(set(result["in_file"])) == len(LOGGERS) * len(LEVELS),
            str(len(set(result["in_file"]))))

    with open(os.path.join(_B20SS, "FMOFP", "startupConfiguration.xml"),
              encoding="utf-8") as fh:
            shipped = fh.read()
    R.check("and the shipped config is still Debug, so nothing changes for "
            "an unmodified install",
            re.search(r"<level>\s*Debug\s*</level>", shipped) is not None)


def test_the_diagnostic_capture_is_deliberately_left_alone():
    R.section("TestLogHandler stays at DEBUG, on purpose")

    src = _run_source()
    R.check("the file handler takes the configured level",
            code_has(src, "file_handler.setLevel(self.log_level)"))
    R.check("the console handler is unchanged",
            code_has(src, "console_handler.setLevel(self.log_level)"))
    R.check("the test capture handler is still DEBUG",
            code_has(src, "test_log_handler.setLevel(logging.DEBUG)"))
    R.check("the root logger is still the DEBUG floor, so handlers decide",
            code_has(src, "self.root_logger.setLevel(logging.DEBUG)"))
    # The hardcoded value this replaced must be gone from the file handler.
    R.check("the file handler no longer hardcodes DEBUG",
            not code_has(src, "file_handler.setLevel(logging.DEBUG)"))
    # And code_has must be able to answer False, or the five above are free.
    R.check("code_has() reports False for a call that is not there",
            not code_has(src, "file_handler.setLevel(logging.NOTSET)"))

    # It writes nothing until a record matches a test-start pattern, which is
    # why leaving it verbose costs an operator nothing.
    handler_src = _handler_source()
    R.check("it only opens a file once a test start is detected",
            code_has(handler_src, "detect_test_start")
            and code_has(handler_src, "setup_startup_file"))


def _run_source():
    with open(os.path.join(_B20SS, "FMOFP", "Utils", "logger", "sys_logger.py"),
              encoding="utf-8") as fh:
        return fh.read()


def _handler_source():
    with open(os.path.join(_B20SS, "FMOFP", "Utils", "logger",
                           "test_log_handler.py"), encoding="utf-8") as fh:
        return fh.read()


def main():
    print("=" * 60)
    print("  The configured log level governs the file too")
    print("=" * 60)

    for test in (test_the_file_honours_the_configured_level,
                 test_the_root_logger_was_the_leak,
                 test_the_console_is_unchanged,
                 test_the_filename_no_longer_misdescribes_its_contents,
                 test_the_shipped_config_is_unaffected,
                 test_the_diagnostic_capture_is_deliberately_left_alone):
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
