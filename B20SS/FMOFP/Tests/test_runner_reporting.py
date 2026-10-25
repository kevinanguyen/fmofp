"""
Test suite: a failing suite has to be diagnosable from the CI log.

One finding.

THE EXCERPT COULD LOSE THE VERDICT ENTIRELY. run_all_tests prints the last
TAIL_LINES (60) lines of a failing suite's output. A suite prints its FAIL
lines and its "N/M passed" score, and then its fixtures shut a real
application down, which logs heavily.

Measured on the CI run of 2026-10-24: test_weather_radar_live failed, emitted
36,130 stdout lines, and the 60-line tail was entirely shutdown records --
"AEWC radar aewc_radar stopped", "BC_sender has been shut down." -- ending in
"[harness] FAILED: body returned 1" with no failing check anywhere in the
excerpt. The suite does not reproduce locally (six consecutive clean runs),
so the only record of what failed was the part that got clipped.

There was already a comment at that site recording the same class of problem
from Sept 2026, when the excerpt showed stderr only and the suite's FAIL
lines were on stdout. That fix widened WHICH stream is shown; this one fixes
WHERE in the stream the harness looks, which is the half that was still
broken.

verdict_lines() lifts the suite's own markers out by pattern wherever they
sit, and main() prints them above the tail.

Run:  python3 -m FMOFP.Tests.test_runner_reporting
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import contextlib
import io
import tempfile
import textwrap
import traceback

from FMOFP.Tests import run_all_tests as runner


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

# A failing suite's output, shaped like the real one: verdict first, then
# thousands of shutdown log records. The noise lines are real lines from the
# CI log that clipped the verdict.
_NOISE = "\n".join(
    "2026-10-24 23:53:59,576 - root - INFO - AEWC radar aewc_radar stopped"
    for _ in range(5000)
)

_BODY = textwrap.dedent("""\
      PASS  the radar reaches SURVEILLANCE
      FAIL  the VIL overlay was withheld  [items=0]

      54/55 passed

      Failures:
          FAIL  the VIL overlay was withheld  [items=0]
    """)


def test_the_tail_alone_loses_the_verdict():
    R.section("NON-TAUTOLOGICAL — what the tail-only excerpt showed")

    text = _BODY + _NOISE
    tail = text.splitlines()[-runner.TAIL_LINES:]

    R.check("the stream is long enough to clip, as the real one was",
            len(text.splitlines()) > runner.TAIL_LINES * 10,
            str(len(text.splitlines())))
    R.check("no FAIL line survives in the last 60 lines",
            not any(line.strip().startswith("FAIL") for line in tail))
    R.check("...and neither does the score",
            not any("passed" in line for line in tail))
    R.check("so the tail-only excerpt named no failing check at all",
            all("VIL overlay" not in line for line in tail))


def test_verdict_lines_finds_them_anyway():
    R.section("verdict_lines() lifts the verdict out of the stream")

    got = runner.verdict_lines(_BODY + _NOISE)

    R.check("the failing check is surfaced",
            any("VIL overlay was withheld" in line for line in got), str(got[:3]))
    R.check("...with its detail intact",
            any("items=0" in line for line in got))
    R.check("the score is surfaced", any("54/55 passed" in line for line in got))
    R.check("the Failures: block header is surfaced",
            any(line.strip().startswith("Failures:") for line in got))
    R.check("and none of the 5000 noise lines came with it",
            not any("aewc_radar stopped" in line for line in got),
            str(len(got)))


def test_it_does_not_match_log_records_that_merely_say_fail():
    R.section("a log record containing the word is not a verdict line")

    # These are real shapes from the suites' own logging. If any of them
    # qualified, the section would fill with noise and be as useless as the
    # tail was.
    decoys = "\n".join([
        "2026-10-24 23:51:29,033 - root - ERROR - send FAILED for request r-1",
        "2026-10-24 23:51:29,033 - root - WARNING - FAILED to bind listener",
        "  the word FAIL appears mid-line here",
        "PASSED: everything",
        "2026-10-24 - root - INFO - 54/55 passed",
    ])
    got = runner.verdict_lines(decoys)
    R.check("no decoy line is taken for a verdict", got == [], str(got))

    # And the scanner is not simply blind: the same call over real verdict
    # lines must return them.
    R.check("the same scanner does find real verdict lines",
            runner.verdict_lines(_BODY) != [])

    # A timed-out suite's marker, and the harness's own, both qualify.
    R.check("the harness's own failure marker qualifies",
            runner.verdict_lines("  [harness] FAILED: body returned 1") != [])


def test_the_section_is_capped_and_keeps_the_end():
    R.section("a suite with many failures does not flood the excerpt")

    many = "\n".join(f"  FAIL  check number {i}" for i in range(200))
    tail_marker = "  7/207 passed"
    got = runner.verdict_lines(many + "\n" + tail_marker)

    R.check("the section is capped", len(got) <= runner.VERDICT_LINES + 1,
            str(len(got)))
    R.check("the score is kept, because it comes last",
            any("7/207 passed" in line for line in got))
    R.check("the last failures are kept rather than the first",
            any("check number 199" in line for line in got)
            and not any("check number 0" in line for line in got))
    R.check("and the excerpt says how many were omitted",
            any("omitted" in line for line in got), str(got[:1]))


def test_main_prints_the_verdict_for_a_real_failing_child():
    R.section("end to end: a failing child suite, through main()")

    # A real subprocess, shaped like the suite that clipped: verdict, then
    # 5000 log lines, then exit 1.
    script = textwrap.dedent(f"""
        import sys
        sys.stdout.write({_BODY!r})
        for _ in range(5000):
            print("2026-10-24 23:53:59,576 - root - INFO - "
                  "AEWC radar aewc_radar stopped")
        sys.exit(1)
    """)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "clipping_suite.py")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(script)

        original = runner.SUITES
        runner.SUITES = [(False, path, 60)]
        buffer = io.StringIO()
        try:
            with contextlib.redirect_stdout(buffer):
                rc = runner.main()
        finally:
            runner.SUITES = original

    printed = buffer.getvalue()
    R.check("main() reports the failure", rc == 1, str(rc))
    R.check("and the excerpt names the failing check",
            "VIL overlay was withheld" in printed)
    R.check("...and shows the score", "54/55 passed" in printed)
    R.check("...under its own heading",
            "the suite's own verdict lines" in printed)
    R.check("the tail is still printed for context",
            "aewc_radar stopped" in printed)


def main():
    print("=" * 60)
    print("  A failing suite must be diagnosable from the CI log")
    print("=" * 60)

    for test in (test_the_tail_alone_loses_the_verdict,
                 test_verdict_lines_finds_them_anyway,
                 test_it_does_not_match_log_records_that_merely_say_fail,
                 test_the_section_is_capped_and_keeps_the_end,
                 test_main_prints_the_verdict_for_a_real_failing_child):
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
