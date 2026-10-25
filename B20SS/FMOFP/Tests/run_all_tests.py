"""Aggregating test runner — runs every standalone-safe test suite with one
real exit code.

Why this exists (August 2026 production-readiness follow-up): the repo had 23
test files but CI executed only 7 of them as individually-maintained workflow
steps, and PLANNING.md had already flagged that several test entry points
discard run_tests()'s result (false-positive risk). This runner is the single
source of truth for "the test suite": it subprocess-isolates each suite,
enforces a per-suite watchdog timeout (a hang is a failure, not a stall —
see ci_test_boot_smoke.py for why that matters here), aggregates results,
and exits nonzero if ANY suite fails, times out, or crashes.

Run from B20SS/:  python FMOFP/Tests/run_all_tests.py

Suites deliberately NOT run here, with reasons:
  - All ten CLI-harness-only suites are gone. They exited 1 by design when run
    standalone, printing "This test should be run via the user CLI 'test'
    command", which kept 6,991 lines -- two thirds of the project's test code --
    out of CI. Story C14.1 removed that blocker with Tests/live_system.py, and
    C14.2 - C14.7 replaced every one of them with state assertions:

      radar_tests/weather_radar_test.py          }
      weather_radar_surveillance_mode_test.py    }  test_weather_radar_live.py
      combined_precipitation_vil_flow_test.py    }

      radar_tests/targeting_radar_test.py        }
      radar_tests/tfr_radar_test.py              }  test_radar_modes_live.py
      radar_tests/aewc_radar_test.py             }
      radar_tests/sar_radar_test.py              }

      fms_system_test.py                         -> test_fms_live.py
      flight_control_system_test.py              -> test_flight_control_live.py
      predefined_messages_test.py                -> test_predefined_messages_live.py

    The radar_tests/ package went with them: it held nothing but those five
    files and a docstring promising "comprehensive tests for all radar systems".

  - test_weather_radar_holographic_display: interactive GUI test — enters
    QApplication.exec() and never exits; visual inspection only.
  - performance_profile: a profiler, not a pass/fail test.
  - setup_env: an import helper, not a test.
"""
import os
import re
import subprocess
import sys
import time

# (module path run with -m?, name, timeout seconds)
SUITES = [
    (True,  "FMOFP.Tests.ci_test_boot_smoke", 300),
    (True,  "FMOFP.Tests.test_bridge_and_coordinator", 300),
    (True,  "FMOFP.Tests.test_displays_headless", 300),
    (False, "FMOFP/Tests/ci_test_weather_radar.py", 300),
    (False, "FMOFP/Tests/ci_test_scenario_engine.py", 300),
    (True,  "FMOFP.Tests.test_power_fuel_thrust", 300),
    (True,  "FMOFP.Tests.test_hydr_airframe_ecs_fdm_fitness_swcm", 300),
    (True,  "FMOFP.Tests.test_toctou_start_race_regression", 600),
    (True,  "FMOFP.Tests.test_precipitation_data_transfer", 300),
    (True,  "FMOFP.Tests.test_install_script", 300),
    (True,  "FMOFP.Tests.test_bus_adapter", 300),
    (True,  "FMOFP.Tests.test_scenario_failure_injection", 300),
    (True,  "FMOFP.Tests.test_db_connection_pool", 300),
    (True,  "FMOFP.Tests.test_thread_and_cli_resilience", 300),
    (True,  "FMOFP.Tests.test_radar_shutdown_health", 300),
    (True,  "FMOFP.Tests.test_health_and_readiness", 300),
    (True,  "FMOFP.Tests.test_listener_retry_and_ports", 300),
    (True,  "FMOFP.Tests.test_data_root", 300),
    (True,  "FMOFP.Tests.test_line_endings", 300),
    # Production blockers B5/B11/B12: singleton re-initialisation leaking a
    # thread pool per construction, run-once markers that outlived the process
    # (so every boot after the first skipped database init and never re-read
    # the message-rate config), and an import-time filesystem walk that indexed
    # site-packages on an installed deployment.
    (True,  "FMOFP.Tests.test_blocker_singleton_and_state", 300),
    # Production blockers B6/B7/B8: FCS mode change calling a method that does
    # not exist, FCS control input reading a bool as a dict (after the surface
    # had already moved), and the radar mode-lookup tables being enum members
    # rather than dicts, so every lookup by mode name fell back to STANDBY.
    (True,  "FMOFP.Tests.test_blocker_command_paths", 300),
    # Production blocker B9: one encoder, five decoders, four different sets of
    # precipitation scale factors -- severe weather decoded ~79x low and
    # rendered in the lightest colour band. Round-trip assertions.
    (True,  "FMOFP.Tests.test_blocker_precip_scale", 300),
    # The weather radar must not invent severity: precipitation, VIL, storm-cell
    # and turbulence readings that never arrived used to be substituted with
    # literals inside real colour bands (intensity 0.7 / rate 20 mm/h, VIL 20.0,
    # category LIGHT, cell intensity 0), so a degraded return looked measured.
    # Also covers two defects that made the whole thing moot: the particle draw
    # path raised AttributeError on self.width() and painted nothing, and the
    # 'collection' link row shadowed every real point on an exact request_id
    # lookup.
    (True,  "FMOFP.Tests.test_weather_radar_truthfulness", 300),
    # H4: send_request guarded itself with `if not self._can_send_request():` on
    # an async method. The un-awaited coroutine is always truthy, so the gate
    # never ran -- losing both the rate limit and, more seriously, the readiness
    # check that decides whether anything is listening. Adding `await` while
    # keeping the surrounding `while` would have turned it into an infinite spin.
    (True,  "FMOFP.Tests.test_radar_send_rate_limit", 300),
    # H17: SystemDatabase.ensure_column_exists had the log prefix "[DBM] "
    # pasted inside its ALTER statement, checked for an existing column with a
    # substring match against the CREATE TABLE text, and swallowed the resulting
    # OperationalError. It added nothing, ever, and said it had.
    (True,  "FMOFP.Tests.test_db_ensure_column", 300),
    # H2/H16: an accepted connection had no idle timeout, so a peer that
    # connected and said nothing held its handler for as long as it stayed
    # connected; and get_listen_endpoint validated the port but passed the host
    # straight to bind().
    (True,  "FMOFP.Tests.test_listener_idle_and_bind", 300),
    # H3: both listeners called handle_connection() inline on the accept thread,
    # so one held connection stopped every other peer being accepted at all.
    # Connections are now served on a bounded pool of worker threads.
    (True,  "FMOFP.Tests.test_listener_concurrency", 420),
    # H5: RadarMessageHandler.send_request held a threading.Lock across three
    # awaits, and FMSResponseService.send_response across its callback await. A
    # thread lock blocks the thread, not the task, so two concurrent calls
    # deadlocked the event loop outright instead of serialising.
    (True,  "FMOFP.Tests.test_handler_lock_discipline", 420),
    # H8/H7: RadarMessageHandler._start_cleanup_timer() had no caller, so
    # pending requests were never expired or retried and the expiry path's own
    # bugs had never executed; and FCS/FMS delivered timeout notifications via
    # asyncio.get_event_loop() from a threading.Timer thread, where it raises.
    (True,  "FMOFP.Tests.test_request_expiry", 420),
    # The interpreters CI covers and the interpreters a shipped offline
    # install supports were not the same set: PyQt6_sip is not abi3, so the
    # bundled wheel is locked to CPython 3.10 and --offline has no PyPI
    # fallback. CI never caught it because it only runs on ubuntu-latest,
    # where WHEEL_DIRS is empty and that path is never exercised.
    (True,  "FMOFP.Tests.test_offline_install_matrix", 180),
    # The bus send path could not finish: the block-transfer split produced
    # chunks that were themselves "large", so any message over 33 frames
    # recursed forever; the transport socket had no timeout; the caller's
    # future.result() had none either; and one dispatch site submitted to an
    # event loop nobody ran, so it burned its full 5 s timeout every call.
    (True,  "FMOFP.Tests.test_send_path_bounds", 300),
    # Production blockers B1/B2/B3/B4: a boot failure that hung the process
    # forever and an error shutdown that exited 0; initialization failures
    # swallowed so boot continued on a half-built system; coroutine stops
    # discarded when the loop was halted straight after scheduling them; and an
    # unbounded EventBus join holding the shared lock.
    (True,  "FMOFP.Tests.test_blocker_lifecycle", 300),
    # Story C14.3: four radars swept across every commandable mode against a
    # live system, plus phase-policy and request-dispatch assertions.
    (True,  "FMOFP.Tests.test_radar_modes_live", 420),
    # Story C14.4: FMS mode contract, the FMS->FCS mode mapping, attitude
    # handling and request dispatch, against a live system.
    (True,  "FMOFP.Tests.test_fms_live", 420),
    # Story C14.5: FCS mode vocabulary, control-input saturation and the
    # one-way FMS -> FCS coupling, against a live system.
    (True,  "FMOFP.Tests.test_flight_control_live", 420),
    # FuelManagementSystem's worker loop called three methods outside its
    # try, so ONE transient error killed fuel management permanently and
    # silently -- reproduced with a single injected RuntimeError -- while
    # get_status() went on reporting running=True over the dead thread. An
    # AST sweep found it was the only one of the nine safety-critical
    # subsystems with unguarded calls in its loop; the sweep is kept here as
    # a regression guard, together with the consecutive-failure reporting
    # that stops the guard from turning a dead subsystem into a HEALTHY one.
    (True,  "FMOFP.Tests.test_worker_loop_resilience", 180),
    # H21: <level> in startupConfiguration.xml reached the console only. The
    # file handler was setLevel(logging.DEBUG), hardcoded, so the one knob
    # for how much to log governed the terminal and not the 250MB file --
    # measured at <level>error</level>: console 6 of 15 records, file 9 of
    # 15, including all five levels from the root logger, which is the logger
    # every SysLogger call delegates to. The file was named ERROR_*.log.
    (True,  "FMOFP.Tests.test_logging_levels", 180),
    # A failing suite has to be diagnosable from the CI log. The 60-line
    # tail excerpt could clip the suite's own verdict entirely: on
    # 2026-10-24 test_weather_radar_live failed, printed 36,130 lines, and
    # the excerpt was sixty lines of shutdown logging with no failing check
    # in it. verdict_lines() lifts the suite's markers out by pattern
    # wherever they sit in the stream.
    (True,  "FMOFP.Tests.test_runner_reporting", 120),
    # Story C25.1, the FCS half: the FCS's command words were invented from
    # a private 0x4000 base rather than derived from the address book, so
    # 0x4007 decoded to RT address 8 -- Power Management -- and every FCS
    # mode change and status request went out addressed there. Its data
    # words had two incompatible encodings, one of them with no callers and
    # nothing to decode it, and an unrecognised mode or surface was encoded
    # as a different one rather than refused.
    (True,  "FMOFP.Tests.test_fcs_wire_format", 180),
    # Story C14.6: the predefined message facade -- initialisation, the three
    # accepted radar-mode input forms, rejection, and request-ID contracts.
    (True,  "FMOFP.Tests.test_predefined_messages_live", 420),
    # Story C14: runs against a REAL booted application via
    # Tests/live_system.py, so it needs a longer budget than the unit
    # suites -- boot plus supervisory-convergence polling.
    (True,  "FMOFP.Tests.test_weather_radar_live", 420),
]


def _force_utf8_output():
    """This runner prints glyphs too, and its own stdout may be a pipe.

    Redirecting run_all_tests.py's output to a file or through a pager on
    Windows would hit the same cp1252 failure the suites hit. Best effort: a
    closed or exotic stream just keeps its current encoding.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError, ValueError):
            pass


def run_suite(as_module: bool, name: str, timeout_s: int):
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    # Every suite prints box-drawing and check glyphs. A child's stdout here is
    # a PIPE, so Python picks the locale encoding for it -- cp1252 on Windows --
    # and the first such print() raises UnicodeEncodeError before the suite has
    # tested anything. Nine of the 28 suites died that way, and the handler then
    # raised again trying to print the failure marker. PYTHONIOENCODING takes
    # precedence over PYTHONUTF8, so both are set; setdefault leaves an explicit
    # choice by the caller alone.
    env.setdefault("PYTHONUTF8", "1")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    cwd = os.getcwd()
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in (cwd, env.get("PYTHONPATH", "")) if p
    )
    cmd = [sys.executable, "-m", name] if as_module else [sys.executable, name]
    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd, timeout=timeout_s, capture_output=True,
            encoding="utf-8", errors="replace",
            env=env, cwd=cwd,
        )
        elapsed = time.monotonic() - start
        return proc.returncode, elapsed, proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as exc:
        elapsed = time.monotonic() - start
        out = exc.stdout or b""
        err = exc.stderr or b""
        if isinstance(out, bytes):
            out = out.decode(errors="replace")
        if isinstance(err, bytes):
            err = err.decode(errors="replace")
        return "TIMEOUT", elapsed, out, err


# How many trailing lines of each stream to show for a failing suite. 20 was
# too few to reach the suite's own verdict past Qt's startup warnings.
TAIL_LINES = 60

# How many verdict lines to lift out of the stream, wherever they sit in it.
VERDICT_LINES = 40


def verdict_lines(text, limit=VERDICT_LINES):
    """The suite's own failure lines and score, wherever they are in `text`.

    WHY A TAIL IS NOT ENOUGH. A suite prints its FAIL lines and its "N/M
    passed" score, and then its fixtures shut a real application down,
    which logs. test_weather_radar_live emitted 36,130 lines, and the
    shutdown logging after the summary was longer than the 60-line tail --
    so the CI excerpt for a genuine failure was sixty lines of
    "AEWC radar aewc_radar stopped" and not one word about what failed.
    Measured on the CI log of 2026-10-24: "[harness] FAILED: body returned
    1" with no failing check anywhere in the excerpt, and the suite does not
    reproduce locally, so the information was simply gone.

    Lifting these lines out by pattern makes a failing suite diagnosable
    from the CI log no matter how much it logs afterwards. The tail is still
    printed, for context around the end.
    """
    wanted = []
    for line in (text or "").splitlines():
        stripped = line.strip()
        # The suites' own markers. "FAIL" is matched at the start of the
        # stripped line so that a log record merely containing the word --
        # and there are plenty -- does not qualify.
        if (stripped.startswith("FAIL")
                or stripped.startswith("✗")
                or stripped.startswith("[harness]")
                or stripped.startswith("Failures:")
                or _SCORE_RE.match(stripped)):
            wanted.append(line)
    if len(wanted) > limit:
        dropped = len(wanted) - limit
        # Keep the LAST ones: the score and the Failures: block come last.
        wanted = [f"... {dropped} earlier verdict line(s) omitted"] + wanted[-limit:]
    return wanted


_SCORE_RE = re.compile(r"^\d+/\d+ passed$")


def main() -> int:
    _force_utf8_output()
    print(f"Running {len(SUITES)} test suites "
          "(subprocess-isolated, per-suite watchdog)\n" + "=" * 60)
    failures = []
    for as_module, name, timeout_s in SUITES:
        rc, elapsed, out, err = run_suite(as_module, name, timeout_s)
        label = name.rsplit("/", 1)[-1]
        if label.endswith(".py"):
            label = label[:-3]
        else:
            label = label.rsplit(".", 1)[-1]
        if rc == 0:
            print(f"  ✓  {label:<45s} {elapsed:6.1f}s")
        else:
            status = "TIMED OUT" if rc == "TIMEOUT" else f"exit {rc}"
            print(f"  ✗  {label:<45s} {elapsed:6.1f}s  ({status})")
            failures.append((name, rc))
            # Show BOTH streams. This used to be `(err or out)`, which threw
            # stdout away whenever stderr had any content at all -- and stderr
            # is never empty here, because Qt's offscreen plugin warns and the
            # shutdown path logs tracebacks. The result was that a failing
            # suite reported 20 lines of Qt noise while the suite's own "FAIL"
            # lines and verdict, which are on stdout, were discarded. That made
            # a CI failure of test_weather_radar_live undiagnosable from the
            # log (Sept 2026).
            print("     ┌─ last output " + "─" * 40)
            shown = False

            # The suite's own verdict first, pulled from anywhere in the
            # stream, because the tail below can miss it entirely.
            verdict = verdict_lines(out) or verdict_lines(err)
            if verdict:
                print("     │ ── the suite's own verdict lines ──────────")
                for line in verdict:
                    print("     │ " + line)
                shown = True

            # A suite can end up emitting every record to both streams (for
            # example by adding a console handler when SysLogger has already
            # attached one), leaving stderr byte-identical to stdout. Printing
            # both then doubles the failure report -- 60 lines becomes 120 of
            # the same thing. Show stderr only when it actually differs.
            # Measured per suite: test_precipitation_data_transfer had 301 of
            # 623 lines repeated before its own handler was fixed, against
            # 0.2% for test_weather_radar_live, so this is a per-suite hazard
            # rather than a property of the logging setup.
            streams = [("stdout", out)]
            if (err or "").strip() and (err or "").strip() != (out or "").strip():
                streams.append(("stderr", err))
            elif (err or "").strip():
                streams.append(("stderr", "(identical to stdout -- not repeated)"))
            for stream_name, text in streams:
                if not (text or "").strip():
                    continue
                shown = True
                lines = text.splitlines()
                clipped = len(lines) - TAIL_LINES
                print(f"     │ ── {stream_name} "
                      + (f"(last {TAIL_LINES} of {len(lines)} lines) "
                         if clipped > 0 else "")
                      + "─" * 10)
                for line in lines[-TAIL_LINES:]:
                    print("     │ " + line)
            if not shown:
                print("     │ (the suite produced no output)")
            print("     └" + "─" * 54)
    print("=" * 60)
    if failures:
        print(f"FAILED: {len(failures)}/{len(SUITES)} suites: "
              + ", ".join(n for n, _ in failures))
        return 1
    print(f"All {len(SUITES)} suites passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
