"""Test suite — the predefined message API against a live system (C14.6).

Replaces predefined_messages_test.py (1,090 lines, 719 of live code, 7 test
methods, 13 `_verify_log_patterns` calls -- the densest log-scraping in the tree).

`Interfaces/predefinedMessages/Messages.py` is the public front door: the facade a
console or another system calls to command a radar, change a flight mode, move a
control surface or ask for data. The original suite verified that calls through it
produced certain log phrases. This one verifies that they produce the effect they
name and return what they document.

WHAT THE FACADE PROMISES, AND WHERE IT DOES NOT DELIVER
-------------------------------------------------------
Every method is annotated `-> str` and documents "Returns: the request ID". Measured
against a live system, that holds for the radar families and does not hold for the
FMS and FCS families, which return None -- so a caller has nothing to correlate a
response against for any flight-mode, control-surface, attitude or navigation
message. Asserted below as a labelled defect against story C25.1; when it is fixed
those assertions must fail.

Standalone-safe: run from B20SS/ as
`python -m FMOFP.Tests.test_predefined_messages_live`.
"""
import asyncio
import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _B20SS not in sys.path:
    sys.path.insert(0, _B20SS)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from FMOFP.Tests.live_system import run_against_live_system, await_condition  # noqa: E402

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


def is_request_id(value):
    return isinstance(value, str) and len(value) == 36 and value.count('-') == 4


async def body(sm):
    """Assert the predefined message facade against an already-running system."""
    global PASS, FAIL
    PASS = FAIL = 0

    from FMOFP.Interfaces.predefinedMessages.Messages import get_messages
    from FMOFP.Systems.radarManagement.radar_enums import (
        aewc_radarMode,
        sar_radarMode,
        targeting_radarMode,
        tfr_radarMode,
    )

    messages = get_messages()
    rm = sm.components.get('radar_management')
    check("radar management is available to observe the effects", rm is not None)
    if rm is None:
        return 1

    # ── initialisation ───────────────────────────────────────────────────
    print("\ninitialisation brings up every message subsystem")

    SUBSYSTEMS = ('weather_radar', 'tfr_radar', 'sar_radar', 'targeting_radar', 'fms', 'fcs')

    before = await messages.is_initialized()
    if not before:
        check("is_initialized() is False before initialize()", before is False, repr(before))
        check("every subsystem is None before initialize()",
              all(getattr(messages, s) is None for s in SUBSYSTEMS),
              str({s: type(getattr(messages, s)).__name__ for s in SUBSYSTEMS}))

    await asyncio.wait_for(messages.initialize(), timeout=60.0)
    check("is_initialized() is True after initialize()",
          await messages.is_initialized() is True)
    for s in SUBSYSTEMS:
        check(f"the {s} message subsystem is constructed",
              getattr(messages, s) is not None, f"{s} is None")

    await asyncio.wait_for(messages.initialize(), timeout=60.0)
    check("initialize() a second time is safe and leaves it initialised",
          await messages.is_initialized() is True)

    # ── radar mode commands: three input forms, one effect ───────────────
    print("\nradar mode commands accept str, int and enum -- and actually land")

    RADAR_CASES = (
        ('targeting_radar', messages.set_targeting_radar_mode, targeting_radarMode.LOCK),
        ('tfr_radar', messages.set_tfr_radar_mode, tfr_radarMode.OBSTACLE_AVOIDANCE),
        ('aewc_radar', messages.set_aewc_radar_mode, aewc_radarMode.SECTOR_SCAN),
        ('sar_radar', messages.set_sar_radar_mode, sar_radarMode.SPOTLIGHT),
    )

    issued = []
    for radar_name, facade, target in RADAR_CASES:
        radar = rm.radars[radar_name]
        enum = type(radar.mode)
        for label, argument in (('name', target.name), ('value', target.value), ('enum', target)):
            # Start somewhere else, so "it landed" cannot be true beforehand.
            radar.set_mode(enum.STANDBY, send_completion=False)
            check(f"{radar_name} starts from STANDBY for the {label} case",
                  radar.mode is enum.STANDBY, radar.mode.name)

            rid = await asyncio.wait_for(facade(argument), timeout=30.0)
            issued.append(rid)
            check(f"{radar_name} mode by {label} returns a request ID", is_request_id(rid), repr(rid))

            landed = await await_condition(
                lambda r=radar, t=target: r.mode.value == t.value, timeout=10.0)
            check(f"...and the radar reaches {target.name}", landed, radar.mode.name)

        radar.set_mode(enum.STANDBY, send_completion=False)

    # The weather radar is deliberately not checked for a landed mode here: its
    # mode is owned by RadarManagementSystem's mission-phase policy, which
    # reasserts a commanded mode every tick. That relationship is asserted in
    # test_weather_radar_live.py; what matters here is the facade's own contract.
    rid = await asyncio.wait_for(messages.set_weather_radar_mode('MAPPING'), timeout=30.0)
    issued.append(rid)
    check("the weather radar mode command returns a request ID too", is_request_id(rid), repr(rid))

    # ── rejection is an exception, with a message that names the input ───
    print("\nan invalid radar mode is refused loudly")

    for bogus in ('NOT_A_MODE', None, 9999):
        try:
            await asyncio.wait_for(messages.set_targeting_radar_mode(bogus), timeout=30.0)
            check(f"set_targeting_radar_mode({bogus!r}) raises", False, "no exception")
        except ValueError as exc:
            check(f"set_targeting_radar_mode({bogus!r}) raises ValueError", True)
            check("...and the message names the offending value",
                  str(bogus) in str(exc), str(exc))
        except Exception as exc:  # noqa: BLE001 - anything else is the wrong error
            check(f"set_targeting_radar_mode({bogus!r}) raises ValueError, not "
                  f"{type(exc).__name__}", False, str(exc))

    # ── radar data requests ──────────────────────────────────────────────
    print("\nradar data requests return correlatable IDs")

    DATA_REQUESTS = (
        ('precipitation', messages.request_precipitation_data()),
        ('VIL', messages.request_vil_data()),
        ('imagery', messages.request_imagery_data()),
        ('elevation', messages.request_elevation_data()),
        ('sector scan', messages.request_sector_scan()),
        ('track data', messages.request_track_data()),
    )
    for label, coro in DATA_REQUESTS:
        rid = await asyncio.wait_for(coro, timeout=30.0)
        issued.append(rid)
        check(f"request for {label} returns a request ID", is_request_id(rid), repr(rid))

    # ── C25.1, the FMS half: these now send and correlate ────────────────
    #
    # All four families used to return None, and the reason was not a missing
    # return statement -- RadarMessageHandler.send_request's data branch
    # enumerated the five radars by hand and refused everything else BEFORE
    # sending, so the command never reached the bus. Measured on a live
    # system: four "Unsupported radar type for data request" errors per run.
    # It consults DATA_REQUEST_MAP now, and the FMS entries were already
    # registered there by command_word_map_fms.update_command_maps().
    print("\nC25.1 — the FMS families send, and return an ID that correlates")

    FMS_FIXED = (
        ('update_attitude', messages.update_attitude({"pitch": 1.0, "roll": 2.0, "yaw": 3.0})),
        ('request_navigation_update', messages.request_navigation_update(latitude=1.0, longitude=2.0)),
    )
    for label, coro in FMS_FIXED:
        rid = await asyncio.wait_for(coro, timeout=30.0)
        issued.append(rid)
        check(f"{label}() returns a request ID", is_request_id(rid), repr(rid))

    # ── C25.1, the FCS half: these now send and carry a payload ──────────
    #
    # Both used to return None, because registering the FCS's data command
    # word made the message travel EMPTY -- fcs_controlSurfaceRequest and
    # fcs_flightModeRequest had no to_data_words(), and _pack_data() handles
    # radars only, so the send fell through to the default format. Measured
    # then: Data words: ['0000000000000000'].
    #
    # They have one now (FMOFP/local_messaging/fcs_wire_format.py), and it is
    # the encoding the FCS's own handler decodes rather than the orphan
    # layout that used to sit in encode_1553b(). The mode change is also
    # routed as "mode_change" rather than "data", so it resolves to
    # FCS_MODE_CHANGE_REQUEST -- the command word the FCS's mode handler is
    # registered against.
    #
    # See FMOFP/Tests/test_fcs_wire_format.py for the wire-level assertions;
    # what this suite adds is that the live facade returns a correlatable ID.
    print("\nC25.1 — the FCS families send, and return an ID that correlates")

    FCS_FIXED = (
        ('request_flight_mode_change', messages.request_flight_mode_change("COMBAT")),
        ('request_control_surface_change', messages.request_control_surface_change("aileron", 0.3)),
    )
    for label, coro in FCS_FIXED:
        rid = await asyncio.wait_for(coro, timeout=30.0)
        issued.append(rid)
        check(f"{label}() returns a request ID", is_request_id(rid), repr(rid))

    # And a mode the FCS does not have is refused rather than being turned
    # into a command to enter NORMAL, which is what the old packer default
    # did. No request ID exists for it, so nothing goes in `issued`.
    refused = await asyncio.wait_for(
        messages.request_flight_mode_change("CRUISE"), timeout=30.0)
    check("a mode the FCS does not have is refused, not silently remapped",
          refused is None, repr(refused))

    # ── non-tautological guards ──────────────────────────────────────────
    print("\nNON-TAUTOLOGICAL — these must not pass by accident")

    check(f"every request ID issued was distinct ({len(issued)} of them)",
          len(set(issued)) == len(issued), f"{len(set(issued))} unique of {len(issued)}")
    check("...and none of them was None or empty",
          all(is_request_id(r) for r in issued), str([r for r in issued if not is_request_id(r)]))

    check("the is_request_id() guard rejects things that are not request IDs",
          not any(is_request_id(x) for x in (None, '', 'abc', 42, 'x' * 36)))

    probe = rm.radars['targeting_radar']
    probe.set_mode(targeting_radarMode.STANDBY, send_completion=False)
    never = await await_condition(
        lambda: probe.mode is targeting_radarMode.LOCK, timeout=2.0)
    check("polling for a mode the radar was NOT commanded into fails",
          not never, probe.mode.name)

    print(f"\nPredefined messages (live): {PASS} passed, {FAIL} failed")
    return FAIL


if __name__ == '__main__':
    sys.exit(run_against_live_system(body, body_timeout=400))
