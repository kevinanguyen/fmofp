"""Test suite — targeting, TFR, AEWC and SAR radars against a live system (C14.3).

Replaces four log-scraping suites (radar_tests/{targeting,tfr,aewc,sar}_radar_test.py,
1,416 lines of live code) with one parameterised suite of state assertions.

WHY ONE SUITE AND NOT FOUR
--------------------------
The four originals were structural clones: identical method structure, identical
helpers, only the radar name and enum differed. They also shared the same weakness.
Two of the four regexes each applied per mode cannot fail:

    (f"Sending mode change completion notification|{mode.name}", "Display mode update")
    (f"Using request ID|request_id",                             "Display processing")

The first is satisfied by the bare mode name, which the test itself logs; the second
by the literal substring `request_id`, which appears in the application's own debug
formatting. At 15-17 modes per radar that is roughly 95 verification points reporting
success unconditionally.

THE CONTRACT THESE FOUR ACTUALLY HAVE
-------------------------------------
It is the opposite of the weather radar's, and neither suite said so.

`RadarManagementSystem._get_expected_mode()` implements a mission-phase policy for the
weather radar only (TAKEOFF/APPROACH -> SURVEILLANCE, else MAPPING) and returns
`radar.mode` -- the radar's own current mode -- for these four. So the supervisor is a
deliberate no-op here: a mode set on one of these radars PERSISTS, where the same
action on the weather radar is reverted within a tick.

That asymmetry is the thing most likely to surprise someone, so it is asserted
directly rather than left to be rediscovered. If a mission-phase policy is added for
any of these radars later, these assertions fail and say exactly what changed.

FOREIGN-ENUM COERCION -- FIXED BY C23.1, NOW ASSERTED
-----------------------------------------------------
Three of the five radars (weather, TFR, SAR) used to silently reinterpret an enum
member from a DIFFERENT enum class by its numeric value:

    elif hasattr(mode, '_value_'):
        mode = tfr_radarMode(mode._value_)

Every enum member has `_value_`, so `RadarMode.SEARCH` (value 3) became
`tfr_radarMode(3)` -- which is TEST. A command meaning "search" put the radar into
built-in test. Measured while fixing it: value 3 is TEST in ALL FIVE radar enums, so
the collision was universal and only the three radars with that branch were exposed.

All five now refuse a foreign enum, report the refusal through set_mode's return
value, and leave the mode untouched. All five also still accept a plain int, which
is a legitimate shape off the 1553B path. Those are the assertions below.

Standalone-safe: run from B20SS/ as
`python -m FMOFP.Tests.test_radar_modes_live`.
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


def mode_name(radar):
    m = getattr(radar, 'mode', None)
    return getattr(m, 'name', str(m))


async def body(sm):
    """Assert the four non-weather radars against an already-running system."""
    global PASS, FAIL
    PASS = FAIL = 0

    from FMOFP.Systems.radarManagement.radarControl import MissionPhase
    from FMOFP.Systems.radarManagement.radar_enums import (
        RadarMode,
        aewc_radarMode,
        sar_radarMode,
        targeting_radarMode,
        tfr_radarMode,
    )

    RADARS = (
        ('targeting_radar', targeting_radarMode),
        ('tfr_radar', tfr_radarMode),
        ('aewc_radar', aewc_radarMode),
        ('sar_radar', sar_radarMode),
    )

    # _configure_enroute_surveillance() is the one phase configuration that
    # names all four of these radars explicitly and uses each radar's OWN enum.
    # TAKEOFF and APPROACH do not -- see the STAND-DOWN block at the end.
    CRUISE_MODE = {
        'targeting_radar': targeting_radarMode.SEARCH,
        'tfr_radar': tfr_radarMode.TERRAIN_FOLLOWING,
        'aewc_radar': aewc_radarMode.SEARCH,
        'sar_radar': sar_radarMode.STRIPMAP,
    }

    rm = sm.components.get('radar_management')
    rh = sm.components.get('radar_message_handler')
    check("radar management is a registered component", rm is not None)
    check("the radar message handler is a registered component", rh is not None)
    if rm is None or rh is None:
        return 1

    modes_exercised = 0

    for name, enum in RADARS:
        print(f"\n{name}")
        radar = rm.radars.get(name)
        check(f"{name} is registered", radar is not None, sorted(rm.radars))
        if radar is None:
            continue

        check(f"{name} reports healthy", radar.is_healthy())
        check(f"{name}.mode is a {enum.__name__}, not a generic RadarMode",
              isinstance(radar.mode, enum), type(radar.mode).__name__)

        # ── every mode the enum defines can actually be entered ──────────
        # INITIALIZING (-1) is documented as a pre-operational state, not a
        # commandable one, so it is excluded deliberately rather than silently.
        settable = [m for m in enum if m.value >= 0]
        failed_modes = []
        for mode in settable:
            radar.set_mode(mode, send_completion=False)
            if radar.mode is not mode:
                failed_modes.append(f"{mode.name}->{mode_name(radar)}")
            modes_exercised += 1
        check(f"all {len(settable)} commandable modes take effect immediately",
              not failed_modes, str(failed_modes[:4]))

        # ── aliases resolve to their canonical member ────────────────────
        alias = getattr(enum, 'SEARCH', None)
        if alias is not None:
            radar.set_mode(alias, send_completion=False)
            check(f"the SEARCH alias resolves to {alias.name} (value {alias.value})",
                  radar.mode is alias, mode_name(radar))

        # ── two supervisors, and only one of them is continuous ─────────
        # Conflating these is how this contract gets misread:
        #
        #   _get_expected_mode()         reasserted every tick of the update
        #                                loop -- weather radar only; returns the
        #                                radar's own current mode for these four
        #   _allocate_radar_resources()  one-shot on a mission PHASE change --
        #                                sets a mode on every radar
        #
        # So a mode set on one of these four persists indefinitely, but a phase
        # change overwrites it once. The weather radar is subject to both, which
        # is why a mode set on it appears to "bounce back" and a mode set on
        # these does not.
        expected = rm._get_expected_mode(radar)
        check("the continuous supervisor leaves this radar alone "
              "(_get_expected_mode returns its own current mode)",
              expected is radar.mode, f"expected={getattr(expected, 'name', expected)}")

        held = settable[len(settable) // 2]
        radar.set_mode(held, send_completion=False)
        drifted = await await_condition(lambda r=radar, h=held: r.mode is not h, timeout=6.0)
        check("a commanded mode PERSISTS -- not reverted tick by tick as the "
              "weather radar's is",
              not drifted, f"{held.name} -> {mode_name(radar)}")

        # ── a phase change DOES drive it, per a fixed policy table ──────
        original_phase = rm.mission_phase
        try:
            rm.update_mission_phase(MissionPhase.CRUISE)
            target = CRUISE_MODE[name]
            reached = await await_condition(
                lambda r=radar, m=target: r.mode is m, timeout=10.0)
            check(f"CRUISE drives it to {target.name}", reached,
                  f"mode is {mode_name(radar)}")
            check("...and the mode it lands in is still its own enum type",
                  isinstance(radar.mode, enum), type(radar.mode).__name__)
        finally:
            rm.update_mission_phase(original_phase)

        # ── a foreign enum is refused, reported, and changes nothing ──────
        # C23.1. RadarMode.SEARCH has value 3, and 3 is TEST in every one of
        # the five radar enums, so a radar that coerces by value puts itself
        # into built-in test on a command meaning "search".
        before_foreign = radar.mode
        try:
            accepted = radar.set_mode(RadarMode.SEARCH, send_completion=False)
            raised = None
        except Exception as exc:  # noqa: BLE001 - the point is that this cannot happen
            accepted, raised = None, f"{type(exc).__name__}: {exc}"
        check("set_mode with a foreign enum does not raise out of the radar",
              raised is None, str(raised))
        check("...it reports the refusal rather than returning nothing",
              accepted is False, repr(accepted))
        check("...the mode is left exactly as it was",
              radar.mode is before_foreign,
              f"{mode_name(radar)} (was {before_foreign.name})")
        check("...and in particular it is not coerced into TEST",
              radar.mode.name != 'TEST' or before_foreign.name == 'TEST',
              mode_name(radar))
        check("...and the stored mode is still this radar's own enum type",
              isinstance(radar.mode, enum), type(radar.mode).__name__)

        # An int is a legitimate shape off the 1553B path, on every radar.
        before_int = radar.mode
        int_target = next(m for m in enum if m is not before_int)
        accepted_int = radar.set_mode(int_target.value, send_completion=False)
        check("a mode delivered as a plain int is accepted",
              accepted_int is True, repr(accepted_int))
        check("...and selects the member with that value",
              radar.mode is int_target, mode_name(radar))

        # ── status surface ───────────────────────────────────────────────
        status = radar.get_status()
        check("get_status() returns a dict reporting the mode",
              isinstance(status, dict) and any('mode' in str(k).lower() for k in status),
              str(type(status).__name__))

        # ── request dispatch correlates by ID ────────────────────────────
        for kind, payload in (("status", None), ("data", {"request_type": "scan"})):
            rid = await asyncio.wait_for(
                rh.send_request(name, kind, payload), timeout=30.0)
            check(f"a {kind} request returns a request ID",
                  isinstance(rid, str) and len(rid) == 36, repr(rid))
            check(f"the handler registers the {kind} request under that exact ID",
                  isinstance(rid, str) and rid in rh.pending_requests)

    # ── the approach stand-down reaches every radar (story C23.1) ────────
    #
    # Both airport configurations used to handle weather and TFR by name and
    # then fall through to
    #     else: radar.set_mode(RadarMode.STANDBY, send_completion=False)
    # with the GENERIC enum. Targeting and AEWC validate strictly and refused
    # it, so they kept whatever mode they had -- on approach, a targeting
    # radar left searching from cruise stayed searching. SAR appeared to work
    # only by coincidence: RadarMode.STANDBY is 0 and sar_radarMode(0) is also
    # STANDBY.
    #
    # These assertions used to pin that defect, and failed exactly as intended
    # when the fix landed:
    #     FAIL  DEFECT: APPROACH does NOT stand targeting_radar down  mode is STANDBY
    #     FAIL  DEFECT: APPROACH does NOT stand aewc_radar down       mode is STANDBY
    print("\nC23.1 — the approach stand-down reaches every radar")

    rm.update_mission_phase(MissionPhase.CRUISE)
    for rname, target in CRUISE_MODE.items():
        await await_condition(lambda r=rm.radars[rname], m=target: r.mode is m, timeout=10.0)

    rm.update_mission_phase(MissionPhase.APPROACH)
    await asyncio.sleep(1.0)

    tfr = rm.radars['tfr_radar']
    check("APPROACH stands TFR down correctly (its own enum is used)",
          tfr.mode is tfr_radarMode.TERRAIN_FOLLOWING, mode_name(tfr))

    for rname, enum_cls in (('sar_radar', sar_radarMode),
                            ('targeting_radar', targeting_radarMode),
                            ('aewc_radar', aewc_radarMode)):
        r = rm.radars[rname]
        check(f"APPROACH stands {rname} down, in its OWN enum",
              r.mode is enum_cls.STANDBY, f"mode is {mode_name(r)}")
        check(f"...and {rname} did not simply hold its cruise mode",
              r.mode is not CRUISE_MODE[rname], f"mode is {mode_name(r)}")

    # AC3: the same for TAKEOFF, which shares the defect and the fix and was
    # never covered here.
    print("\nC23.1 — and the takeoff stand-down reaches every radar")

    rm.update_mission_phase(MissionPhase.CRUISE)
    for rname, target in CRUISE_MODE.items():
        await await_condition(lambda r=rm.radars[rname], m=target: r.mode is m,
                              timeout=10.0)

    rm.update_mission_phase(MissionPhase.TAKEOFF)
    await asyncio.sleep(1.0)

    check("TAKEOFF leaves TFR in TERRAIN_FOLLOWING",
          rm.radars['tfr_radar'].mode is tfr_radarMode.TERRAIN_FOLLOWING,
          mode_name(rm.radars['tfr_radar']))
    for rname, enum_cls in (('sar_radar', sar_radarMode),
                            ('targeting_radar', targeting_radarMode),
                            ('aewc_radar', aewc_radarMode)):
        r = rm.radars[rname]
        check(f"TAKEOFF stands {rname} down, in its OWN enum",
              r.mode is enum_cls.STANDBY, f"mode is {mode_name(r)}")

    # ── non-tautological guards ──────────────────────────────────────────
    print("\nNON-TAUTOLOGICAL — these must not pass by accident")

    check(f"the sweep actually exercised modes ({modes_exercised}), not an empty enum",
          modes_exercised >= 50, str(modes_exercised))

    probe = rm.radars['targeting_radar']
    wrong = next(m for m in targeting_radarMode if m is not probe.mode and m.value >= 0)
    detected = await await_condition(lambda: probe.mode is wrong, timeout=2.0)
    check("polling for a mode the radar is NOT in fails (wrong state is detectable)",
          not detected, f"mode={mode_name(probe)} probed={wrong.name}")

    import uuid as _uuid
    check("an ID that was never issued is absent from the pending table",
          str(_uuid.uuid4()) not in rh.pending_requests)

    check("the four radars are distinct objects, not one aliased four ways",
          len({id(rm.radars[n]) for n, _ in RADARS}) == 4)

    print(f"\nRadar modes (live): {PASS} passed, {FAIL} failed")
    return FAIL


if __name__ == '__main__':
    sys.exit(run_against_live_system(body, body_timeout=400))
