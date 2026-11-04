"""Test suite — flight control system against a live system (C14.5).

Replaces flight_control_system_test.py (1,049 lines, 773 of live code).

The original verified by `re.search` over captured log output, so it could observe
that a phrase like "[FCS] Mode changed from" appeared, but not what the mode became,
whether an out-of-range control input was clamped or refused, or whether either
method's documented boolean return was correct. Both `set_mode()` and
`set_control_input()` return a real bool that the original never checked.

THE TWO CONTRACTS ASSERTED HERE

1. Mode. Six named modes, a True/False return, an unchanged mode on refusal, and
   -- documented and easy to get wrong -- setting the mode the system is ALREADY in
   returns True rather than False.

2. Control inputs are SATURATED, not rejected. Out-of-range values are clamped to
   the control's range and the call still reports success:

       throttle          clamped to [0.0, 1.0]
       everything else   clamped to [-1.0, 1.0]

   So `set_control_input('aileron', 5.0)` returns True and leaves the aileron at
   1.0. A caller checking only the return value cannot tell that its command was
   modified, which is a design worth pinning down rather than rediscovering.

Standalone-safe: run from B20SS/ as `python -m FMOFP.Tests.test_flight_control_live`.
"""
import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _B20SS not in sys.path:
    sys.path.insert(0, _B20SS)

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from FMOFP.Tests.live_system import run_against_live_system  # noqa: E402

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


async def body(sm):
    """Assert flight control behaviour against an already-running system."""
    global PASS, FAIL
    PASS = FAIL = 0

    from FMOFP.Systems.flightManagementSys.flightControlSys.flight_control_system import (
        FlightControlModes,
    )
    from FMOFP.Systems.flightManagementSys.flightManagementSystem import (
        get_flightManagementSystem,
    )

    fms = get_flightManagementSystem()
    fcs = getattr(fms, 'flight_control_system', None)
    check("the FMS exposes a flight control system", fcs is not None)
    if fcs is None:
        return 1

    # ── the mode vocabulary is what the class says it is ─────────────────
    print("\nmode vocabulary")

    modes = FlightControlModes.get_all_modes()
    check("get_all_modes() lists six modes", len(modes) == 6, str(modes))
    for name in ('NORMAL', 'COMBAT', 'PRECISION', 'AUTOPILOT', 'TERRAIN', 'EMERGENCY'):
        check(f"...including {name}", getattr(FlightControlModes, name, None) in modes, name)
    check("every listed mode is a plain string",
          all(isinstance(m, str) for m in modes), str([type(m).__name__ for m in modes]))

    # ── mode changes are checked operations ──────────────────────────────
    print("\nset_mode reports what it did")

    original_mode = fcs.mode
    try:
        for mode in modes:
            accepted = fcs.set_mode(mode, send_completion=False)
            check(f"set_mode({mode!r}) returns True", accepted is True, repr(accepted))
            check(f"...and fcs.mode becomes {mode!r}", fcs.mode == mode, repr(fcs.mode))

        fcs.set_mode(FlightControlModes.COMBAT, send_completion=False)
        again = fcs.set_mode(FlightControlModes.COMBAT, send_completion=False)
        check("setting the mode it is already in returns True, not False "
              "(a no-op is success, not failure)",
              again is True, repr(again))
        check("...and leaves the mode alone", fcs.mode == FlightControlModes.COMBAT, repr(fcs.mode))

        # ── refusal ──────────────────────────────────────────────────────
        print("\nan invalid mode is refused, and the mode is left alone")

        before = fcs.mode
        for bogus in ("HYPERSONIC", "normal", "", None, 2):
            refused = fcs.set_mode(bogus, send_completion=False)
            check(f"set_mode({bogus!r}) returns False", refused is False, repr(refused))
        check("...and the mode survived every refusal", fcs.mode == before, repr(fcs.mode))
    finally:
        fcs.set_mode(original_mode, send_completion=False)

    # ── control inputs ───────────────────────────────────────────────────
    print("\ncontrol inputs are stored as commanded, within range")

    original_inputs = dict(fcs.control_inputs)
    try:
        for control in ('aileron', 'elevator', 'rudder'):
            ok = fcs.set_control_input(control, 0.25, send_completion=False)
            check(f"set_control_input({control!r}, 0.25) returns True", ok is True, repr(ok))
            check(f"...and {control} holds 0.25",
                  abs(fcs.control_inputs[control] - 0.25) < 1e-9, fcs.control_inputs[control])

        ok = fcs.set_control_input('throttle', 0.75, send_completion=False)
        check("set_control_input('throttle', 0.75) returns True", ok is True, repr(ok))
        check("...and throttle holds 0.75",
              abs(fcs.control_inputs['throttle'] - 0.75) < 1e-9, fcs.control_inputs['throttle'])

        # ── saturation, not rejection ────────────────────────────────────
        print("\nout-of-range inputs are CLAMPED, and still report success")

        for control, sent, expected in (('aileron', 5.0, 1.0),
                                        ('aileron', -5.0, -1.0),
                                        ('elevator', 99.0, 1.0),
                                        ('rudder', -2.5, -1.0),
                                        ('throttle', -1.0, 0.0),
                                        ('throttle', 5.0, 1.0)):
            ok = fcs.set_control_input(control, sent, send_completion=False)
            held = fcs.control_inputs[control]
            check(f"{control} commanded {sent} is clamped to {expected}",
                  abs(held - expected) < 1e-9, f"held {held}")
            check("...and the clamped call still returns True (saturation, not refusal)",
                  ok is True, repr(ok))

        # ── an unknown control is refused ────────────────────────────────
        print("\nan unknown control is refused")

        for bogus in ('spoiler', 'AILERON', '', None):
            refused = fcs.set_control_input(bogus, 0.5, send_completion=False)
            check(f"set_control_input({bogus!r}, ...) returns False", refused is False, repr(refused))
        check("...and no key was created for it",
              set(fcs.control_inputs) == {'aileron', 'elevator', 'rudder', 'throttle'},
              str(sorted(fcs.control_inputs)))
    finally:
        for control, value in original_inputs.items():
            fcs.set_control_input(control, value, send_completion=False)

    # ── the FMS drives this system, not the other way round ──────────────
    print("\ncoupling direction")

    fms_mode_before = fms.get_flight_data()['status']['mode']
    fcs.set_mode(FlightControlModes.TERRAIN, send_completion=False)
    check("setting the FCS mode directly does NOT change the FMS mode "
          "(the mapping runs one way, FMS -> FCS)",
          fms.get_flight_data()['status']['mode'] == fms_mode_before,
          f"{fms_mode_before} -> {fms.get_flight_data()['status']['mode']}")

    fms.set_mode("STEALTH")
    check("...while an FMS mode change does drive the FCS (STEALTH -> PRECISION)",
          fcs.mode == FlightControlModes.PRECISION, repr(fcs.mode))

    # ── non-tautological guards ──────────────────────────────────────────
    print("\nNON-TAUTOLOGICAL — these must not pass by accident")

    fcs.set_mode(FlightControlModes.AUTOPILOT, send_completion=False)
    check("asserting the WRONG mode fails (mode comparison can detect a mismatch)",
          fcs.mode != FlightControlModes.NORMAL, repr(fcs.mode))

    fcs.set_control_input('aileron', 0.5, send_completion=False)
    check("the clamp assertions above used values that a MISSING clamp would fail "
          "(0.5 is stored verbatim, so clamping is not simply flooring everything)",
          abs(fcs.control_inputs['aileron'] - 0.5) < 1e-9, fcs.control_inputs['aileron'])

    check("an unknown control is absent from control_inputs, not silently defaulted",
          fcs.control_inputs.get('spoiler') is None)

    fms.set_mode("NORMAL")
    for control, value in original_inputs.items():
        fcs.set_control_input(control, value, send_completion=False)

    print(f"\nFlight control (live): {PASS} passed, {FAIL} failed")
    return FAIL


if __name__ == '__main__':
    sys.exit(run_against_live_system(body, body_timeout=300))
