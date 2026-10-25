"""
Flight Control System 1553B wire format -- one encoding, in one place.

WHY THIS MODULE EXISTS (story C25.1, the FCS half)
--------------------------------------------------
The FCS had TWO data-word encodings in the tree, and they disagreed.

  (A) The live one, in FCSMessageHandler: _pack_control_input_data() /
      _pack_mode_data() on the way out, and the inline dicts inside
      _handle_control_input_request() / _handle_mode_change_request() on the
      way back in. This one round-trips, because both halves exist.

  (B) An orphan, in the predefined-message classes:
      fcs_controlSurfaceRequest.encode_1553b() and
      fcs_flightModeRequest.encode_1553b(). Package-wide these had ZERO
      callers, and nothing anywhere decoded them.

They are not two spellings of the same thing. Measured against (A)'s decoder:

  * (B) numbered the surfaces  aileron 1, elevator 2, rudder 3, flaps 4,
    spoilers 5, speedbrake 6, trim 7.  (A) numbers them  aileron 1,
    elevator 2, rudder 3, throttle 4, flaps 5, speedbrake 6, gear 7.
    So a "flaps" command encoded by (B) carries code 4, and (A)'s decoder
    reads code 4 as THROTTLE. A flap command would have moved the throttle.

  * (B) scaled position as int(position * 100). (A) scales as
    int((value + 1.0) * 500), i.e. 0..1000 with 500 as centre. (B)'s 0.3
    becomes 30, which (A) decodes as -0.94 -- near full negative deflection
    for a request for 30% positive.

  * (B)'s mode_map was  normal 1, approach 2, takeoff 3, landing 4, cruise 5,
    combat 6, emergency 7.  The FCS's own FlightControlModes are NORMAL,
    COMBAT, PRECISION, AUTOPILOT, TERRAIN, EMERGENCY -- there is no approach,
    takeoff, landing or cruise mode to command -- and (A) numbers them
    NORMAL 0 .. EMERGENCY 5. Every single value disagreed, and (B)'s
    "combat" (6) decodes under (A) as nothing at all.

(A) is therefore the authoritative layout: it is the one the FCS actually
decodes and acts on. This module is (A), lifted out of the handler so that
the predefined-message classes encode the same bytes the handler decodes,
and so the next person finds one definition instead of two.

encode_1553b() now delegates here as well, so (B) no longer exists.

WHAT IS AND IS NOT CARRIED
--------------------------
The control-input encoding carries exactly two words: the surface code and
the scaled value. It has no field for a rate, and the mode encoding has no
field for flaps/gear parameters. Callers that supply those are told so at
WARNING level rather than having them silently dropped -- see
control_input_data_words() and mode_data_words().
"""

from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()


# ---------------------------------------------------------------------------
# Control surfaces
# ---------------------------------------------------------------------------
# These seven codes are the wire contract: they are what
# _handle_control_input_request() decodes, and changing a number here changes
# which surface a command moves. Codes 5, 6 and 7 are encodable but NOT
# applicable -- FlightControlSystem.control_inputs holds only aileron,
# elevator, rudder and throttle, so set_control_input() answers "Invalid
# control type" and returns False for flaps, speedbrake and gear. That
# refusal is visible and correct; it is better than this layer pretending
# those surfaces do not exist.
CONTROL_SURFACE_TO_CODE = {
    "aileron": 1,
    "elevator": 2,
    "rudder": 3,
    "throttle": 4,
    "flaps": 5,
    "speedbrake": 6,
    "gear": 7,
}

CODE_TO_CONTROL_SURFACE = {code: name for name, code in CONTROL_SURFACE_TO_CODE.items()}

# The surfaces the FCS can actually apply (FlightControlSystem.control_inputs).
APPLICABLE_CONTROL_SURFACES = frozenset({"aileron", "elevator", "rudder", "throttle"})

# Value quantisation. A control value in [-1.0, 1.0] is carried as an integer
# in [0, 1000] with 500 as the zero point, so one count is 0.002 of full
# deflection.
CONTROL_VALUE_CENTRE = 500
CONTROL_VALUE_SPAN = 500           # counts per 1.0 of deflection
CONTROL_VALUE_MAX = 1000
CONTROL_VALUE_QUANTUM = 1.0 / CONTROL_VALUE_SPAN   # 0.002


def encode_control_value(value):
    """
    Scale a control value in [-1.0, 1.0] to the wire's [0, 1000].

    Rounds rather than truncates. The previous implementation used
    int((value + 1.0) * 500), which truncates toward zero and so biased
    EVERY encoded value downward by up to one quantum -- a one-sided error
    on a flight control input. Rounding is symmetric and costs nothing;
    the decoder is unchanged either way.
    """
    scaled = round((float(value) + 1.0) * CONTROL_VALUE_SPAN)
    return max(0, min(CONTROL_VALUE_MAX, scaled))


def decode_control_value(scaled):
    """Convert a wire value in [0, 1000] back to [-1.0, 1.0]."""
    value = (int(scaled) / float(CONTROL_VALUE_SPAN)) - 1.0
    return max(-1.0, min(1.0, value))


def control_input_data_words(surface_name, value, rate=None):
    """
    Build the data words for an FCS control input.

    Args:
        surface_name: one of CONTROL_SURFACE_TO_CODE's keys (case-insensitive)
        value: control value in [-1.0, 1.0]
        rate: optional, and NOT CARRIED -- see the module docstring. Passing
              one logs a warning so the caller learns the rate will not
              reach the FCS, instead of the request looking complete.

    Returns:
        list[str]: two 16-bit binary strings.

    Raises:
        ValueError: if surface_name is not an encodable surface. This is
            deliberate. The earlier packer fell back to code 0 for an
            unknown name, and code 0 is not in CODE_TO_CONTROL_SURFACE, so
            the receiving end read "no control surface specified" and
            answered ERROR -- after the command had been accepted, sent and
            given a request ID. Refusing at encode time means the caller
            finds out before anything goes on the bus.
    """
    if surface_name is None:
        raise ValueError("control input requires a surface name, got None")

    key = str(surface_name).lower()
    code = CONTROL_SURFACE_TO_CODE.get(key)
    if code is None:
        raise ValueError(
            f"'{surface_name}' is not an FCS control surface; the encodable "
            f"surfaces are {sorted(CONTROL_SURFACE_TO_CODE)}"
        )

    if key not in APPLICABLE_CONTROL_SURFACES:
        logger.warning(
            f"[FCS_WIRE] '{key}' encodes to code {code}, but the FCS holds "
            f"only {sorted(APPLICABLE_CONTROL_SURFACES)}, so it will refuse "
            f"the command on arrival."
        )

    if rate is not None:
        logger.warning(
            f"[FCS_WIRE] A rate of {rate} was given for '{key}', but the FCS "
            f"control-input encoding has no rate field. The rate is NOT being "
            f"transmitted and will not reach the FCS."
        )

    return [
        format(code, '016b'),
        format(encode_control_value(value), '016b'),
    ]


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------
# The numeric values are the ones used by FCSMessageHandler's packer, by
# FMSMessageHandler, and by the mode-change decoder. The names are
# FlightControlModes constants, which is what FlightControlSystem.set_mode()
# validates against -- the wire works in numbers, the system works in names.
MODE_NAME_TO_VALUE = {
    "NORMAL": 0,
    "COMBAT": 1,
    "PRECISION": 2,
    "AUTOPILOT": 3,
    "TERRAIN": 4,
    "EMERGENCY": 5,
    "STANDBY": 0,      # no FCS standby mode; treated as NORMAL
}

# Reverse direction. STANDBY deliberately does not appear: 0 decodes to
# NORMAL, which is the mode the FCS is actually placed in.
MODE_VALUE_TO_NAME = {
    0: "NORMAL",
    1: "COMBAT",
    2: "PRECISION",
    3: "AUTOPILOT",
    4: "TERRAIN",
    5: "EMERGENCY",
}


def mode_data_words(mode_name, extra_params=None):
    """
    Build the data words for an FCS mode change.

    Args:
        mode_name: an FCS mode name (case-insensitive), or an enum-like
            object carrying .name / .value.
        extra_params: optional dict of additional mode parameters, which the
            encoding has no room for. Passing a non-empty one logs a warning
            naming the keys that will not be transmitted.

    Returns:
        list[str]: one 16-bit binary string.

    Raises:
        ValueError: if the mode is not an FCS mode. The earlier packer
            defaulted an unrecognised name to 0, which is NORMAL -- so a
            request for a mode the FCS does not have would have commanded it
            into NORMAL and reported success. Commanding a different mode
            than the one asked for is worse than refusing.
    """
    if mode_name is None:
        raise ValueError("mode change requires a mode name, got None")

    # Enum-like payloads carry the number directly.
    value = getattr(mode_name, 'value', None)
    if isinstance(value, int):
        if value not in MODE_VALUE_TO_NAME:
            raise ValueError(
                f"mode value {value} is not an FCS mode; the FCS modes are "
                f"{MODE_VALUE_TO_NAME}"
            )
        resolved = value
    else:
        key = str(getattr(mode_name, 'name', mode_name)).upper()
        if key not in MODE_NAME_TO_VALUE:
            raise ValueError(
                f"'{mode_name}' is not an FCS mode; the FCS modes are "
                f"{sorted(n for n in MODE_NAME_TO_VALUE if n != 'STANDBY')}"
            )
        resolved = MODE_NAME_TO_VALUE[key]

    if extra_params:
        carried = sorted(k for k in extra_params if k != 'mode')
        if carried:
            logger.warning(
                f"[FCS_WIRE] The FCS mode encoding carries the mode value "
                f"only; {carried} are NOT being transmitted and will not "
                f"reach the FCS."
            )

    return [format(resolved, '016b')]


__all__ = [
    'CONTROL_SURFACE_TO_CODE',
    'CODE_TO_CONTROL_SURFACE',
    'APPLICABLE_CONTROL_SURFACES',
    'CONTROL_VALUE_CENTRE',
    'CONTROL_VALUE_SPAN',
    'CONTROL_VALUE_MAX',
    'CONTROL_VALUE_QUANTUM',
    'encode_control_value',
    'decode_control_value',
    'control_input_data_words',
    'MODE_NAME_TO_VALUE',
    'MODE_VALUE_TO_NAME',
    'mode_data_words',
]
