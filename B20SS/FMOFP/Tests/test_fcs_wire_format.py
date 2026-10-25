"""
Test suite: the FCS's 1553B command words and data words (story C25.1).

Three findings, one story.

1. THE FCS SPOKE AT SOMEBODY ELSE'S ADDRESS. Its eight command words were
   invented from a private base, FCS_BASE_COMMAND = 0x4000, plus 0x01..0x08.
   A command word in this codebase is not a counter: it is RT address (5) |
   T/R (1) | subaddress (5) | word count (5), and BusControllerModule's
   formatCommand() slices exactly those fields out of whatever it is handed.
   0x4007 -- FCS_MODE_CHANGE_REQUEST -- is 0100000000000111, which is RT
   address 8, subaddress 0. RT 8 is Power Management, and the RT message
   extractor's own address map agrees (8 -> power_management). So FCS mode
   changes and status requests, the two FCS request types that did reach the
   bus, were addressed to Power Management.

   The real address was in the tree: rtAddressConfig.xml has RT 5 for
   "flightcontrolsys" and subaddress 10 for "flight_control". It was
   unreachable because address_book.xml had left the FCS system entry under
   the duplicate id "unnamed_system" -- the same id as the address-0 entry --
   so get_rt_address() could not resolve the FCS at all.

2. THE FCS HAD TWO DATA-WORD ENCODINGS, AND THEY DISAGREED. The live one is
   FCSMessageHandler's packers plus the matching decoders in
   _handle_control_input_request / _handle_mode_change_request. The other was
   encode_1553b() on fcs_controlSurfaceRequest and fcs_flightModeRequest:
   zero callers package-wide, and nothing decoded it. Measured against the
   live decoder, the orphan's "flaps" (code 4) reads as THROTTLE, its
   position*100 scaling turns a request for +0.3 into -0.94, and its mode
   numbering (normal 1 .. emergency 7, over names the FCS does not have)
   disagrees with NORMAL 0 .. EMERGENCY 5 on every value.

   The encoding now lives once, in FMOFP/local_messaging/fcs_wire_format.py.
   Five sites held literal copies of these tables; all five now import them.

3. UNRECOGNISED INPUTS WERE ENCODED AS SOMETHING ELSE. An unknown mode name
   mapped to 0, which is NORMAL -- so a command for a mode the FCS does not
   have was transmitted as a command to enter NORMAL. An unknown control
   surface mapped to code 0, which is not a surface, so the FCS answered "no
   control surface specified" AFTER the request had been accepted, sent and
   given an ID. Both are refused at encode time now.

Run:  python3 -m FMOFP.Tests.test_fcs_wire_format
"""

import os
import sys

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from FMOFP.Tests import code_only  # shared source-inspection helper

import ast
import asyncio
import glob
import inspect
import logging
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

_FMOFP = os.path.join(_B20SS, 'FMOFP')


def _src(relpath):
    with open(os.path.join(_FMOFP, relpath), encoding='utf-8') as fh:
        return fh.read()


def _fields(word):
    """Split a 16-bit command word the way BusControllerModule does."""
    return {
        'rt': int(word[:5], 2),
        'tr': int(word[5], 2),
        'sa': int(word[6:11], 2),
        'wc': int(word[11:], 2),
    }


class _Capture(logging.Handler):
    """Collect log records emitted while the block is active."""

    def __init__(self):
        super().__init__(level=logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)

    def __enter__(self):
        logging.getLogger().addHandler(self)
        return self

    def __exit__(self, *exc):
        logging.getLogger().removeHandler(self)
        return False

    def text(self):
        return "\n".join(r.getMessage() for r in self.records)


# ---------------------------------------------------------------------------
# 1. command word geometry
# ---------------------------------------------------------------------------

def test_command_words_carry_the_fcs_address():
    R.section("the FCS's command words address the FCS")

    from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS
    from FMOFP.local_messaging.address_utils import get_rt_address, get_subaddress

    R.check("all eight FCS command words exist", len(FCS_COMMAND_WORDS) == 8,
            str(sorted(FCS_COMMAND_WORDS)))

    expected_rt = get_rt_address('flightcontrolsys')
    expected_sa = get_subaddress('flight_control')
    R.check("the address book resolves the FCS to RT 5", expected_rt == 5,
            str(expected_rt))
    R.check("...and the flight_control subaddress to 10", expected_sa == 10,
            str(expected_sa))

    for name, cmd in sorted(FCS_COMMAND_WORDS.items()):
        word = format(cmd.value, '016b')
        f = _fields(word)
        R.check(f"{name} is addressed to RT {expected_rt}, subaddress {expected_sa}",
                f['rt'] == expected_rt and f['sa'] == expected_sa,
                f"{word} -> rt={f['rt']} sa={f['sa']}")
        expect_tr = 1 if name.endswith('RESPONSE') else 0
        R.check(f"...and its T/R bit is {expect_tr}", f['tr'] == expect_tr,
                f"tr={f['tr']}")

    values = [cmd.value for cmd in FCS_COMMAND_WORDS.values()]
    R.check("the eight words are distinct", len(set(values)) == 8,
            f"{len(set(values))} distinct")


def test_pre_fix_words_addressed_power_management():
    R.section("NON-TAUTOLOGICAL — what the invented words actually decoded to")

    # The pre-fix expression, reproduced inline. If this suite ever stops
    # being able to state the defect, the defect is what changed.
    pre_fix_base = 0x4000
    pre_fix_mode = pre_fix_base + 0x07         # FCS_MODE_CHANGE_REQUEST
    pre_fix_status = pre_fix_base + 0x05       # FCS_STATUS_REQUEST
    pre_fix_control = pre_fix_base + 0x01      # FCS_CONTROL_INPUT_REQUEST

    mode_fields = _fields(format(pre_fix_mode, '016b'))
    R.check("0x4007 decoded to RT address 8, not 5", mode_fields['rt'] == 8,
            str(mode_fields))
    R.check("...and to subaddress 0, not 10", mode_fields['sa'] == 0,
            str(mode_fields))

    for label, value in (("status", pre_fix_status), ("control input", pre_fix_control)):
        R.check(f"the {label} word decoded to RT 8 too",
                _fields(format(value, '016b'))['rt'] == 8)

    # RT 8 is a real, different system -- this is why it mattered.
    from FMOFP.local_messaging.address_utils import get_rt_address
    R.check("RT 8 belongs to Power Management", get_rt_address('powermanagement') == 8,
            str(get_rt_address('powermanagement')))

    # And the receive side would have believed it. Raw source here, not
    # code_only(): the thing being matched IS a string literal, and
    # code_only() strips those (its own docstring says so).
    extractor = _src('MIL_STD_1553B/Remote_Terminal/rt_message_extractor.py')
    R.check("the RT extractor maps 8 to power_management",
            "8: 'power_management'" in extractor or '8: "power_management"' in extractor)
    R.check("...and maps 5 to flight_control, which is where the FCS now sends",
            "5: 'flight_control'" in extractor or '5: "flight_control"' in extractor)

    from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS
    now = {cmd.value for cmd in FCS_COMMAND_WORDS.values()}
    R.check("none of the invented values survives",
            not ({pre_fix_mode, pre_fix_status, pre_fix_control} & now),
            str(sorted(now)))

    src = code_only(_src('local_messaging/command_word_map_fcs.py'))
    R.check("FCS_BASE_COMMAND is gone from the code", "FCS_BASE_COMMAND" not in src)


def test_fcs_words_collide_with_nothing_else():
    R.section("the new words collide with no other system's")

    from FMOFP.local_messaging.command_word_map import (
        MODE_REQUEST_MAP, STATUS_REQUEST_MAP, DATA_REQUEST_MAP,
        DISPLAY_MODE_REQUEST_MAP, WEATHER_DATA_REQUEST_MAP)
    from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS
    from FMOFP.local_messaging.command_word_map import FCS_TYPES

    fcs_words = {format(c.value, '016b') for c in FCS_COMMAND_WORDS.values()}

    others = set()
    for m in (MODE_REQUEST_MAP, STATUS_REQUEST_MAP, DATA_REQUEST_MAP,
              DISPLAY_MODE_REQUEST_MAP, WEATHER_DATA_REQUEST_MAP):
        for key, value in m.items():
            if key in FCS_TYPES:
                continue
            others.add(value)

    R.check("no FCS command word is also some other system's",
            not (fcs_words & others), str(sorted(fcs_words & others)))

    # Nothing else is at RT 5 either, which is the stronger statement.
    at_rt5 = sorted(w for w in others
                    if isinstance(w, str) and len(w) == 16 and _fields(w)['rt'] == 5)
    R.check("RT address 5 belongs to the FCS alone", not at_rt5, str(at_rt5))


def test_every_map_entry_is_a_binary_string():
    R.section("the request maps hold one type, not two")

    from FMOFP.local_messaging.command_word_map import (
        MODE_REQUEST_MAP, STATUS_REQUEST_MAP, DATA_REQUEST_MAP)

    # Pre-fix, the four FCS keys in MODE_REQUEST_MAP and STATUS_REQUEST_MAP
    # held bare ints (16391, 16389) while every radar and FMS key held a
    # 16-bit binary string. Nothing caught it because commandCheck() quietly
    # accepts an int -- which is also why the wrong address travelled instead
    # of raising.
    for label, m in (("mode", MODE_REQUEST_MAP), ("status", STATUS_REQUEST_MAP),
                     ("data", DATA_REQUEST_MAP)):
        bad = {k: v for k, v in m.items()
               if not (isinstance(v, str) and len(v) == 16 and set(v) <= {'0', '1'})}
        R.check(f"every {label} map entry is a 16-bit binary string",
                not bad, str(bad))

    from FMOFP.local_messaging.command_word_map import FCS_TYPES
    for fcs_type in FCS_TYPES:
        R.check(f"{fcs_type} has a data command word now",
                fcs_type in DATA_REQUEST_MAP, str(sorted(DATA_REQUEST_MAP)))

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.\
        RadarMessageHandler import resolve_command_word
    word, matched = resolve_command_word(DATA_REQUEST_MAP, "flightControlSystem")
    R.check("'flightControlSystem' resolves to a data command word",
            word is not None, f"matched={matched}")


def test_empty_command_words_do_not_crash_the_import():
    R.section("a missing FCS command word refuses rather than raising")

    # The registration used `.get(name, {}).value`, so the one case the
    # default exists to cover -- no FCS command word -- raised
    # AttributeError at import, taking the whole application down.
    src = code_only(_src('local_messaging/command_word_map.py'))
    R.check("the `.get(name, {}).value` registration is gone",
            ".get(FCS_MODE_CHANGE_REQUEST, {}).value" not in src
            and ".get(FCS_STATUS_REQUEST, {}).value" not in src)

    from FMOFP.local_messaging import command_word_map as cwm
    from FMOFP.local_messaging.message_types_fcs import FCS_MODE_CHANGE_REQUEST

    R.check("_fcs_map_entry returns None for an unknown command name, "
            "where the old expression raised AttributeError",
            cwm._fcs_map_entry("NO_SUCH_FCS_COMMAND") is None)

    known = cwm._fcs_map_entry(FCS_MODE_CHANGE_REQUEST)
    R.check("...and a 16-bit binary string for a known one",
            isinstance(known, str) and len(known) == 16, repr(known))

    # The pre-fix expression, inline: this is the AttributeError the default
    # was supposed to prevent.
    try:
        {}.get(FCS_MODE_CHANGE_REQUEST, {}).value
        R.check("the pre-fix expression raises on its own default", False)
    except AttributeError:
        R.check("the pre-fix expression raises AttributeError on its own "
                "default, which happened at import time", True)


# ---------------------------------------------------------------------------
# 2. data words
# ---------------------------------------------------------------------------

def test_control_input_round_trips_through_the_real_decoder():
    R.section("control input encodes to what the FCS decodes")

    from FMOFP.local_messaging.fcs_wire_format import (
        CONTROL_SURFACE_TO_CODE, CODE_TO_CONTROL_SURFACE, decode_control_value,
        CONTROL_VALUE_QUANTUM)
    from FMOFP.local_messaging.messageConfigurations.fcs_data import (
        fcs_controlSurfaceRequest)

    for surface in sorted(CONTROL_SURFACE_TO_CODE):
        for value in (-1.0, -0.5, -0.003, 0.0, 0.3, 0.5, 1.0):
            req = fcs_controlSurfaceRequest(
                message_header="data_request", sending_system="test",
                destination="flightControlSystem",
                surface_data={"surface": surface, "position": value})
            words = req.to_data_words()
            if len(words) != 2:
                R.check(f"{surface}@{value} encodes to two words", False, str(words))
                continue
            got_surface = CODE_TO_CONTROL_SURFACE.get(int(words[0], 2))
            got_value = decode_control_value(int(words[1], 2))
            R.check(f"{surface} at {value:+.3f} round-trips",
                    got_surface == surface
                    and abs(got_value - value) <= CONTROL_VALUE_QUANTUM,
                    f"-> {got_surface} {got_value}")

    # Out-of-range values clamp to the rail rather than wrapping.
    req = fcs_controlSurfaceRequest(
        message_header="data_request", sending_system="test",
        destination="flightControlSystem",
        surface_data={"surface": "elevator", "position": 7.5})
    value = decode_control_value(int(req.to_data_words()[1], 2))
    R.check("a value past the rail clamps to +1.0, it does not wrap",
            value == 1.0, str(value))


def test_the_orphan_encoding_was_wrong_not_merely_different():
    R.section("NON-TAUTOLOGICAL — why the orphan layout was not adopted")

    from FMOFP.local_messaging.fcs_wire_format import (
        CODE_TO_CONTROL_SURFACE, decode_control_value, MODE_VALUE_TO_NAME)

    # The orphan encode_1553b() layout, reproduced inline.
    orphan_surface_map = {"aileron": 0x01, "elevator": 0x02, "rudder": 0x03,
                          "flaps": 0x04, "spoilers": 0x05, "speedbrake": 0x06,
                          "trim": 0x07}
    orphan_code = orphan_surface_map["flaps"]
    R.check("the orphan's 'flaps' code decodes as throttle",
            CODE_TO_CONTROL_SURFACE.get(orphan_code) == "throttle",
            str(CODE_TO_CONTROL_SURFACE.get(orphan_code)))

    orphan_scaled = int(0.3 * 100)             # its position scaling
    orphan_value = decode_control_value(orphan_scaled)
    R.check("the orphan's +0.3 decodes as near-full negative deflection",
            orphan_value < -0.9, f"{orphan_value:.3f}")

    orphan_mode_map = {"normal": 0x01, "approach": 0x02, "takeoff": 0x03,
                       "landing": 0x04, "cruise": 0x05, "combat": 0x06,
                       "emergency": 0x07}
    R.check("the orphan's 'normal' (1) decodes as COMBAT",
            MODE_VALUE_TO_NAME.get(orphan_mode_map["normal"]) == "COMBAT",
            str(MODE_VALUE_TO_NAME.get(orphan_mode_map["normal"])))
    R.check("the orphan's 'combat' (6) decodes as nothing at all",
            orphan_mode_map["combat"] not in MODE_VALUE_TO_NAME)
    R.check("four of the orphan's seven mode names are not FCS modes at all",
            not ({"approach", "takeoff", "landing", "cruise"}
                 & {n.lower() for n in MODE_VALUE_TO_NAME.values()}))

    # And it is gone from the source now.
    for relpath in ("local_messaging/messageConfigurations/fcs_data.py",
                    "local_messaging/messageConfigurations/fcs_flight_mode.py"):
        src = code_only(_src(relpath))
        R.check(f"{os.path.basename(relpath)} no longer defines its own table",
                "spoilers" not in src and "approach" not in src, relpath)


def test_encode_1553b_delegates_to_one_encoding():
    R.section("encode_1553b() and to_data_words() cannot drift apart")

    from FMOFP.local_messaging.command_word_map_fcs import FCS_COMMAND_WORDS
    from FMOFP.local_messaging.message_types_fcs import (
        FCS_CONTROL_INPUT_REQUEST, FCS_MODE_CHANGE_REQUEST)
    from FMOFP.local_messaging.messageConfigurations.fcs_data import (
        fcs_controlSurfaceRequest)
    from FMOFP.local_messaging.messageConfigurations.fcs_flight_mode import (
        fcs_flightModeRequest)

    cases = (
        (fcs_controlSurfaceRequest(
            message_header="data_request", sending_system="test",
            destination="flightControlSystem",
            surface_data={"surface": "rudder", "position": -0.25}),
         FCS_CONTROL_INPUT_REQUEST),
        (fcs_flightModeRequest(
            message_header="data_request", sending_system="test",
            destination="flightControlSystem",
            mode_data={"mode": "TERRAIN"}),
         FCS_MODE_CHANGE_REQUEST),
    )
    for msg, command_name in cases:
        frames = msg.encode_1553b()
        label = type(msg).__name__
        R.check(f"{label}.encode_1553b() leads with its command word",
                frames[0] == FCS_COMMAND_WORDS[command_name].value,
                f"{frames[0]} vs {FCS_COMMAND_WORDS[command_name].value}")
        R.check(f"...and the rest is exactly to_data_words()",
                frames[1:] == [int(w, 2) for w in msg.to_data_words()],
                f"{frames[1:]} vs {msg.to_data_words()}")


def test_unrecognised_inputs_are_refused_not_substituted():
    R.section("an unknown surface or mode is refused, not turned into another")

    from FMOFP.local_messaging.fcs_wire_format import (
        control_input_data_words, mode_data_words, MODE_NAME_TO_VALUE)

    for bogus in ("canard", "", "AILERON ", None):
        try:
            words = control_input_data_words(bogus, 0.5)
            R.check(f"control_input_data_words({bogus!r}) raises", False, str(words))
        except ValueError:
            R.check(f"control_input_data_words({bogus!r}) raises ValueError", True)
        except Exception as exc:  # noqa: BLE001 - anything else is the wrong error
            R.check(f"control_input_data_words({bogus!r}) raises ValueError, not "
                    f"{type(exc).__name__}", False, str(exc))

    for bogus in ("CRUISE", "approach", "", None, 99):
        try:
            words = mode_data_words(bogus)
            R.check(f"mode_data_words({bogus!r}) raises", False, str(words))
        except ValueError:
            R.check(f"mode_data_words({bogus!r}) raises ValueError", True)
        except Exception as exc:  # noqa: BLE001
            R.check(f"mode_data_words({bogus!r}) raises ValueError, not "
                    f"{type(exc).__name__}", False, str(exc))

    # The specific substitution that used to happen.
    R.check("the value an unknown mode used to become is NORMAL",
            MODE_NAME_TO_VALUE["NORMAL"] == 0)
    R.check("...and 0 is still NORMAL, so the old default really did command it",
            mode_data_words("NORMAL") == [format(0, '016b')])


def test_what_cannot_be_carried_is_reported():
    R.section("fields the encoding has no room for are not dropped silently")

    from FMOFP.local_messaging.fcs_wire_format import (
        control_input_data_words, mode_data_words)

    with _Capture() as cap:
        words = control_input_data_words("aileron", 0.2, rate=15.0)
    R.check("a control input still encodes to two words with a rate given",
            len(words) == 2, str(words))
    R.check("...and the caller is told the rate is not transmitted",
            "rate" in cap.text().lower() and "not" in cap.text().lower(),
            cap.text()[:200])

    with _Capture() as cap:
        words = mode_data_words("COMBAT", extra_params={"mode": "COMBAT",
                                                        "flaps": 2,
                                                        "gear": "down"})
    R.check("a mode change still encodes to one word with extra params given",
            len(words) == 1, str(words))
    text = cap.text()
    R.check("...and the caller is told which params are not transmitted",
            "flaps" in text and "gear" in text, text[:200])

    with _Capture() as cap:
        control_input_data_words("flaps", 0.5)
    R.check("a surface the FCS cannot apply is flagged at encode time",
            "flaps" in cap.text(), cap.text()[:200])


# ---------------------------------------------------------------------------
# 3. one definition, five call sites
# ---------------------------------------------------------------------------

def test_the_tables_are_defined_exactly_once():
    R.section("five literal copies became one definition")

    paths = sorted(glob.glob(os.path.join(_FMOFP, '**', '*.py'), recursive=True))
    R.check("the sweep found the package's sources", len(paths) > 100, str(len(paths)))

    # Raw source, not code_only(): every needle below IS a string literal or
    # contains one, and code_only() strips those. This suite is the fifth
    # place that mistake has been made in this test tree; the note in
    # FMOFP/Tests/__init__.py documents the other four.
    def count(needle, skip_self=True):
        hits = []
        for path in paths:
            if skip_self and os.path.abspath(path) == os.path.abspath(__file__):
                continue
            try:
                with open(path, encoding='utf-8') as fh:
                    src = fh.read()
            except (OSError, UnicodeDecodeError):
                continue
            if needle in src:
                hits.append(os.path.relpath(path, _FMOFP))
        return hits

    # The control-surface table. Both spellings, since the copies differed.
    surface_hits = count('"aileron": 1') + count("'aileron': 1")
    R.check("the control-surface codes are defined in one file",
            len(surface_hits) == 1, str(surface_hits))
    if surface_hits:
        R.check("...and that file is fcs_wire_format.py",
                surface_hits[0].endswith('fcs_wire_format.py'), surface_hits[0])

    # "COMBAT": 1 is NOT a discriminating needle: the FMS vocabulary agrees
    # with the FCS on COMBAT and differs only from value 2 upward. PRECISION
    # is the first value where they part company.
    mode_hits = count('"PRECISION": 2') + count("'PRECISION': 2")
    R.check("the FCS mode values are defined in one file",
            len(mode_hits) == 1, str(mode_hits))
    if mode_hits:
        R.check("...and that file is fcs_wire_format.py",
                mode_hits[0].endswith('fcs_wire_format.py'), mode_hits[0])

    # The scanner must be able to see a copy when there is one, or the
    # checks above pass for the wrong reason. This suite itself holds the
    # orphan table inline (test_the_orphan_encoding_was_wrong...), so point
    # the scanner at this file and it must find it.
    R.check("the scanner finds a table when one is there",
            count('"aileron": 0x01', skip_self=False), "scanner is blind")

    # The scaling arithmetic, which was written out three times.
    scale_hits = count('+ 1.0) * 500')
    R.check("the control-value scaling is written once",
            len(scale_hits) == 1, str(scale_hits))

    # The FMS's mode vocabulary is a DIFFERENT table, and it is also now
    # defined once. FMSMessageHandler's mode packer carried a copy of the
    # FCS's table under a comment claiming they were aligned; they are not.
    fms_mode_hits = count('"STEALTH": 2')
    R.check("the FMS mode values are defined in one file",
            len(fms_mode_hits) == 1, str(fms_mode_hits))
    if fms_mode_hits:
        R.check("...and that file is fms_tactical_data.py",
                fms_mode_hits[0].endswith('fms_tactical_data.py'),
                fms_mode_hits[0])

    # And the four sites that held copies now import them.
    for relpath, names in (
        ('local_messaging/routing/handlers/system_message_handlers/FCSMessageHandler.py',
         ('MODE_NAME_TO_VALUE', 'control_input_data_words')),
        ('local_messaging/routing/handlers/system_message_handlers/FMSMessageHandler.py',
         ('mode_data_words', 'control_input_data_words')),
    ):
        src = code_only(_src(relpath))
        R.check(f"{os.path.basename(relpath)} imports from fcs_wire_format",
                'fcs_wire_format' in src, relpath)
        for name in names:
            R.check(f"...and uses {name}", name in src, relpath)


def test_the_mode_vocabulary_is_the_fcs_s_own():
    R.section("the encoded modes are the modes the FCS has")

    from FMOFP.local_messaging.fcs_wire_format import (
        MODE_NAME_TO_VALUE, MODE_VALUE_TO_NAME, APPLICABLE_CONTROL_SURFACES)

    # Read FlightControlModes and control_inputs out of the source rather
    # than constructing a FlightControlSystem, which starts threads.
    tree = ast.parse(_src('Systems/flightManagementSys/flightControlSys/'
                          'flight_control_system.py'))

    declared_modes = set()
    declared_inputs = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == 'FlightControlModes':
            for stmt in node.body:
                if (isinstance(stmt, ast.Assign)
                        and isinstance(stmt.value, ast.Constant)
                        and isinstance(stmt.value.value, str)):
                    declared_modes.add(stmt.value.value)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (isinstance(target, ast.Attribute)
                        and target.attr == 'control_inputs'
                        and isinstance(node.value, ast.Dict)):
                    for key in node.value.keys:
                        if isinstance(key, ast.Constant):
                            declared_inputs.add(key.value)

    R.check("FlightControlModes was found in the source",
            len(declared_modes) == 6, str(sorted(declared_modes)))
    encodable = {n for n in MODE_NAME_TO_VALUE if n != 'STANDBY'}
    R.check("every FCS mode is encodable", declared_modes <= encodable,
            str(sorted(declared_modes - encodable)))
    R.check("...and nothing else is", encodable <= declared_modes,
            str(sorted(encodable - declared_modes)))
    R.check("the reverse map covers every value once",
            sorted(MODE_VALUE_TO_NAME) == sorted(set(MODE_VALUE_TO_NAME)),
            str(MODE_VALUE_TO_NAME))
    R.check("STANDBY is an alias of NORMAL, not a seventh mode",
            MODE_NAME_TO_VALUE['STANDBY'] == MODE_NAME_TO_VALUE['NORMAL']
            and 'STANDBY' not in MODE_VALUE_TO_NAME.values())

    R.check("control_inputs was found in the source",
            declared_inputs == {'aileron', 'elevator', 'rudder', 'throttle'},
            str(sorted(declared_inputs)))
    R.check("APPLICABLE_CONTROL_SURFACES matches what the FCS actually holds",
            set(APPLICABLE_CONTROL_SURFACES) == declared_inputs,
            str(sorted(set(APPLICABLE_CONTROL_SURFACES) ^ declared_inputs)))


def test_the_handlers_pack_through_the_shared_encoding():
    R.section("both handlers' packers produce the shared words")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.\
        FCSMessageHandler import FCSMessageHandler
    from FMOFP.local_messaging.routing.handlers.system_message_handlers.\
        FMSMessageHandler import FMSMessageHandler
    from FMOFP.local_messaging.fcs_wire_format import (
        control_input_data_words, mode_data_words)
    from FMOFP.local_messaging.messageConfigurations.fms_tactical_data import (
        fms_mode_data_words)

    # __new__ rather than __init__: these handlers start timers and bind
    # sockets, and the packers are pure.
    #
    # The control-input encoding is the FCS's in BOTH handlers -- the FMS
    # handler forwards control input to the FCS. The MODE encoding is not:
    # the FCS packs FCS modes and the FMS packs FMS modes, and the two
    # vocabularies differ from value 2 upward.
    for cls, mode_packer, mode_words in (
            (FCSMessageHandler, '_pack_mode_data', mode_data_words),
            (FMSMessageHandler, '_pack_data', fms_mode_data_words)):
        handler = cls.__new__(cls)
        name = cls.__name__

        got = handler._pack_control_input_data({'control_type': 'elevator',
                                                'value': -0.4})
        R.check(f"{name}._pack_control_input_data matches the FCS encoding",
                got == control_input_data_words('elevator', -0.4), str(got))

        for payload, label in (({'control_type': 'canard', 'value': 0.1},
                                'an unknown surface'),
                               ({'value': 0.1}, 'a missing surface')):
            try:
                got = handler._pack_control_input_data(payload)
                R.check(f"{name} refuses {label}", False, str(got))
            except ValueError:
                R.check(f"{name} refuses {label}", True)

        pack = getattr(handler, mode_packer)
        for mode in ('NORMAL', 'COMBAT', 'EMERGENCY'):
            got = pack(mode)
            R.check(f"{name}.{mode_packer}({mode}) matches its own vocabulary",
                    got == mode_words(mode), f"{got} vs {mode_words(mode)}")

        # The dict form callers actually pass. {"mode": ...} matched none of
        # the extraction branches in the FMS packer, so the mode name stayed
        # None and the old default turned it into 0 -- NORMAL. Both packers
        # accept 'mode' and 'mode_name' now.
        for key in ('mode', 'mode_name'):
            got = pack({key: 'COMBAT'})
            R.check(f"{name}.{mode_packer}({{{key!r}: 'COMBAT'}}) encodes COMBAT",
                    got == mode_words('COMBAT'),
                    f"{got} vs {mode_words('COMBAT')}")
            R.check(f"...and not NORMAL, which is what the old default gave",
                    got != mode_words('NORMAL'), str(got))

        try:
            got = pack('NO_SUCH_MODE')
            R.check(f"{name} refuses an unknown mode", False, str(got))
        except ValueError:
            R.check(f"{name} refuses an unknown mode", True)


def test_the_two_mode_vocabularies_are_not_interchangeable():
    R.section("NON-TAUTOLOGICAL — the FMS's modes are not the FCS's")

    from FMOFP.local_messaging.fcs_wire_format import (
        MODE_NAME_TO_VALUE, MODE_VALUE_TO_NAME)
    from FMOFP.local_messaging.messageConfigurations.fms_tactical_data import (
        FMS_MODE_NAME_TO_VALUE, FMS_MODE_VALUE_TO_NAME, fms_mode_data_words)

    fcs_names = {n for n in MODE_NAME_TO_VALUE if n != 'STANDBY'}
    fms_names = set(FMS_MODE_NAME_TO_VALUE)

    R.check("the FMS has modes the FCS does not", fms_names - fcs_names,
            str(sorted(fms_names - fcs_names)))
    R.check("...and the FCS has modes the FMS does not", fcs_names - fms_names,
            str(sorted(fcs_names - fms_names)))

    # The value that matters most, and it differs.
    R.check("EMERGENCY is 4 to the FMS and 5 to the FCS",
            FMS_MODE_NAME_TO_VALUE['EMERGENCY'] == 4
            and MODE_NAME_TO_VALUE['EMERGENCY'] == 5,
            f"fms={FMS_MODE_NAME_TO_VALUE['EMERGENCY']} "
            f"fcs={MODE_NAME_TO_VALUE['EMERGENCY']}")
    R.check("an FMS EMERGENCY encoded with the FCS table decodes as TERRAIN",
            MODE_VALUE_TO_NAME[MODE_NAME_TO_VALUE['EMERGENCY'] - 1] == 'TERRAIN')
    R.check("an FMS EMERGENCY decoded with the FCS table would name TERRAIN",
            MODE_VALUE_TO_NAME[FMS_MODE_NAME_TO_VALUE['EMERGENCY']] == 'TERRAIN',
            MODE_VALUE_TO_NAME[FMS_MODE_NAME_TO_VALUE['EMERGENCY']])

    # STEALTH and TRAINING are real FMS modes and were absent from the table
    # the FMS packer used, so they fell through its default to 0 = NORMAL.
    for mode in ('STEALTH', 'TRAINING'):
        R.check(f"{mode} is a real FMS mode", mode in FMS_MODE_NAME_TO_VALUE)
        R.check(f"...and was not in the FCS table the FMS packer used",
                mode not in MODE_NAME_TO_VALUE)
        words = fms_mode_data_words(mode)
        R.check(f"...and now encodes as itself, not as NORMAL",
                words != [format(0, '016b')], str(words))
        R.check(f"...and decodes back to {mode}",
                FMS_MODE_VALUE_TO_NAME[int(words[0], 2)] == mode)

    # The FMS's own set_mode is the authority for that list.
    src = _src('Systems/flightManagementSys/flightManagementSystem.py')
    R.check("FlightManagementSystem.set_mode validates against these five",
            '["NORMAL", "COMBAT", "STEALTH", "TRAINING", "EMERGENCY"]' in src)

    R.check("the FMS encode and decode halves are inverses",
            all(FMS_MODE_VALUE_TO_NAME[v] == n
                for n, v in FMS_MODE_NAME_TO_VALUE.items()))


def test_an_unknown_mode_name_on_arrival_is_refused():
    R.section("the receive side no longer substitutes NORMAL either")

    from FMOFP.local_messaging.routing.handlers.system_message_handlers.\
        FCSMessageHandler import FCSMessageHandler

    handler = FCSMessageHandler.__new__(FCSMessageHandler)
    handler.loop_prevention = None
    handler.fcs = None

    result = asyncio.run(handler._handle_mode_change_request(
        {'parameters': {'mode_name': 'CRUISE'}, 'request_id': 'r-1'}))
    R.check("a mode change naming a mode the FCS lacks returns ERROR",
            isinstance(result, dict) and result.get('status') == 'ERROR',
            str(result))
    R.check("...and the message names the mode it refused",
            isinstance(result, dict) and 'CRUISE' in str(result.get('message', '')),
            str(result))

    src = code_only(inspect.getsource(FCSMessageHandler._handle_mode_change_request))
    R.check("the inline mode table is gone from the handler",
            'mode_map' not in src)


def main():
    print("=" * 60)
    print("  The FCS on the bus: addressing and data words")
    print("=" * 60)

    for test in (test_command_words_carry_the_fcs_address,
                 test_pre_fix_words_addressed_power_management,
                 test_fcs_words_collide_with_nothing_else,
                 test_every_map_entry_is_a_binary_string,
                 test_empty_command_words_do_not_crash_the_import,
                 test_control_input_round_trips_through_the_real_decoder,
                 test_the_orphan_encoding_was_wrong_not_merely_different,
                 test_encode_1553b_delegates_to_one_encoding,
                 test_unrecognised_inputs_are_refused_not_substituted,
                 test_what_cannot_be_carried_is_reported,
                 test_the_tables_are_defined_exactly_once,
                 test_the_mode_vocabulary_is_the_fcs_s_own,
                 test_the_handlers_pack_through_the_shared_encoding,
                 test_the_two_mode_vocabularies_are_not_interchangeable,
                 test_an_unknown_mode_name_on_arrival_is_refused):
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
