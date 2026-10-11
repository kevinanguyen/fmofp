"""Check, render and generate code from the FMOFP bus ICD (B20SS/icd/fmofp_icd.xml).

    py B20SS/icd/tools/icd_tool.py check              # validate; exit code 1 on any error
    py B20SS/icd/tools/icd_tool.py render             # check, then regenerate B20SS/icd/ICD_TABLES.md
    py B20SS/icd/tools/icd_tool.py gen-cpp --out DIR  # check, then write the C++ headers and tests
    py B20SS/icd/tools/icd_tool.py gen-py             # check, then regenerate B20SS/fmofp_bus/icd/

Standard library only, so it runs anywhere the Python side runs, including as a
build step of the C++ (B20SS/ofp/icd/CMakeLists.txt).

`check` enforces the rules the old message_schemas.py could not: every bit of
every word is defined, every enum value fits its field and defines a fail-safe
zero, scaled ranges are representable, no two messages share an
RT/subaddress/direction, and every legacy message type is accounted for.

`gen-cpp` writes, under DIR:
    include/fmofp/icd/version.hpp   the ICD version the code was built from
    include/fmofp/icd/enums.hpp     one enum class per ICD enum, with validity checks
    include/fmofp/icd/messages.hpp  one struct per message with encode()/decode()
    icd_roundtrip_test.cpp          a GoogleTest round-trip test for every message
The generated code uses only fmofp/bus/word.hpp for bit access, so the headers are
the only place a message layout exists on the C++ side.
"""

import argparse
import ast
import sys
import xml.etree.ElementTree as ET
from fractions import Fraction
from pathlib import Path

ICD_DIR = Path(__file__).resolve().parent.parent
ICD_XML = ICD_DIR / "fmofp_icd.xml"
TABLES_MD = ICD_DIR / "ICD_TABLES.md"
LEGACY_SCHEMAS = ICD_DIR.parent / "FMOFP" / "MIL_STD_1553B" / "message_schemas.py"

RATES = {"50", "10", "5", "1", "aperiodic"}
FIELD_TYPES = {"uint", "sint", "enum", "bool", "ssm", "spare"}
MIN_SA, MAX_SA = 1, 29          # 0 and 31 are mode codes, 30 is data wraparound
MAX_RT = 30                     # 31 is broadcast
MAX_WORDS = 32


class Icd:
    """Parsed ICD plus the errors found while checking it."""

    def __init__(self, path):
        self.path = path
        self.root = ET.parse(path).getroot()
        self.errors = []
        self.enums = {}
        self.terminals = {}
        self.messages = []
        self.blocks = []
        self.mode_codes = []
        self.legacy = []

    def error(self, where, text):
        self.errors.append(f"{where}: {text}")


# --------------------------------------------------------------------- parsing

def parse_bits(text):
    """'15-12' -> (15, 12); '7' -> (7, 7). Raises ValueError on bad input."""
    if "-" in text:
        hi, lo = (int(part) for part in text.split("-"))
    else:
        hi = lo = int(text)
    if not (0 <= lo <= hi <= 15):
        raise ValueError(f"bad bit range '{text}' (need 15 >= high >= low >= 0)")
    return hi, lo


def tokens(text):
    return text.split() if text else []


def load(path):
    icd = Icd(path)
    root = icd.root

    for enum in root.iter("enum"):
        values = [(v.get("name"), int(v.get("value")), v.get("legacy"), v.get("doc"))
                  for v in enum.iter("value")]
        if enum.get("id") in icd.enums:
            icd.error(f"enum {enum.get('id')}", "duplicate enum id")
        icd.enums[enum.get("id")] = {"bits": int(enum.get("bits")), "doc": enum.get("doc"), "values": values}

    for term in root.iter("terminal"):
        rt = int(term.get("rt"))
        if rt in icd.terminals:
            icd.error(f"terminal RT {rt}", "duplicate RT address")
        icd.terminals[rt] = dict(term.attrib, rt=rt)

    by_id = {}
    for msg in root.find("messages").iter("message"):
        entry = dict(msg.attrib)
        entry["doc"] = (msg.findtext("doc") or "").strip()
        entry["fields"] = [dict(f.attrib) for f in msg.iter("field")]
        if entry["id"] in by_id:
            icd.error(f"message {entry['id']}", "duplicate message id")
        by_id[entry["id"]] = entry
        icd.messages.append(entry)

    for entry in icd.messages:
        source = entry.get("layout_of")
        if source:
            if source not in by_id:
                icd.error(f"message {entry['id']}", f"layout_of names unknown message '{source}'")
                continue
            entry["fields"] = by_id[source]["fields"]
            entry["words"] = by_id[source]["words"]

    icd.blocks = [dict(b.attrib) for b in root.iter("block")]
    icd.mode_codes = [dict(m.attrib) for m in root.iter("mode_code")]
    icd.legacy = [dict(lg.attrib) for lg in root.iter("legacy")]
    return icd


def message_rts(icd, msg):
    """RT addresses a message lives on ('all' = every non-reserved terminal)."""
    if msg["rt"] == "all":
        return sorted(rt for rt, t in icd.terminals.items() if t.get("status") != "reserved")
    return [int(msg["rt"])]


# --------------------------------------------------------------------- checks

def check_enums(icd):
    for eid, enum in icd.enums.items():
        names, values = set(), set()
        if not 1 <= enum["bits"] <= 8:
            icd.error(f"enum {eid}", f"{enum['bits']} bits; enums are 1-8 bits wide")
        for name, value, _, _ in enum["values"]:
            where = f"enum {eid}.{name}"
            if name in names:
                icd.error(where, "duplicate name")
            if value in values:
                icd.error(where, f"duplicate value {value}")
            if not 0 <= value < (1 << enum["bits"]):
                icd.error(where, f"value {value} does not fit in {enum['bits']} bits")
            names.add(name)
            values.add(value)
        # An all-zero word must never read as good data (README section 4), so
        # every enum needs a defined, safe meaning for 0.
        if 0 not in values:
            icd.error(f"enum {eid}", "no value 0; an uninitialised field must decode to a defined, safe value")


def check_terminals(icd):
    ids = set()
    for rt, term in icd.terminals.items():
        if not 0 <= rt <= MAX_RT:
            icd.error(f"terminal {term.get('id')}", f"RT {rt} outside 0-{MAX_RT} (31 is broadcast)")
        if term.get("id") in ids:
            icd.error(f"terminal RT {rt}", f"duplicate id '{term.get('id')}'")
        ids.add(term.get("id"))


def check_scaled(icd, where, field, width):
    lsb_text = field.get("lsb")
    if lsb_text is None:
        if field.get("min") is not None or field.get("max") is not None:
            icd.error(where, "min/max given without an lsb")
        return
    lsb = Fraction(lsb_text)
    if lsb <= 0:
        icd.error(where, "lsb must be positive")
        return
    if field["type"] == "sint":
        lowest, highest = -(1 << (width - 1)) * lsb, ((1 << (width - 1)) - 1) * lsb
    else:
        lowest, highest = Fraction(0), ((1 << width) - 1) * lsb
    for key, limit_ok in (("min", lambda v: v >= lowest), ("max", lambda v: v <= highest)):
        if field.get(key) is not None and not limit_ok(Fraction(field[key])):
            icd.error(where, f"{key} {field[key]} is outside the representable range "
                             f"[{float(lowest):g}, {float(highest):g}]")


def check_message(icd, msg):
    mid = msg["id"]
    if msg.get("rt") != "all" and int(msg["rt"]) not in icd.terminals:
        icd.error(f"message {mid}", f"unknown RT {msg['rt']}")
    elif msg.get("rt") != "all" and icd.terminals[int(msg["rt"])].get("status") == "reserved":
        icd.error(f"message {mid}", f"RT {msg['rt']} is reserved")
    sa = int(msg["sa"])
    if not MIN_SA <= sa <= MAX_SA:
        icd.error(f"message {mid}", f"SA {sa} outside {MIN_SA}-{MAX_SA}")
    if msg.get("dir") not in ("T", "R"):
        icd.error(f"message {mid}", "dir must be T (terminal transmits) or R (terminal receives)")
    if msg.get("rate") not in RATES:
        icd.error(f"message {mid}", f"rate '{msg.get('rate')}' not one of {sorted(RATES)}")
    if msg.get("layout_of"):
        return

    words = int(msg["words"])
    if not 1 <= words <= MAX_WORDS:
        icd.error(f"message {mid}", f"{words} words; must be 1-{MAX_WORDS}")
    used = {w: set() for w in range(words)}
    names = {f.get("name") for f in msg["fields"]}
    seen = set()

    for field in msg["fields"]:
        where = f"message {mid}.{field.get('name')}"
        if field.get("name") in seen:
            icd.error(where, "duplicate field name")
        seen.add(field.get("name"))
        ftype = field.get("type")
        if ftype not in FIELD_TYPES:
            icd.error(where, f"unknown type '{ftype}'")
            continue
        word = int(field["word"])
        try:
            hi, lo = parse_bits(field["bits"])
        except ValueError as exc:
            icd.error(where, str(exc))
            continue
        width = hi - lo + 1
        if not 0 <= word < words:
            icd.error(where, f"word {word} outside 0-{words - 1}")
            continue
        bits = set(range(lo, hi + 1))
        if used[word] & bits:
            icd.error(where, f"overlaps another field in word {word}")
        used[word] |= bits

        if ftype == "bool" and width != 1:
            icd.error(where, "bool must be 1 bit")
        elif ftype == "ssm":
            if width != 2:
                icd.error(where, "ssm must be 2 bits")
            if field.get("for") not in names:
                icd.error(where, f"ssm 'for' names unknown field '{field.get('for')}'")
        elif ftype == "enum":
            enum = icd.enums.get(field.get("enum"))
            if enum is None:
                icd.error(where, f"unknown enum '{field.get('enum')}'")
            elif any(value >= (1 << width) for _, value, _, _ in enum["values"]):
                icd.error(where, f"enum '{field['enum']}' has values that do not fit in {width} bits")
        elif ftype in ("uint", "sint"):
            check_scaled(icd, where, field, width)

    for word, bits in used.items():
        missing = sorted(set(range(16)) - bits, reverse=True)
        if missing:
            icd.error(f"message {mid}", f"word {word} bits {missing} undefined; declare them as spare")


def check_addresses(icd):
    owner = {}

    def claim(rt, sa, direction, label):
        key = (rt, sa, direction)
        if key in owner:
            icd.error(label, f"RT {rt} SA {sa} {direction} already used by {owner[key]}")
        owner[key] = label

    for msg in icd.messages:
        for rt in message_rts(icd, msg):
            claim(rt, int(msg["sa"]), msg.get("dir"), f"message {msg['id']}")
    for block in icd.blocks:
        where = f"block {block['id']}"
        rt = int(block["rt"])
        if rt not in icd.terminals:
            icd.error(where, f"unknown RT {rt}")
        if block.get("rate") not in RATES - {"aperiodic"}:
            icd.error(where, f"rate '{block.get('rate')}' must be periodic")
        for key in ("desc_sa", "seg_sa"):
            sa = int(block[key])
            if not MIN_SA <= sa <= MAX_SA:
                icd.error(where, f"{key} {sa} outside {MIN_SA}-{MAX_SA}")
            claim(rt, sa, "T", f"{where} ({key})")

    codes = set()
    for mc in icd.mode_codes:
        code = int(mc["code"])
        if not 0 <= code <= 31 or code in codes:
            icd.error(f"mode code {code}", "outside 0-31 or duplicated")
        codes.add(code)


def legacy_message_types(path):
    """Keys of MESSAGE_SCHEMAS in message_schemas.py, read with ast (no import, no side effects)."""
    constants, keys = {}, set()
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)):
            continue
        name, value = node.targets[0].id, node.value
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            constants[name] = value.value
        elif name == "MESSAGE_SCHEMAS" and isinstance(value, ast.Dict):
            for key in value.keys:
                if isinstance(key, ast.Name):
                    keys.add(constants[key.id])
                elif isinstance(key, ast.Constant):
                    keys.add(key.value)
    return keys


def legacy_destinations(icd):
    """legacy type -> list of places it went."""
    dest = {}
    for msg in icd.messages:
        for t in tokens(msg.get("replaces")):
            dest.setdefault(t, []).append(msg["id"])
    for block in icd.blocks:
        for t in tokens(block.get("replaces")):
            dest.setdefault(t, []).append(f"block {block['id']}")
    for mc in icd.mode_codes:
        for t in tokens(mc.get("replaces")):
            dest.setdefault(t, []).append(f"mode code {mc['code']}")
    for lg in icd.legacy:
        dest.setdefault(lg["type"], []).append(f"{lg['disposition']}: {lg['reason']}")
    return dest


def check_legacy(icd):
    if not LEGACY_SCHEMAS.exists():
        icd.error("legacy", f"{LEGACY_SCHEMAS} not found; cannot check coverage")
        return
    legacy = legacy_message_types(LEGACY_SCHEMAS)
    dest = legacy_destinations(icd)
    for t in sorted(legacy - dest.keys()):
        icd.error("legacy", f"'{t}' from message_schemas.py is not mapped anywhere")
    for t in sorted(dest.keys() - legacy):
        icd.error("legacy", f"'{t}' is mapped but is not a MESSAGE_SCHEMAS type (typo?)")
    icd.legacy_count = len(legacy)


def check(icd):
    check_enums(icd)
    check_terminals(icd)
    for msg in icd.messages:
        check_message(icd, msg)
    check_addresses(icd)
    check_legacy(icd)
    return not icd.errors


# --------------------------------------------------------------------- rendering

def md_escape(text):
    return (text or "").replace("|", "\\|").replace("\n", " ")


def scaling(field):
    parts = []
    if field.get("enum"):
        parts.append(f"enum `{field['enum']}`")
    if field.get("lsb"):
        lsb = Fraction(field["lsb"])
        shown = field["lsb"] if "/" in field["lsb"] else f"{float(lsb):g}"
        parts.append(f"LSB {shown} {field.get('units', '')}".strip())
    if field.get("min") is not None or field.get("max") is not None:
        parts.append(f"range {field.get('min', '?')} to {field.get('max', '?')}")
    if field.get("type") in ("uint", "sint") and not field.get("lsb"):
        parts.append("raw counts")
    if field.get("type") == "ssm":
        parts.append(f"validity of `{field.get('for')}`")
    return ", ".join(parts)


def where_text(icd, msg):
    rts = "every terminal" if msg["rt"] == "all" else f"RT {msg['rt']}"
    direction = "transmit (terminal to bus)" if msg["dir"] == "T" else "receive (bus to terminal)"
    rate = "aperiodic" if msg["rate"] == "aperiodic" else f"{msg['rate']} Hz"
    return f"{rts} · SA {msg['sa']} · {direction} · {msg.get('words')} words · {rate}"


def render(icd):
    root = icd.root
    out = []
    w = out.append
    w("# FMOFP 1553B ICD — tables")
    w("")
    w(f"> Generated from `fmofp_icd.xml` by `tools/icd_tool.py render`. **Do not edit by hand.** "
      f"ICD version {root.get('version_major')}.{root.get('version_minor')} ({root.get('status')}, "
      f"{root.get('date')}). Conventions, timing and transport are in [README.md](README.md).")
    w("")

    w("## Terminals")
    w("")
    w("| RT | ID | Name | Status | Source | Note |")
    w("|---|---|---|---|---|---|")
    for rt in sorted(icd.terminals):
        t = icd.terminals[rt]
        w(f"| {rt} | `{t['id']}` | {md_escape(t.get('name'))} | {t.get('status')} | "
          f"{md_escape(t.get('source'))} | {md_escape(t.get('note'))} |")
    w("")
    w("RT 31 is broadcast. Unused: " + ", ".join(
        str(rt) for rt in range(MAX_RT + 1) if rt not in icd.terminals) + ".")
    w("")

    w("## Message summary")
    w("")
    w("| ID | RT | SA | Dir | Words | Rate (Hz) | Title | Status |")
    w("|---|---|---|---|---|---|---|---|")
    for msg in sorted(icd.messages, key=lambda m: (999 if m["rt"] == "all" else int(m["rt"]), int(m["sa"]), m["dir"])):
        w(f"| [`{msg['id']}`](#{msg['id'].lower()}) | {msg['rt']} | {msg['sa']} | {msg['dir']} | "
          f"{msg.get('words')} | {msg['rate']} | {md_escape(msg.get('title'))} | {msg.get('status', 'DRAFT')} |")
    w("")

    w("## Block transfers")
    w("")
    w("Layout of the descriptor and segment messages is in README.md. Payload layouts come in v2.")
    w("")
    w("| ID | RT | Descriptor SA | Segment SA | Rate (Hz) | Title | Payload |")
    w("|---|---|---|---|---|---|---|")
    for b in icd.blocks:
        w(f"| `{b['id']}` | {b['rt']} | {b['desc_sa']} | {b['seg_sa']} | {b['rate']} | "
          f"{md_escape(b.get('title'))} | {b.get('payload')} |")
    w("")

    w("## Mode codes")
    w("")
    w("| Code | Name | Dir | Data words | Broadcast | Use |")
    w("|---|---|---|---|---|---|")
    for mc in icd.mode_codes:
        w(f"| {mc['code']} | {mc['name']} | {mc['dir']} | {mc['data_words']} | "
          f"{mc.get('broadcast_allowed')} | {md_escape(mc.get('doc'))} |")
    w("")

    w("## Enumerations")
    w("")
    for eid, enum in icd.enums.items():
        w(f"### `{eid}` ({enum['bits']} bits)")
        w("")
        if enum.get("doc"):
            w(md_escape(enum["doc"]))
            w("")
        w("| Value | Name | Legacy value | Note |")
        w("|---|---|---|---|")
        for name, value, legacy, doc in enum["values"]:
            w(f"| {value} | {name} | {legacy if legacy is not None else ''} | {md_escape(doc)} |")
        w("")

    w("## Messages")
    w("")
    for msg in icd.messages:
        w(f"### {msg['id']}")
        w("")
        w(f"**{md_escape(msg.get('title'))}** — {where_text(icd, msg)}"
          + (f" · **{msg['status']}**" if msg.get("status") else ""))
        w("")
        if msg.get("doc"):
            w(md_escape(msg["doc"]))
            w("")
        if msg.get("replaces"):
            w("Replaces: " + ", ".join(f"`{t}`" for t in tokens(msg["replaces"])))
            w("")
        if msg.get("layout_of"):
            w(f"Same layout as [`{msg['layout_of']}`](#{msg['layout_of'].lower()}).")
            w("")
            continue
        w("| Word | Bits | Field | Type | Scaling | Notes |")
        w("|---|---|---|---|---|---|")
        for f in msg["fields"]:
            w(f"| {f['word']} | {f['bits']} | `{f['name']}` | {f['type']} | {md_escape(scaling(f))} | "
              f"{md_escape(f.get('doc'))} |")
        w("")

    w("## Where every legacy message type went")
    w("")
    w(f"All {getattr(icd, 'legacy_count', '?')} `MESSAGE_SCHEMAS` types from "
      "`FMOFP/MIL_STD_1553B/message_schemas.py`.")
    w("")
    w("| Legacy type | Now |")
    w("|---|---|")
    for t, places in sorted(legacy_destinations(icd).items(), key=lambda kv: kv[0].lower()):
        w(f"| `{t}` | {md_escape('; '.join(places))} |")
    w("")
    return "\n".join(out)


# --------------------------------------------------------------------- C++ generation

def pascal(identifier):
    """RDR_WX_STATUS -> RdrWxStatus, terminal_state -> TerminalState."""
    return "".join(part.capitalize() for part in identifier.lower().split("_"))


def enum_type(enum_id):
    return pascal(enum_id)


def cpp_int_type(ftype, width):
    bits = 8 if width <= 8 else 16
    return f"std::{'u' if ftype == 'uint' else ''}int{bits}_t"


def raw_limits(field, width):
    """(raw_min, raw_max, lsb or None) for a uint/sint field.

    Limits come from the field's min/max in engineering units, rounded to the
    nearest raw count, and never exceed what the bit width can hold.
    """
    signed = field["type"] == "sint"
    width_min = -(1 << (width - 1)) if signed else 0
    width_max = (1 << (width - 1)) - 1 if signed else (1 << width) - 1
    if field.get("lsb") is None:
        return width_min, width_max, None
    lsb = Fraction(field["lsb"])
    raw_min = width_min if field.get("min") is None else max(width_min, round(Fraction(field["min"]) / lsb))
    raw_max = width_max if field.get("max") is None else min(width_max, round(Fraction(field["max"]) / lsb))
    return raw_min, raw_max, lsb


def member_type(field, width):
    ftype = field["type"]
    if ftype == "bool":
        return "bool"
    if ftype in ("enum", "ssm"):
        return enum_type(field["enum"] if ftype == "enum" else "ssm")
    if field.get("lsb") is not None:
        return "double"
    return cpp_int_type(ftype, width)


def member_comment(field):
    parts = []
    if field.get("units"):
        parts.append(field["units"])
    if field.get("lsb"):
        parts.append(f"LSB {field['lsb']}")
    if field.get("min") is not None or field.get("max") is not None:
        parts.append(f"{field.get('min', '')} .. {field.get('max', '')}")
    if field["type"] == "ssm":
        parts.append(f"validity of {field['for']}")
    if field.get("doc"):
        parts.append(field["doc"])
    return ", ".join(parts)


def cpp_header(icd, what):
    root = icd.root
    return [
        f"// {what}",
        "//",
        f"// GENERATED by B20SS/icd/tools/icd_tool.py gen-cpp from fmofp_icd.xml",
        f"// (ICD {root.get('version_major')}.{root.get('version_minor')} {root.get('status')}, "
        f"{root.get('date')}). Do not edit; change the ICD and rebuild.",
        "",
    ]


def gen_version_hpp(icd):
    root = icd.root
    out = cpp_header(icd, "The ICD version this code was generated from; TERM_STATUS reports it.")
    out += [
        "#ifndef FMOFP_ICD_VERSION_HPP",
        "#define FMOFP_ICD_VERSION_HPP",
        "",
        "#include <cstdint>",
        "",
        "namespace fmofp::icd {",
        "",
        f"constexpr std::uint8_t kIcdMajor = {int(root.get('version_major'))}U;",
        f"constexpr std::uint8_t kIcdMinor = {int(root.get('version_minor'))}U;",
        f"constexpr const char* kIcdStatus = \"{root.get('status')}\";",
        f"constexpr const char* kIcdDate = \"{root.get('date')}\";",
        "",
        "}  // namespace fmofp::icd",
        "",
        "#endif",
    ]
    return "\n".join(out) + "\n"


def gen_enums_hpp(icd):
    out = cpp_header(icd, "Enumerations of the FMOFP bus ICD.")
    out += [
        "#ifndef FMOFP_ICD_ENUMS_HPP",
        "#define FMOFP_ICD_ENUMS_HPP",
        "",
        "#include <cstdint>",
        "",
        "#include <fmofp/bus/word.hpp>",
        "",
        "namespace fmofp::icd {",
        "",
        "// Enumerator names mirror the ICD exactly (ICD_TABLES.md, Enumerations).",
        "// is_valid() says whether a value is one the ICD defines; decode() rejects",
        "// anything else with Status::kInvalidEnum.",
    ]
    for eid, enum in icd.enums.items():
        name = enum_type(eid)
        out.append("")
        if enum.get("doc"):
            out.append(f"// {enum['doc']}")
        out.append(f"enum class {name} : std::uint8_t {{")
        for vname, value, _, vdoc in enum["values"]:
            comment = f"  // {vdoc}" if vdoc else ""
            out.append(f"    {vname} = {value}U,{comment}")
        out.append("};")
        out.append("")
        out.append(f"constexpr bool is_valid({name} v) noexcept {{")
        out.append("    switch (v) {")
        for vname, _, _, _ in enum["values"]:
            out.append(f"        case {name}::{vname}:")
        out.append("            return true;")
        out.append("    }")
        out.append("    return false;")
        out.append("}")
        out.append("")
        out.append(f"constexpr bool is_valid_raw_{eid}(bus::Word raw) noexcept {{")
        out.append(f"    return raw <= 0xFFU && is_valid(static_cast<{name}>(raw));")
        out.append("}")
        out.append("")
        out.append(f"constexpr const char* to_string({name} v) noexcept {{")
        out.append("    switch (v) {")
        for vname, _, _, _ in enum["values"]:
            out.append(f"        case {name}::{vname}:")
            out.append(f"            return \"{vname}\";")
        out.append("    }")
        out.append("    return \"?\";")
        out.append("}")
    out += ["", "}  // namespace fmofp::icd", "", "#endif"]
    return "\n".join(out) + "\n"


def field_geometry(field):
    hi, lo = parse_bits(field["bits"])
    return hi, lo, hi - lo + 1


def emit_encode(field):
    """Lines that place one field into w[]; `result` accumulates the Status."""
    name, ftype = field["name"], field["type"]
    if ftype == "spare":
        return []
    hi, lo, width = field_geometry(field)
    word = f"w[{field['word']}]"
    if ftype == "bool":
        return [f"    {word} = bus::set_bit({word}, {hi}U, {name});"]
    if ftype in ("enum", "ssm"):
        return [
            f"    if (!is_valid({name})) {{",
            "        result = worst(result, Status::kInvalidArgument);",
            "    }",
            f"    {word} = bus::set_field({word}, {hi}U, {lo}U, static_cast<bus::Word>({name}));",
        ]
    raw_min, raw_max, lsb = raw_limits(field, width)
    if lsb is not None:
        placed = f"bus::to_twos(raw, {width}U)" if ftype == "sint" else "static_cast<bus::Word>(raw)"
        return [
            "    {",
            "        std::int32_t raw = 0;",
            f"        result = worst(result, bus::encode_scaled({name}, {lsb.numerator}, {lsb.denominator}, "
            f"{raw_min}, {raw_max}, raw));",
            f"        {word} = bus::set_field({word}, {hi}U, {lo}U, {placed});",
            "    }",
        ]
    if ftype == "uint":
        if width == 16:
            return [f"    {word} = bus::set_field({word}, {hi}U, {lo}U, {name});"]
        return [
            "    {",
            f"        const std::uint32_t v = {name};",
            f"        if (v > {raw_max}U) {{",
            "            result = worst(result, Status::kClamped);",
            "        }",
            f"        {word} = bus::set_field({word}, {hi}U, {lo}U, "
            f"static_cast<bus::Word>(v > {raw_max}U ? {raw_max}U : v));",
            "    }",
        ]
    return [
        "    {",
        f"        std::int32_t v = {name};",
        f"        if (v < {raw_min} || v > {raw_max}) {{",
        "            result = worst(result, Status::kClamped);",
        f"            v = v < {raw_min} ? {raw_min} : {raw_max};",
        "        }",
        f"        {word} = bus::set_field({word}, {hi}U, {lo}U, bus::to_twos(v, {width}U));",
        "    }",
    ]


def emit_decode(field):
    """Lines that read one field from in[] into m; returns early on a bad enum."""
    name, ftype = field["name"], field["type"]
    if ftype == "spare":
        return []
    hi, lo, width = field_geometry(field)
    word = f"in[{field['word']}]"
    get = f"bus::get_field({word}, {hi}U, {lo}U)"
    if ftype == "bool":
        return [f"    m.{name} = bus::get_bit({word}, {hi}U);"]
    if ftype in ("enum", "ssm"):
        eid = field["enum"] if ftype == "enum" else "ssm"
        return [
            "    {",
            f"        const bus::Word raw = {get};",
            f"        if (!is_valid_raw_{eid}(raw)) {{",
            "            return Status::kInvalidEnum;",
            "        }",
            f"        m.{name} = static_cast<{enum_type(eid)}>(raw);",
            "    }",
        ]
    _, _, lsb = raw_limits(field, width)
    if lsb is not None:
        raw = f"bus::from_twos({get}, {width}U)" if ftype == "sint" else f"static_cast<std::int32_t>({get})"
        return [f"    m.{name} = bus::decode_scaled({raw}, {lsb.numerator}, {lsb.denominator});"]
    ctype = cpp_int_type(ftype, width)
    if ftype == "uint":
        return [f"    m.{name} = static_cast<{ctype}>({get});"]
    return [f"    m.{name} = static_cast<{ctype}>(bus::from_twos({get}, {width}U));"]


def message_constants(msg, indent="    "):
    rt = "kEveryTerminal" if msg["rt"] == "all" else f"{int(msg['rt'])}U"
    direction = "Direction::kTransmit" if msg["dir"] == "T" else "Direction::kReceive"
    rate = "kAperiodic" if msg["rate"] == "aperiodic" else f"{int(msg['rate'])}U"
    return [
        f"{indent}static constexpr std::uint8_t kRt = {rt};",
        f"{indent}static constexpr std::uint8_t kSubaddress = {int(msg['sa'])}U;",
        f"{indent}static constexpr Direction kDirection = {direction};",
        f"{indent}static constexpr std::uint8_t kRateHz = {rate};",
    ]


def gen_messages_hpp(icd):
    out = cpp_header(icd, "Messages of the FMOFP bus ICD: one struct per message with encode() and decode().")
    out += [
        "#ifndef FMOFP_ICD_MESSAGES_HPP",
        "#define FMOFP_ICD_MESSAGES_HPP",
        "",
        "#include <array>",
        "#include <cstdint>",
        "",
        "#include <fmofp/bus/status.hpp>",
        "#include <fmofp/bus/word.hpp>",
        "#include <fmofp/icd/enums.hpp>",
        "",
        "namespace fmofp::icd {",
        "",
        "enum class Direction : std::uint8_t { kTransmit, kReceive };",
        "",
        "// kRt of a message that every terminal carries (TERM_STATUS, TERM_CMD).",
        "constexpr std::uint8_t kEveryTerminal = 0xFFU;",
        "// kRateHz of a message sent only on demand.",
        "constexpr std::uint8_t kAperiodic = 0U;",
        "",
        "// encode(): fills `out` from the members. Returns kOk; kClamped if a value",
        "// had to be limited to its field's range (then mark that field's SSM",
        "// NO_COMPUTED_DATA); kInvalidArgument for NaN/infinity or an enum value",
        "// the ICD does not define. `out` is always fully written.",
        "// decode(): fills `out` from the words. Returns kOk, or kInvalidEnum",
        "// (leaving `out` untouched) if an enum field holds an undefined value.",
    ]
    for msg in icd.messages:
        name = pascal(msg["id"])
        out.append("")
        out.append(f"// {msg['id']} - {msg.get('title', '')}")
        out.append(f"// {where_text(icd, msg)}" + (f" - {msg['status']}" if msg.get("status") else ""))
        if msg.get("doc"):
            out.append(f"// {msg['doc']}")
        if msg.get("layout_of"):
            base = pascal(msg["layout_of"])
            out.append(f"// Same word layout as {msg['layout_of']}, carried in `payload`; its own address and rate.")
            out.append(f"struct {name} {{")
            out += message_constants(msg)
            out.append(f"    using Layout = {base};")
            out.append("    static constexpr std::uint8_t kWords = Layout::kWords;")
            out.append("    using Words = Layout::Words;")
            out.append("")
            out.append("    Layout payload{};")
            out.append("")
            out.append("    [[nodiscard]] Status encode(Words& out) const noexcept { return payload.encode(out); }")
            out.append(f"    [[nodiscard]] static Status decode(const Words& in, {name}& out) noexcept {{")
            out.append("        return Layout::decode(in, out.payload);")
            out.append("    }")
            out.append("};")
            continue
        out.append(f"struct {name} {{")
        out += message_constants(msg)
        out.append(f"    static constexpr std::uint8_t kWords = {int(msg['words'])}U;")
        out.append("    using Words = std::array<bus::Word, kWords>;")
        out.append("")
        for field in msg["fields"]:
            if field["type"] == "spare":
                continue
            _, _, width = field_geometry(field)
            comment = member_comment(field)
            out.append(f"    {member_type(field, width)} {field['name']}{{}};" + (f"  // {comment}" if comment else ""))
        out.append("")
        out.append("    [[nodiscard]] Status encode(Words& out) const noexcept;")
        out.append(f"    [[nodiscard]] static Status decode(const Words& in, {name}& out) noexcept;")
        out.append("};")
        out.append("")
        out.append(f"inline Status {name}::encode(Words& out) const noexcept {{")
        out.append("    Words w{};")
        out.append("    Status result = Status::kOk;")
        for field in msg["fields"]:
            out += emit_encode(field)
        out.append("    out = w;")
        out.append("    return result;")
        out.append("}")
        out.append("")
        out.append(f"inline Status {name}::decode(const Words& in, {name}& out) noexcept {{")
        out.append(f"    {name} m{{}};")
        for field in msg["fields"]:
            out += emit_decode(field)
        out.append("    out = m;")
        out.append("    return Status::kOk;")
        out.append("}")

    out += [
        "",
        "// Every message in the ICD, for schedule building and tests.",
        "struct MessageInfo {",
        "    const char* id;",
        "    std::uint8_t rt;  // kEveryTerminal for TERM_STATUS / TERM_CMD",
        "    std::uint8_t subaddress;",
        "    Direction direction;",
        "    std::uint8_t words;",
        "    std::uint8_t rate_hz;  // kAperiodic for on-demand messages",
        "};",
        "",
        f"inline constexpr std::array<MessageInfo, {len(icd.messages)}> kMessages = {{{{",
    ]
    for msg in icd.messages:
        name = pascal(msg["id"])
        out.append(f"    {{\"{msg['id']}\", {name}::kRt, {name}::kSubaddress, {name}::kDirection, "
                   f"{name}::kWords, {name}::kRateHz}},")
    out += ["}};", "", "}  // namespace fmofp::icd", "", "#endif"]
    return "\n".join(out) + "\n"


def probe_value(icd, field):
    """A representative non-zero value for a field, and how the test compares it."""
    ftype = field["type"]
    _, _, width = field_geometry(field)
    if ftype == "bool":
        return "true", None
    if ftype in ("enum", "ssm"):
        eid = field["enum"] if ftype == "enum" else "ssm"
        values = icd.enums[eid]["values"]
        pick = values[1] if len(values) > 1 else values[0]
        if eid == "ssm":
            pick = next(v for v in values if v[0] == "NORMAL")
        return f"{enum_type(eid)}::{pick[0]}", None
    raw_min, raw_max, lsb = raw_limits(field, width)
    if lsb is not None:
        raw = raw_min + (raw_max - raw_min) * 5 // 8
        return repr(float(Fraction(raw) * lsb)), float(lsb) * 0.5 + 1e-9
    if ftype == "uint":
        return f"{0xA5A5 & ((1 << width) - 1)}U", None
    return str(-(1 << (width - 2))) if width >= 2 else "0", None


def undefined_enum_raw(icd, field):
    """A raw value that fits the field's bits but is not an enumerator, or None if every value is defined."""
    eid = field["enum"] if field["type"] == "enum" else "ssm"
    _, _, width = field_geometry(field)
    defined = {value for _, value, _, _ in icd.enums[eid]["values"]}
    return next((raw for raw in range(1 << width) if raw not in defined), None)


def gen_roundtrip_test(icd):
    out = cpp_header(icd, "Tests for every generated enum and message: round trips, fail-safe zeros, "
                          "and rejection of undefined enum values.")
    out += [
        "#include <gtest/gtest.h>",
        "",
        "#include <fmofp/icd/messages.hpp>",
        "",
        "namespace {",
        "",
        "using fmofp::Status;",
        "using fmofp::bus::Word;",
        "using namespace fmofp::icd;  // NOLINT: generated test",
    ]
    for eid, enum in icd.enums.items():
        name = enum_type(eid)
        out += ["", f"TEST(IcdEnum, {name}) {{"]
        for vname, value, _, _ in enum["values"]:
            out.append(f"    EXPECT_TRUE(is_valid({name}::{vname}));")
            out.append(f"    EXPECT_TRUE(is_valid_raw_{eid}(Word{{{value}U}}));")
            out.append(f"    EXPECT_STREQ(\"{vname}\", to_string({name}::{vname}));")
        defined = {value for _, value, _, _ in enum["values"]}
        for raw in range(1 << enum["bits"]):
            if raw not in defined:
                out.append(f"    EXPECT_FALSE(is_valid_raw_{eid}(Word{{{raw}U}}));")
                out.append(f"    EXPECT_FALSE(is_valid(static_cast<{name}>({raw}U)));")
                out.append(f"    EXPECT_STREQ(\"?\", to_string(static_cast<{name}>({raw}U)));")
                break
        out.append("    EXPECT_FALSE(is_valid_raw_%s(Word{0x100U}));" % eid)
        out.append("}")
    for msg in icd.messages:
        if msg.get("layout_of"):
            continue
        name = pascal(msg["id"])
        fields = [f for f in msg["fields"] if f["type"] != "spare"]
        out += [
            "",
            f"TEST(IcdRoundTrip, {name}) {{",
            f"    {name} m{{}};",
        ]
        comparisons = []
        for field in fields:
            value, tol = probe_value(icd, field)
            out.append(f"    m.{field['name']} = {value};")
            if tol is None:
                comparisons.append(f"    EXPECT_EQ(m.{field['name']}, d.{field['name']});")
            else:
                comparisons.append(f"    EXPECT_NEAR(m.{field['name']}, d.{field['name']}, {tol!r});")
        out += [
            f"    {name}::Words w{{}};",
            "    ASSERT_EQ(Status::kOk, m.encode(w));",
            f"    {name} d{{}};",
            f"    ASSERT_EQ(Status::kOk, {name}::decode(w, d));",
        ]
        out += comparisons
        out += [
            f"    {name}::Words again{{}};",
            "    ASSERT_EQ(Status::kOk, d.encode(again));",
            "    EXPECT_EQ(w, again);",
            "}",
            "",
            f"TEST(IcdZeroWords, {name}) {{",
            "    // All-zero words are the fail-safe state and must always decode.",
            f"    const {name}::Words zero{{}};",
            f"    {name} d{{}};",
            f"    EXPECT_EQ(Status::kOk, {name}::decode(zero, d));",
            "}",
        ]
        # The first enum field with an undefined raw value available gives a
        # corrupted-word test for the message; encode must refuse the same value.
        for field in fields:
            if field["type"] not in ("enum", "ssm"):
                continue
            raw = undefined_enum_raw(icd, field)
            if raw is None:
                continue
            hi, lo, _ = field_geometry(field)
            etype = enum_type(field["enum"] if field["type"] == "enum" else "ssm")
            out += [
                "",
                f"TEST(IcdUndefinedEnum, {name}) {{",
                f"    // {field['name']} holds {raw}, which the ICD does not define.",
                f"    {name}::Words w{{}};",
                f"    w[{field['word']}] = fmofp::bus::set_field(w[{field['word']}], {hi}U, {lo}U, Word{{{raw}U}});",
                f"    {name} d{{}};",
                f"    EXPECT_EQ(Status::kInvalidEnum, {name}::decode(w, d));",
                f"    {name} m{{}};",
                f"    m.{field['name']} = static_cast<{etype}>({raw}U);",
                f"    {name}::Words out{{}};",
                "    EXPECT_EQ(Status::kInvalidArgument, m.encode(out));",
                "}",
            ]
            break
    out += ["", "}  // namespace"]
    return "\n".join(out) + "\n"


def gen_cpp(icd, out_dir):
    include = out_dir / "include" / "fmofp" / "icd"
    include.mkdir(parents=True, exist_ok=True)
    written = []
    for path, text in (
        (include / "version.hpp", gen_version_hpp(icd)),
        (include / "enums.hpp", gen_enums_hpp(icd)),
        (include / "messages.hpp", gen_messages_hpp(icd)),
        (out_dir / "icd_roundtrip_test.cpp", gen_roundtrip_test(icd)),
    ):
        path.write_text(text, encoding="utf-8", newline="\n")
        written.append(path)
    return written


# --------------------------------------------------------------------- Python generation

PY_OUT_DIR = ICD_DIR.parent / "fmofp_bus" / "icd"


def py_header(icd, what):
    root = icd.root
    return [
        f'"""{what}',
        "",
        "GENERATED by B20SS/icd/tools/icd_tool.py gen-py from fmofp_icd.xml",
        f"(ICD {root.get('version_major')}.{root.get('version_minor')} {root.get('status')}, "
        f"{root.get('date')}). Do not edit; change the ICD and run `icd_tool.py gen-py`.",
        '"""',
        "",
    ]


def py_enum_default(icd, eid):
    """The enumerator with value 0: the fail-safe default every field starts in."""
    for name, value, _, _ in icd.enums[eid]["values"]:
        if value == 0:
            return f"{enum_type(eid)}.{name}"
    raise ValueError(f"enum {eid} has no zero")


def gen_py_enums(icd):
    out = py_header(icd, "Enumerations of the FMOFP bus ICD. Enumerator names mirror the ICD exactly.")
    out += ["import enum", ""]
    for eid, enum in icd.enums.items():
        out.append("")
        out.append(f"class {enum_type(eid)}(enum.IntEnum):")
        if enum.get("doc"):
            out.append(f'    """{enum["doc"]}"""')
        for vname, value, _, vdoc in enum["values"]:
            comment = f"  # {vdoc}" if vdoc else ""
            out.append(f"    {vname} = {value}{comment}")
        out.append("")
    out += [
        "",
        "def is_valid(enum_cls, value):",
        '    """True if `value` is one of the enumerators the ICD defines for enum_cls."""',
        "    try:",
        "        enum_cls(value)",
        "    except ValueError:",
        "        return False",
        "    return True",
    ]
    return "\n".join(out) + "\n"


def py_default(icd, field):
    ftype = field["type"]
    if ftype == "bool":
        return "False"
    if ftype in ("enum", "ssm"):
        return py_enum_default(icd, field["enum"] if ftype == "enum" else "ssm")
    return "0.0" if field.get("lsb") is not None else "0"


def py_annotation(icd, field):
    ftype = field["type"]
    if ftype == "bool":
        return "bool"
    if ftype in ("enum", "ssm"):
        return enum_type(field["enum"] if ftype == "enum" else "ssm")
    return "float" if field.get("lsb") is not None else "int"


def py_emit_encode(field):
    name, ftype = field["name"], field["type"]
    if ftype == "spare":
        return []
    hi, lo, width = field_geometry(field)
    word = f"w[{field['word']}]"
    if ftype == "bool":
        return [f"        {word} = _w.set_bit({word}, {hi}, bool(self.{name}))"]
    if ftype in ("enum", "ssm"):
        etype = enum_type(field["enum"] if ftype == "enum" else "ssm")
        return [
            f"        if not is_valid({etype}, self.{name}):",
            "            result = worst(result, Status.INVALID_ARGUMENT)",
            f"        {word} = _w.set_field({word}, {hi}, {lo}, int(self.{name}))",
        ]
    raw_min, raw_max, lsb = raw_limits(field, width)
    if lsb is not None:
        placed = f"_w.to_twos(raw, {width})" if ftype == "sint" else "raw"
        return [
            f"        s, raw = _w.encode_scaled(self.{name}, {lsb.numerator}, {lsb.denominator}, {raw_min}, {raw_max})",
            "        result = worst(result, s)",
            f"        {word} = _w.set_field({word}, {hi}, {lo}, {placed})",
        ]
    if ftype == "uint":
        return [
            f"        s, v = _w.check_uint(self.{name}, {width})",
            "        result = worst(result, s)",
            f"        {word} = _w.set_field({word}, {hi}, {lo}, v)",
        ]
    return [
        f"        s, v = _w.check_sint(self.{name}, {width})",
        "        result = worst(result, s)",
        f"        {word} = _w.set_field({word}, {hi}, {lo}, _w.to_twos(v, {width}))",
    ]


def py_emit_decode(field):
    name, ftype = field["name"], field["type"]
    if ftype == "spare":
        return []
    hi, lo, width = field_geometry(field)
    get = f"_w.get_field(words[{field['word']}], {hi}, {lo})"
    if ftype == "bool":
        return [f"        m.{name} = _w.get_bit(words[{field['word']}], {hi})"]
    if ftype in ("enum", "ssm"):
        etype = enum_type(field["enum"] if ftype == "enum" else "ssm")
        return [
            f"        raw = {get}",
            f"        if not is_valid({etype}, raw):",
            "            return Status.INVALID_ENUM, None",
            f"        m.{name} = {etype}(raw)",
        ]
    _, _, lsb = raw_limits(field, width)
    if lsb is not None:
        raw = f"_w.from_twos({get}, {width})" if ftype == "sint" else get
        return [f"        m.{name} = _w.decode_scaled({raw}, {lsb.numerator}, {lsb.denominator})"]
    if ftype == "uint":
        return [f"        m.{name} = {get}"]
    return [f"        m.{name} = _w.from_twos({get}, {width})"]


def py_constants(msg):
    rt = "EVERY_TERMINAL" if msg["rt"] == "all" else str(int(msg["rt"]))
    rate = "APERIODIC" if msg["rate"] == "aperiodic" else str(int(msg["rate"]))
    return [
        f"    RT: ClassVar[int] = {rt}",
        f"    SUBADDRESS: ClassVar[int] = {int(msg['sa'])}",
        f"    DIRECTION: ClassVar[str] = \"{msg['dir']}\"",
        f"    RATE_HZ: ClassVar[int] = {rate}",
    ]


def gen_py_messages(icd):
    root = icd.root
    out = py_header(icd, "Messages of the FMOFP bus ICD: one dataclass per message with encode() and decode().")
    out += [
        "from collections import namedtuple",
        "from dataclasses import dataclass, field",
        "from typing import ClassVar, List, Optional, Tuple",
        "",
        "from fmofp_bus import word as _w",
        "from fmofp_bus.icd.enums import *  # noqa: F401,F403 - the enums are part of this interface",
        "from fmofp_bus.icd.enums import is_valid",
        "from fmofp_bus.status import Status, worst",
        "",
        f"ICD_MAJOR = {int(root.get('version_major'))}",
        f"ICD_MINOR = {int(root.get('version_minor'))}",
        f"ICD_STATUS = \"{root.get('status')}\"",
        f"ICD_DATE = \"{root.get('date')}\"",
        "",
        "# RT of a message that every terminal carries (TERM_STATUS, TERM_CMD).",
        "EVERY_TERMINAL = 0xFF",
        "# RATE_HZ of a message sent only on demand.",
        "APERIODIC = 0",
        "",
        "# encode() -> (Status, words): OK; CLAMPED if a value was limited to its",
        "# field's range (mark that field's SSM NO_COMPUTED_DATA); INVALID_ARGUMENT",
        "# for NaN/infinity or an undefined enum value. The words are always complete.",
        "# decode(words) -> (Status, message or None): BAD_LENGTH for the wrong",
        "# number of words, INVALID_ENUM for an undefined enum value.",
    ]
    for msg in icd.messages:
        name = pascal(msg["id"])
        out.append("")
        out.append("")
        out.append("@dataclass")
        out.append(f"class {name}:")
        title = msg.get("title", "")
        out.append(f'    """{msg["id"]} - {title}. {where_text(icd, msg)}.')
        if msg.get("doc"):
            out.append("")
            out.append(f"    {msg['doc']}")
        out.append('    """')
        out += py_constants(msg)
        if msg.get("layout_of"):
            base = pascal(msg["layout_of"])
            out.append(f"    WORDS: ClassVar[int] = {base}.WORDS")
            out.append("")
            out.append(f"    payload: {base} = field(default_factory={base})")
            out.append("")
            out.append("    def encode(self) -> Tuple[Status, List[int]]:")
            out.append("        return self.payload.encode()")
            out.append("")
            out.append("    @classmethod")
            out.append(f"    def decode(cls, words) -> Tuple[Status, Optional[\"{name}\"]]:")
            out.append(f"        status, payload = {base}.decode(words)")
            out.append("        return status, (cls(payload=payload) if payload is not None else None)")
            continue
        out.append(f"    WORDS: ClassVar[int] = {int(msg['words'])}")
        out.append("")
        fields = [f for f in msg["fields"] if f["type"] != "spare"]
        for f in fields:
            comment = member_comment(f)
            out.append(f"    {f['name']}: {py_annotation(icd, f)} = {py_default(icd, f)}" + (f"  # {comment}" if comment else ""))
        out.append("")
        out.append("    def encode(self) -> Tuple[Status, List[int]]:")
        out.append(f"        w = [0] * {int(msg['words'])}")
        out.append("        result = Status.OK")
        for f in msg["fields"]:
            out += py_emit_encode(f)
        out.append("        return result, w")
        out.append("")
        out.append("    @classmethod")
        out.append(f"    def decode(cls, words) -> Tuple[Status, Optional[\"{name}\"]]:")
        out.append("        if len(words) != cls.WORDS:")
        out.append("            return Status.BAD_LENGTH, None")
        out.append("        m = cls()")
        for f in msg["fields"]:
            out += py_emit_decode(f)
        out.append("        return Status.OK, m")

    out += [
        "",
        "",
        'MessageInfo = namedtuple("MessageInfo", "id rt subaddress direction words rate_hz cls")',
        "",
        "# Every message in the ICD, in ICD order.",
        "MESSAGES = (",
    ]
    for msg in icd.messages:
        name = pascal(msg["id"])
        out.append(f"    MessageInfo(\"{msg['id']}\", {name}.RT, {name}.SUBADDRESS, {name}.DIRECTION, "
                   f"{name}.WORDS, {name}.RATE_HZ, {name}),")
    out += [
        ")",
        "",
        "MESSAGE_CLASSES = {info.id: info.cls for info in MESSAGES}",
    ]
    return "\n".join(out) + "\n"


def gen_py(icd, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    init = py_header(icd, "Generated message layouts of the FMOFP bus ICD (see enums.py and messages.py).")
    written = []
    for path, text in (
        (out_dir / "__init__.py", "\n".join(init) + "\n"),
        (out_dir / "enums.py", gen_py_enums(icd)),
        (out_dir / "messages.py", gen_py_messages(icd)),
    ):
        path.write_text(text, encoding="utf-8", newline="\n")
        written.append(path)
    return written


# --------------------------------------------------------------------- main

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["check", "render", "gen-cpp", "gen-py"])
    parser.add_argument("--icd", type=Path, default=ICD_XML)
    parser.add_argument("--out", type=Path,
                        help="gen-cpp: directory to write into (required); gen-py: defaults to B20SS/fmofp_bus/icd")
    args = parser.parse_args(argv)
    if args.command == "gen-cpp" and args.out is None:
        parser.error("gen-cpp needs --out DIR")

    icd = load(args.icd)
    ok = check(icd)
    if not ok:
        for err in icd.errors:
            print(f"ERROR {err}")
        print(f"{len(icd.errors)} error(s) in {args.icd.name}")
        return 1

    expanded = sum(len(message_rts(icd, m)) for m in icd.messages)
    print(f"{args.icd.name}: OK - {len(icd.terminals)} terminals, {len(icd.messages)} message definitions "
          f"({expanded} after expanding per-terminal ones), {len(icd.blocks)} block streams, "
          f"{len(icd.mode_codes)} mode codes, {icd.legacy_count} legacy types all mapped")

    if args.command == "render":
        TABLES_MD.write_text(render(icd) + "\n", encoding="utf-8", newline="\n")
        print(f"wrote {TABLES_MD}")
    elif args.command == "gen-cpp":
        for path in gen_cpp(icd, args.out):
            print(f"wrote {path}")
    elif args.command == "gen-py":
        for path in gen_py(icd, args.out or PY_OUT_DIR):
            print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
