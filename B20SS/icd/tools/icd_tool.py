"""Check and render the FMOFP bus ICD (B20SS/icd/fmofp_icd.xml).

    py B20SS/icd/tools/icd_tool.py check    # validate; exit code 1 on any error
    py B20SS/icd/tools/icd_tool.py render   # check, then regenerate B20SS/icd/ICD_TABLES.md

Standard library only, so it runs anywhere the Python side runs.

`check` enforces the rules the old message_schemas.py could not: every bit of
every word is defined, every enum value fits its field, scaled ranges are
representable, no two messages share an RT/subaddress/direction, and every
legacy message type is accounted for.
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


# --------------------------------------------------------------------- main

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["check", "render"])
    parser.add_argument("--icd", type=Path, default=ICD_XML)
    args = parser.parse_args(argv)

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
    return 0


if __name__ == "__main__":
    sys.exit(main())
