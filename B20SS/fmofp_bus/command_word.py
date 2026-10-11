"""MIL-STD-1553B command and status words (ICD README section 3).

Mirrors fmofp/bus/command_word.hpp and status_word.hpp.
"""

from dataclasses import dataclass

from fmofp_bus import word as w
from fmofp_bus.status import Status

BROADCAST_RT = 31
MAX_RT = 31
MAX_SUBADDRESS = 31
MAX_DATA_WORDS = 32
MAX_MODE_CODE = 31


@dataclass
class CommandWord:
    rt: int = 0
    transmit: bool = False
    subaddress: int = 0
    # Data word count 1..32 for a data message; the mode code 0..31 when the
    # subaddress is 0 or 31.
    count_or_mode: int = 0

    @property
    def is_mode_code(self):
        return self.subaddress in (0, MAX_SUBADDRESS)

    @property
    def is_broadcast(self):
        return self.rt == BROADCAST_RT


def encode_command_word(cw):
    """(Status, word). INVALID_ARGUMENT for a field out of range."""
    if cw.rt > MAX_RT or cw.subaddress > MAX_SUBADDRESS or cw.rt < 0 or cw.subaddress < 0:
        return Status.INVALID_ARGUMENT, 0
    if cw.is_mode_code:
        if not 0 <= cw.count_or_mode <= MAX_MODE_CODE:
            return Status.INVALID_ARGUMENT, 0
        count_field = cw.count_or_mode
    else:
        if not 1 <= cw.count_or_mode <= MAX_DATA_WORDS:
            return Status.INVALID_ARGUMENT, 0
        count_field = 0 if cw.count_or_mode == MAX_DATA_WORDS else cw.count_or_mode
    word = 0
    word = w.set_field(word, 15, 11, cw.rt)
    word = w.set_bit(word, 10, cw.transmit)
    word = w.set_field(word, 9, 5, cw.subaddress)
    word = w.set_field(word, 4, 0, count_field)
    return Status.OK, word


def decode_command_word(word):
    """Every pattern is a valid command word; a zero count decodes as 32."""
    cw = CommandWord()
    cw.rt = w.get_field(word, 15, 11)
    cw.transmit = w.get_bit(word, 10)
    cw.subaddress = w.get_field(word, 9, 5)
    low = w.get_field(word, 4, 0)
    cw.count_or_mode = MAX_DATA_WORDS if (not cw.is_mode_code and low == 0) else low
    return cw


@dataclass
class StatusWord:
    rt: int = 0
    message_error: bool = False
    service_request: bool = False
    broadcast_received: bool = False
    busy: bool = False
    subsystem_flag: bool = False
    terminal_flag: bool = False


def encode_status_word(sw):
    if not 0 <= sw.rt <= MAX_RT:
        return Status.INVALID_ARGUMENT, 0
    word = 0
    word = w.set_field(word, 15, 11, sw.rt)
    word = w.set_bit(word, 10, sw.message_error)
    word = w.set_bit(word, 8, sw.service_request)
    word = w.set_bit(word, 4, sw.broadcast_received)
    word = w.set_bit(word, 3, sw.busy)
    word = w.set_bit(word, 2, sw.subsystem_flag)
    word = w.set_bit(word, 0, sw.terminal_flag)
    return Status.OK, word


def decode_status_word(word):
    """(Status, StatusWord): MALFORMED if a reserved or always-zero bit is set."""
    sw = StatusWord()
    sw.rt = w.get_field(word, 15, 11)
    sw.message_error = w.get_bit(word, 10)
    sw.service_request = w.get_bit(word, 8)
    sw.broadcast_received = w.get_bit(word, 4)
    sw.busy = w.get_bit(word, 3)
    sw.subsystem_flag = w.get_bit(word, 2)
    sw.terminal_flag = w.get_bit(word, 0)
    reserved = w.get_field(word, 7, 5) != 0 or w.get_bit(word, 9) or w.get_bit(word, 1)
    return (Status.MALFORMED if reserved else Status.OK), sw
