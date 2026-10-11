"""The datagram that carries one bus transaction (ICD README section 7).

Mirrors fmofp/bus/transport.hpp: four big-endian header words (magic 0x1553,
frame type and flags, transaction number, word count), then the bus words.
"""

import enum
import struct
from dataclasses import dataclass, field
from typing import List

from fmofp_bus.status import Status

FRAME_MAGIC = 0x1553
FRAME_HEADER_WORDS = 4
MAX_FRAME_WORDS = 36
MAX_FRAME_BYTES = 2 * (FRAME_HEADER_WORDS + MAX_FRAME_WORDS)
FLAG_PARITY_ERROR = 0x01


class FrameType(enum.IntEnum):
    BC_TO_RT = 1
    RT_TO_BC = 2
    MONITOR = 3


@dataclass
class Frame:
    type: FrameType = FrameType.BC_TO_RT
    flags: int = 0
    transaction: int = 0
    words: List[int] = field(default_factory=list)

    @property
    def word_count(self):
        return len(self.words)


def encode_frame(frame):
    """(Status, bytes). INVALID_ARGUMENT for too many words or an unknown type."""
    if len(frame.words) > MAX_FRAME_WORDS or int(frame.type) not in (1, 2, 3):
        return Status.INVALID_ARGUMENT, b""
    header = struct.pack(">HHHH", FRAME_MAGIC, (int(frame.type) << 8) | (frame.flags & 0xFF),
                         frame.transaction & 0xFFFF, len(frame.words))
    body = struct.pack(">%dH" % len(frame.words), *[x & 0xFFFF for x in frame.words])
    return Status.OK, header + body


def decode_frame(data):
    """(Status, Frame or None): BAD_LENGTH, BAD_MAGIC or MALFORMED on damaged bytes."""
    if data is None or len(data) % 2 != 0 or len(data) < 2 * FRAME_HEADER_WORDS:
        return Status.BAD_LENGTH, None
    magic, type_and_flags, transaction, count = struct.unpack_from(">HHHH", data, 0)
    if magic != FRAME_MAGIC:
        return Status.BAD_MAGIC, None
    type_raw = type_and_flags >> 8
    if type_raw not in (1, 2, 3):
        return Status.MALFORMED, None
    if count > MAX_FRAME_WORDS or len(data) != 2 * (FRAME_HEADER_WORDS + count):
        return Status.BAD_LENGTH, None
    words = list(struct.unpack_from(">%dH" % count, data, 2 * FRAME_HEADER_WORDS))
    return Status.OK, Frame(FrameType(type_raw), type_and_flags & 0xFF, transaction, words)
