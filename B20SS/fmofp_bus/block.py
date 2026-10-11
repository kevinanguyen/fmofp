"""Block transfers (ICD README section 8).

A payload longer than one message travels as a 4-word descriptor and 32-word
segments on two transmit subaddresses. BlockTransmitter is the producer's side,
BlockReceiver the consumer's; blocks_from_records reassembles what a monitor
saw. The C++ twin is ofp/bus/include/fmofp/bus/block.hpp.
"""

from dataclasses import dataclass

from fmofp_bus.status import Status

DESCRIPTOR_WORDS = 4
SEGMENT_WORDS = 32
SEGMENT_PAYLOAD_WORDS = 31  # word 0 of a segment is version and index
MAX_SEGMENTS = 256
MAX_PAYLOAD_WORDS = MAX_SEGMENTS * SEGMENT_PAYLOAD_WORDS


def crc16_ccitt_false_bytes(data):
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no reflection, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def crc16_ccitt_false(words):
    """The CRC over bus words as big-endian bytes, which is how the C++ computes it."""
    return crc16_ccitt_false_bytes(b"".join(int(w).to_bytes(2, "big") for w in words))


def segment_count(length):
    return (length + SEGMENT_PAYLOAD_WORDS - 1) // SEGMENT_PAYLOAD_WORDS


@dataclass
class BlockDescriptor:
    version: int = 0  # 0 = no block yet; otherwise 1..255, bumped with every payload
    length: int = 0   # payload words
    segments: int = 0
    crc: int = 0

    def encode(self):
        return [(self.version & 0xFF) << 8, self.length & 0xFFFF, self.segments & 0xFFFF, self.crc & 0xFFFF]

    @classmethod
    def decode(cls, words):
        return cls(version=words[0] >> 8, length=words[1], segments=words[2], crc=words[3])


class BlockTransmitter:
    def __init__(self, capacity=MAX_PAYLOAD_WORDS):
        self.capacity = capacity
        self._payload = []
        self.version = 0
        self.crc = 0
        self.next_index = 0

    @property
    def has_payload(self):
        return self.version != 0

    @property
    def length(self):
        return len(self._payload)

    @property
    def segment_count(self):
        return segment_count(len(self._payload))

    def set_payload(self, words):
        """A new payload: bumps the version (1..255, never 0) and starts again at segment 0."""
        if len(words) > self.capacity:
            return Status.BAD_LENGTH
        self._payload = [int(w) & 0xFFFF for w in words]
        self.crc = crc16_ccitt_false(self._payload)
        self.next_index = 0
        self.version = 1 if self.version == 255 else self.version + 1
        return Status.OK

    def descriptor(self):
        return BlockDescriptor(self.version, len(self._payload), self.segment_count, self.crc).encode()

    def segment(self, index):
        """Segment `index`: version and index, then 31 payload words, zero-padded."""
        start = index * SEGMENT_PAYLOAD_WORDS
        body = self._payload[start:start + SEGMENT_PAYLOAD_WORDS] if self.version else []
        return [((self.version & 0xFF) << 8) | (index & 0xFF)] + body + [0] * (SEGMENT_PAYLOAD_WORDS - len(body))

    def next_segment(self):
        return self.segment(self.next_index)

    def advance(self):
        """The bus took the current segment: on to the next, wrapping after the last."""
        n = self.segment_count
        self.next_index = 0 if n == 0 else (self.next_index + 1) % n


class BlockReceiver:
    def __init__(self, capacity=MAX_PAYLOAD_WORDS):
        self.capacity = capacity
        self.completed_blocks = 0
        self._storage = [0] * capacity
        self._restart(0)

    def _restart(self, version):
        self.version = version
        self._descriptor = None
        self._received = set()
        self.complete = False

    @property
    def length(self):
        return self._descriptor.length if self._descriptor else 0

    @property
    def payload(self):
        """The complete payload (a copy), or None."""
        return list(self._storage[:self._descriptor.length]) if self.complete else None

    def feed_descriptor(self, words):
        """True when this completed the block of the version being collected. Version 0 is ignored."""
        d = BlockDescriptor.decode(words)
        if d.version == 0:
            return False
        if d.version != self.version:
            self._restart(d.version)
        self._descriptor = d
        return self._check_complete()

    def feed_segment(self, words):
        version, index = words[0] >> 8, words[0] & 0xFF
        if version == 0:
            return False
        if version != self.version:
            self._restart(version)
        start = index * SEGMENT_PAYLOAD_WORDS
        for i, w in enumerate(words[1:1 + SEGMENT_PAYLOAD_WORDS]):
            if start + i < self.capacity:
                self._storage[start + i] = w
        self._received.add(index)
        return self._check_complete()

    def _check_complete(self):
        was_complete = self.complete
        self.complete = False
        d = self._descriptor
        if d is None or d.length > self.capacity or d.segments != segment_count(d.length) or d.segments > MAX_SEGMENTS:
            return False
        if any(i not in self._received for i in range(d.segments)):
            return False
        self.complete = crc16_ccitt_false(self._storage[:d.length]) == d.crc
        if self.complete and not was_complete:
            self.completed_blocks += 1
            return True
        return False


def blocks_from_records(records, rt, descriptor_sa, segment_sa, capacity=MAX_PAYLOAD_WORDS):
    """Every block a monitor saw complete on one stream, in order: [(record index, version, payload)].

    `records` are MonitorRecords (fmofp_bus.monitor); a transmit of the
    descriptor or segment subaddress that the terminal answered feeds the receiver.
    """
    rx = BlockReceiver(capacity)
    out = []
    for i, record in enumerate(records):
        cw = record.command
        if cw.is_mode_code or not cw.transmit or cw.rt != rt or not record.answered:
            continue
        if cw.subaddress == descriptor_sa and len(record.received) == DESCRIPTOR_WORDS:
            done = rx.feed_descriptor(record.received)
        elif cw.subaddress == segment_sa and len(record.received) == SEGMENT_WORDS:
            done = rx.feed_segment(record.received)
        else:
            continue
        if done:
            out.append((i, rx.version, rx.payload))
    return out
