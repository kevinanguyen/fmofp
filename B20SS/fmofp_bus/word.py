"""The 16-bit bus word and the only sanctioned ways of putting fields into it.

Mirrors fmofp/bus/word.hpp. Bits are numbered as in the ICD: 15 is the most
significant, 0 the least. The scaled-quantity functions round exactly as the
C++ does (half away from zero, like llround), so both sides produce the same
raw count for the same engineering value.
"""

import math

from fmofp_bus.status import Status

WORD_BITS = 16
WORD_MASK = 0xFFFF


def field_mask(hi, lo):
    """Mask covering bits hi..lo inclusive; 15 >= hi >= lo >= 0."""
    width = hi - lo + 1
    ones = WORD_MASK if width >= WORD_BITS else (1 << width) - 1
    return (ones << lo) & WORD_MASK


def get_field(word, hi, lo):
    return (word & field_mask(hi, lo)) >> lo


def set_field(word, hi, lo, value):
    """`word` with bits hi..lo replaced by the low bits of `value`."""
    mask = field_mask(hi, lo)
    return ((word & ~mask) | ((value << lo) & mask)) & WORD_MASK


def get_bit(word, bit):
    return get_field(word, bit, bit) != 0


def set_bit(word, bit, value):
    return set_field(word, bit, bit, 1 if value else 0)


def to_twos(value, width):
    """Two's-complement representation of `value` in a `width`-bit field."""
    return value & field_mask(width - 1, 0)


def from_twos(raw, width):
    """Sign-extends the low `width` bits of `raw`."""
    bits = raw & field_mask(width - 1, 0)
    sign = 1 << (width - 1)
    return bits - (sign << 1) if bits & sign else bits


def to_twos32(value):
    """Two's-complement representation of `value` in a 32-bit double word."""
    return value & 0xFFFFFFFF


def from_twos32(raw):
    """Sign-extends a 32-bit two's-complement value."""
    raw &= 0xFFFFFFFF
    return raw - 0x100000000 if raw & 0x80000000 else raw


def _llround(x):
    """Round half away from zero, as C's llround does (Python's round() is to even)."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def encode_scaled(value, lsb_num, lsb_den, raw_min, raw_max):
    """(Status, raw) for value = raw * lsb_num / lsb_den.

    OK, or CLAMPED when the value had to be limited to [raw_min, raw_max], or
    INVALID_ARGUMENT (raw 0) for NaN, infinity or a bad scale.
    """
    try:
        value = float(value)
    except (TypeError, ValueError):
        return Status.INVALID_ARGUMENT, 0
    if not math.isfinite(value) or lsb_num <= 0 or lsb_den <= 0 or raw_min > raw_max:
        return Status.INVALID_ARGUMENT, 0
    counts = value * lsb_den / lsb_num
    if counts < raw_min - 1.0:
        return Status.CLAMPED, raw_min
    if counts > raw_max + 1.0:
        return Status.CLAMPED, raw_max
    rounded = _llround(counts)
    if rounded < raw_min:
        return Status.CLAMPED, raw_min
    if rounded > raw_max:
        return Status.CLAMPED, raw_max
    return Status.OK, rounded


def decode_scaled(raw, lsb_num, lsb_den):
    # Multiply first, as the C++ does, so the single division is the only rounding step.
    return raw * lsb_num / lsb_den


def check_uint(value, width):
    """(Status, clipped) for a raw unsigned field."""
    limit = (1 << width) - 1
    v = int(value)
    if v < 0:
        return Status.CLAMPED, 0
    if v > limit:
        return Status.CLAMPED, limit
    return Status.OK, v


def check_sint(value, width):
    """(Status, clipped) for a raw signed field."""
    low, high = -(1 << (width - 1)), (1 << (width - 1)) - 1
    v = int(value)
    if v < low:
        return Status.CLAMPED, low
    if v > high:
        return Status.CLAMPED, high
    return Status.OK, v
