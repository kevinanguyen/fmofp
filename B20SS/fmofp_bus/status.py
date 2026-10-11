"""Operation outcomes, mirroring fmofp::Status (B20SS/ofp/bus/include/fmofp/bus/status.hpp).

Values are ordered: a larger value is a worse outcome, so worst() combines them.
"""

import enum


class Status(enum.IntEnum):
    OK = 0
    CLAMPED = 1           # a value was limited to its field's range; mark the SSM NO_COMPUTED_DATA
    INVALID_ENUM = 2      # a received enum field holds a value the ICD does not define
    INVALID_ARGUMENT = 3  # NaN, infinity, an address out of range, ...
    MALFORMED = 4         # a reserved bit set, or a word that breaks the format
    BAD_LENGTH = 5
    BAD_MAGIC = 6
    TIMEOUT = 7
    NOT_READY = 8
    IO_ERROR = 9


def ok(status):
    """True for the two outcomes that still produced usable output."""
    return status in (Status.OK, Status.CLAMPED)


def worst(a, b):
    return a if int(a) >= int(b) else b
