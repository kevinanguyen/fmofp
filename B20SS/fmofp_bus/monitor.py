"""Reading the bus controller's monitor port (ICD README section 7).

The controller copies every completed transaction to port_base + 33 as a
frame of type MONITOR: the command words as sent, then the reply words
(status word first) if there was one. This is what a bus recorder, and the
tests, listen to.
"""

import select
import socket
from dataclasses import dataclass, field
from typing import List, Optional

from fmofp_bus.command_word import CommandWord, StatusWord, decode_command_word, decode_status_word
from fmofp_bus.rt_process import LOOPBACK, MONITOR_PORT_OFFSET
from fmofp_bus.status import Status
from fmofp_bus.transport import MAX_FRAME_BYTES, FrameType, decode_frame


@dataclass
class MonitorRecord:
    transaction: int
    command: CommandWord
    sent: List[int] = field(default_factory=list)      # data words the BC sent with the command
    status: Optional[StatusWord] = None                # None when nobody answered (or broadcast)
    received: List[int] = field(default_factory=list)  # data words that came back after the status

    @property
    def answered(self):
        return self.status is not None


def parse_record(frame):
    """A MonitorRecord from a MONITOR frame, or None if the frame is not one."""
    if frame.type != FrameType.MONITOR or frame.word_count < 1:
        return None
    cw = decode_command_word(frame.words[0])
    sent_count = 0
    if cw.is_mode_code:
        sent_count = 1 if (not cw.transmit and frame.word_count >= 2) else 0
    elif not cw.transmit:
        sent_count = cw.count_or_mode
    sent = frame.words[1:1 + sent_count]
    rest = frame.words[1 + sent_count:]
    record = MonitorRecord(frame.transaction, cw, sent)
    if rest:
        _, record.status = decode_status_word(rest[0])
        record.received = rest[1:]
    return record


class Monitor:
    """A listener on the monitor port."""

    def __init__(self, port_base=51000):
        self.port = port_base + MONITOR_PORT_OFFSET
        self._sock = None

    def open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((LOOPBACK, self.port))
        except OSError:
            sock.close()
            return Status.IO_ERROR
        sock.setblocking(False)
        self._sock = sock
        return Status.OK

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def read(self, timeout_s):
        """The next record within timeout_s, or None."""
        if self._sock is None:
            return None
        ready, _, _ = select.select([self._sock], [], [], max(0.0, timeout_s))
        if not ready:
            return None
        try:
            data, _ = self._sock.recvfrom(MAX_FRAME_BYTES * 2)
        except OSError:
            return None
        status, frame = decode_frame(data)
        if status != Status.OK:
            return None
        return parse_record(frame)

    def drain(self, idle_s=0.05):
        """Every record until the port has been quiet for idle_s."""
        records = []
        while True:
            record = self.read(idle_s)
            if record is None:
                return records
            records.append(record)
