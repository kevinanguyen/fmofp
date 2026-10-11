"""The socket end of a Python terminal, mirroring fmofp::rt::RtProcess.

Receives the bus controller's datagrams, hands them to the RemoteTerminal and
sends back whatever it answers. A Python partition (the plant model, a display)
owns one of these and calls poll() in its loop; its own work happens when
terminal.take_sync() fires.
"""

import select
import socket
from dataclasses import dataclass

from fmofp_bus.remote_terminal import RemoteTerminal
from fmofp_bus.status import Status
from fmofp_bus.transport import MAX_FRAME_BYTES, decode_frame, encode_frame

DEFAULT_PORT_BASE = 51000
BC_PORT_OFFSET = 32
MONITOR_PORT_OFFSET = 33
LOOPBACK = "127.0.0.1"


@dataclass
class RtProcessStats:
    datagrams: int = 0
    replies: int = 0
    bad_frames: int = 0
    send_failures: int = 0


class RtProcess:
    def __init__(self):
        self.terminal = RemoteTerminal()
        self.stats = RtProcessStats()
        self.port_base = DEFAULT_PORT_BASE
        self._sock = None

    def init(self, rt, port_base=DEFAULT_PORT_BASE):
        self.shutdown()
        status = self.terminal.init(rt)
        if status != Status.OK:
            return status
        self.port_base = port_base
        self.stats = RtProcessStats()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((LOOPBACK, port_base + rt))
        except OSError:
            sock.close()
            return Status.IO_ERROR
        sock.setblocking(False)
        self._sock = sock
        return Status.OK

    @property
    def is_open(self):
        return self._sock is not None

    def shutdown(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def poll(self, timeout_s):
        """Handles at most one datagram, waiting up to timeout_s for it.

        TIMEOUT when nothing arrived; OK when a datagram was processed, valid or not.
        """
        if self._sock is None:
            return Status.NOT_READY
        ready, _, _ = select.select([self._sock], [], [], max(0.0, timeout_s))
        if not ready:
            return Status.TIMEOUT
        try:
            data, _ = self._sock.recvfrom(MAX_FRAME_BYTES * 2)
        except OSError:
            return Status.OK
        self.stats.datagrams += 1
        status, frame = decode_frame(data)
        if status != Status.OK:
            self.stats.bad_frames += 1
            return Status.OK
        status, reply = self.terminal.handle(frame)
        if status != Status.OK:
            self.stats.bad_frames += 1
            return Status.OK
        if reply is None:
            return Status.OK
        status, out = encode_frame(reply)
        if status != Status.OK:
            self.stats.send_failures += 1
            return Status.OK
        try:
            self._sock.sendto(out, (LOOPBACK, self.port_base + BC_PORT_OFFSET))
        except OSError:
            self.stats.send_failures += 1
            return Status.OK
        self.stats.replies += 1
        return Status.OK
