"""A display's end of the bus: a Python terminal in a thread beside Qt.

The bus controller relays what a display needs into its receive subaddresses
and receive blocks (ICD v1.5); the display's own commands sit on its transmit
subaddresses for the controller to relay on. DisplayTerminal runs the socket
loop in a background thread and keeps the latest decoded copy of every message
and block it is told to watch, with the time it arrived. It answers the
display's own status message, its mode command (the lighting mode) and
TERM_STATUS. Nothing here touches Qt: the Qt side reads latest() from a timer,
and the headless tests drive the terminal directly.

A display is test equipment answering a 50 Hz bus from Python, next to a Qt
paint loop: give the bus controller a reply timeout of 20 ms when displays are
on the bus, and run it without --strict.
"""

import threading
import time

from fmofp_bus.block import BlockCollector
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import DisplayMode, TerminalState
from fmofp_bus.remote_terminal import CommandSequence
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status

# data_validity.DEFAULT_STALE_AFTER_S: a display shows a value only this long after it last arrived.
STALE_AFTER_S = 2.0
POLL_S = 0.002

# A state the display can be commanded into (DSP_*_MODE_CMD requested_state).
COMMANDABLE = (TerminalState.STANDBY, TerminalState.NORMAL, TerminalState.TEST, TerminalState.MAINTENANCE,
               TerminalState.CALIBRATION)


class DisplayTerminal:
    """One display terminal: `rt` on the bus at `port_base`, watching `receives` and `blocks`.

    `status_class` / `mode_cmd_class` are the display's own DSP_*_STATUS and
    DSP_*_MODE_CMD; `receives` the relayed messages to keep the latest of;
    `blocks` the receive blocks to reassemble.
    """

    def __init__(self, rt, port_base, status_class, mode_cmd_class, receives=(), blocks=()):
        self.rt = rt
        self.port_base = port_base
        self.status_class = status_class
        self.mode_cmd_class = mode_cmd_class
        self.receives = tuple(receives)
        self.blocks = tuple(blocks)
        self.process = RtProcess()
        self._lock = threading.Lock()
        self._latest = {}
        self._block_latest = {}
        self._collectors = {cls: BlockCollector(cls.MAX_PAYLOAD_WORDS) for cls in self.blocks}
        self._pending = []
        self._mode_seq = CommandSequence()
        self._mode_requests = []
        self.state = TerminalState.INITIALIZING
        self.display_mode = DisplayMode.DAY
        self.last_cmd_seq = 0
        self.cmd_rejected = False
        self.syncs = 0
        self.resets = 0
        self.blocks_completed = 0
        self.last_sync_word = 0
        self._command_seq = 0
        self._thread = None
        self._stop = threading.Event()

    # ---- lifecycle ---------------------------------------------------------------
    def start(self):
        """Opens the socket and starts the loop. IO_ERROR if the port is taken."""
        status = self.process.init(self.rt, self.port_base)
        if status != Status.OK:
            return status
        self._load_status()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"display-rt{self.rt}", daemon=True)
        self._thread.start()
        return Status.OK

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.process.shutdown()

    def _run(self):
        while not self._stop.is_set():
            self.process.poll(POLL_S)
            self.service()

    # ---- what the display reads ----------------------------------------------------
    def latest(self, cls):
        """(message, age in seconds) of the newest `cls` received, or (None, None)."""
        with self._lock:
            entry = self._latest.get(cls)
        if entry is None:
            return None, None
        return entry[0], time.monotonic() - entry[1]

    def fresh(self, cls, stale_after_s=STALE_AFTER_S):
        """The newest `cls` if it arrived within `stale_after_s`, else None."""
        message, age = self.latest(cls)
        return message if age is not None and age <= stale_after_s else None

    def latest_block(self, cls):
        """(decoded payload, version, age) of the newest complete `cls` block, or (None, 0, None)."""
        with self._lock:
            entry = self._block_latest.get(cls)
        if entry is None:
            return None, 0, None
        return entry[0], entry[1], time.monotonic() - entry[2]

    # ---- what the display sends ----------------------------------------------------
    def next_command_seq(self):
        """1..255, wrapping to 1: cmd_seq 0 is no command (ICD README section 5)."""
        self._command_seq = 1 if self._command_seq >= 255 else self._command_seq + 1
        return self._command_seq

    def send(self, message):
        """Loads a transmit message (a DSP_RDR_*_CMD) for the controller to relay on; it stays loaded."""
        with self._lock:
            self._pending.append(message)

    def take_mode_request(self):
        """The newest lighting mode the bus asked for, once, or None."""
        with self._lock:
            if not self._mode_requests:
                return None
            request = self._mode_requests[-1]
            self._mode_requests.clear()
            return request

    # ---- the loop's work ------------------------------------------------------------
    def service(self):
        """Handles a reset and, on a sync, takes what arrived and reloads what the display sends."""
        term = self.process.terminal
        if term.take_reset_request():
            self._reset()
        got, word = term.take_sync()
        if not got:
            return
        self.syncs += 1
        self.last_sync_word = word
        now = time.monotonic()
        with self._lock:
            for cls in self.receives:
                status, words, fresh = term.get_received(cls.SUBADDRESS)
                if status == Status.OK and fresh and len(words) == cls.WORDS:
                    decoded, message = cls.decode(words)
                    if decoded == Status.OK:
                        self._latest[cls] = (message, now)
            for cls in self.blocks:
                collector = self._collectors[cls]
                if collector.collect(term, cls.DESCRIPTOR_SUBADDRESS, cls.SEGMENT_SUBADDRESS):
                    decoded, payload = cls.LAYOUT.decode(collector.receiver.payload)
                    if decoded == Status.OK:
                        self._block_latest[cls] = (payload, collector.receiver.version, now)
                        self.blocks_completed += 1
            status, words, fresh = term.get_received(self.mode_cmd_class.SUBADDRESS)
            if status == Status.OK and fresh and len(words) == self.mode_cmd_class.WORDS:
                decoded, command = self.mode_cmd_class.decode(words)
                if decoded == Status.OK:
                    self._command(command)
            pending, self._pending = self._pending, []
        for message in pending:
            status, words = message.encode()
            if status in (Status.OK, Status.CLAMPED):
                term.set_transmit(message.SUBADDRESS, words)
        if self.state == TerminalState.INITIALIZING:
            self.state = TerminalState.NORMAL
        self._load_status()

    def _command(self, command):
        if not self._mode_seq.accept(command.cmd_seq):
            return
        self.last_cmd_seq = command.cmd_seq
        if command.requested_state not in COMMANDABLE or command.requested_mode == DisplayMode.NOT_SET:
            self.cmd_rejected = True
            return
        self.cmd_rejected = False
        self.state = command.requested_state
        self.display_mode = command.requested_mode
        self._mode_requests.append(command.requested_mode)

    def _reset(self):
        self.resets += 1
        with self._lock:
            self._latest.clear()
            self._block_latest.clear()
            for cls in self.blocks:
                self._collectors[cls] = BlockCollector(cls.MAX_PAYLOAD_WORDS)
            self._mode_seq = CommandSequence()
        self.state = TerminalState.INITIALIZING
        self.last_cmd_seq = 0
        self.cmd_rejected = False
        self._load_status()

    def _load_status(self):
        term = self.process.terminal
        status = self.status_class(state=self.state, mode=self.display_mode, last_cmd_seq=self.last_cmd_seq,
                                   cmd_rejected=self.cmd_rejected, healthy=True)
        encoded, words = status.encode()
        if encoded in (Status.OK, Status.CLAMPED):
            term.set_transmit(status.SUBADDRESS, words)
        ts = m.TermStatus(state=self.state, last_cmd_seq=self.last_cmd_seq, cmd_rejected=self.cmd_rejected, healthy=True,
                          icd_major=m.ICD_MAJOR, icd_minor=m.ICD_MINOR, last_sync=self.last_sync_word)
        encoded, words = ts.encode()
        if encoded in (Status.OK, Status.CLAMPED):
            term.set_transmit(ts.SUBADDRESS, words)
