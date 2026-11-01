"""A display's bus terminal in a process of its own.

A display program is a PyQt6 window that paints at up to 60 frames a second;
the weather picture animates thousands of particles in Python. Measured on
2026-11-01 with the terminal in a thread of the window's process: the moment
the first weather scan reached the radar display, the display missed every
poll (the bus controller's 20 ms reply timeout, then a retry) and the bus
overran every frame. The Python interpreter lets only one thread run at a
time, and the paint code holds it for longer than a reply may take.

So the terminal (DisplayTerminal, unchanged) runs in a helper process that
does nothing but answer the bus, started with the spawn method on every
platform, and the window keeps a mirror of what the terminal holds:
TerminalProcess has DisplayTerminal's public interface (latest, fresh,
latest_block, send, take_mode_request, the counters) and refreshes the mirror
from the helper's queue every time the window reads it. Commands go the other
way through a pipe. The helper leaves with its window: it watches for the
parent to disappear, it never waits for its queue on the way out, and it is a
daemon process, so nothing is left answering the bus (or idling) after the
window closes.
"""

import multiprocessing
import queue
import time

from fmofp_bus.icd.enums import DisplayMode, TerminalState
from fmofp_bus.status import Status
from fmofp_display.terminal import POLL_S, STALE_AFTER_S, DisplayTerminal

START_TIMEOUT_S = 20.0  # a fresh interpreter importing the bus library on a loaded CI machine
PARENT_CHECK_LOOPS = 250  # the helper checks its parent is alive every this many socket polls (~0.5 s)


def _run_terminal(updates, commands, rt, port_base, status_class, mode_cmd_class, receives, blocks):
    """The helper process: a DisplayTerminal's loop, forwarding what changes to the window."""
    terminal = DisplayTerminal(rt, port_base, status_class, mode_cmd_class, receives, blocks)
    status = terminal.process.init(rt, port_base)
    if status == Status.OK:
        terminal._accept_broadcasts()
        terminal._load_status()
    updates.put(("started", status))
    if status != Status.OK:
        updates.close()
        return
    # This process must be able to exit with updates still queued. The window reads the queue only between
    # paints, a window that was killed never reads it again, and the pipe behind the queue never breaks (this
    # process holds its read end too), so once the pipe is full the queue's feeder thread is stuck for good,
    # and the default exit would wait for that thread for ever. Seen on 2026-11-01: a helper outlived its
    # terminated window, off the bus, doing nothing.
    updates.cancel_join_thread()
    parent = multiprocessing.parent_process()
    forwarded = {}
    forwarded_blocks = {}
    last_syncs = -1
    loops = 0
    try:
        while True:
            terminal.process.poll(POLL_S)
            terminal.service()
            now = time.monotonic()
            for cls, (message, at) in terminal._latest.items():
                if forwarded.get(cls) != at:
                    forwarded[cls] = at
                    updates.put(("message", cls, message, now - at))
            for cls, (payload, version, at) in terminal._block_latest.items():
                if forwarded_blocks.get(cls) != at:
                    forwarded_blocks[cls] = at
                    updates.put(("block", cls, payload, version, now - at))
            mode = terminal.take_mode_request()
            if mode is not None:
                updates.put(("mode", mode))
            if terminal.syncs != last_syncs:
                last_syncs = terminal.syncs
                updates.put(("state", (terminal.syncs, terminal.resets, terminal.blocks_completed, terminal.state,
                                       terminal.display_mode)))
            while commands.poll():
                kind, payload = commands.recv()
                if kind == "send":
                    terminal.send(payload)
                elif kind == "stop":
                    return
            loops += 1
            if loops % PARENT_CHECK_LOOPS == 0 and parent is not None and not parent.is_alive():
                return
    except (EOFError, OSError, KeyboardInterrupt):
        return
    finally:
        terminal.process.shutdown()
        updates.close()


class TerminalProcess:
    """DisplayTerminal's interface, backed by a helper process that answers the bus."""

    def __init__(self, rt, port_base, status_class, mode_cmd_class, receives=(), blocks=()):
        self.rt = rt
        self.port_base = port_base
        self.status_class = status_class
        self.mode_cmd_class = mode_cmd_class
        self.receives = tuple(receives)
        self.blocks = tuple(blocks)
        self._process = None
        self._updates = None
        self._commands = None
        self._latest = {}
        self._block_latest = {}
        self._mode_requests = []
        self._syncs = 0
        self._resets = 0
        self._blocks_completed = 0
        self._state = TerminalState.INITIALIZING
        self._display_mode = DisplayMode.DAY
        self._command_seq = 0

    # ---- lifecycle ---------------------------------------------------------------
    def start(self):
        """Starts the helper and waits for it to be on the bus. IO_ERROR if the port is taken or it never came up."""
        context = multiprocessing.get_context("spawn")
        self._updates = context.Queue()
        self._commands, child_end = context.Pipe()
        self._process = context.Process(
            target=_run_terminal,
            args=(self._updates, child_end, self.rt, self.port_base, self.status_class, self.mode_cmd_class,
                  self.receives, self.blocks),
            name=f"display-rt{self.rt}-terminal", daemon=True)
        self._process.start()
        child_end.close()
        try:
            kind, status = self._updates.get(timeout=START_TIMEOUT_S)
        except queue.Empty:
            self.stop()
            return Status.IO_ERROR
        if kind != "started" or status != Status.OK:
            self._process.join(timeout=5.0)
            self._close()
            return status if kind == "started" else Status.IO_ERROR
        return Status.OK

    def stop(self):
        if self._process is None:
            return
        try:
            self._commands.send(("stop", None))
        except (OSError, ValueError, EOFError):
            pass
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=3.0)
        self._drain()
        self._close()

    def _close(self):
        if self._updates is not None:
            self._updates.close()
            self._updates.cancel_join_thread()
        if self._commands is not None:
            self._commands.close()
        self._updates = None
        self._commands = None
        self._process = None

    def _drain(self):
        """Takes everything the helper has forwarded since the last read."""
        if self._updates is None:
            return
        while True:
            try:
                item = self._updates.get_nowait()
            except (queue.Empty, OSError, ValueError, EOFError):
                return
            now = time.monotonic()
            kind = item[0]
            if kind == "message":
                _, cls, message, age = item
                self._latest[cls] = (message, now - age)
            elif kind == "block":
                _, cls, payload, version, age = item
                self._block_latest[cls] = (payload, version, now - age)
            elif kind == "mode":
                self._mode_requests.append(item[1])
            elif kind == "state":
                self._syncs, self._resets, self._blocks_completed, self._state, self._display_mode = item[1]

    # ---- the counters the window shows ------------------------------------------------
    @property
    def syncs(self):
        self._drain()
        return self._syncs

    @property
    def resets(self):
        self._drain()
        return self._resets

    @property
    def blocks_completed(self):
        self._drain()
        return self._blocks_completed

    @property
    def state(self):
        self._drain()
        return self._state

    @property
    def display_mode(self):
        self._drain()
        return self._display_mode

    # ---- what the display reads ----------------------------------------------------
    def latest(self, cls):
        """(message, age in seconds) of the newest `cls` received, or (None, None)."""
        self._drain()
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
        self._drain()
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
        if self._commands is not None:
            self._commands.send(("send", message))

    def take_mode_request(self):
        """The newest lighting mode the bus asked for, once, or None."""
        self._drain()
        if not self._mode_requests:
            return None
        request = self._mode_requests[-1]
        self._mode_requests.clear()
        return request
