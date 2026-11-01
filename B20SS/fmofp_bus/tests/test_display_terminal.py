"""The display terminal in its helper process: it comes up on its port, mirrors what arrives, takes commands,
refuses a taken port, and leaves with its parent even when its queue to the window is backed up."""

import ctypes
import multiprocessing
import os
import signal
import subprocess
import sys
import time
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import DisplayMode, TerminalState, WeatherMode
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_display.terminal_process import TerminalProcess

PORT_BASE = 64000
ORPHAN_PORT_BASE = PORT_BASE + 100
ORPHAN_FRAMES = 1500  # 30 s of bus, more than the test needs; the controller is killed at the end


def radar_display(port_base=PORT_BASE):
    return TerminalProcess(m.DspRadarStatus.RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd,
                           receives=(m.BcastRdrWxStatus,), blocks=(b.BcastRdrWxProducts,))


class TerminalProcessTest(unittest.TestCase):
    def test_it_comes_up_mirrors_the_bus_and_stops(self):
        terminal = radar_display()
        self.assertEqual(Status.OK, terminal.start())
        try:
            self.assertEqual((0, 0, 0, TerminalState.INITIALIZING), (terminal.syncs, terminal.resets,
                                                                     terminal.blocks_completed, terminal.state))
            self.assertEqual((None, None), terminal.latest(m.BcastRdrWxStatus))
            self.assertIsNone(terminal.fresh(m.BcastRdrWxStatus))
            self.assertEqual((None, 0, None), terminal.latest_block(b.BcastRdrWxProducts))
            self.assertIsNone(terminal.take_mode_request())
            self.assertEqual(1, terminal.next_command_seq())
            terminal.send(m.DspRdrWxCmd(payload=m.RdrWxModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL,
                                                                requested_mode=WeatherMode.SURVEILLANCE,
                                                                range_km=100.0)))
            self.assertEqual(DisplayMode.DAY, terminal.display_mode)
            taken = radar_display()
            self.assertEqual(Status.IO_ERROR, taken.start(), "the same RT on the same port base is taken")
        finally:
            terminal.stop()
        again = radar_display()
        self.assertEqual(Status.OK, again.start(), "the port is free again after stop()")
        again.stop()


@needs("FMOFP_BC")
class OrphanedHelperTest(unittest.TestCase):
    """A window killed outright (no stop()) must leave neither a terminal on the bus nor a helper process.

    The window's queue is where this can go wrong: the window reads it only between paints, a killed window
    never reads it again, and the pipe behind it never breaks (the helper holds its read end too). The first
    full demo on 2026-11-01 left a helper behind that had left the bus but hung on exit, waiting for its queue
    thread to push a backlog nobody would read. So the stand-in window here stops reading, fills the pipe,
    and the bus keeps the helper writing behind it."""

    def test_the_helper_leaves_when_its_parent_does(self):
        bc = subprocess.Popen([os.environ["FMOFP_BC"], "--rt", str(m.DspRadarStatus.RT),
                               "--port-base", str(ORPHAN_PORT_BASE), "--frames", str(ORPHAN_FRAMES),
                               "--timeout-ms", "20"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, env=binary_env())
        context = multiprocessing.get_context("spawn")
        ready = context.Queue()
        parent = context.Process(target=_hold_a_terminal, args=(ready, ORPHAN_PORT_BASE), daemon=False)
        parent.start()
        helper_pid = 0
        try:
            status, helper_pid = ready.get(timeout=60.0)
            self.assertEqual(Status.OK, status)
            self.assertEqual("backed up", ready.get(timeout=30.0))
            parent.terminate()
            parent.join(timeout=10.0)
            # The helper notices within about half a second, frees the port and exits.
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and _alive(helper_pid):
                time.sleep(0.2)
            self.assertFalse(_alive(helper_pid), f"the helper process (pid {helper_pid}) outlived its window")
            probe = RtProcess()
            self.assertEqual(Status.OK, probe.init(m.DspRadarStatus.RT, ORPHAN_PORT_BASE),
                             "the port is free again")
            probe.shutdown()
        finally:
            if parent.is_alive():
                parent.terminate()
            if helper_pid and _alive(helper_pid):
                os.kill(helper_pid, signal.SIGTERM)
            bc.kill()
            bc.wait(timeout=10.0)


def _hold_a_terminal(ready, port_base):
    """A stand-in window: starts a terminal, reports, stops reading its queue, fills the pipe behind it from
    this side too (both ends feed the same pipe, so the helper's queue thread is blocked behind it whatever
    the pipe's size), then waits to be killed."""
    terminal = TerminalProcess(m.DspRadarStatus.RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd)
    status = terminal.start()
    ready.put((status, terminal._process.pid if terminal._process is not None else 0))
    if status == Status.OK:
        for _ in range(4):
            terminal._updates.put(bytes(1 << 16))
        time.sleep(2.0)  # the helper queues a state update every sync; by now it is stuck behind the fill
        ready.put("backed up")
    time.sleep(120.0)


def _alive(pid):
    """Whether a process with this pid is still running."""
    if sys.platform == "win32":
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            kernel32.GetExitCodeProcess(handle, ctypes.byref(code))
            return code.value == 259  # STILL_ACTIVE
        finally:
            kernel32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


if __name__ == "__main__":
    unittest.main()
