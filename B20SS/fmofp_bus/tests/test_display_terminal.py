"""The display terminal in its helper process: it comes up on its port, mirrors what arrives, takes commands,
refuses a taken port, and leaves with its parent."""

import multiprocessing
import time
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import DisplayMode, TerminalState, WeatherMode
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status
from fmofp_display.terminal_process import TerminalProcess

PORT_BASE = 64000


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

    def test_the_helper_leaves_when_its_parent_does(self):
        """A window killed outright (no stop()) must not leave a terminal answering the bus."""
        context = multiprocessing.get_context("spawn")
        ready = context.Queue()
        parent = context.Process(target=_hold_a_terminal, args=(ready, PORT_BASE + 100), daemon=False)
        parent.start()
        try:
            status, helper_pid = ready.get(timeout=60.0)
            self.assertEqual(Status.OK, status)
            parent.terminate()
            parent.join(timeout=10.0)
            # The helper notices within about half a second and frees the port.
            probe = RtProcess()
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                if probe.init(m.DspRadarStatus.RT, PORT_BASE + 100) == Status.OK:
                    probe.shutdown()
                    return
                time.sleep(0.2)
            self.fail(f"the helper process (pid {helper_pid}) kept the port after its parent died")
        finally:
            if parent.is_alive():
                parent.terminate()


def _hold_a_terminal(ready, port_base):
    """A stand-in window: starts a terminal, reports, then waits to be killed."""
    terminal = TerminalProcess(m.DspRadarStatus.RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd)
    status = terminal.start()
    ready.put((status, terminal._process.pid if terminal._process is not None else 0))
    time.sleep(120.0)


if __name__ == "__main__":
    unittest.main()
