"""A Python terminal on the bus with the real C++ bus controller.

Runs only when FMOFP_BC names the fmofp_bc binary (CI sets it after the C++
build; locally, point it at build/<preset>/bc/fmofp_bc). The controller runs
60 free-running frames in --strict mode: it exits 1 if the Python terminal
missed a single reply, so the pass is the cross-language proof that both
sides speak the same ICD.
"""

import os
import subprocess
import sys
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import TerminalState
from fmofp_bus.rt_process import RtProcess
from fmofp_bus.status import Status

PORT_BASE = 54000
RT = 1
FRAMES = 60


class StubTerminal(threading.Thread):
    """Answers the bus and refreshes TERM_STATUS with every sync word."""

    def __init__(self, process):
        super().__init__(daemon=True)
        self.process = process
        self.stop = threading.Event()
        self.syncs = 0

    def run(self):
        while not self.stop.is_set():
            self.process.poll(0.002)
            got, sync = self.process.terminal.take_sync()
            if got:
                self.syncs += 1
                status, words = m.TermStatus(state=TerminalState.NORMAL, healthy=True, last_sync=sync,
                                             icd_major=m.ICD_MAJOR).encode()
                if status == Status.OK:
                    self.process.terminal.set_transmit(m.TermStatus.SUBADDRESS, words)


@unittest.skipUnless(os.environ.get("FMOFP_BC"), "set FMOFP_BC to the fmofp_bc binary to run")
class BusIntegrationTest(unittest.TestCase):
    def test_python_terminal_follows_the_cpp_bus_controller(self):
        process = RtProcess()
        self.assertEqual(Status.OK, process.init(RT, PORT_BASE))
        stub = StubTerminal(process)
        stub.start()
        env = dict(os.environ)
        if sys.platform == "win32" and os.path.isdir(r"C:\msys64\ucrt64\bin"):
            env["PATH"] = r"C:\msys64\ucrt64\bin;" + env.get("PATH", "")
        try:
            result = subprocess.run(
                [os.environ["FMOFP_BC"], "--rt", str(RT), "--port-base", str(PORT_BASE), "--frames", str(FRAMES),
                 "--free-run", "--no-monitor", "--strict"],
                capture_output=True, text=True, timeout=60, env=env)
            # The controller exits the moment its last frame is sent; the stub may
            # still have the last datagrams to read. Let it catch up before stopping it.
            deadline = time.monotonic() + 2.0
            while stub.syncs < FRAMES and time.monotonic() < deadline:
                time.sleep(0.01)
        finally:
            stub.stop.set()
            stub.join(timeout=2)
            process.shutdown()
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertIn("rt  1: transactions=2 retries=0 no_response=0", result.stdout)
        self.assertEqual(FRAMES, stub.syncs)
        self.assertEqual(FRAMES, process.terminal.counters.syncs)
        self.assertEqual(2, process.terminal.counters.transmits)  # TERM_STATUS at 1 Hz over 60 frames


if __name__ == "__main__":
    unittest.main()
