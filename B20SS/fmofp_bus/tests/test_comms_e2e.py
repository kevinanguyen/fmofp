"""The communications terminal on the bus, end to end.

The plant (Python, in-process, with a satellite scripted to come into view after
two seconds) and the real fmofp_comms under the real fmofp_bc with a script of
commands: the radio is tuned and switched to USB, a transmission fails while
the field is quiet, a satellite is acquired and the data link comes up on it,
messages go out on both, the channel is set. The monitor port shows
COM_STATUS and COM_LINK_STATUS, and the plant's PLANT_RF reaching the terminal
by relay.

Needs FMOFP_BC and FMOFP_COMMS (CI sets them).
"""

import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import CmdStatus, CommsCommand, DatalinkMode, LinkState, RadioMode, SatcomState, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 63500
FRAMES = 330
SYNC_MODE_CODE = 17
COMMS_RT = m.ComStatus.RT

SCRIPT = [
    (5, m.ComCmd(cmd_seq=1, command=CommsCommand.SET_RADIO_FREQUENCY, frequency=121.5)),
    (6, m.ComCmd(cmd_seq=2, command=CommsCommand.SET_RADIO_MODE, radio_mode=RadioMode.USB)),
    (10, m.ComCmd(cmd_seq=3, command=CommsCommand.RADIO_TRANSMIT, message_id=7)),
    (70, m.ComCmd(cmd_seq=4, command=CommsCommand.ACQUIRE_SATELLITE)),
    (250, m.ComCmd(cmd_seq=5, command=CommsCommand.DATALINK_SEND, message_id=1, priority=2)),
    (260, m.ComCmd(cmd_seq=6, command=CommsCommand.SATCOM_SEND, message_id=3)),
    (270, m.ComCmd(cmd_seq=7, command=CommsCommand.SET_DATALINK_CHANNEL, channel=7)),
]


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.05)


@needs("FMOFP_BC", "FMOFP_COMMS")
class CommsEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_comms_"), "commands.txt")
        write_script(cls.script)
        try:
            model = PlantModel(seed=5, traffic=False, ground=False, weather=False, satellite_after_s=2)
            cls.plant = PlantTerminal(model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            cls.processes.append(subprocess.Popen(
                [os.environ["FMOFP_COMMS"], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, env=binary_env()))
            time.sleep(0.3)
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        for p in cls.processes:
            p.kill()
            p.wait(timeout=10)
            p.stdout.close()
        cls.processes = []
        if cls.plant_thread is not None:
            cls.plant_thread.stop.set()
            cls.plant_thread.join(timeout=2)
        if cls.plant is not None:
            cls.plant.shutdown()
        if cls.monitor is not None:
            cls.monitor.close()

    def run_bus(self):
        bc = subprocess.Popen(
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", str(COMMS_RT), "--port-base", str(PORT_BASE), "--frames",
             str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script", self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records, output

    @staticmethod
    def by_frame(records):
        frame = -1
        out = []
        for record in records:
            cw = record.command
            if cw.is_mode_code and cw.count_or_mode == SYNC_MODE_CODE:
                frame += 1
                continue
            out.append((frame, record))
        return out

    def last_in_window(self, framed, cls, first, last, rt=None):
        rt = cls.RT if rt is None else rt
        for frame, record in reversed(framed):
            cw = record.command
            if (first <= frame < last and cw.transmit and not cw.is_mode_code and cw.rt == rt
                    and cw.subaddress == cls.SUBADDRESS and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                return msg
        self.fail(f"no {cls.__name__} from RT {rt} in frames {first}..{last - 1}")

    def test_the_links_follow_the_plant_and_the_commands(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)
        self.assertIn("script: 7 commands sent of 7", output)

        # The plant's RF environment reaches the terminal by relay: PLANT_RF polled, written into COM_RF.
        relayed = [r for _, r in framed if not r.command.transmit and not r.command.is_mode_code
                   and r.command.rt == COMMS_RT and r.command.subaddress == m.ComRf.SUBADDRESS]
        self.assertGreaterEqual(len(relayed), 5, "once a second")

        # Frames 5-10: tuned to 121.5 USB; the transmission fails while the field is under 20 %.
        status = self.last_in_window(framed, m.ComStatus, 12, 70)
        self.assertEqual((CmdStatus.FAILED, False, False), (status.comms_cmd_status, status.radio_active,
                                                            status.satcom_connected))
        self.assertEqual(0, status.radio_transmissions)
        link = self.last_in_window(framed, m.ComLinkStatus, 12, 70)
        self.assertEqual((121.5, RadioMode.USB, SatcomState.DISCONNECTED, LinkState.INACTIVE),
                         (link.frequency, link.radio_mode, link.satcom_state, link.datalink_state))
        self.assertLess(link.radio_signal, 20.0)

        # Frame 70: acquiring; the plant's satellite comes into view at its second step (two seconds): connected,
        # with the satellite's numbers, and the data link up on it.
        link = self.last_in_window(framed, m.ComLinkStatus, 150, 250)
        self.assertEqual((SatcomState.CONNECTED, LinkState.ACTIVE), (link.satcom_state, link.datalink_state))
        self.assertTrue(1 <= link.satellite_id <= 12)
        self.assertTrue(40.0 <= link.satcom_signal <= 100.0, link.satcom_signal)
        self.assertTrue(0.0 <= link.data_rate <= 300.0, link.data_rate)
        self.assertGreaterEqual(link.latency, 200.0)
        status = self.last_in_window(framed, m.ComStatus, 150, 250)
        self.assertTrue(status.satcom_connected and status.datalink_active)
        self.assertEqual(CmdStatus.ACCEPTED, status.comms_cmd_status, "the acquisition")

        # Frames 250-270: a data-link message, a satcom message, the channel.
        status = self.last_in_window(framed, m.ComStatus, 280, FRAMES)
        self.assertEqual((CmdStatus.ACCEPTED, 1), (status.comms_cmd_status, status.satcom_transmissions))
        link = self.last_in_window(framed, m.ComLinkStatus, 280, FRAMES)
        self.assertEqual((7, DatalinkMode.LOS), (link.channel, link.datalink_mode))
        self.assertGreaterEqual(link.packets_sent, 1)
        term = self.last_in_window(framed, m.TermStatus, 280, FRAMES, rt=COMMS_RT)
        self.assertEqual(TerminalState.NORMAL, term.state)
        self.assertIn("overruns=0", output)


if __name__ == "__main__":
    unittest.main()
