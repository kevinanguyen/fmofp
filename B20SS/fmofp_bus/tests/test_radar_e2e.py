"""The ported weather radar on the bus.

Plant (Python, in-process, with its weather), the real fmofp_radar and the real
fmofp_bc with a command script: SURVEILLANCE, then WINDSHEAR, then STANDBY go to
the radar over the bus at known frames. The monitor port shows the relayed
weather truth reaching the radar, the status and scan summary it reports, and
the RDR_WX_PRODUCTS block it publishes a segment per frame; the block is
reassembled here and decoded with the generated layout. The arithmetic itself
is checked against the Python in test_weather_oracle.py; this test checks the
plumbing and the cadence.

The radar answers the bus from the same thread that finishes a scan, so the
controller is given a 20 ms reply timeout here (a real terminal answers from
its buffers while its processor works); locally the worst frame is under 3 ms.

Needs FMOFP_BC and FMOFP_RADAR (CI sets them).
"""

import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.block import blocks_from_records
from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import TerminalState, WeatherMode
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 58000
FRAMES = 260
SYNC_MODE_CODE = 17
RADAR_RT = m.RdrWxStatus.RT

SCRIPT = [
    (5, m.RdrWxModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL, requested_mode=WeatherMode.SURVEILLANCE,
                       range_km=200.0, tilt=0.0)),
    (150, m.RdrWxModeCmd(cmd_seq=2, requested_state=TerminalState.NORMAL, requested_mode=WeatherMode.WINDSHEAR,
                         range_km=200.0, tilt=5.0)),
    (230, m.RdrWxModeCmd(cmd_seq=3, requested_state=TerminalState.STANDBY, requested_mode=WeatherMode.NONE,
                         range_km=200.0, tilt=0.0)),
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
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_RADAR")
class WeatherRadarEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_radar_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=9)
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=30000.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            cls.processes.append(subprocess.Popen(
                [os.environ["FMOFP_RADAR"], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
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
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", str(RADAR_RT), "--port-base", str(PORT_BASE), "--frames",
             str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script", self.script],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        self.assertIn(f"script: {len(SCRIPT)} commands", output)
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

    def transmitted(self, framed, cls, first, last, rt=None):
        """Every `cls` the terminal transmitted in frames first..last-1, as (frame, message)."""
        rt = cls.RT if rt is None else rt
        out = []
        for frame, record in framed:
            cw = record.command
            if (first <= frame < last and cw.transmit and not cw.is_mode_code and cw.rt == rt
                    and cw.subaddress == cls.SUBADDRESS and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                out.append((frame, msg))
        return out

    def test_the_radar_scans_commands_and_publishes_its_products(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # The weather truth is relayed to the radar once a second, 32 words each time.
        relayed = [(f, r) for f, r in framed if not r.command.transmit and not r.command.is_mode_code
                   and r.command.rt == RADAR_RT and r.command.subaddress == m.RdrWxWeather.SUBADDRESS]
        self.assertGreaterEqual(len(relayed), 4)
        self.assertTrue(all(len(r.sent) == 32 and r.answered for _, r in relayed))
        truth = m.RdrWxWeather.decode(relayed[0][1].sent)[1].payload
        self.assertGreaterEqual(truth.cell_count, 3)

        # Status: SURVEILLANCE after the first command, WINDSHEAR after the second, STANDBY after the third.
        status = self.transmitted(framed, m.RdrWxStatus, 20, 40)
        self.assertTrue(status)
        self.assertEqual((TerminalState.NORMAL, WeatherMode.SURVEILLANCE, 1, False),
                         (status[-1][1].state, status[-1][1].mode, status[-1][1].last_cmd_seq, status[-1][1].cmd_rejected))
        self.assertTrue(status[-1][1].mode_transition, "no scan in the new mode has completed by frame 40")
        later = self.transmitted(framed, m.RdrWxStatus, 100, 150)[-1][1]
        self.assertFalse(later.mode_transition)
        windshear = self.transmitted(framed, m.RdrWxStatus, 160, 230)[-1][1]
        self.assertEqual((TerminalState.NORMAL, WeatherMode.WINDSHEAR, 2), (windshear.state, windshear.mode, windshear.last_cmd_seq))
        standby = self.transmitted(framed, m.RdrWxStatus, 240, FRAMES)[-1][1]
        self.assertEqual((TerminalState.STANDBY, WeatherMode.NONE, 3), (standby.state, standby.mode, standby.last_cmd_seq))

        # The scan summary: the first scan completes about 60 frames after the command.
        scans = self.transmitted(framed, m.RdrWxScan, 0, FRAMES)
        scan_one = next((f, s) for f, s in scans if s.scan_id == 1)
        self.assertTrue(60 <= scan_one[0] <= 80, f"first scan reported at frame {scan_one[0]}")
        self.assertEqual(WeatherMode.SURVEILLANCE, scan_one[1].mode)
        # The plant's weather reaches the radar once a second, in whichever frame the schedule gives that poll; a scan
        # that started before the first delivery sees only ground clutter (a peak near 15 dBZ, still a few VIL points).
        # The products are checked on the first scan with weather in it.
        first = next((f, s) for f, s in scans if s.mode == WeatherMode.SURVEILLANCE and s.max_reflectivity > 30.0)
        self.assertLessEqual(first[0], 140, "a surveillance scan with weather completes before the wind shear command")
        self.assertEqual(4, first[1].elevations)
        self.assertEqual(200.0, first[1].range_km)
        self.assertGreater(first[1].storm_cells, 0)
        self.assertGreater(first[1].vil_points, 0)
        self.assertGreater(first[1].max_reflectivity, 30.0)
        second = next(s for _, s in scans if s.scan_id == 2)
        self.assertEqual(WeatherMode.SURVEILLANCE, second.mode)
        shear_scan = next(s for _, s in scans if s.mode == WeatherMode.WINDSHEAR)
        self.assertEqual(1, shear_scan.elevations)
        self.assertAlmostEqual(5.0, shear_scan.tilt, delta=0.01)
        cleared = scans[-1][1]
        self.assertEqual(WeatherMode.NONE, cleared.mode)
        self.assertEqual(0, cleared.vil_points)

        # The products block: reassembled from the segments, one per frame.
        blocks = blocks_from_records(records, RADAR_RT, b.RdrWxProducts.DESCRIPTOR_SUBADDRESS,
                                     b.RdrWxProducts.SEGMENT_SUBADDRESS)
        self.assertGreaterEqual(len(blocks), 4, "empty, two surveillance scans, a wind shear scan, empty again")
        decoded = []
        for _, version, payload in blocks:
            status, products = b.RdrWxProducts.decode(payload)
            self.assertEqual(Status.OK, status, version)
            decoded.append((version, products))
        versions = [v for v, _ in decoded]
        self.assertEqual(sorted(versions), versions)
        surveillance = [p for _, p in decoded if p.mode == WeatherMode.SURVEILLANCE]
        self.assertGreaterEqual(len(surveillance), 2)
        p = next(p for p in surveillance if p.scan_id == first[1].scan_id)
        self.assertEqual(first[1].scan_id, p.scan_id)
        self.assertEqual(first[1].vil_points, len(p.vil))
        self.assertEqual(first[1].precip_points, len(p.precip))
        self.assertEqual(min(12, first[1].storm_cells), len(p.cells))
        self.assertEqual(0, len(p.windshear))
        for point in p.vil:
            self.assertEqual(0, round(point.azimuth) % 8)
            self.assertLess(point.range, 200.0)
            self.assertTrue(0.0 < point.intensity <= 1.0)
        self.assertEqual(first[1].max_reflectivity, p.max_reflectivity)
        version_of_first = next(v for v, q in decoded if q is p)
        self.assertEqual(version_of_first, first[1].products_version)
        shear = next(q for _, q in decoded if q.mode == WeatherMode.WINDSHEAR)
        self.assertEqual(0, len(shear.vil))
        self.assertEqual(shear_scan.windshear_events > 0, len(shear.windshear) > 0)
        self.assertEqual(WeatherMode.NONE, decoded[-1][1].mode)
        self.assertEqual(0, len(decoded[-1][1].vil))

        # The terminal's own status says what it is and which ICD it speaks.
        term = self.transmitted(framed, m.TermStatus, 100, 150, rt=RADAR_RT)[-1][1]
        self.assertEqual(TerminalState.NORMAL, term.state)
        self.assertEqual((m.ICD_MAJOR, m.ICD_MINOR), (term.icd_major, term.icd_minor))
        self.assertIn("overruns=0", output)


if __name__ == "__main__":
    unittest.main()
