"""The ported synthetic aperture radar on the bus.

Plant (Python, in-process, holding a northbound track at 450 kt over a scene of its
own: the road beside the track, a lake with a ship under way), the real flight
control, navigation and flight management partitions (the SAR places its pictures
by the FMS flight data broadcast), the real fmofp_radar and the real fmofp_bc with a
command script: STRIPMAP at 1 m, then SPOTLIGHT at 0.3 m with the lake abeam, then
STANDBY. The monitor port shows the plant's scene reaching the radar by relay, the
status the radar reports, and the RDR_SAR_IMAGE blocks: the strip with the road in
it, two spotlight pictures of the same patch with the ship three pixels further
on, and the empty block of standby. The picture's levels and the change detector
are checked against the Python in test_sar_oracle.py; this test checks the
plumbing, the cadence and the geometry on the bus.

Needs FMOFP_BC, FMOFP_FCS, FMOFP_NAV, FMOFP_FMS and FMOFP_RADAR (CI sets them).
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
from fmofp_bus.icd.enums import GroundFeature, SarMode, TerminalState
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.ground import Area, GroundScene, Line, Point
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 52000
FRAMES = 950
SYNC_MODE_CODE = 17
RADAR_RT = m.RdrSarStatus.RT
PLANT_RT = m.PlantSensors.RT
BINARIES = ("FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS", "FMOFP_RADAR")
SPOTLIGHT_FRAME = 400
STANDBY_FRAME = 860

SCRIPT = [
    (5, m.RdrSarModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL, requested_mode=SarMode.STRIPMAP,
                        resolution=1.0)),
    (SPOTLIGHT_FRAME, m.RdrSarModeCmd(cmd_seq=2, requested_state=TerminalState.NORMAL,
                                      requested_mode=SarMode.SPOTLIGHT, resolution=0.3)),
    (STANDBY_FRAME, m.RdrSarModeCmd(cmd_seq=3, requested_state=TerminalState.STANDBY, requested_mode=SarMode.NONE,
                                    resolution=0.3)),
]


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


def test_scene():
    """The road 2.5 km east of the track; a lake 1.9 km east, abeam 8 s into the run; a ship on it heading north
    at 5 m/s, inside the 307 m spotlight patch (east 2000-2307, the aircraft's north plus or minus 154 m)."""
    return GroundScene(points=[Point(GroundFeature.SHIP, 190, 90.0, (2154.0, 1760.0), (2154.0, 2200.0), 5.0)],
                       lines=[Line(GroundFeature.ROAD, 20, 12.0, (2500.0, -2000.0), (2500.0, 30000.0))],
                       areas=[Area(GroundFeature.LAKE, 6, 500.0, (1900.0, 1850.0))])


def pixels_of(image):
    out = []
    for p in image.pixel:
        out.append(p.even)
        out.append(p.odd)
    return out


def column_mean(pixels, col):
    return sum(pixels[row * 64 + col] for row in range(64)) / 64.0


def bright_rows(pixels, threshold=120):
    rows = [i // 64 for i, v in enumerate(pixels) if v > threshold]
    return rows


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", *BINARIES)
class SarEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_sar_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=5, ground=test_scene())
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=10000.0, vertical_speed=0.0, pitch=0.0,
                                  roll=0.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for name in BINARIES:
                cls.processes.append(subprocess.Popen(
                    [os.environ[name], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
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
            [os.environ["FMOFP_BC"], "--rt", str(PLANT_RT), "--rt", str(m.FcsStatus.RT), "--rt", str(m.NavSolution.RT),
             "--rt", str(m.FmsStatus.RT), "--rt", str(RADAR_RT), "--port-base", str(PORT_BASE), "--frames",
             str(FRAMES), "--free-run", "--strict", "--timeout-ms", "40", "--script", self.script],  # the plant answers five terminals from one Python thread for 950 frames
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=120), output)
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

    def test_the_radar_pictures_the_plants_ground_and_spotlights_the_ship(self):
        self.monitor.drain()
        records, _ = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # The plant's scene reaches the radar by relay, on SA 26 and 27.
        scenes = []
        for _, version, payload in blocks_from_records(records, PLANT_RT, b.PlantGround.DESCRIPTOR_SUBADDRESS,
                                                       b.PlantGround.SEGMENT_SUBADDRESS):
            status, scene = b.PlantGround.decode(payload)
            self.assertEqual(Status.OK, status, version)
            scenes.append(scene)
        self.assertGreaterEqual(len(scenes), FRAMES // 50 - 2, "a publication a second")
        self.assertEqual((1, 1, 1), (len(scenes[0].point), len(scenes[0].line), len(scenes[0].area)))
        relayed = [r for _, r in framed if not r.command.transmit and not r.command.is_mode_code
                   and r.command.rt == RADAR_RT
                   and r.command.subaddress in (b.RdrSarScene.DESCRIPTOR_SUBADDRESS, b.RdrSarScene.SEGMENT_SUBADDRESS)]
        self.assertGreaterEqual(len(relayed), FRAMES // 5 - 10)
        self.assertTrue(all(r.answered for r in relayed))

        # Status: imaging the strip once it has the scene and a fix, the spotlight from frame 400, standby from 870.
        stripping = self.transmitted(framed, m.RdrSarStatus, 200, SPOTLIGHT_FRAME)
        self.assertTrue(stripping)
        s = stripping[-1][1]
        self.assertEqual((TerminalState.NORMAL, SarMode.STRIPMAP, 1, False, True, True),
                         (s.state, s.mode, s.last_cmd_seq, s.cmd_rejected, s.scene_valid, s.nav_valid))
        self.assertGreaterEqual(s.image_seq, 1, "a strip image by frame 400")
        spotting = self.transmitted(framed, m.RdrSarStatus, SPOTLIGHT_FRAME + 20, STANDBY_FRAME)
        t = spotting[-1][1]
        self.assertEqual((SarMode.SPOTLIGHT, 2, False), (t.mode, t.last_cmd_seq, t.mode_transition))
        self.assertGreaterEqual(t.image_seq, s.image_seq + 2, "two spotlight images")
        standby = self.transmitted(framed, m.RdrSarStatus, STANDBY_FRAME + 10, FRAMES)[-1][1]
        self.assertEqual((TerminalState.STANDBY, SarMode.NONE, 3, False), (standby.state, standby.mode,
                                                                           standby.last_cmd_seq, standby.cmd_rejected))
        self.assertEqual(t.image_seq, standby.image_seq, "the last image's number stays")
        term = self.transmitted(framed, m.TermStatus, STANDBY_FRAME + 10, FRAMES, rt=RADAR_RT)[-1][1]
        self.assertEqual((3, False), (term.last_cmd_seq, term.cmd_rejected), "TERM_STATUS reports the SAR's command")

        # The image blocks: the strip with the road in column 31, two spotlights of one patch with the ship moving.
        images = []
        for _, version, payload in blocks_from_records(records, RADAR_RT, b.RdrSarImage.DESCRIPTOR_SUBADDRESS,
                                                       b.RdrSarImage.SEGMENT_SUBADDRESS):
            status, image = b.RdrSarImage.decode(payload)
            self.assertEqual(Status.OK, status, version)
            images.append(image)
        self.assertEqual(SarMode.NONE, images[0].mode, "the empty block first")
        self.assertEqual(0, len(images[0].pixel))
        strips = [i for i in images if i.mode == SarMode.STRIPMAP]
        self.assertGreaterEqual(len(strips), 1)
        strip = strips[0]
        self.assertEqual(2048, len(strip.pixel))
        self.assertEqual((1.0, 16.0, 0.0, 2512.0), (strip.resolution, strip.pixel_size, strip.centre_along,
                                                    strip.centre_cross))
        self.assertLess(min(strip.heading, 360.0 - strip.heading), 1.0, "the picture points along the held heading")
        self.assertLess(abs(strip.centre_latitude), 0.05)
        self.assertAlmostEqual(2512.0 / 111195.0, strip.centre_longitude, delta=0.0005, msg="2.5 km east")
        px = pixels_of(strip)
        self.assertLess(column_mean(px, 31) + 10.0, column_mean(px, 10), "the road, 2.5 km east, darkens column 31")
        self.assertEqual(image_seqs(strips), sorted(image_seqs(strips)))
        spots = [i for i in images if i.mode == SarMode.SPOTLIGHT]
        self.assertGreaterEqual(len(spots), 2, "two spotlight pictures reached the bus")
        self.assertEqual((0.3, 4.8), (spots[0].resolution, spots[0].pixel_size))
        self.assertAlmostEqual(2000.0 + 32.0 * 4.8, spots[0].centre_cross, delta=0.5)  # a 1 m field
        self.assertEqual((spots[0].centre_latitude, spots[0].centre_longitude),
                         (spots[1].centre_latitude, spots[1].centre_longitude), "a spotlight stares at one patch")
        self.assertAlmostEqual(1855.0 / 111195.0, spots[0].centre_latitude, delta=60.0 / 111195.0,
                               msg="the patch abeam where the aircraft was at 8 s")
        first, second = pixels_of(spots[0]), pixels_of(spots[1])
        self.assertLess(sorted(first)[2048], 25, "the lake fills most of the patch")
        rows_first, rows_second = bright_rows(first), bright_rows(second)
        self.assertGreater(len(rows_first), 200, "the ship, 19 pixels square")
        shift = sum(rows_first) / len(rows_first) - sum(rows_second) / len(rows_second)
        self.assertTrue(2.0 <= shift <= 4.5, f"15 m north in 3 s is three 4.8 m pixels up the picture: {shift}")
        for image in spots:
            self.assertLessEqual(len(image.change), b.RdrSarImage.MAX_CHANGE)
            self.assertLessEqual(image.changes, 15)
        self.assertEqual(SarMode.NONE, images[-1].mode, "standby empties the block")
        self.assertEqual(0, len(images[-1].pixel))


def image_seqs(images):
    return [i.image_seq for i in images]


if __name__ == "__main__":
    unittest.main()
