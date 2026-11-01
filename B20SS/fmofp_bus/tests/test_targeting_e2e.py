"""The ported targeting radar and the track fusion on the bus.

Plant (Python, in-process, flying its traffic), the real fmofp_radar and the real
fmofp_bc with a command script: the targeting radar is switched to TARGET_SEARCH,
then TARGET_TRACK with a lock on track 1, then STANDBY. The monitor port shows
the plant's traffic block reaching the radar by relay, the status the radar
reports, the RDR_TGT_TRACKS block it publishes and the RDR_FUSED_TRACKS block the
fusion makes of it. The processing itself is checked against the Python in
test_targeting_oracle.py; this test checks the plumbing, the cadence and the
consistency of what arrives.

Needs FMOFP_BC and FMOFP_RADAR (CI sets them).
"""

import math
import os
import subprocess
import tempfile
import threading
import time
import unittest

from fmofp_bus.block import blocks_from_records
from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import LockStatus, TargetingMode, TerminalState, Threat, TrackFilter
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal
from fmofp_plant.traffic import MAX_TARGETS

PORT_BASE = 60000
FRAMES = 300
SYNC_MODE_CODE = 17
RADAR_RT = m.RdrTgtStatus.RT
PLANT_RT = m.PlantSensors.RT

SCRIPT = [
    (5, m.RdrTgtModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL, requested_mode=TargetingMode.TARGET_SEARCH,
                        track_filter=TrackFilter.ALL)),
    (120, m.RdrTgtModeCmd(cmd_seq=2, requested_state=TerminalState.NORMAL, requested_mode=TargetingMode.TARGET_TRACK,
                          track_filter=TrackFilter.ALL, lock_track_id=1)),
    (240, m.RdrTgtModeCmd(cmd_seq=3, requested_state=TerminalState.STANDBY, requested_mode=TargetingMode.NONE)),
]


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


def position_of(record):
    """The metres east, north and up a radar reconstructs from a PLANT_TRAFFIC record."""
    range_m = record.range * 1000.0
    az = math.radians(record.azimuth)
    el = math.radians(record.elevation)
    return (range_m * math.cos(el) * math.sin(az), range_m * math.cos(el) * math.cos(az), range_m * math.sin(el))


def confidence_of(x, y, z):
    snr = 30.0 - 20.0 * math.log10(math.sqrt(x * x + y * y + z * z) / 1000.0)
    return max(0.0, min(1.0, snr / 30.0))


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_RADAR")
class TargetingEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_tgt_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=5)
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=10000.0, vertical_speed=0.0)
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
            [os.environ["FMOFP_BC"], "--rt", str(PLANT_RT), "--rt", str(RADAR_RT), "--port-base", str(PORT_BASE),
             "--frames", str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script", self.script],
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

    def decoded_blocks(self, records, rt, cls):
        out = []
        for _, version, payload in blocks_from_records(records, rt, cls.DESCRIPTOR_SUBADDRESS, cls.SEGMENT_SUBADDRESS):
            status, block = cls.decode(payload)
            self.assertEqual(Status.OK, status, version)
            out.append(block)
        return out

    def test_the_radar_tracks_the_plants_traffic_and_the_fusion_follows(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # The plant's traffic: twelve aircraft, five times a second, relayed to the radar on SA 8 and 9.
        traffic = self.decoded_blocks(records, PLANT_RT, b.PlantTraffic)
        self.assertGreaterEqual(len(traffic), FRAMES // 10 - 4)
        seqs = [t.traffic_seq for t in traffic]
        self.assertEqual(sorted(set(seqs)), seqs, "every publication once, in order")
        for t in traffic:
            self.assertEqual(MAX_TARGETS, len(t.target))
            self.assertTrue(all(0.0 < r.range <= 400.0 for r in t.target))
        relayed = [r for _, r in framed if not r.command.transmit and not r.command.is_mode_code
                   and r.command.rt == RADAR_RT
                   and r.command.subaddress in (b.RdrTraffic.DESCRIPTOR_SUBADDRESS, b.RdrTraffic.SEGMENT_SUBADDRESS)]
        self.assertGreaterEqual(len(relayed), FRAMES - 20)
        self.assertTrue(all(r.answered for r in relayed))
        near = {t.traffic_seq: [r for r in t.target if r.range <= 100.0] for t in traffic}
        self.assertTrue(all(near.values()), "the targeting generator's aircraft stay within 100 km for a while")

        # Status: searching with tracks from frame 5, locked on track 1 from frame 120, standby from 240.
        searching = self.transmitted(framed, m.RdrTgtStatus, 40, 120)
        self.assertTrue(searching)
        s = searching[-1][1]
        self.assertEqual((TerminalState.NORMAL, TargetingMode.TARGET_SEARCH, 1, False, LockStatus.NONE, 0),
                         (s.state, s.mode, s.last_cmd_seq, s.cmd_rejected, s.lock_status, s.locked_track_id))
        self.assertGreaterEqual(s.track_count, 1)
        self.assertEqual(TrackFilter.ALL, s.track_filter)
        tracking = self.transmitted(framed, m.RdrTgtStatus, 160, 240)
        t = tracking[-1][1]
        self.assertEqual((TargetingMode.TARGET_TRACK, 2, 1), (t.mode, t.last_cmd_seq, t.locked_track_id))
        self.assertIn(t.lock_status, (LockStatus.ACQUIRING, LockStatus.LOCKED))
        self.assertEqual(t.lock_status == LockStatus.LOCKED, t.lock_quality > 0.5)
        standby = self.transmitted(framed, m.RdrTgtStatus, 260, FRAMES)[-1][1]
        self.assertEqual((TerminalState.STANDBY, TargetingMode.NONE, 3, 0, LockStatus.NONE),
                         (standby.state, standby.mode, standby.last_cmd_seq, standby.track_count, standby.lock_status))

        # The tracks block: what the radar made of the traffic, consistent with the plant's records.
        blocks = self.decoded_blocks(records, RADAR_RT, b.RdrTgtTracks)
        self.assertGreaterEqual(len(blocks), 6, "empty, several searching, locked, empty again")
        searching_blocks = [blk for blk in blocks if blk.mode == TargetingMode.TARGET_SEARCH]
        self.assertGreaterEqual(len(searching_blocks), 3)
        checked = 0
        all_positions = [position_of(r) for t in traffic for r in t.target if r.range <= 100.0]
        for blk in searching_blocks:
            self.assertEqual(LockStatus.NONE, blk.lock_status)
            self.assertGreaterEqual(len(blk.track), 1)
            ids = [tr.track_id for tr in blk.track]
            self.assertEqual(sorted(ids), ids, "in track id order")
            for tr in blk.track:
                nearest = min(math.dist((tr.x, tr.y, tr.z), p) for p in all_positions)
                self.assertLess(nearest, 60.0, "a track sits on one of the plant's aircraft (within the words' resolution)")
                self.assertAlmostEqual(confidence_of(tr.x, tr.y, tr.z), tr.confidence, delta=0.011)
                speed = math.sqrt(tr.velocity_east ** 2 + tr.velocity_north ** 2 + tr.velocity_up ** 2)
                self.assertEqual(Threat.UNKNOWN if speed <= 200.0 else tr.threat, tr.threat)
                self.assertFalse(tr.locked)
                checked += 1
        self.assertGreaterEqual(checked, 3)
        locked_blocks = [blk for blk in blocks if blk.mode == TargetingMode.TARGET_TRACK]
        self.assertGreaterEqual(len(locked_blocks), 1)
        last_locked = locked_blocks[-1]
        self.assertEqual(1, last_locked.locked_track_id)
        self.assertIn(last_locked.lock_status, (LockStatus.ACQUIRING, LockStatus.LOCKED))
        flagged = [tr for tr in last_locked.track if tr.locked]
        self.assertEqual([1], [tr.track_id for tr in flagged])
        self.assertAlmostEqual(flagged[0].confidence, last_locked.lock_quality, delta=0.011, msg="both are snr / 30")
        self.assertEqual(TargetingMode.NONE, blocks[-1].mode)
        self.assertEqual(0, len(blocks[-1].track))

        # The fusion: one fused track per targeting track (nothing else contributes yet), confidence 0.8.
        fused = self.decoded_blocks(records, RADAR_RT, b.RdrFusedTracks)
        self.assertGreaterEqual(len(fused), 6, "an epoch every half second")
        epochs = [f.epoch for f in fused]
        self.assertEqual(sorted(epochs), epochs)
        present = [f for f in fused if f.targeting_present]
        self.assertGreaterEqual(len(present), 4)
        for f in present:
            self.assertFalse(f.aewc_present)
            self.assertEqual(sum(1 for tr in f.track if tr.threat == Threat.HOSTILE), f.hostile_count)
            for tr in f.track:
                self.assertAlmostEqual(0.8, tr.confidence, delta=0.006)
                self.assertTrue(tr.from_targeting)
                self.assertFalse(tr.from_aewc)
                nearest = min(math.dist((tr.x, tr.y, tr.z), p) for p in all_positions)
                self.assertLess(nearest, 60.0)
        counts = [len(f.track) for f in present]
        self.assertGreaterEqual(max(counts), 1)
        self.assertFalse(fused[-1].targeting_present, "standby: the sensor no longer contributes")
        self.assertEqual(0, len(fused[-1].track))

        term = self.transmitted(framed, m.TermStatus, 160, 240, rt=RADAR_RT)[-1][1]
        self.assertEqual(TerminalState.NORMAL, term.state)
        self.assertEqual((m.ICD_MAJOR, m.ICD_MINOR, 2), (term.icd_major, term.icd_minor, term.last_cmd_seq))
        self.assertIn("overruns=0", output)


if __name__ == "__main__":
    unittest.main()
