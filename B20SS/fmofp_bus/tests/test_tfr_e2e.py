"""The ported terrain-following radar on the bus.

Plant (Python, in-process, over its hills), the real fmofp_radar and fmofp_fcs,
and the real fmofp_bc with a command script: terrain following is switched on
at a known frame and off again later. The monitor port shows the plant's
terrain block reaching the radar through the first block relay, the status
and guidance the radar reports, the RDR_TFR_ELEVATION block it publishes, and
the flight control computer's TERRAIN alert following the radar's clearance.
The arithmetic itself is checked against the Python in test_tfr_oracle.py;
this test checks the plumbing, the cadence and the consistency of what arrives.

Needs FMOFP_BC, FMOFP_RADAR and FMOFP_FCS (CI sets them).
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
from fmofp_bus.icd.enums import ClearanceLevel, Ssm, TerminalState, TfrMode
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_plant.plant import PlantModel, PlantTerminal
from fmofp_plant.terrain import PROFILE_SAMPLES, sample_distance

PORT_BASE = 59000
FRAMES = 230
SYNC_MODE_CODE = 17
RADAR_RT = m.RdrTfrStatus.RT
ALTITUDE_FT = 5000.0  # 1524 m: below the highest hills, so the clearance goes negative at times
M_PER_FT = 0.3048
FT_PER_M = 3.28084  # tfr_processor._FT_PER_M

SCRIPT = [
    (5, m.RdrTfrModeCmd(cmd_seq=1, requested_state=TerminalState.NORMAL, requested_mode=TfrMode.TERRAIN_FOLLOWING,
                        set_clearance=984.0)),
    (180, m.RdrTfrModeCmd(cmd_seq=2, requested_state=TerminalState.STANDBY, requested_mode=TfrMode.NONE,
                          set_clearance=984.0)),
]


def write_script(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# frame rt sa words (hex)\n")
        for frame, msg in SCRIPT:
            status, words = msg.encode()
            assert status == Status.OK, (msg, status)
            f.write(f"{frame} {msg.RT} {msg.SUBADDRESS} " + " ".join(f"{w:04X}" for w in words) + "\n")


def clearance_level(clearance_m):
    """tfr_processor's bands."""
    if clearance_m >= 600.0:
        return ClearanceLevel.CLEAR
    if clearance_m >= 300.0:
        return ClearanceLevel.LOW
    if clearance_m >= 150.0:
        return ClearanceLevel.CAUTION
    return ClearanceLevel.PULL_UP


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", "FMOFP_RADAR", "FMOFP_FCS")
class TerrainFollowingEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None

    @classmethod
    def setUpClass(cls):
        cls.script = os.path.join(tempfile.mkdtemp(prefix="fmofp_tfr_"), "commands.txt")
        write_script(cls.script)
        try:
            cls.model = PlantModel(seed=3)
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=ALTITUDE_FT, vertical_speed=0.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            for binary in ("FMOFP_RADAR", "FMOFP_FCS"):
                cls.processes.append(subprocess.Popen(
                    [os.environ[binary], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
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
            [os.environ["FMOFP_BC"], "--rt", "27", "--rt", str(RADAR_RT), "--rt", str(m.FcsStatus.RT), "--port-base",
             str(PORT_BASE), "--frames", str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20", "--script",
             self.script],
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

    def test_the_radar_follows_the_plants_terrain_and_the_fcs_follows_the_radar(self):
        self.monitor.drain()
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # The plant's terrain block is relayed to the radar: descriptor and segment receives on SA 21 and 22.
        relayed = [r for _, r in framed if not r.command.transmit and not r.command.is_mode_code
                   and r.command.rt == RADAR_RT
                   and r.command.subaddress in (b.RdrTfrTerrain.DESCRIPTOR_SUBADDRESS, b.RdrTfrTerrain.SEGMENT_SUBADDRESS)]
        self.assertGreaterEqual(len(relayed), 2 * (FRAMES - 10))
        self.assertTrue(all(r.answered for r in relayed))

        # What the plant published: one profile every five frames, sequence numbers in order.
        plant_blocks = blocks_from_records(records, m.PlantSensors.RT, b.PlantTerrain.DESCRIPTOR_SUBADDRESS,
                                           b.PlantTerrain.SEGMENT_SUBADDRESS)
        self.assertGreaterEqual(len(plant_blocks), FRAMES // 5 - 4)
        profiles = {}
        last_seq = None
        for _, _, payload in plant_blocks:
            status, terrain = b.PlantTerrain.decode(payload)
            self.assertEqual(Status.OK, status)
            self.assertEqual(PROFILE_SAMPLES, len(terrain.elevation))
            self.assertEqual(10000.0, terrain.range_m)
            if last_seq is not None:
                self.assertEqual((last_seq + 1) % 256, terrain.terrain_seq)
            last_seq = terrain.terrain_seq
            profiles[terrain.terrain_seq] = [e.elevation for e in terrain.elevation]
        self.assertGreater(max(max(p) for p in profiles.values()), 1500.0, "the hills have high terrain")

        # Status: NORMAL / TERRAIN_FOLLOWING after the first command, STANDBY after the second.
        status = self.transmitted(framed, m.RdrTfrStatus, 20, 60)
        self.assertTrue(status)
        self.assertEqual((TerminalState.NORMAL, TfrMode.TERRAIN_FOLLOWING, 1, False),
                         (status[-1][1].state, status[-1][1].mode, status[-1][1].last_cmd_seq, status[-1][1].cmd_rejected))
        self.assertFalse(self.transmitted(framed, m.RdrTfrStatus, 100, 170)[-1][1].mode_transition)
        standby = self.transmitted(framed, m.RdrTfrStatus, 195, FRAMES)[-1][1]
        self.assertEqual((TerminalState.STANDBY, TfrMode.NONE, 2), (standby.state, standby.mode, standby.last_cmd_seq))

        # Guidance: the assessment of the profile it names, against the held altitude.
        altitude_m = ALTITUDE_FT * M_PER_FT
        guidance = self.transmitted(framed, m.RdrTfrGuidance, 60, 170)
        self.assertGreaterEqual(len(guidance), 20)
        checked = 0
        for _, g in guidance:
            self.assertEqual(Ssm.NORMAL, g.clearance_ssm)
            self.assertEqual(TfrMode.TERRAIN_FOLLOWING, g.mode)
            self.assertEqual(984.0, g.set_clearance)
            self.assertTrue(g.commands_valid)
            self.assertEqual(g.pull_up, g.level == ClearanceLevel.PULL_UP)
            self.assertNotEqual(ClearanceLevel.NONE, g.level)
            heights = profiles.get(g.profile_seq)
            if heights is None:
                continue  # a profile from before the monitor was listening
            clearances = [altitude_m - h for h in heights]
            lowest = min(clearances)
            self.assertAlmostEqual(lowest * FT_PER_M, g.min_clearance, delta=1.0)
            self.assertAlmostEqual(sample_distance(clearances.index(lowest)), g.min_clearance_distance, delta=1.0)
            self.assertAlmostEqual(clearances[0] * FT_PER_M, g.terrain_clearance, delta=1.0)
            self.assertEqual(g.high_terrain, any(h > 1500.0 for h in heights))
            self.assertGreaterEqual(g.level.value, clearance_level(lowest).value, "the latch never shows better than now")
            checked += 1
        self.assertGreaterEqual(checked, 10)

        # The elevation block: the profile the radar assessed, with its levels, and the path commands.
        blocks = blocks_from_records(records, RADAR_RT, b.RdrTfrElevation.DESCRIPTOR_SUBADDRESS,
                                     b.RdrTfrElevation.SEGMENT_SUBADDRESS)
        self.assertGreaterEqual(len(blocks), 4, "empty, several assessed profiles, empty again")
        decoded = []
        for _, version, payload in blocks:
            status, products = b.RdrTfrElevation.decode(payload)
            self.assertEqual(Status.OK, status, version)
            decoded.append(products)
        assessed = [p for p in decoded if p.mode == TfrMode.TERRAIN_FOLLOWING]
        self.assertGreaterEqual(len(assessed), 3)
        matched = 0
        for p in assessed:
            self.assertEqual(PROFILE_SAMPLES, len(p.profile))
            self.assertEqual(10000.0, p.range_m)
            self.assertEqual(ALTITUDE_FT, p.altitude)
            self.assertEqual(984.0, p.set_clearance)
            self.assertTrue(1 <= len(p.commands) <= b.RdrTfrElevation.MAX_COMMANDS)
            distances = [c.distance for c in p.commands]
            self.assertEqual(sorted(distances), distances, "nearest first")
            self.assertTrue(all(c.reason.name in ("TERRAIN_AVOIDANCE", "CLEARANCE") for c in p.commands))
            heights = profiles.get(p.profile_seq)
            if heights is None:
                continue
            self.assertEqual(heights, [s.elevation for s in p.profile], "the radar reports what it was given")
            for s, h in zip(p.profile, heights):
                self.assertEqual(clearance_level(altitude_m - h), s.level)
                self.assertEqual(h > 1500.0, s.warning.name == "HIGH_TERRAIN")
            self.assertAlmostEqual(min(altitude_m - h for h in heights) * FT_PER_M, p.min_clearance, delta=1.0)
            matched += 1
        self.assertGreaterEqual(matched, 2)
        self.assertEqual(TfrMode.NONE, decoded[-1].mode)
        self.assertEqual(0, len(decoded[-1].profile))

        # The flight control computer: TERRAIN follows the radar's clearance, and goes away with it.
        alerts = self.transmitted(framed, m.FcsGcasAlerts, 60, 170)
        self.assertGreaterEqual(len(alerts), 10)
        recent = {}
        for frame, g in guidance:
            recent[frame] = g.min_clearance
        for frame, a in alerts:
            self.assertEqual(Ssm.NORMAL, a.clearance_ssm)
            self.assertEqual(a.terrain_clearance < 500.0, a.terrain)
            candidates = [v for f, v in recent.items() if frame - 20 <= f <= frame]
            self.assertTrue(candidates)
            self.assertTrue(any(abs(v - a.terrain_clearance) <= 1.0 for v in candidates),
                            f"frame {frame}: {a.terrain_clearance} not among {candidates}")
        after = self.transmitted(framed, m.FcsGcasAlerts, 205, FRAMES)[-1][1]
        self.assertEqual(Ssm.NO_COMPUTED_DATA, after.clearance_ssm)
        self.assertFalse(after.terrain)

        term = self.transmitted(framed, m.TermStatus, 100, 170, rt=RADAR_RT)[-1][1]
        self.assertEqual(TerminalState.NORMAL, term.state)
        self.assertEqual((m.ICD_MAJOR, m.ICD_MINOR), (term.icd_major, term.icd_minor))
        self.assertIn("overruns=0", output)


if __name__ == "__main__":
    unittest.main()
