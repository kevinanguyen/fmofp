"""The AEWC radar's arithmetic against the frozen Python.

aewc_radar._calculate_snr and _get_target_sector, and aewc_processor.py's
SectorPriorityManager.update and ElectronicProtection.assess_sector, run beside
fmofp_radar_tool's aewc_snr / aewc_sector / aewc_priority / aewc_ecm commands on
the same inputs. The Python's priority manager reads the wall clock for the
sectors' ages; the test patches it. The classification rule lives inline in
_generate_target (no function to call): aewc_radar_test.cpp pins its thresholds.

Needs FMOFP_RADAR_TOOL (CI sets it).
"""

import unittest
from unittest import mock

from fmofp_bus.tests._support import RadarTool, needs

SECTOR_IDS = [f"SECTOR_{i}" for i in range(1, 7)]


@needs("FMOFP_RADAR_TOOL")
class AewcOracleTest(unittest.TestCase):
    def test_the_radar_equation_and_the_sectors_match(self):
        from FMOFP.Systems.radarManagement.aewc.aewc_radar import aewc_radar
        radar = aewc_radar("aewc_radar", None, None)
        snr_cases = [(rcs, r) for rcs in (0.01, 0.05, 1.0, 10.0, 100.0)
                     for r in (5000.0, 8000.0, 20000.0, 50000.0, 100000.0, 400000.0)]
        positions = [(1000.0, 0.0, 0.0), (0.0, 1000.0, 0.0), (-1000.0, 0.0, 0.0), (0.0, -1000.0, 0.0),
                     (1000.0, -1.0, 0.0), (1000.0, 0.0, -200.0), (1000.0, 0.0, -300.0), (1000.0, 0.0, 1500.0),
                     (-500.0, -866.0, 100.0), (300000.0, 0.0, 20000.0), (59999.0, 103920.0, 0.0), (1.0, 0.0, 0.0)]
        tool = RadarTool()
        for rcs, r in snr_cases:
            tool.send(f"aewc_snr {rcs!r} {r!r}")
        for p in positions:
            tool.send(f"aewc_sector {p[0]!r} {p[1]!r} {p[2]!r}")
        tool.run()
        for rcs, r in snr_cases:
            python = float(radar._calculate_snr({"position": (r, 0.0, 0.0), "rcs": rcs}))
            self.assertAlmostEqual(python, float(tool.line()[0]), delta=1e-9, msg=(rcs, r))
        for p in positions:
            sector = radar._get_target_sector(p)
            expected = 0 if sector is None else int(sector.split("_")[1])
            self.assertEqual(expected, int(tool.line()[0]), p)

    def test_the_electronic_protection_matches(self):
        from FMOFP.Systems.radarManagement.aewc.aewc_processor import ElectronicProtection
        exceedances = [0.0, 5.99, 6.0, 7.5, 10.0, 10.1, 20.0, 20.1, 30.0, 30.1, 40.0, 45.0, 2.5]
        tool = RadarTool()
        for x in exceedances:
            tool.send(f"aewc_ecm {x!r}")
        tool.run()
        for x in exceedances:
            protection = ElectronicProtection()
            contact = protection.assess_sector("SECTOR_1", ElectronicProtection.THERMAL_NOISE_DBM + x)
            jammed, severity, jam_type, mitigation = tool.line()
            if contact is None:
                self.assertEqual(("0", "NONE", "NONE"), (jammed, jam_type, mitigation), x)
            else:
                self.assertEqual("1", jammed, x)
                self.assertAlmostEqual(contact.severity, float(severity), delta=1e-12, msg=x)
                self.assertEqual((contact.jam_type, contact.mitigation_active), (jam_type, mitigation), x)

    def test_the_sector_priorities_match(self):
        from FMOFP.Systems.radarManagement.aewc import aewc_processor
        cases = [
            # (track counts, hostile counts, age in seconds, jammed sectors)
            ([2, 0, 0, 0, 0, 0], [0] * 6, 10.0, set()),
            ([2, 0, 0, 0, 0, 0], [0] * 6, 10.0, {"SECTOR_3"}),
            ([0] * 6, [0] * 6, 0.0, set()),
            ([1, 1, 1, 1, 1, 1], [0, 0, 1, 0, 0, 0], 3.0, {"SECTOR_1", "SECTOR_6"}),
            ([5, 0, 3, 0, 1, 0], [0] * 6, 120.5, set()),
        ]
        tool = RadarTool()
        for counts, hostile, age, jammed in cases:
            mask = sum(1 << i for i, sid in enumerate(SECTOR_IDS) if sid in jammed)
            tool.send("aewc_priority " + " ".join(str(c) for c in counts) + " " + " ".join(str(h) for h in hostile)
                      + " " + " ".join(repr(float(age)) for _ in range(6)) + f" {mask}")
        tool.run()
        for counts, hostile, age, jammed in cases:
            with mock.patch.object(aewc_processor, "time") as clock:
                clock.time.return_value = 1000.0
                manager = aewc_processor.SectorPriorityManager(SECTOR_IDS)
                clock.time.return_value = 1000.0 + age
                order = manager.update(dict(zip(SECTOR_IDS, counts)), dict(zip(SECTOR_IDS, hostile)), jammed)
                dwell = manager.get_dwell_allocation()
                scores = [manager.get_status(sid).priority_score for sid in SECTOR_IDS]
            row = tool.line()
            self.assertEqual("order", row[0])
            self.assertEqual([int(s.split("_")[1]) for s in order], [int(x) for x in row[1:7]], (counts, age, jammed))
            self.assertEqual("dwell", row[7])
            for sid, value in zip(SECTOR_IDS, row[8:14]):
                self.assertAlmostEqual(dwell[sid], float(value), delta=1e-12, msg=(sid, counts, age, jammed))
            self.assertEqual("score", row[14])
            for expected, value in zip(scores, row[15:21]):
                self.assertAlmostEqual(expected, float(value), delta=1e-9)


if __name__ == "__main__":
    unittest.main()
