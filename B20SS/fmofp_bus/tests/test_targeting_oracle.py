"""The targeting radar's processing and the track fusion against the frozen Python.

target_processor.py's SignatureAnalyzer and StealthDetector, and radar_data_fusion.py's
RadarDataFusion (_correlate_and_merge, _reassign_ids, _assess_threat), run beside
fmofp_radar_tool's signature / stealth / threat / fusion commands on the same
inputs. The Python drew its scintillation from the process-wide generator; the
tool takes it as an argument and the Python gets the same value through a patch.

Needs FMOFP_RADAR_TOOL (CI sets it).
"""

import unittest
from unittest import mock

from fmofp_bus.tests._support import RadarTool, needs

SENSOR_BITS = {"targeting_radar": 1, "aewc_radar": 2}


def report(x, y, z, vx, vy, vz, classification, rcs, weight, sensor):
    """RadarDataFusion._collect_reports' dictionary."""
    return {"position": (float(x), float(y), float(z)), "velocity": (float(vx), float(vy), float(vz)),
            "classification": classification, "identity": "UNKNOWN", "rcs": float(rcs), "is_stealth": False,
            "sensor": sensor, "weight": weight, "source_id": "s", "timestamp": 0.0}


@needs("FMOFP_RADAR_TOOL")
class TargetProcessingOracleTest(unittest.TestCase):
    def test_signature_analysis_matches(self):
        from FMOFP.Systems.radarManagement.targeting.target_processor import SignatureAnalyzer
        cases = [(rcs, s, speed, alt)
                 for rcs in (0.02, 0.3, 2.0, 7.5, 60.0)
                 for s in (0.0, 0.1, 0.7, 1.0, 2.5)
                 for speed, alt in ((100.0, 0.0), (300.0, 6000.0), (300.0, 1000.0), (600.0, 0.0), (120.0, 16000.0))]
        tool = RadarTool()
        for rcs, s, speed, alt in cases:
            tool.send(f"signature {rcs!r} {s!r} {speed!r} {alt!r}")
        tool.run()
        analyzer = SignatureAnalyzer()
        for i, (rcs, s, speed, alt) in enumerate(cases):
            track = {"rcs": rcs, "velocity": (speed, 0.0, 0.0), "position": (0.0, 0.0, alt)}
            with mock.patch("random.expovariate", return_value=s):
                analyzer.analyse(i, track)
            python = analyzer.get_report(i)
            row = tool.line()
            self.assertEqual(track["classification"], row[0], (rcs, s, speed, alt))
            self.assertAlmostEqual(python.confidence, float(row[1]), delta=1e-12, msg=(rcs, s, speed, alt))
            self.assertAlmostEqual(python.rcs_measured, float(row[2]), delta=1e-12, msg=(rcs, s, speed, alt))

    def test_stealth_detection_matches(self):
        from FMOFP.Systems.radarManagement.targeting.target_processor import StealthDetector
        sequence = [(0.05, 20.0), (0.05, 18.0), (0.3, 15.0), (5.0, 25.0), (0.02, 10.0), (0.02, 10.0), (0.02, 10.0),
                    (0.08, 30.0), (0.6, 12.0)]
        tool = RadarTool()
        tool.send("stealth_reset")
        for rcs, snr in sequence:
            tool.send(f"stealth {rcs!r} {snr!r}")
        tool.run()
        self.assertEqual(["ok"], tool.line())
        detector = StealthDetector()
        for rcs, snr in sequence:
            track = {"rcs": rcs, "snr": snr}
            detector.assess(1, track, None)
            row = tool.line()
            self.assertAlmostEqual(track["stealth_prob"], round(float(row[0]), 3), delta=1e-12, msg=(rcs, snr))
            self.assertAlmostEqual(track["snr_integrated"], round(float(row[1]), 1), delta=1e-12, msg=(rcs, snr))
            self.assertEqual(detector._hit_counts[1], int(row[2]))
            self.assertEqual(track["is_stealth"], row[3] == "1", (rcs, snr))
            self.assertAlmostEqual(detector._stealth_probs[1], float(row[0]), delta=1e-12)

    def test_the_threat_rule_matches(self):
        from FMOFP.Systems.radarManagement.radar_data_fusion import RadarDataFusion
        cases = [("FIGHTER", "UNKNOWN", 2.9, 250.0), ("FIGHTER", "UNKNOWN", 3.0, 250.0),
                 ("FIGHTER", "UNKNOWN", 2.9, 200.0), ("UNKNOWN", "HOSTILE", 50.0, 10.0),
                 ("BOMBER", "FRIENDLY", 1.0, 400.0), ("STEALTH", "NEUTRAL", 0.05, 100.0),
                 ("MISSILE", "UNKNOWN", 0.05, 800.0), ("HIGH_ALT", "UNKNOWN", 10.0, 300.0)]
        tool = RadarTool()
        for case in cases:
            tool.send("threat %s %s %r %r" % case)
        tool.run()
        for classification, identity, rcs, speed in cases:
            cluster = [{"rcs": rcs, "velocity": (speed, 0.0, 0.0)}]
            self.assertEqual(RadarDataFusion._assess_threat(classification, identity, cluster), tool.line()[0],
                             (classification, identity, rcs, speed))


@needs("FMOFP_RADAR_TOOL")
class TrackFusionOracleTest(unittest.TestCase):
    def test_fusion_matches_cluster_by_cluster_and_id_by_id(self):
        from FMOFP.Systems.radarManagement.radar_data_fusion import RadarDataFusion
        epochs = [
            [report(10000, 0, 0, 0, 300, 0, "FIGHTER", 2.0, 0.8, "targeting_radar"),
             report(10200, 100, 0, 0, 280, 0, "UNKNOWN", 2.0, 0.7, "aewc_radar"),
             report(-30000, 5000, 0, 100, 0, 0, "BOMBER", 40.0, 0.8, "targeting_radar"),
             report(0, 50000, 1000, 0, -200, 0, "HIGH_ALT", 10.0, 0.7, "aewc_radar"),
             report(300, 50100, 1000, 0, -200, 0, "HIGH_ALT", 12.0, 0.7, "aewc_radar"),
             report(80000, -20000, -3000, -400, 50, 0, "STEALTH", 0.05, 0.7, "aewc_radar")],
            # Half a second on: four moved, the bomber gone, a drone new, the stealth report now from both sensors.
            [report(10000, 150, 0, 0, 300, 0, "FIGHTER", 2.0, 0.8, "targeting_radar"),
             report(10200, 240, 0, 0, 280, 0, "UNKNOWN", 2.0, 0.7, "aewc_radar"),
             report(0, 49900, 1000, 0, -200, 0, "HIGH_ALT", 10.0, 0.7, "aewc_radar"),
             report(300, 50000, 1000, 0, -200, 0, "HIGH_ALT", 12.0, 0.7, "aewc_radar"),
             report(79800, -19975, -3000, -400, 50, 0, "STEALTH", 0.05, 0.7, "aewc_radar"),
             report(79850, -19960, -3010, -395, 52, 0, "FIGHTER", 0.06, 0.8, "targeting_radar"),
             report(-5000, -5000, 200, 20, 20, 0, "DRONE", 0.01, 0.8, "targeting_radar")],
            [],
            [report(-5000, -4990, 200, 20, 20, 0, "DRONE", 0.01, 0.8, "targeting_radar")],
        ]
        tool = RadarTool()
        tool.send("fusion_reset")
        for epoch in epochs:
            tool.send(f"fusion {len(epoch)}")
            for r in epoch:
                tool.send("%r %r %r %r %r %r %s %s %r %r %d" % (*r["position"], *r["velocity"], r["classification"],
                                                               r["identity"], r["rcs"], r["weight"],
                                                               SENSOR_BITS[r["sensor"]]))
        tool.run()
        self.assertEqual(["ok"], tool.line())
        fusion = RadarDataFusion()
        fusion._fused_tracks = {}
        previous_python = set()
        previous_cpp = set()
        for n, epoch in enumerate(epochs):
            table = fusion._correlate_and_merge(epoch)
            fusion._fused_tracks = table  # what _update() does with it
            python = list(table.values())  # cluster order
            header = tool.line()
            self.assertEqual("n", header[0])
            rows = tool.block()
            self.assertEqual(len(python), int(header[1]), f"epoch {n}")
            self.assertEqual(len(python), len(rows), f"epoch {n}")
            self.assertEqual(sum(1 for t in python if t.threat == "HOSTILE"), int(header[2]), f"epoch {n}")
            for t, row in zip(python, rows):
                for i in range(3):
                    self.assertAlmostEqual(t.position_m[i], float(row[1 + i]), delta=1e-6, msg=f"epoch {n}")
                    self.assertAlmostEqual(t.velocity_ms[i], float(row[4 + i]), delta=1e-9, msg=f"epoch {n}")
                self.assertAlmostEqual(t.confidence, float(row[7]), delta=1e-12)
                self.assertEqual((t.classification, t.identity, t.threat), (row[8], row[9], row[10]), f"epoch {n}")
                bits = int(row[11])
                self.assertEqual(sorted(t.sources), sorted(name for name, bit in SENSOR_BITS.items() if bits & bit))
            # The same tracks carried their ids over from the previous epoch (the Python's are UUIDs, so the pattern).
            self.assertEqual([t.track_id in previous_python for t in python],
                             [int(row[0]) in previous_cpp for row in rows], f"epoch {n}")
            self.assertEqual(len(python), len({t.track_id for t in python}), "ids are unique")
            self.assertEqual(len(rows), len({row[0] for row in rows}))
            previous_python = {t.track_id for t in python}
            previous_cpp = {row[0] for row in rows}
            previous_cpp = {int(x) for x in previous_cpp}


if __name__ == "__main__":
    unittest.main()
