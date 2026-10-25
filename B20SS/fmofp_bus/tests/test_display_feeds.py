"""The display feeds: bus words into what the Python displays draw. No Qt, no FMOFP imports."""

import math
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import (ClearanceLevel, FmsFlightMode, PathReason, PrecipType, ShearSeverity, Ssm, TerminalState,
                                 TfrMode, TurbulenceCategory, WeatherMode)
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed, cell_velocity_nm_s, polar_to_nm, weather_items


class FakeTerminal:
    """What the feeds ask a DisplayTerminal: fresh(cls), latest(cls), latest_block(cls)."""

    def __init__(self):
        self.messages = {}  # cls -> (message, age)
        self.blocks = {}    # cls -> (payload, version, age)

    def put(self, message, age=0.1):
        self.messages[type(message)] = (message, age)

    def put_block(self, cls, payload, version, age=0.1):
        self.blocks[cls] = (payload, version, age)

    def latest(self, cls):
        return self.messages.get(cls, (None, None))

    def fresh(self, cls, stale_after_s=2.0):
        message, age = self.latest(cls)
        return message if age is not None and age <= stale_after_s else None

    def latest_block(self, cls):
        return self.blocks.get(cls, (None, 0, None))


class FakeCoordinator:
    def __init__(self):
        self.stores = {}
        self.calls = []

    def store_data(self, data_type, items, request_id):
        self.stores[data_type] = list(items)
        self.calls.append(("store", data_type, len(items), request_id))
        return len(items)

    def reset_data(self, data_type=None):
        self.stores[data_type] = []
        self.calls.append(("reset", data_type))


def sample_products(scan_id=3):
    p = b.RdrWxProducts(scan_id=scan_id, mode=WeatherMode.SURVEILLANCE, max_reflectivity=55.0)
    p.vil = [b.RdrWxProductsVil(azimuth=90.0, range=18.52, value=12.0, intensity=0.4, layer_count=4, show_values=True),
             b.RdrWxProductsVil(azimuth=0.0, range=37.04, value=30.0, intensity=0.9, layer_count=9, show_values=True)]
    p.precip = [b.RdrWxProductsPrecip(azimuth=180.0, range=9.26, rate=60.0, intensity=1.0, type=PrecipType.HAIL,
                                      show_values=True)]
    p.cells = [b.RdrWxProductsCells(cell_id=7, azimuth=45.0, range=74.08, reflectivity=52.0, size=4.0, intensity=0.6,
                                    azimuth_rate=0.0, range_rate=0.001852, altitude=18000.0, vertical_development=100.0)]
    p.windshear = [b.RdrWxProductsWindshear(azimuth=270.0, range=1.852, shear=15.0, severity=ShearSeverity.LOW,
                                            microburst=True, divergence=0.1, altitude=900.0)]
    p.turbulence = [b.RdrWxProductsTurbulence(azimuth=0.0, range=1.852, intensity=0.5, spectrum_width=2.0,
                                              category=TurbulenceCategory.MODERATE, altitude=1200.0)]
    return p


def status(state, mode):
    return m.DspRdrWxStatus(payload=m.RdrWxStatus(state=state, mode=mode))


def sample_elevation(samples=100, range_m=10000.0, mode=TfrMode.TERRAIN_FOLLOWING):
    e = b.RdrTfrElevation(profile_seq=5, mode=mode, level=ClearanceLevel.CAUTION, range_m=range_m, altitude=4000.0,
                          set_clearance=984.0, min_clearance=600.0)
    e.profile = [b.RdrTfrElevationProfile(elevation=1000.0 + 500.0 * math.sin(i / 10.0), level=ClearanceLevel.CLEAR)
                 for i in range(samples)]
    e.commands = [b.RdrTfrElevationCommands(distance=0.0, altitude=1219.0, reason=PathReason.TERRAIN_AVOIDANCE)]
    return e


class WeatherItemsTest(unittest.TestCase):
    def test_positions_are_east_and_north_in_nautical_miles(self):
        self.assertAlmostEqual(10.0, polar_to_nm(90.0, 18.52)[0])
        self.assertAlmostEqual(0.0, polar_to_nm(90.0, 18.52)[1])
        self.assertAlmostEqual(20.0, polar_to_nm(0.0, 37.04)[1])
        self.assertAlmostEqual(-5.0, polar_to_nm(180.0, 9.26)[1])
        vx, vy = cell_velocity_nm_s(90.0, 18.52, 0.0, 0.001852)
        self.assertAlmostEqual(0.001, vx, msg="a radial rate at azimuth 90 is eastward")
        self.assertAlmostEqual(0.0, vy)
        vx, vy = cell_velocity_nm_s(0.0, 18.52, 1.0, 0.0)
        self.assertAlmostEqual(10.0 * math.radians(1.0), vx, msg="an azimuth rate at azimuth 0 is eastward")

    def test_items_have_the_shapes_the_bridge_stored(self):
        items = weather_items(sample_products(), "wx3v1", 1000.0)
        vil = items["vil"]
        self.assertEqual(2, len(vil))
        self.assertAlmostEqual(10.0, vil[0]["position"][0])
        self.assertEqual({"position", "value", "layer_count", "intensity", "show_values", "id", "timestamp"}, set(vil[0]))
        self.assertEqual(("wx3v1_vil_0", "wx3v1_vil_1"), (vil[0]["id"], vil[1]["id"]))
        precip = items["precipitation"][0]
        self.assertEqual(("hail", "hail", 60.0, 1.0), (precip["type"], precip["precip_type"], precip["rate"],
                                                         precip["intensity"]))
        cell = items["cells"][0]
        self.assertEqual(7, cell["cell_id"])
        self.assertEqual(cell["position"], (cell["x"], cell["y"]))
        self.assertAlmostEqual(40.0 * math.sin(math.radians(45.0)), cell["x"])
        self.assertEqual((52.0, 4.0, 0.6, 18000.0, 100.0), (cell["reflectivity"], cell["size"], cell["intensity"],
                                                             cell["altitude"], cell["vertical_development"]))
        shear = items["windshear"][0]
        self.assertEqual(("LOW", True, 15.0, 900.0, 0), (shear["severity"], shear["is_microburst"], shear["shear_knots"],
                                                         shear["altitude_ft"], shear["event_id"]))
        self.assertAlmostEqual(0.5, shear["intensity"])
        turb = items["turbulence"][0]
        self.assertEqual(("MODERATE", 0.5, 1200.0, 0), (turb["category"], turb["intensity"], turb["altitude_ft"],
                                                         turb["cell_id"]))
        ids = [item["id"] for store in items.values() for item in store]
        self.assertEqual(len(ids), len(set(ids)))


class RadarFeedTest(unittest.TestCase):
    def setUp(self):
        self.terminal = FakeTerminal()
        self.coordinator = FakeCoordinator()
        self.feed = RadarFeed(self.terminal, self.coordinator, clock=lambda: 1000.0)

    def test_mode_and_range_follow_the_radar(self):
        self.feed.update()
        self.assertEqual("STANDBY", self.feed.weather_draw_mode)
        self.assertEqual(40.0, self.feed.weather_range_nm)
        self.terminal.put(status(TerminalState.NORMAL, WeatherMode.SURVEILLANCE))
        self.terminal.put(m.DspRdrWxScan(payload=m.RdrWxScan(scan_id=1, range_km=100.0)))
        self.feed.update()
        self.assertEqual("SURVEILLANCE", self.feed.weather_draw_mode)
        self.assertAlmostEqual(100.0 / 1.852, self.feed.weather_range_nm)
        self.terminal.put(status(TerminalState.NORMAL, WeatherMode.PRECIPITATION))
        self.feed.update()
        self.assertEqual("MAPPING", self.feed.weather_draw_mode, "the display has no precipitation picture of its own")
        self.terminal.put(status(TerminalState.STANDBY, WeatherMode.NONE))
        self.feed.update()
        self.assertEqual("STANDBY", self.feed.weather_draw_mode)
        self.terminal.put(status(TerminalState.NORMAL, WeatherMode.TURBULENCE), age=3.0)
        self.feed.update()
        self.assertEqual("STANDBY", self.feed.weather_draw_mode, "a stale status is no status")

    def test_each_new_block_is_stored_once_and_stale_data_is_cleared(self):
        self.terminal.put_block(b.DspRdrWxProducts, sample_products(), 1)
        self.feed.update()
        self.assertEqual(["vil", "precipitation", "cells", "windshear", "turbulence"],
                         [c[1] for c in self.coordinator.calls])
        self.assertTrue(all(c[0] == "store" for c in self.coordinator.calls))
        self.assertEqual("wx3v1", self.coordinator.calls[0][3])
        self.feed.update()
        self.assertEqual(5, len(self.coordinator.calls), "the same version is not stored again")
        empty = b.RdrWxProducts(scan_id=4, mode=WeatherMode.NONE)
        self.terminal.put_block(b.DspRdrWxProducts, empty, 2)
        self.feed.update()
        self.assertEqual([("reset", s) for s in ("vil", "precipitation", "cells", "windshear", "turbulence")],
                         self.coordinator.calls[5:])
        self.assertTrue(self.feed.stores_cleared)
        self.terminal.put_block(b.DspRdrWxProducts, sample_products(scan_id=5), 3)
        self.feed.update()
        self.assertEqual(("store", "vil", 2, "wx5v3"), self.coordinator.calls[10])
        self.terminal.put_block(b.DspRdrWxProducts, sample_products(scan_id=5), 3, age=5.0)
        self.feed.update()
        self.assertEqual(("reset", "vil"), self.coordinator.calls[15], "a block older than the stale limit is cleared")
        self.assertEqual(3, self.feed.blocks_applied)

    def test_terrain_display_data(self):
        data = self.feed.tfr_data()
        self.assertEqual(("STANDBY", [], None, "UNKNOWN"), (data["mode_name"], data["tfr_data"],
                                                            data["terrain_clearance"], data["system_health"]))
        self.terminal.put(m.DspRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL,
                                                                    mode=TfrMode.TERRAIN_FOLLOWING)))
        self.terminal.put(m.DspRdrTfrGuidance(payload=m.RdrTfrGuidance(
            clearance_ssm=Ssm.NORMAL, level=ClearanceLevel.CAUTION, mode=TfrMode.TERRAIN_FOLLOWING, pull_up=False,
            commands_valid=True, min_clearance=600.0, commanded_altitude=4000.0, set_clearance=984.0)))
        self.terminal.put_block(b.DspRdrTfrElevation, sample_elevation(), 2)
        data = self.feed.tfr_data()
        self.assertEqual("TRACK", data["mode_name"], "terrain following is drawn as the tracked profile")
        self.assertEqual("NORMAL", data["system_health"])
        self.assertAlmostEqual(600.0 * 0.3048, data["terrain_clearance"])
        self.assertAlmostEqual(4000.0 * 0.3048, data["clearance_altitude"])
        self.assertAlmostEqual(4000.0 * 0.3048, data["current_altitude"])
        self.assertEqual(("CAUTION", False), (data["level"], data["pull_up"]))
        points = data["tfr_data"]
        self.assertEqual(100, len(points))
        self.assertEqual(0.0, points[0]["distance"])
        self.assertAlmostEqual(10000.0 / 99.0, points[1]["distance"])
        self.assertEqual(10000.0, points[-1]["distance"])
        self.assertAlmostEqual(1000.0 + 500.0 * math.sin(0.1), points[1]["elevation"])
        self.terminal.put(m.DspRdrTfrGuidance(payload=m.RdrTfrGuidance(clearance_ssm=Ssm.NO_COMPUTED_DATA)))
        self.terminal.put(m.DspRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.STANDBY, mode=TfrMode.NONE)))
        data = self.feed.tfr_data()
        self.assertEqual(("STANDBY", None, None), (data["mode_name"], data["terrain_clearance"],
                                                   data["clearance_altitude"]))
        self.assertEqual(100, len(data["tfr_data"]), "the last profile stays until it goes stale")
        self.assertIn("TFR STANDBY NONE", self.feed.tfr_status_text())


class FlightDataFeedTest(unittest.TestCase):
    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = FlightDataFeed(self.terminal)

    def flight_data(self, **overrides):
        p = m.FmsFlightData(roll_ssm=Ssm.NORMAL, pitch_ssm=Ssm.NORMAL, heading_ssm=Ssm.NORMAL, airspeed_ssm=Ssm.NORMAL,
                            altitude_ssm=Ssm.NORMAL, vertical_speed_ssm=Ssm.NORMAL, g_ssm=Ssm.NORMAL, aoa_ssm=Ssm.NORMAL,
                            mach_ssm=Ssm.NORMAL, energy_ssm=Ssm.NORMAL, flight_mode=FmsFlightMode.COMBAT, roll=-12.0,
                            pitch=3.5, heading=275.0, airspeed=420.0, altitude=12345.0, vertical_speed=800.0,
                            mach=0.71, g_force=1.3, angle_of_attack=4.0, energy_state=61.0)
        for key, value in overrides.items():
            setattr(p, key, value)
        return m.PfdFlightData(payload=p)

    def test_the_dictionary_the_pfd_reads(self):
        self.assertEqual({}, self.feed.get_flight_data(), "nothing yet: every tape is invalid")
        self.terminal.put(self.flight_data())
        data = self.feed.get_flight_data()
        self.assertEqual({"attitude", "velocity", "navigation", "tactical", "status"}, set(data))
        self.assertEqual({"roll": -12.0, "pitch": 3.5}, data["attitude"])
        self.assertEqual({"airspeed": 420.0, "vertical_speed": 800.0, "mach": 0.71}, data["velocity"])
        self.assertEqual({"heading": 275.0, "altitude": 12345.0}, data["navigation"])
        self.assertEqual({"g_force": 1.3, "aoa": 4.0, "energy_state": 61.0}, data["tactical"])
        self.assertEqual({"mode": "COMBAT"}, data["status"])

    def test_an_invalid_field_is_left_out(self):
        self.terminal.put(self.flight_data(mach_ssm=Ssm.NO_COMPUTED_DATA, roll_ssm=Ssm.FAILURE_WARNING,
                                           pitch_ssm=Ssm.NO_COMPUTED_DATA))
        data = self.feed.get_flight_data()
        self.assertNotIn("attitude", data, "no valid attitude field at all: the group is absent")
        self.assertEqual({"airspeed": 420.0, "vertical_speed": 800.0}, data["velocity"])
        self.terminal.put(self.flight_data(), age=2.5)
        self.assertEqual({}, self.feed.get_flight_data(), "stale data is no data")
        self.assertIn("(stale)", self.feed.status_text())

    def test_tactical_status_from_the_fms_status_and_the_limited_bits(self):
        tactical = self.feed.get_tactical_status()
        self.assertEqual(("UNKNOWN", [], {"max_g_force": 9.0, "max_aoa": 25.0}),
                         (tactical["mode"], tactical["envelope_warnings"], tactical["profile_limits"]))
        self.terminal.put(m.PfdFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.STEALTH)))
        self.terminal.put(m.PfdAttitudeStatus(payload=m.FmsAttitudeStatus(bank_limited=True, yaw_rate_limited=True)))
        tactical = self.feed.get_tactical_status()
        self.assertEqual("STEALTH", tactical["mode"])
        self.assertEqual(["BANK_ANGLE", "YAW_RATE"], tactical["envelope_warnings"])
        self.assertEqual({"max_g_force": 5.0, "max_aoa": 20.0}, tactical["profile_limits"])


if __name__ == "__main__":
    unittest.main()
