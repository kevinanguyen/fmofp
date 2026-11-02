"""The display feeds: bus words into what the Python displays draw. No Qt, no FMOFP imports."""

import math
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import (AewcMode, ClearanceLevel, CountermeasuresState, EcmMitigation, FmsFlightMode, GcasStatus,
                                 IffIdentity, JamType, LockStatus, PathReason, PrecipType, ShearSeverity, Ssm, TargetClass,
                                 TargetingMode, TargetingState, TerminalState, TfrMode, Threat, TurbulenceCategory,
                                 WeaponsState, WeatherMode)
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import (BROADCAST_SOURCES, SYMBOL_CLASS, RadarFeed, aewc_item, cell_velocity_nm_s,
                                      polar_to_nm, targeting_item, weather_items)
from fmofp_display.systems_feed import LIMITS, FusedTrackView, SystemsFeed, TerrainClearance, WeatherSeverity


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
    return m.BcastRdrWxStatus(payload=m.RdrWxStatus(state=state, mode=mode))


def sample_elevation(samples=100, range_m=10000.0, mode=TfrMode.TERRAIN_FOLLOWING):
    e = b.RdrTfrElevation(profile_seq=5, mode=mode, level=ClearanceLevel.CAUTION, range_m=range_m, altitude=4000.0,
                          set_clearance=984.0, min_clearance=600.0)
    e.profile = [b.RdrTfrElevationProfile(elevation=1000.0 + 500.0 * math.sin(i / 10.0), level=ClearanceLevel.CLEAR)
                 for i in range(samples)]
    e.commands = [b.RdrTfrElevationCommands(distance=0.0, altitude=1219.0, reason=PathReason.TERRAIN_AVOIDANCE)]
    return e


def sample_tracks(mode=TargetingMode.TARGET_SEARCH):
    """An RDR_TGT_TRACKS block: a fighter 10 nm east, 1000 m up, and a locked friendly helicopter 20 nm south."""
    t = b.RdrTgtTracks(update_seq=7, mode=mode, lock_status=LockStatus.LOCKED, locked_track_id=2, lock_quality=0.8)
    t.track = [b.RdrTgtTracksTrack(track_id=1, x=18520.0, y=0.0, z=1000.0, velocity_east=0.0, velocity_north=250.0,
                                   classification=TargetClass.FIGHTER, identity=IffIdentity.UNKNOWN,
                                   threat=Threat.HOSTILE, confidence=0.7, stealth_probability=0.05),
               b.RdrTgtTracksTrack(track_id=2, x=0.0, y=-37040.0, z=-500.0, velocity_east=-100.0, velocity_north=0.0,
                                   classification=TargetClass.HELICOPTER, identity=IffIdentity.FRIENDLY,
                                   threat=Threat.FRIENDLY, locked=True, confidence=0.5)]
    return t


def sample_fused(epoch=3):
    """An RDR_FUSED_TRACKS block: a hostile fighter 10 nm east and 10 nm north, an unknown 5 nm south."""
    f = b.RdrFusedTracks(epoch=epoch, hostile_count=1, targeting_present=True)
    f.track = [b.RdrFusedTracksTrack(track_id=1, x=18520.0, y=18520.0, z=0.0, velocity_north=250.0,
                                     classification=TargetClass.FIGHTER, identity=IffIdentity.UNKNOWN,
                                     threat=Threat.HOSTILE, from_targeting=True, confidence=0.8),
               b.RdrFusedTracksTrack(track_id=2, x=0.0, y=-9260.0, z=0.0, classification=TargetClass.UNKNOWN,
                                     identity=IffIdentity.UNKNOWN, threat=Threat.UNKNOWN, from_targeting=True,
                                     confidence=0.8)]
    return f


def sample_aewc(mode=AewcMode.AEWC_SEARCH):
    """An RDR_AEWC_TRACKS block: a conventional fighter 100 nm north-east (no echo at that range in the Python's
    arithmetic) and a stealth aircraft 10 nm east, and six sectors with the first busy and the fourth jammed."""
    t = b.RdrAewcTracks(update_seq=3, mode=mode, scan_azimuth_start=-30.0, scan_azimuth_end=60.0)
    t.track = [b.RdrAewcTracksTrack(track_id=1, x=130968.0, y=130968.0, z=9000.0, velocity_north=-450.0,
                                    classification=TargetClass.FIGHTER, identity=IffIdentity.UNKNOWN,
                                    threat=Threat.UNKNOWN, detected=True, sector=1, confidence=0.0, snr=0.0),
               b.RdrAewcTracksTrack(track_id=2, x=18520.0, y=0.0, z=0.0, velocity_north=250.0,
                                    classification=TargetClass.STEALTH, identity=IffIdentity.UNKNOWN,
                                    threat=Threat.HOSTILE, stealth=True, detected=False, sector=0, confidence=0.3,
                                    snr=9.0)]
    t.sector = [b.RdrAewcTracksSector(azimuth_start=60.0 * i, active_tracks=1 if i == 0 else 0, jammed=(i == 3),
                                      scan_progress=0.3, dwell_fraction=0.25 if i == 0 else 0.15,
                                      priority_score=12.0 if i == 0 else 5.0,
                                      jam_type=JamType.DECEPTIVE if i == 3 else JamType.NONE,
                                      mitigation=EcmMitigation.FREQ_AGILITY if i == 3 else EcmMitigation.NONE,
                                      severity=0.2 if i == 3 else 0.0) for i in range(6)]
    return t


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
        self.terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=1, range_km=100.0)))
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
        self.terminal.put_block(b.BcastRdrWxProducts, sample_products(), 1)
        self.feed.update()
        self.assertEqual(["vil", "precipitation", "cells", "windshear", "turbulence"],
                         [c[1] for c in self.coordinator.calls])
        self.assertTrue(all(c[0] == "store" for c in self.coordinator.calls))
        self.assertEqual("wx3v1", self.coordinator.calls[0][3])
        self.feed.update()
        self.assertEqual(5, len(self.coordinator.calls), "the same version is not stored again")
        empty = b.RdrWxProducts(scan_id=4, mode=WeatherMode.NONE)
        self.terminal.put_block(b.BcastRdrWxProducts, empty, 2)
        self.feed.update()
        self.assertEqual([("reset", s) for s in ("vil", "precipitation", "cells", "windshear", "turbulence")],
                         self.coordinator.calls[5:])
        self.assertTrue(self.feed.stores_cleared)
        self.terminal.put_block(b.BcastRdrWxProducts, sample_products(scan_id=5), 3)
        self.feed.update()
        self.assertEqual(("store", "vil", 2, "wx5v3"), self.coordinator.calls[10])
        self.terminal.put_block(b.BcastRdrWxProducts, sample_products(scan_id=5), 3, age=5.0)
        self.feed.update()
        self.assertEqual(("reset", "vil"), self.coordinator.calls[15], "a block older than the stale limit is cleared")
        self.assertEqual(3, self.feed.blocks_applied)

    def test_terrain_display_data(self):
        data = self.feed.tfr_data()
        self.assertEqual(("STANDBY", [], None, "UNKNOWN"), (data["mode_name"], data["tfr_data"],
                                                            data["terrain_clearance"], data["system_health"]))
        self.terminal.put(m.BcastRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL,
                                                                    mode=TfrMode.TERRAIN_FOLLOWING)))
        self.terminal.put(m.BcastRdrTfrGuidance(payload=m.RdrTfrGuidance(
            clearance_ssm=Ssm.NORMAL, level=ClearanceLevel.CAUTION, mode=TfrMode.TERRAIN_FOLLOWING, pull_up=False,
            commands_valid=True, min_clearance=600.0, commanded_altitude=4000.0, set_clearance=984.0)))
        self.terminal.put_block(b.BcastRdrTfrElevation, sample_elevation(), 2)
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
        self.terminal.put(m.BcastRdrTfrGuidance(payload=m.RdrTfrGuidance(clearance_ssm=Ssm.NO_COMPUTED_DATA)))
        self.terminal.put(m.BcastRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.STANDBY, mode=TfrMode.NONE)))
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
        return m.BcastFmsFlightData(payload=p)

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
        self.assertEqual(("UNKNOWN", [], {"max_g_force": 9.0, "max_aoa": 25.0, "engine_power": 0.7}),
                         (tactical["mode"], tactical["envelope_warnings"], tactical["profile_limits"]))
        self.terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.STEALTH)))
        self.terminal.put(m.BcastFmsAttitudeStatus(payload=m.FmsAttitudeStatus(bank_limited=True, yaw_rate_limited=True)))
        tactical = self.feed.get_tactical_status()
        self.assertEqual("STEALTH", tactical["mode"])
        self.assertEqual(["BANK_ANGLE", "YAW_RATE"], tactical["envelope_warnings"])
        self.assertEqual({"max_g_force": 5.0, "max_aoa": 20.0, "engine_power": 0.6}, tactical["profile_limits"])


def flight_payload(**overrides):
    p = m.FmsFlightData(roll_ssm=Ssm.NORMAL, pitch_ssm=Ssm.NORMAL, heading_ssm=Ssm.NORMAL, airspeed_ssm=Ssm.NORMAL,
                        altitude_ssm=Ssm.NORMAL, vertical_speed_ssm=Ssm.NORMAL, g_ssm=Ssm.NORMAL, aoa_ssm=Ssm.NORMAL,
                        mach_ssm=Ssm.NORMAL, energy_ssm=Ssm.NORMAL, position_ssm=Ssm.NORMAL, flight_mode=FmsFlightMode.NORMAL,
                        roll=-12.0, pitch=3.5, heading=275.0, airspeed=420.0, altitude=5000.0, vertical_speed=800.0,
                        mach=0.71, g_force=1.3, angle_of_attack=4.0, energy_state=61.0, latitude=1.5, longitude=-2.25)
    for key, value in overrides.items():
        setattr(p, key, value)
    return p


class BroadcastSourcesTest(unittest.TestCase):
    def test_every_display_hears_the_same_broadcasts_and_invalidate_restores(self):
        terminal = FakeTerminal()
        coordinator = FakeCoordinator()
        feed = RadarFeed(terminal, coordinator, clock=lambda: 1000.0)
        self.assertIs(BROADCAST_SOURCES, feed.sources)
        self.assertEqual((31, 1, 31, 7), (m.BcastRdrWxStatus.RT, m.BcastRdrWxStatus.SUBADDRESS, b.BcastRdrWxProducts.RT,
                                           b.BcastRdrWxProducts.SEGMENT_SUBADDRESS))
        terminal.put(m.BcastRdrWxStatus(payload=m.RdrWxStatus(state=TerminalState.NORMAL, mode=WeatherMode.MAPPING)))
        terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=2, range_km=50.0)))
        terminal.put_block(b.BcastRdrWxProducts, sample_products(), 4)
        feed.update()
        self.assertEqual("MAPPING", feed.weather_draw_mode)
        self.assertAlmostEqual(50.0 / 1.852, feed.weather_range_nm)
        self.assertEqual(1, feed.blocks_applied)
        feed.update()
        self.assertEqual(1, feed.blocks_applied)
        feed.invalidate()
        feed.update()
        self.assertEqual(2, feed.blocks_applied, "after invalidate() the same block is stored again")
        terminal.put(m.BcastRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL, mode=TfrMode.TFR_SEARCH)))
        terminal.put_block(b.BcastRdrTfrElevation, sample_elevation(mode=TfrMode.TFR_SEARCH), 1)
        data = feed.tfr_data()
        self.assertEqual(("SEARCH", 100), (data["mode_name"], len(data["tfr_data"])))
        self.assertIn("WX NORMAL MAPPING", feed.weather_status_text())
        self.assertIn("TFR NORMAL TFR_SEARCH", feed.tfr_status_text())


class SystemsFeedTest(unittest.TestCase):
    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = SystemsFeed(self.terminal, clock=lambda: 1000.0)

    @staticmethod
    def gcas(**fields):
        return m.BcastFcsGcasAlerts(payload=m.FcsGcasAlerts(**fields))

    def test_alerts_are_the_pythons_dictionaries(self):
        self.assertEqual([], self.feed.get_alerts())
        self.assertEqual("FCS: no data", self.feed.status_text())
        self.terminal.put(self.gcas(status=GcasStatus.WARNING, pullup=True, terrain=True, vertical_speed=-4500.0,
                                    altitude=1500.0, terrain_clearance=300.0, vertical_speed_ssm=Ssm.NORMAL,
                                    altitude_ssm=Ssm.NORMAL, clearance_ssm=Ssm.NORMAL))
        alerts = self.feed.get_alerts()
        self.assertEqual([("PULLUP", 1, "PULL UP  VS -4500 fpm", 1000.0), ("TERRAIN", 1, "TERRAIN  300 ft clearance", 1000.0)],
                         [(a["code"], a["severity"], a["message"], a["ts"]) for a in alerts])
        self.assertEqual("GCAS WARNING PULLUP,TERRAIN", self.feed.status_text())
        self.terminal.put(self.gcas(status=GcasStatus.CAUTION, sink_rate=True, alt_low=True, vertical_speed=-2500.0,
                                    altitude=150.0))
        self.assertEqual([("SINK RATE", 1, "SINK RATE  -2500 fpm"), ("ALT LOW", 2, "ALT LOW  150 ft")],
                         [(a["code"], a["severity"], a["message"]) for a in self.feed.get_alerts()])
        self.terminal.put(self.gcas(status=GcasStatus.ARMED))
        self.assertEqual([], self.feed.get_alerts())
        self.assertEqual("GCAS ARMED", self.feed.status_text())
        self.terminal.put(self.gcas(status=GcasStatus.WARNING, pullup=True), age=3.0)
        self.assertEqual([], self.feed.get_alerts(), "stale alerts are no alerts")
        self.assertEqual(self.feed.get_alerts(), self.feed.gcas.get_alerts(), "the stand-in the page calls")

    def test_exceedances_pair_the_bits_with_the_magnitudes(self):
        self.assertEqual([], self.feed.get_exceedances())
        self.terminal.put(m.BcastFcsPerfStatus(payload=m.FcsPerfStatus(exceed_g=True, exceed_roll=True,
                                                                  exceed_vertical_speed=True, g_ssm=Ssm.NORMAL,
                                                                  aoa_ssm=Ssm.NORMAL, g_force=-9.5, angle_of_attack=10.0)))
        rows = self.feed.get_exceedances()
        self.assertEqual(["g_force", "roll", "vertical_speed"], [r["parameter"] for r in rows])
        self.assertEqual((9.5, 9.0, 1000.0), (rows[0]["value"], rows[0]["limit"], rows[0]["ts"]))
        self.assertTrue(math.isnan(rows[1]["value"]), "no flight data yet: the roll's magnitude is unknown")
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload(roll=-72.456, vertical_speed=-6500.0)))
        rows = self.feed.get_exceedances()
        self.assertEqual((72.46, 60.0), (rows[1]["value"], rows[1]["limit"]))
        self.assertEqual((6500.0, 6000.0), (rows[2]["value"], rows[2]["limit"]))
        self.assertEqual([(bit, name) for bit, name, _ in LIMITS],
                         [("exceed_g", "g_force"), ("exceed_aoa", "angle_of_attack"), ("exceed_roll", "roll"),
                          ("exceed_pitch", "pitch"), ("exceed_vertical_speed", "vertical_speed")])
        self.assertEqual(rows, self.feed.performance.get_exceedances())
        self.assertIn("exceed g_force,roll,vertical_speed", self.feed.status_text())

    def test_pages_without_a_producer_and_the_position_from_the_fms(self):
        self.assertEqual({}, self.feed.engine.get_data(), "no engine computer yet")
        self.assertEqual({}, self.feed.comms.get_data())
        self.assertEqual({}, self.feed.mission.get_data())
        self.assertEqual({"summary": {}, "friendly": [], "enemy": [], "neutral": []}, self.feed.mission.get_oob())
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload()))
        self.assertEqual({"current_position": {"lat": 1.5, "lon": -2.25, "alt_ft": 5000.0}}, self.feed.mission.get_data())
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload(altitude_ssm=Ssm.NO_COMPUTED_DATA)))
        self.assertEqual({"lat": 1.5, "lon": -2.25}, self.feed.mission.get_data()["current_position"])
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload(position_ssm=Ssm.NO_COMPUTED_DATA)))
        self.assertEqual({}, self.feed.mission.get_data())

    def test_engine_rows_are_the_ecus_dictionary_and_the_fuel_is_the_tanks(self):
        engine = m.EngEngineStatus(engine_1_ssm=Ssm.NORMAL, engine_2_ssm=Ssm.NORMAL, thrust_1=80.0, n1_1=82.004,
                                   n2_1=82.0, egt_1=710.25, fuel_flow_1=3360.0, oil_pressure_1=61.237,
                                   oil_temperature_1=95.5, vibration_1=0.3124, compressor_efficiency_1=88.0,
                                   thrust_2=40.0, n1_2=50.0, total_fuel_flow=5040.0, egt_high_1=False)
        self.terminal.put(m.BcastEngEngineStatus(payload=engine))
        data = self.feed.engine.get_data()
        self.assertEqual({"thrust_pct": 80.0, "n1_pct": 82.0, "n2_pct": 82.0, "egt_c": 710.2, "ff_kgh": 3360.0,
                          "oil_psi": 61.24, "oil_temp_c": 95.5, "vibration": 0.312, "compressor_eff": 88.0,
                          "timestamp": 1000.0}, data, "engine 1, rounded as EngineControlUnit._snapshot did")
        self.assertIn("ENG N1 82.0% EGT 710C", self.feed.status_text())
        engine.engine_1_ssm = Ssm.NO_COMPUTED_DATA
        self.terminal.put(m.BcastEngEngineStatus(payload=engine))
        self.assertEqual({}, self.feed.engine.get_data(), "an invalid engine is no engine")
        self.terminal.put(m.BcastEngEngineStatus(payload=m.EngEngineStatus(engine_1_ssm=Ssm.NORMAL, n1_1=58.0)), age=3.0)
        self.assertEqual({}, self.feed.engine.get_data(), "stale data is no data")

        self.assertEqual({}, self.feed.fuel_data())
        self.terminal.put(m.BcastEngFuelStatus(payload=m.EngFuelStatus(main_left=1500.0, main_right=9000.0, center=900.0,
                                                                       total=11400.0, burn_rate=8400.0, transferred=1000.0,
                                                                       low_left=True, critical_center=True,
                                                                       leak_suspected=True, transfer_left=True)))
        fuel = self.feed.fuel_data()
        self.assertEqual((1500.0, 9000.0, 900.0, 11400.0, 8400.0, 1000.0),
                         (fuel["Main Left"], fuel["Main Right"], fuel["Center"], fuel["total"], fuel["burn_rate_kgh"],
                          fuel["transferred_kg"]))
        self.assertEqual((["Main Left"], ["Center"], True), (fuel["low"], fuel["critical"], fuel["leak_suspected"]))
        self.assertIn("fuel 11400 kg CRITICAL Center LEAK?", self.feed.status_text())

    def test_the_terrain_clearance_stand_in(self):
        radar = next(iter(self.feed.radar_management.radars.values()))
        self.assertIsInstance(radar, TerrainClearance)
        self.assertEqual((0.0, "UNKNOWN"), (radar.clearance_manager.min_clearance_m, radar.clearance_manager.master_level),
                         "the terrain display cannot draw a None clearance")
        self.feed.terrain = (182.88, "CAUTION")
        self.assertEqual((182.88, "CAUTION"), (radar.clearance_manager.min_clearance_m,
                                                radar.clearance_manager.master_level))
        self.feed.terrain = (None, "NONE")
        self.assertEqual((0.0, "UNKNOWN"), (radar.clearance_manager.min_clearance_m, radar.clearance_manager.master_level))


class EicasFeedTest(unittest.TestCase):
    """What eicas._poll_data() reads beyond the MFD's getters, from the same SystemsFeed."""

    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = SystemsFeed(self.terminal, clock=lambda: 1000.0)

    def test_the_fms_getters_are_the_flight_data_feed(self):
        self.assertIsInstance(self.feed.fms, FlightDataFeed)
        self.assertIs(self.feed.fms, self.feed.fms_control)
        self.assertEqual({}, self.feed.fms.get_flight_data())
        self.assertEqual(0.7, self.feed.fms_control.get_tactical_status()["profile_limits"]["engine_power"],
                         "the EICAS compares the thrust unconditionally: NORMAL's power until the FMS says otherwise")
        self.terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT)))
        self.assertEqual(1.0, self.feed.fms_control.get_tactical_status()["profile_limits"]["engine_power"])

    def test_engine_readings_are_what_set_engine_readings_takes(self):
        self.assertEqual({}, self.feed.engine_readings())
        self.terminal.put(m.BcastEngEngineStatus(payload=m.EngEngineStatus(
            engine_1_ssm=Ssm.NORMAL, oil_pressure_1=61.237, oil_temperature_1=95.5, vibration_1=0.3124)))
        self.assertEqual({"oil_psi": 61.24, "oil_temp_c": 95.5, "vib": 0.312}, self.feed.engine_readings())
        self.terminal.put(m.BcastEngFuelStatus(payload=m.EngFuelStatus(main_left=9000.0, main_right=10000.0,
                                                                       center=20000.0, total=39000.0)))
        readings = self.feed.engine_readings()
        self.assertEqual((39000.0, -1000.0), (readings["total_kg"], readings["balance_kg"]),
                         "balance = Main Left - Main Right, the imbalance the EICAS alarms on")
        self.terminal.put(m.BcastEngEngineStatus(payload=m.EngEngineStatus(engine_1_ssm=Ssm.NO_COMPUTED_DATA)))
        self.assertEqual({"total_kg", "balance_kg"}, set(self.feed.engine_readings()),
                         "an invalid engine gives no oil readings")

    def test_weather_severity_from_the_scan_summary(self):
        self.assertEqual(("NONE", "NONE", False), self.feed.weather_severity())
        radar = self.feed.radar_management.radars["weather_radar"]
        self.assertIsInstance(radar, WeatherSeverity)
        self.assertEqual(([], False, "NONE"), (radar.windshear_proc.get_last_result(), radar.windshear_proc.has_microburst(),
                                               radar.turbulence_proc.max_category()))
        self.terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=4, mode=WeatherMode.WINDSHEAR,
                                                             max_shear=ShearSeverity.MODERATE, microburst=True,
                                                             max_turbulence=TurbulenceCategory.LIGHT)))
        self.assertEqual(("MODERATE", "LIGHT", True), self.feed.weather_severity())
        events = radar.windshear_proc.get_last_result()
        self.assertEqual(["MODERATE"], [e.severity for e in events])
        self.assertEqual("MODERATE", max(events, key=lambda e: e.severity_level()).severity, "what eicas.py does with the list")
        self.assertTrue(radar.windshear_proc.has_microburst())
        self.assertEqual("LIGHT", radar.turbulence_proc.max_category())
        self.terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=5, mode=WeatherMode.WINDSHEAR,
                                                             max_shear=ShearSeverity.SEVERE)), age=3.0)
        self.assertEqual(("NONE", "NONE", False), self.feed.weather_severity(), "stale data is no data")

    def test_the_sources_not_on_the_bus_yet(self):
        self.assertEqual([], self.feed.bit.self_tests, "no BITS section until the avionics terminal")
        self.assertEqual({}, self.feed.defensive.get_data())
        self.assertEqual({}, self.feed.sensors.get_data())
        self.assertEqual([], self.feed.fusion.get_threat_tracks())
        self.assertEqual({"tfr_radar", "weather_radar"}, set(self.feed.radar_management.radars))


class TargetingFeedTest(unittest.TestCase):
    """The MFD's targeting picture: RDR_TGT_STATUS and the RDR_TGT_TRACKS block as TargetingRadarDisplay draws them."""

    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = RadarFeed(self.terminal, FakeCoordinator(), clock=lambda: 1000.0)

    def test_standby_until_the_radar_reports(self):
        data = self.feed.targeting_data()
        self.assertEqual(("STANDBY", [], 0, "NONE", 0),
                         (data["mode_name"], data["targets"], data["track_count"], data["lock_status"],
                          data["locked_track_id"]))
        self.assertAlmostEqual(100.0 / 1.852, data["range_nm"], msg="the targeting radar's 100 km")
        self.assertEqual("TGT: no data", self.feed.targeting_status_text())

    def test_targets_in_the_displays_vocabulary(self):
        self.terminal.put(m.BcastRdrTgtStatus(payload=m.RdrTgtStatus(
            state=TerminalState.NORMAL, mode=TargetingMode.TARGET_TRACK, track_count=2, lock_status=LockStatus.LOCKED,
            locked_track_id=2, lock_quality=0.8)))
        self.terminal.put_block(b.BcastRdrTgtTracks, sample_tracks(TargetingMode.TARGET_TRACK), 3)
        data = self.feed.targeting_data()
        self.assertEqual(("TARGET_TRACK", 2, "LOCKED", 2),
                         (data["mode_name"], data["track_count"], data["lock_status"], data["locked_track_id"]))
        one, two = data["targets"]
        self.assertEqual(1, one["id"])
        self.assertAlmostEqual(10.0, one["position"][0])
        self.assertAlmostEqual(0.0, one["position"][1])
        self.assertAlmostEqual(1000.0 / 1852.0, one["position"][2])
        self.assertEqual((0.0, 250.0, 0.0), one["velocity"])
        self.assertEqual(("FIXED_WING", None, "HOSTILE", False),
                         (one["classification"], one["identity"], one["threat"], one["locked"]))
        self.assertEqual(("ROTARY_WING", "FRIENDLY", True), (two["classification"], two["identity"], two["locked"]))
        self.assertAlmostEqual(-20.0, two["position"][1])
        self.assertIn("TGT NORMAL TARGET_TRACK  2 tracks  lock 2 LOCKED", self.feed.targeting_status_text())
        # Out of an operating mode the picture reads STANDBY; the last block stays until it is stale.
        self.terminal.put(m.BcastRdrTgtStatus(payload=m.RdrTgtStatus(state=TerminalState.STANDBY, mode=TargetingMode.NONE)))
        self.assertEqual("STANDBY", self.feed.targeting_data()["mode_name"])
        self.assertEqual(2, len(self.feed.targeting_data()["targets"]))
        self.terminal.put_block(b.BcastRdrTgtTracks, sample_tracks(), 4, age=3.0)
        self.assertEqual([], self.feed.targeting_data()["targets"], "stale data is no data")

    def test_the_symbol_vocabulary(self):
        for name in ("FIGHTER", "BOMBER", "HIGH_ALT", "DRONE", "STEALTH"):
            self.assertEqual("FIXED_WING", SYMBOL_CLASS[name], name)
        self.assertEqual("ROTARY_WING", SYMBOL_CLASS["HELICOPTER"])
        self.assertNotIn("MISSILE", SYMBOL_CLASS, "no missile shape: the display's '?'")
        self.assertNotIn("UNKNOWN", SYMBOL_CLASS)
        item = targeting_item(b.RdrTgtTracksTrack(track_id=9, x=-1852.0, y=1852.0, z=0.0, classification=TargetClass.MISSILE,
                                                  identity=IffIdentity.HOSTILE, threat=Threat.HOSTILE, stealth=True))
        self.assertEqual((9, (-1.0, 1.0, 0.0), None, "HOSTILE", True),
                         (item["id"], item["position"], item["classification"], item["identity"], item["stealth"]))


class AewcFeedTest(unittest.TestCase):
    """The MFD's AEWC picture: RDR_AEWC_STATUS and the RDR_AEWC_TRACKS block as AEWCRadarDisplay draws them."""

    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = RadarFeed(self.terminal, FakeCoordinator(), clock=lambda: 1000.0)

    def test_standby_until_the_radar_reports(self):
        data = self.feed.aewc_data()
        self.assertEqual(("STANDBY", [], 0, 0, [], 0, []),
                         (data["mode_name"], data["targets"], data["track_count"], data["stealth_tracks"],
                          data["jammed_sectors"], data["priority_sector"], data["sectors"]))
        self.assertAlmostEqual(400.0 / 1.852, data["range_nm"], msg="the AEWC radar's 400 km")
        self.assertEqual("AEWC: no data", self.feed.aewc_status_text())

    def test_targets_sectors_and_the_draw_modes(self):
        self.terminal.put(m.BcastRdrAewcStatus(payload=m.RdrAewcStatus(
            state=TerminalState.NORMAL, mode=AewcMode.AEWC_SEARCH, track_count=2, stealth_tracks=1, jammed_sectors=0x08,
            priority_sector=1)))
        self.terminal.put_block(b.BcastRdrAewcTracks, sample_aewc(), 2)
        data = self.feed.aewc_data()
        self.assertEqual(("SEARCH", 2, 1, [4], 1), (data["mode_name"], data["track_count"], data["stealth_tracks"],
                                                   data["jammed_sectors"], data["priority_sector"]))
        one, two = data["targets"]
        self.assertEqual((1, "FIXED_WING", None, False, True, 1), (one["id"], one["classification"], one["identity"],
                                                                   one["is_stealth"], one["detected"], one["sector"]))
        self.assertAlmostEqual(130968.0 / 1852.0, one["position"][0])
        self.assertEqual((2, "FIXED_WING", True, False, 0, 9.0), (two["id"], two["classification"], two["is_stealth"],
                                                                  two["detected"], two["sector"], two["snr_db"]))
        self.assertAlmostEqual(10.0, two["position"][0])
        sectors = data["sectors"]
        self.assertEqual([0.0, 60.0, 120.0, 180.0, 240.0, 300.0], [s["azimuth_start"] for s in sectors])
        self.assertEqual((1, False, 0.25, 12.0, "NONE"), (sectors[0]["active_tracks"], sectors[0]["jammed"],
                                                         sectors[0]["dwell_fraction"], sectors[0]["priority_score"],
                                                         sectors[0]["jam_type"]))
        self.assertEqual((True, "DECEPTIVE", "FREQ_AGILITY", 0.2), (sectors[3]["jammed"], sectors[3]["jam_type"],
                                                                   sectors[3]["mitigation"], sectors[3]["severity"]))
        self.assertIn("AEWC NORMAL AEWC_SEARCH  2 tracks (1 stealth)  jammed 4", self.feed.aewc_status_text())
        for mode, name in ((AewcMode.AEWC_SURVEILLANCE, "SEARCH"), (AewcMode.SECTOR_SCAN, "SEARCH"),
                           (AewcMode.STEALTH_DETECTION, "SEARCH"), (AewcMode.AEWC_TRACK, "TRACK"),
                           (AewcMode.GROUND_MAPPING, "SEARCH"), (AewcMode.NONE, "STANDBY")):
            self.terminal.put(m.BcastRdrAewcStatus(payload=m.RdrAewcStatus(state=TerminalState.NORMAL, mode=mode)))
            self.assertEqual(name, self.feed.aewc_data()["mode_name"], mode.name)
        self.terminal.put_block(b.BcastRdrAewcTracks, sample_aewc(), 3, age=3.0)
        self.assertEqual(([], []), (self.feed.aewc_data()["targets"], self.feed.aewc_data()["sectors"]),
                         "stale data is no data")
        item = aewc_item(b.RdrAewcTracksTrack(track_id=7, x=-1852.0, y=1852.0, z=0.0, classification=TargetClass.UNKNOWN,
                                              identity=IffIdentity.FRIENDLY, threat=Threat.FRIENDLY))
        self.assertEqual((7, (-1.0, 1.0, 0.0), None, "FRIENDLY", False),
                         (item["id"], item["position"], item["classification"], item["identity"], item["is_stealth"]))


class TsdFeedTest(unittest.TestCase):
    """What tsd._poll_data() reads: the FMS getters' tactical systems and energy metrics, the fusion's tracks as
    to_tsd_dict() dictionaries, and the services not on the bus yet."""

    def setUp(self):
        self.terminal = FakeTerminal()
        self.feed = SystemsFeed(self.terminal, clock=lambda: 1000.0)

    def test_the_tactical_status_grows_the_tsd_fields(self):
        tactical = self.feed.fms_control.get_tactical_status()
        self.assertNotIn("tactical_systems", tactical, "the TSD keeps its own defaults until the FMS status says")
        self.assertNotIn("energy_metrics", tactical)
        self.terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(
            state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT, countermeasures=CountermeasuresState.ACTIVE,
            targeting=TargetingState.ACTIVE, weapons=WeaponsState.ARMED, stealth_mode=False)))
        self.terminal.put(m.BcastFmsEnergy(payload=m.FmsEnergy(
            ssm=Ssm.NORMAL, specific_excess_power=42.5, max_sustained_turn_rate=12.3, max_instantaneous_turn_rate=20.1,
            max_climb_rate=6000.0, energy_advantage=425.0)))
        tactical = self.feed.fms_control.get_tactical_status()
        self.assertEqual({"countermeasures": "ACTIVE", "targeting": "ACTIVE", "weapons": "ARMED", "stealth_mode": "OFF"},
                         tactical["tactical_systems"])
        self.assertEqual({"specific_excess_power": 42.5, "max_sustained_turn_rate": 12.3,
                          "max_instantaneous_turn_rate": 20.1, "max_climb_rate_fpm": 6000.0, "energy_advantage": 425.0},
                         tactical["energy_metrics"])
        self.terminal.put(m.BcastFmsEnergy(payload=m.FmsEnergy(ssm=Ssm.NO_COMPUTED_DATA)))
        self.assertNotIn("energy_metrics", self.feed.fms_control.get_tactical_status())

    def test_the_position_reaches_the_navigation_group(self):
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload()))
        nav = self.feed.fms.get_flight_data()["navigation"]
        self.assertEqual((1.5, -2.25), (nav["latitude"], nav["longitude"]))
        self.terminal.put(m.BcastFmsFlightData(payload=flight_payload(position_ssm=Ssm.NO_COMPUTED_DATA)))
        self.assertNotIn("latitude", self.feed.fms.get_flight_data()["navigation"])

    def test_fused_tracks_are_the_pythons_tsd_dictionaries(self):
        self.assertEqual([], self.feed.fusion.get_fused_tracks())
        self.assertIsNone(self.feed.fused_summary())
        self.terminal.put_block(b.BcastRdrFusedTracks, sample_fused(), 5)
        tracks = self.feed.fusion.get_fused_tracks()
        self.assertEqual(2, len(tracks))
        self.assertIsInstance(tracks[0], FusedTrackView)
        d = tracks[0].to_tsd_dict()
        self.assertEqual({"bearing", "range_nm", "type", "hostile", "confidence", "sources", "track_id", "identity"},
                         set(d), "FusedTrack.to_tsd_dict()'s keys")
        self.assertAlmostEqual(45.0, d["bearing"])
        self.assertAlmostEqual(math.hypot(10.0, 10.0), d["range_nm"])
        self.assertEqual(("FIGHTER", True, ["targeting_radar"], 1, "UNKNOWN"),
                         (d["type"], d["hostile"], d["sources"], d["track_id"], d["identity"]))
        self.assertAlmostEqual(0.8, d["confidence"])
        self.assertAlmostEqual(180.0, tracks[1].to_tsd_dict()["bearing"])
        self.assertEqual([1], [t.track_id for t in self.feed.fusion.get_threat_tracks()])
        self.assertEqual((2, 1, 3), self.feed.fused_summary())
        self.assertIn("fused 2 (1 hostile)", self.feed.status_text())
        self.terminal.put_block(b.BcastRdrFusedTracks, sample_fused(), 6, age=3.0)
        self.assertEqual([], self.feed.fusion.get_fused_tracks(), "stale data is no data")

    def test_the_services_not_on_the_bus_yet(self):
        self.assertEqual({"route": []}, self.feed.mission.get_nav_data())
        self.assertEqual([], self.feed.defensive.get_rwr_contacts())


if __name__ == "__main__":
    unittest.main()
