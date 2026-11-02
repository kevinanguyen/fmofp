"""The display windows paint bus data with the real Python display classes, offscreen.

Skipped where PyQt6 or the FMOFP display code cannot be imported. The feeds
are given sample bus data through a fake terminal; the windows' pictures are
rendered to an image and must light up more than their standby pictures, with
no paint error.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PyQt6.QtWidgets import QApplication
    HAVE_QT = True
except ImportError:  # pragma: no cover - depends on the machine
    HAVE_QT = False

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import (AewcMode, SarMode, ClearanceLevel, CountermeasuresState, FmsFlightMode, GcasStatus, ShearSeverity,
                                 Ssm, TargetingMode, TargetingState, TerminalState, TfrMode, WeaponsState, WeatherMode)
from fmofp_bus.tests.test_display_feeds import (FakeTerminal, flight_payload, sample_aewc, sample_elevation, sample_fused, sample_sar_image,
                                                sample_products, sample_tracks)
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed
from fmofp_display.systems_feed import SystemsFeed


def lit_pixels(image, step=2):
    n = 0
    for y in range(0, image.height(), step):
        for x in range(0, image.width(), step):
            if image.pixel(x, y) & 0xFFFFFF:
                n += 1
    return n


class StubTerminal(FakeTerminal):
    """A FakeTerminal with the few attributes the windows show in their status line."""
    rt = 15
    state = TerminalState.NORMAL
    syncs = 0

    def take_mode_request(self):
        return None

    def send(self, message):
        self.sent = getattr(self, "sent", []) + [message]

    def next_command_seq(self):
        return 1


@unittest.skipUnless(HAVE_QT, "PyQt6 is not installed")
class DisplayWindowsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from fmofp_display import app
        app.bootstrap()
        try:
            from FMOFP.Interfaces.userInterface.displays.radar.radar_display_data_coordinator import (
                get_radar_display_data_coordinator)
        except Exception as exc:  # pragma: no cover - depends on the machine
            raise unittest.SkipTest(f"FMOFP display code not importable here: {exc}")
        cls.app = QApplication.instance() or QApplication([])
        cls.app_module = app
        cls.coordinator = get_radar_display_data_coordinator()

    def test_the_radar_window_draws_both_radars_from_the_bus_data(self):
        terminal = StubTerminal()
        feed = RadarFeed(terminal, self.coordinator)
        window = self.app_module.RadarWindow(terminal, feed)
        window.resize(1200, 700)
        window.show()
        self.app.processEvents()
        window.tick()
        standby = lit_pixels(window.picture.grab().toImage())
        self.assertEqual(0, window.picture.paint_errors)

        terminal.put(m.BcastRdrWxStatus(payload=m.RdrWxStatus(state=TerminalState.NORMAL, mode=WeatherMode.SURVEILLANCE)))
        terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=3, mode=WeatherMode.SURVEILLANCE, range_km=200.0,
                                                        max_reflectivity=55.0)))
        terminal.put_block(b.BcastRdrWxProducts, sample_products(), 1)
        terminal.put(m.BcastRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL, mode=TfrMode.TERRAIN_FOLLOWING)))
        terminal.put(m.BcastRdrTfrGuidance(payload=m.RdrTfrGuidance(
            clearance_ssm=Ssm.NORMAL, level=ClearanceLevel.CAUTION, mode=TfrMode.TERRAIN_FOLLOWING, commands_valid=True,
            min_clearance=600.0, commanded_altitude=4000.0, set_clearance=984.0)))
        terminal.put_block(b.BcastRdrTfrElevation, sample_elevation(), 2)
        terminal.put(m.BcastRdrTgtStatus(payload=m.RdrTgtStatus(state=TerminalState.NORMAL,
                                                              mode=TargetingMode.TARGET_SEARCH, track_count=4)))
        terminal.put(m.BcastRdrAewcStatus(payload=m.RdrAewcStatus(state=TerminalState.NORMAL, mode=AewcMode.AEWC_SEARCH,
                                                                track_count=9, stealth_tracks=2)))
        terminal.put(m.BcastRdrSarStatus(payload=m.RdrSarStatus(state=TerminalState.NORMAL, mode=SarMode.STRIPMAP,
                                                              scene_valid=True, nav_valid=True, image_seq=2)))
        window.tick()
        self.app.processEvents()
        active = lit_pixels(window.picture.grab().toImage())
        self.assertEqual(0, window.picture.paint_errors)
        self.assertGreater(active, standby * 2, f"standby {standby}, active {active}")
        self.assertEqual("SURVEILLANCE", window.picture._weather_mode_name)
        self.assertAlmostEqual(200.0 / 1.852, window.picture.weather.range_scale)
        self.assertIn("WX NORMAL SURVEILLANCE", window.status.text())
        self.assertIn("TFR NORMAL TERRAIN_FOLLOWING", window.status.text())
        self.assertIn("TGT NORMAL TARGET_SEARCH  4 tracks", window.status.text())
        self.assertIn("AEWC NORMAL AEWC_SEARCH  9 tracks (2 stealth)", window.status.text())
        self.assertIn("SAR NORMAL STRIPMAP  image 2", window.status.text())
        # The control panel sends commands through the terminal.
        window.findChildren(type(window.clearance))  # the spin box exists
        buttons = {button.text(): button for button in window.findChildren(self.app_module.QPushButton)}
        buttons["WINDSHEAR"].click()
        buttons["TFR_SEARCH"].click()
        buttons["TARGET_SEARCH"].click()
        window.lock_track.setValue(3)
        buttons["TARGET_TRACK"].click()
        buttons["AEWC_SEARCH"].click()
        window.sar_resolution.setValue(0.3)
        buttons["SPOTLIGHT"].click()
        sent = getattr(terminal, "sent", [])
        self.assertEqual(6, len(sent))
        self.assertEqual((WeatherMode.WINDSHEAR, TerminalState.NORMAL), (sent[0].payload.requested_mode,
                                                                          sent[0].payload.requested_state))
        self.assertEqual((TfrMode.TFR_SEARCH, 1000.0), (sent[1].payload.requested_mode, sent[1].payload.set_clearance))
        self.assertEqual((m.DspRdrTgtCmd, TargetingMode.TARGET_SEARCH, 0), (type(sent[2]), sent[2].payload.requested_mode,
                                                                          sent[2].payload.lock_track_id))
        self.assertEqual((TargetingMode.TARGET_TRACK, 3), (sent[3].payload.requested_mode, sent[3].payload.lock_track_id))
        self.assertEqual((m.DspRdrAewcCmd, AewcMode.AEWC_SEARCH, 90.0), (type(sent[4]), sent[4].payload.requested_mode,
                                                                      sent[4].payload.azimuth_end))
        self.assertEqual((m.DspRdrSarCmd, SarMode.SPOTLIGHT, 0.3), (type(sent[5]), sent[5].payload.requested_mode,
                                                                round(sent[5].payload.resolution, 6)))
        window.timer.stop()
        window.close()

    def test_the_pfd_window_draws_the_flight_data(self):
        terminal = StubTerminal()
        terminal.rt = 11
        feed = FlightDataFeed(terminal)
        window = self.app_module.PfdWindow(terminal, feed)
        window.show()
        self.app.processEvents()
        window.pfd.update_flight_data()
        empty = lit_pixels(window.pfd.grab().toImage())
        self.assertFalse(window.pfd.validity.is_valid("heading"), "nothing from the bus yet: every field invalid")
        terminal.put(m.BcastFmsFlightData(payload=m.FmsFlightData(
            roll_ssm=Ssm.NORMAL, pitch_ssm=Ssm.NORMAL, heading_ssm=Ssm.NORMAL, airspeed_ssm=Ssm.NORMAL,
            altitude_ssm=Ssm.NORMAL, vertical_speed_ssm=Ssm.NORMAL, g_ssm=Ssm.NORMAL, aoa_ssm=Ssm.NORMAL,
            mach_ssm=Ssm.NORMAL, energy_ssm=Ssm.NORMAL, flight_mode=FmsFlightMode.COMBAT, roll=-12.0, pitch=3.5,
            heading=275.0, airspeed=420.0, altitude=12345.0, vertical_speed=800.0, mach=0.71, g_force=1.3,
            angle_of_attack=4.0, energy_state=61.0)))
        terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT)))
        terminal.put(m.BcastFmsAttitudeStatus(payload=m.FmsAttitudeStatus(bank_limited=True)))
        window.pfd.update_flight_data()
        window.tick()
        self.app.processEvents()
        full = lit_pixels(window.pfd.grab().toImage())
        self.assertEqual(275.0, window.pfd.heading)
        self.assertEqual(12345.0, window.pfd.altitude)
        self.assertEqual("COMBAT", window.pfd.flight_mode)
        self.assertEqual(["BANK_ANGLE"], window.pfd.envelope_warnings)
        self.assertTrue(window.pfd.validity.is_valid("heading"))
        self.assertFalse(getattr(window.pfd, "_paint_error", False))
        self.assertNotEqual(empty, full)
        self.assertIn("FMS COMBAT", window.status.text())
        window.timer.stop()
        window.close()

    def test_the_mfd_window_draws_every_page_from_the_bus_data(self):
        from PyQt6.QtCore import QPoint, Qt
        from PyQt6.QtTest import QTest
        terminal = StubTerminal()
        terminal.rt = 13
        systems = SystemsFeed(terminal)
        radar = RadarFeed(terminal, self.coordinator)
        window = self.app_module.MfdWindow(terminal, radar, systems)
        try:
            window.show()
            self.app.processEvents()
            mfd = window.mfd
            pages = window._pages
            kinds = window._radar_types
            # The getters the pages import answer with the feed's stand-ins; the real module is still there for
            # anything else that asks.
            from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys import (
                GCAS, get_gcas)
            self.assertIs(systems.gcas, get_gcas())
            self.assertTrue(hasattr(GCAS, "_evaluate"))
            self.assertEqual((0.0, "UNKNOWN"), mfd._read_tfr_clearance())

            terminal.put(m.BcastFcsGcasAlerts(payload=m.FcsGcasAlerts(
                status=GcasStatus.WARNING, sink_rate=True, vertical_speed=-2500.0, altitude=1800.0,
                vertical_speed_ssm=Ssm.NORMAL, altitude_ssm=Ssm.NORMAL)))
            terminal.put(m.BcastFcsPerfStatus(payload=m.FcsPerfStatus(exceed_roll=True, g_ssm=Ssm.NORMAL, aoa_ssm=Ssm.NORMAL)))
            terminal.put(m.BcastFmsFlightData(payload=m.FmsFlightData(
                position_ssm=Ssm.NORMAL, roll_ssm=Ssm.NORMAL, altitude_ssm=Ssm.NORMAL, latitude=1.5, longitude=-2.25,
                altitude=5000.0, roll=-70.0)))
            for page in (pages.NAV, pages.SYSTEMS, pages.WEAPONS, pages.COMMS, pages.MISSION, pages.SETTINGS):
                mfd.current_page = page
                window.tick()
                mfd.update()
                self.app.processEvents()
                self.assertGreater(lit_pixels(mfd.grab().toImage()), 0, page.name)
                self.assertFalse(mfd._paint_error, page.name)
            self.assertIn("GCAS WARNING SINK RATE", window.status.text())
            self.assertIn("exceed roll", window.status.text())
            self.assertIn("pos 1.5000 -2.2500", window.status.text())

            # The Radar menu item, clicked as the pilot would: the weather picture, standby until the radar reports.
            title_height = mfd.height() / 12
            item_height = min((mfd.height() - title_height - 20) / len(mfd.pages), 40)
            QTest.mouseClick(mfd, Qt.MouseButton.LeftButton,
                             pos=QPoint(int(mfd.width() / 10), int(title_height + 10 + item_height * 1.5)))
            self.app.processEvents()
            self.assertEqual((pages.RADAR, kinds.WEATHER), (mfd.current_page, mfd.current_radar_type))
            window.tick()
            mfd.update()
            self.app.processEvents()
            standby = lit_pixels(mfd.grab().toImage())
            self.assertEqual("STANDBY", mfd.radar_data.mode.name)
            terminal.put(m.BcastRdrWxStatus(payload=m.RdrWxStatus(state=TerminalState.NORMAL, mode=WeatherMode.SURVEILLANCE)))
            terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=3, mode=WeatherMode.SURVEILLANCE, range_km=200.0,
                                                            max_reflectivity=55.0)))
            terminal.put_block(b.BcastRdrWxProducts, sample_products(), 1)
            window.tick()
            mfd.update()
            self.app.processEvents()
            active = lit_pixels(mfd.grab().toImage())
            self.assertFalse(mfd._paint_error)
            self.assertGreater(active, standby, f"standby {standby}, active {active}")
            self.assertEqual("SURVEILLANCE", mfd.radar_data.mode.name)
            self.assertAlmostEqual(200.0 / 1.852, mfd._current_radar_display.range_scale)
            applied = radar.blocks_applied

            # The terrain picture through the sub-menu's switch: the MFD empties the coordinator and builds a new
            # picture; the profile, mode and clearance come from the bus.
            terminal.put(m.BcastRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL,
                                                                  mode=TfrMode.TERRAIN_FOLLOWING)))
            terminal.put(m.BcastRdrTfrGuidance(payload=m.RdrTfrGuidance(
                clearance_ssm=Ssm.NORMAL, level=ClearanceLevel.CAUTION, mode=TfrMode.TERRAIN_FOLLOWING,
                commands_valid=True, min_clearance=600.0, commanded_altitude=4000.0, set_clearance=984.0)))
            terminal.put_block(b.BcastRdrTfrElevation, sample_elevation(), 2)
            mfd.current_radar_type = kinds.TFR
            mfd._update_radar_type()
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual("TFRRadarDisplay", type(mfd._current_radar_display).__name__)
            self.assertEqual(window._terrain_modes["TRACK"], mfd.radar_data.mode)
            self.assertEqual(100, len(mfd.radar_data.tfr_data))
            clearance_m, level = mfd._read_tfr_clearance()
            self.assertAlmostEqual(600.0 * 0.3048, clearance_m)
            self.assertEqual("CAUTION", level)

            # The targeting picture: standby until the radar reports, then its tracks in the display's vocabulary.
            mfd.current_radar_type = kinds.TARGETING
            mfd._update_radar_type()
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual("TargetingRadarDisplay", type(mfd._current_radar_display).__name__)
            self.assertEqual("STANDBY", mfd.radar_data.mode.name)
            standby_targets = lit_pixels(mfd.grab().toImage())
            terminal.put(m.BcastRdrTgtStatus(payload=m.RdrTgtStatus(state=TerminalState.NORMAL,
                                                                  mode=TargetingMode.TARGET_SEARCH, track_count=2)))
            terminal.put_block(b.BcastRdrTgtTracks, sample_tracks(), 1)
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual(window._targeting_modes.TARGET_SEARCH, mfd.radar_data.mode)
            self.assertEqual([1, 2], [target["id"] for target in mfd.radar_data.targets])
            self.assertAlmostEqual(100.0 / 1.852, mfd._current_radar_display.range_scale)
            self.assertGreater(lit_pixels(mfd.grab().toImage()), standby_targets, "two targets and their labels")

            # The AEWC picture: standby, then its tracks (a conventional one and a stealth one) from the bus.
            mfd.current_radar_type = kinds.AEWC
            mfd._update_radar_type()
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual("AEWCRadarDisplay", type(mfd._current_radar_display).__name__)
            self.assertEqual("STANDBY", mfd.radar_data.mode.name)
            standby_aewc = lit_pixels(mfd.grab().toImage())
            terminal.put(m.BcastRdrAewcStatus(payload=m.RdrAewcStatus(state=TerminalState.NORMAL,
                                                                    mode=AewcMode.AEWC_SEARCH, track_count=2,
                                                                    stealth_tracks=1)))
            terminal.put_block(b.BcastRdrAewcTracks, sample_aewc(), 1)
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual(window._aewc_modes.AEWC_SEARCH, mfd.radar_data.mode)
            self.assertEqual([False, True], [target["is_stealth"] for target in mfd.radar_data.aewc_tracks])
            self.assertAlmostEqual(400.0 / 1.852, mfd._current_radar_display.range_scale)
            self.assertGreater(lit_pixels(mfd.grab().toImage()), standby_aewc, "two targets, one of them a stealth diamond")

            # The SAR picture: standby, then an image from the bus, which the display decodes from the PGM bytes.
            mfd.current_radar_type = kinds.SAR
            mfd._update_radar_type()
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual("SARRadarDisplay", type(mfd._current_radar_display).__name__)
            self.assertEqual("STANDBY", mfd.radar_data.mode.name)
            standby_sar = lit_pixels(mfd.grab().toImage())
            terminal.put(m.BcastRdrSarStatus(payload=m.RdrSarStatus(state=TerminalState.NORMAL, mode=SarMode.STRIPMAP,
                                                                  scene_valid=True, nav_valid=True, image_seq=4)))
            terminal.put_block(b.BcastRdrSarImage, sample_sar_image(), 1)
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertEqual(window._sar_modes.STRIPMAP, mfd.radar_data.mode)
            self.assertEqual(13 + 4096, len(mfd.radar_data.sar_data["image_data"]))
            picture = mfd._current_radar_display._current_image
            self.assertIsNotNone(picture, "QImage.fromData decoded the PGM")
            self.assertEqual((64, 64), (picture.width(), picture.height()))
            self.assertGreater(lit_pixels(mfd.grab().toImage()), standby_sar, "the picture of the ground")

            # Back to the weather: the picture returns from the bus data at once.
            mfd.current_radar_type = kinds.WEATHER
            mfd._update_radar_type()
            window.tick()
            mfd.update()
            self.app.processEvents()
            self.assertFalse(mfd._paint_error)
            self.assertGreater(radar.blocks_applied, applied, "the block was stored again after the coordinator was emptied")
            self.assertEqual("SURVEILLANCE", mfd.radar_data.mode.name)
        finally:
            window.timer.stop()
            window.close()
            self.app_module.uninstall_system_sources()
        from FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys import groundCollisionAvoidanceSys
        self.assertFalse(getattr(groundCollisionAvoidanceSys, "_fmofp_stand_in", False),
                         "after the window the real module is back for everyone else")

    def test_the_eicas_window_draws_the_engine_the_fuel_and_the_alerts_from_the_bus_data(self):
        terminal = StubTerminal()
        terminal.rt = 14
        systems = SystemsFeed(terminal)
        window = self.app_module.EicasWindow(terminal, systems)
        try:
            window.show()
            self.app.processEvents()
            eicas = window.eicas
            eicas._poll_data()
            window.tick()
            self.app.processEvents()
            empty = lit_pixels(eicas.grab().toImage())
            self.assertGreater(empty, 0)
            self.assertFalse(eicas.has_valid("total_kg"), "nothing from the bus yet")
            self.assertIn("FUEL QTY UNKNOWN", [a["text"] for a in eicas._alerts])

            terminal.put(m.BcastFmsFlightData(payload=flight_payload(airspeed=420.0, altitude=12345.0, g_force=1.3)))
            terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT)))
            terminal.put(m.BcastEngEngineStatus(payload=m.EngEngineStatus(
                engine_1_ssm=Ssm.NORMAL, engine_2_ssm=Ssm.NORMAL, thrust_1=80.0, n1_1=82.0, n2_1=82.0, egt_1=710.0,
                fuel_flow_1=3360.0, oil_pressure_1=61.2, oil_temperature_1=95.5, vibration_1=0.31)))
            terminal.put(m.BcastEngFuelStatus(payload=m.EngFuelStatus(main_left=9000.0, main_right=10000.0,
                                                                     center=20000.0, total=39000.0, burn_rate=6720.0)))
            terminal.put(m.BcastFcsGcasAlerts(payload=m.FcsGcasAlerts(
                status=GcasStatus.WARNING, sink_rate=True, vertical_speed=-2500.0, altitude=1800.0,
                vertical_speed_ssm=Ssm.NORMAL, altitude_ssm=Ssm.NORMAL)))
            terminal.put(m.BcastRdrWxScan(payload=m.RdrWxScan(scan_id=4, mode=WeatherMode.WINDSHEAR,
                                                            max_shear=ShearSeverity.SEVERE)))
            window.tick()        # the oil and fuel readings go in through set_engine_readings()
            eicas._poll_data()   # the 10 Hz poll reads the getters
            window.tick()
            self.app.processEvents()
            full = lit_pixels(eicas.grab().toImage())
            self.assertFalse(getattr(eicas, "_paint_error", False))
            self.assertNotEqual(empty, full)
            self.assertEqual((82.0, 710.0, 61.2), (eicas._engine["n1_pct"], eicas._engine["egt_c"], eicas._engine["oil_psi"]))
            self.assertEqual(100.0, eicas._engine["thrust_pct"], "COMBAT's engine power, as the Python showed")
            for field in ("oil_psi", "oil_temp_c", "vib", "total_kg", "balance_kg", "sys_a_psi"):
                self.assertTrue(eicas.has_valid(field), field)
            self.assertFalse(eicas.has_valid("sys_c_psi"), "no hydraulics terminal yet")
            self.assertEqual((39000.0, -1000.0), (eicas._fuel["total_kg"], eicas._fuel["balance_kg"]))
            texts = [a["text"] for a in eicas._alerts]
            self.assertIn("FUEL  IMBALANCE", texts)
            self.assertIn("FCS  SINK RATE  -2500 fpm", texts)
            self.assertIn("WX   SEVERE WINDSHEAR", texts)
            self.assertIn("ENG  MAX CONTINUOUS THRUST", texts)
            self.assertIn("HYD C  PRESSURE UNKNOWN", texts, "honest dashes for what has no terminal")
            self.assertNotIn("FUEL QTY UNKNOWN", texts)
            self.assertNotIn("ENG  OIL PRESS UNKNOWN", texts)
            self.assertIn("ENG  EGT CAUTION", texts, "graded on the thrust model (350 + 4.5 x 100 C at COMBAT power), which eicas.py computes before its ECU override")
            self.assertIn("2 warnings 6 cautions", window.status.text())
            self.assertIn("ENG N1 82.0%", window.status.text())
        finally:
            window.timer.stop()
            window.close()
            self.app_module.uninstall_system_sources()

    def test_the_tsd_window_draws_the_fused_tracks_the_metrics_and_the_strip_from_the_bus_data(self):
        terminal = StubTerminal()
        terminal.rt = 16
        systems = SystemsFeed(terminal)
        window = self.app_module.TsdWindow(terminal, systems)
        try:
            window.show()
            self.app.processEvents()
            tsd = window.tsd
            tsd._poll_data()
            window.tick()
            self.app.processEvents()
            before = tsd.grab().toImage()
            self.assertGreater(lit_pixels(before), 0)
            self.assertEqual([], tsd._threats)
            self.assertFalse(tsd._ownship_position_known)
            self.assertIn("fusion: no data", window.status.text())

            terminal.put(m.BcastFmsFlightData(payload=flight_payload(flight_mode=FmsFlightMode.COMBAT, heading=45.0,
                                                                     g_force=3.2, angle_of_attack=8.0, energy_state=70.0)))
            terminal.put(m.BcastFmsStatus(payload=m.FmsStatus(
                state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT, countermeasures=CountermeasuresState.ACTIVE,
                targeting=TargetingState.ACTIVE, weapons=WeaponsState.ARMED)))
            terminal.put(m.BcastFmsAttitudeStatus(payload=m.FmsAttitudeStatus(bank_limited=True)))
            terminal.put(m.BcastFmsEnergy(payload=m.FmsEnergy(
                ssm=Ssm.NORMAL, specific_excess_power=42.5, max_sustained_turn_rate=12.3, max_instantaneous_turn_rate=20.1,
                max_climb_rate=6000.0, energy_advantage=425.0)))
            terminal.put_block(b.BcastRdrFusedTracks, sample_fused(), 5)
            tsd._poll_data()   # the 10 Hz poll reads the getters
            window.tick()
            self.app.processEvents()
            after = tsd.grab().toImage()
            self.assertFalse(getattr(tsd, "_paint_error", False))
            # The TSD paints every pixel of its dark panels, so a lit-pixel count cannot tell the two states apart
            # (it did not on Linux); the pictures themselves differ.
            self.assertNotEqual(before, after, "the data changed the picture")
            self.assertEqual(("COMBAT", 45.0, 3.2, 8.0, 70.0), (tsd._mode, tsd._heading, tsd._g_force, tsd._aoa, tsd._energy))
            self.assertTrue(tsd._ownship_position_known)
            self.assertEqual((42.5, 12.3, 20.1, 6000.0),
                             (tsd._sep, tsd._max_turn_sustained, tsd._max_turn_instant, tsd._max_climb_fpm))
            self.assertEqual(100.0, tsd._throttle, "COMBAT's engine power")
            self.assertEqual({"countermeasures": "ACTIVE", "targeting": "ACTIVE", "weapons": "ARMED", "stealth_mode": "OFF"},
                             tsd._tac)
            self.assertEqual(["BANK_ANGLE"], tsd._warnings)
            self.assertEqual([("FIGHTER", True), ("UNKNOWN", False)], [(t["type"], t["hostile"]) for t in tsd._threats])
            self.assertAlmostEqual(45.0, tsd._threats[0]["bearing"])
            self.assertEqual(([], []), (tsd._waypoints, tsd._rwr_contacts), "no mission or defensive terminal yet")
            self.assertIn("fused 2 (1 hostile), epoch 3", window.status.text())
        finally:
            window.timer.stop()
            window.close()
            self.app_module.uninstall_system_sources()


if __name__ == "__main__":
    unittest.main()
