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
from fmofp_bus.icd.enums import ClearanceLevel, FmsFlightMode, Ssm, TerminalState, TfrMode, WeatherMode
from fmofp_bus.tests.test_display_feeds import FakeTerminal, sample_elevation, sample_products
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed


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

        terminal.put(m.DspRdrWxStatus(payload=m.RdrWxStatus(state=TerminalState.NORMAL, mode=WeatherMode.SURVEILLANCE)))
        terminal.put(m.DspRdrWxScan(payload=m.RdrWxScan(scan_id=3, mode=WeatherMode.SURVEILLANCE, range_km=200.0,
                                                        max_reflectivity=55.0)))
        terminal.put_block(b.DspRdrWxProducts, sample_products(), 1)
        terminal.put(m.DspRdrTfrStatus(payload=m.RdrTfrStatus(state=TerminalState.NORMAL, mode=TfrMode.TERRAIN_FOLLOWING)))
        terminal.put(m.DspRdrTfrGuidance(payload=m.RdrTfrGuidance(
            clearance_ssm=Ssm.NORMAL, level=ClearanceLevel.CAUTION, mode=TfrMode.TERRAIN_FOLLOWING, commands_valid=True,
            min_clearance=600.0, commanded_altitude=4000.0, set_clearance=984.0)))
        terminal.put_block(b.DspRdrTfrElevation, sample_elevation(), 2)
        window.tick()
        self.app.processEvents()
        active = lit_pixels(window.picture.grab().toImage())
        self.assertEqual(0, window.picture.paint_errors)
        self.assertGreater(active, standby * 2, f"standby {standby}, active {active}")
        self.assertEqual("SURVEILLANCE", window.picture._weather_mode_name)
        self.assertAlmostEqual(200.0 / 1.852, window.picture.weather.range_scale)
        self.assertIn("WX NORMAL SURVEILLANCE", window.status.text())
        self.assertIn("TFR NORMAL TERRAIN_FOLLOWING", window.status.text())
        # The control panel sends commands through the terminal.
        window.findChildren(type(window.clearance))  # the spin box exists
        buttons = {button.text(): button for button in window.findChildren(self.app_module.QPushButton)}
        buttons["WINDSHEAR"].click()
        buttons["TFR_SEARCH"].click()
        sent = getattr(terminal, "sent", [])
        self.assertEqual(2, len(sent))
        self.assertEqual((WeatherMode.WINDSHEAR, TerminalState.NORMAL), (sent[0].payload.requested_mode,
                                                                          sent[0].payload.requested_state))
        self.assertEqual((TfrMode.TFR_SEARCH, 1000.0), (sent[1].payload.requested_mode, sent[1].payload.set_clearance))
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
        terminal.put(m.PfdFlightData(payload=m.FmsFlightData(
            roll_ssm=Ssm.NORMAL, pitch_ssm=Ssm.NORMAL, heading_ssm=Ssm.NORMAL, airspeed_ssm=Ssm.NORMAL,
            altitude_ssm=Ssm.NORMAL, vertical_speed_ssm=Ssm.NORMAL, g_ssm=Ssm.NORMAL, aoa_ssm=Ssm.NORMAL,
            mach_ssm=Ssm.NORMAL, energy_ssm=Ssm.NORMAL, flight_mode=FmsFlightMode.COMBAT, roll=-12.0, pitch=3.5,
            heading=275.0, airspeed=420.0, altitude=12345.0, vertical_speed=800.0, mach=0.71, g_force=1.3,
            angle_of_attack=4.0, energy_state=61.0)))
        terminal.put(m.PfdFmsStatus(payload=m.FmsStatus(state=TerminalState.NORMAL, flight_mode=FmsFlightMode.COMBAT)))
        terminal.put(m.PfdAttitudeStatus(payload=m.FmsAttitudeStatus(bank_limited=True)))
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


if __name__ == "__main__":
    unittest.main()
