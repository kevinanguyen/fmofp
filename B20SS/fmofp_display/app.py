"""The display windows and the entry point: py -m fmofp_display radar|pfd [--port-base N].

The windows own a DisplayTerminal (the bus, in a thread) and a feed, and paint
with the Python display classes from FMOFP/Interfaces/userInterface/displays,
unchanged. A timer applies what the feed makes of the latest bus data; the
display classes draw it exactly as they drew the bridge's data before.
"""

import argparse
import os
import sys
import time
from pathlib import Path

from PyQt6.QtCore import QRectF, Qt, QTimer
from PyQt6.QtGui import QPainter
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import TerminalState, TfrMode, WeatherMode
from fmofp_bus.status import Status
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed
from fmofp_display.terminal import DisplayTerminal

B20SS = Path(__file__).resolve().parent.parent
RADAR_RT = m.DspRadarStatus.RT
PFD_RT = m.DspPfdStatus.RT
TICK_MS = 50
DEFAULT_RANGE_KM = 200.0


def bootstrap():
    """The FMOFP display code expects B20SS as the working directory (its config paths are relative) and on sys.path."""
    os.chdir(B20SS)
    if str(B20SS) not in sys.path:
        sys.path.insert(0, str(B20SS))


def radar_terminal(port_base):
    return DisplayTerminal(RADAR_RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd,
                           receives=(m.DspRdrWxStatus, m.DspRdrWxScan, m.DspRdrTfrStatus, m.DspRdrTfrGuidance),
                           blocks=(b.DspRdrWxProducts, b.DspRdrTfrElevation))


def pfd_terminal(port_base):
    return DisplayTerminal(PFD_RT, port_base, m.DspPfdStatus, m.DspPfdModeCmd,
                           receives=(m.PfdFlightData, m.PfdFmsStatus, m.PfdAttitudeStatus))


def weather_command(terminal, state, mode, range_km=DEFAULT_RANGE_KM):
    cmd = m.RdrWxModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                         range_km=range_km, tilt=0.0)
    return m.DspRdrWxCmd(payload=cmd)


def tfr_command(terminal, state, mode, set_clearance_ft):
    cmd = m.RdrTfrModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                          set_clearance=float(set_clearance_ft))
    return m.DspRdrTfrCmd(payload=cmd)


class RadarPicture(QWidget):
    """Both radar pictures side by side: the weather display on the left, the terrain display on the right."""

    def __init__(self, feed, parent=None):
        super().__init__(parent)
        bootstrap()
        from FMOFP.Interfaces.userInterface.displays.radar.tfr_radar_display import TFRRadarDisplay
        from FMOFP.Interfaces.userInterface.displays.radar.weather_radar_display import WeatherRadarDisplay
        from FMOFP.Systems.radarManagement.radar_enums import tfr_radarMode, weather_radarMode
        self.feed = feed
        self.weather = WeatherRadarDisplay()
        self.terrain = TFRRadarDisplay()
        self._weather_modes = weather_radarMode
        self._terrain_modes = tfr_radarMode
        self._weather_mode_name = None
        self.paint_errors = 0
        self.setMinimumSize(1200, 600)

    def apply(self):
        """Puts the weather display into the mode and range the radar reports, then repaints."""
        name = self.feed.weather_draw_mode
        if name != self._weather_mode_name:
            self.weather._current_mode = self._weather_modes[name]
            self.weather._settings_manager.apply_mode_settings(name)
            self.weather._visual_elements = self.weather._settings_manager.get_settings()
            self._weather_mode_name = name
        self.weather.range_scale = max(10.0, self.feed.weather_range_nm)
        self.update()

    def terrain_display_data(self):
        """The terrain display's dictionary: the feed's data with the display's enum and no None values."""
        data = self.feed.tfr_data()
        data["mode"] = self._terrain_modes[data.pop("mode_name")]
        for key in ("current_altitude", "clearance_altitude"):
            if data.get(key) is None:
                data.pop(key, None)
        if data.get("terrain_clearance") is None:
            data["terrain_clearance"] = 0.0  # the display compares it; None would stop the whole picture
        return data

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            painter.fillRect(self.rect(), Qt.GlobalColor.black)
            half = self.width() / 2.0
            left = QRectF(0.0, 0.0, half, float(self.height()))
            right = QRectF(half, 0.0, half, float(self.height()))
            painter.save()
            painter.setClipRect(left)
            self.weather.draw_radar_elements(painter, left, {})
            painter.restore()
            painter.save()
            painter.setClipRect(right)
            self.terrain.draw_display(painter, right, self.terrain_display_data())
            painter.restore()
        except Exception:  # noqa: BLE001 - a display class raised; the window must stay up
            self.paint_errors += 1
        finally:
            if painter.isActive():
                painter.end()


class RadarWindow(QWidget):
    """The radar display, RT 15: the pictures, the bus status, and the radars' control panel."""

    def __init__(self, terminal, feed=None):
        super().__init__()
        bootstrap()
        from FMOFP.Interfaces.userInterface.displays.radar.radar_display_data_coordinator import (
            get_radar_display_data_coordinator)
        self.terminal = terminal
        self.feed = feed if feed is not None else RadarFeed(terminal, get_radar_display_data_coordinator())
        self.setWindowTitle(f"FMOFP radar display - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.picture = RadarPicture(self.feed)
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.picture, 1)
        layout.addLayout(self._weather_controls())
        layout.addLayout(self._terrain_controls())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(TICK_MS)

    def _weather_controls(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Weather radar:"))
        standby = QPushButton("STANDBY")
        standby.clicked.connect(lambda: self.terminal.send(weather_command(self.terminal, TerminalState.STANDBY,
                                                                           WeatherMode.NONE)))
        row.addWidget(standby)
        for mode in (WeatherMode.SURVEILLANCE, WeatherMode.MAPPING, WeatherMode.PRECIPITATION, WeatherMode.WINDSHEAR,
                     WeatherMode.TURBULENCE):
            button = QPushButton(mode.name)
            button.clicked.connect(lambda _=False, mode=mode: self.terminal.send(
                weather_command(self.terminal, TerminalState.NORMAL, mode)))
            row.addWidget(button)
        row.addStretch(1)
        return row

    def _terrain_controls(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("Terrain-following radar:"))
        self.clearance = QSpinBox()
        self.clearance.setRange(30, 5000)
        self.clearance.setValue(1000)
        self.clearance.setSuffix(" ft")
        standby = QPushButton("STANDBY")
        standby.clicked.connect(lambda: self.terminal.send(tfr_command(self.terminal, TerminalState.STANDBY, TfrMode.NONE,
                                                                       self.clearance.value())))
        row.addWidget(standby)
        for mode in (TfrMode.TFR_SEARCH, TfrMode.TFR_TRACK, TfrMode.TERRAIN_FOLLOWING, TfrMode.GROUND_MAPPING):
            button = QPushButton(mode.name)
            button.clicked.connect(lambda _=False, mode=mode: self.terminal.send(
                tfr_command(self.terminal, TerminalState.NORMAL, mode, self.clearance.value())))
            row.addWidget(button)
        row.addWidget(QLabel("set clearance"))
        row.addWidget(self.clearance)
        row.addStretch(1)
        return row

    def tick(self):
        self.feed.update()
        self.picture.apply()
        self.terminal.take_mode_request()  # the radar picture has no lighting modes; the status reports it
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.feed.weather_status_text()}  |  {self.feed.tfr_status_text()}")


def make_pfd(feed):
    """A PrimaryFlightDisplay whose FMS and FMS control are the bus feed."""
    bootstrap()
    import FMOFP.Interfaces.userInterface.displays.pfd as pfd_module
    pfd_module.get_flightManagementSystem = lambda: feed
    pfd_module.get_fms_control = lambda: feed
    pfd = pfd_module.PrimaryFlightDisplay()
    pfd.start()
    return pfd


class PfdWindow(QWidget):
    """The primary flight display, RT 11."""

    def __init__(self, terminal, feed=None):
        super().__init__()
        self.terminal = terminal
        self.feed = feed if feed is not None else FlightDataFeed(terminal)
        self.setWindowTitle(f"FMOFP primary flight display - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.pfd = make_pfd(self.feed)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayMode as PfdDisplayMode
        self._display_modes = PfdDisplayMode
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.pfd, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(250)

    def tick(self):
        requested = self.terminal.take_mode_request()
        if requested is not None and requested.name in self._display_modes.__members__:
            self.pfd.set_mode(self._display_modes[requested.name])
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.feed.status_text()}")

    def closeEvent(self, event):
        self.pfd.cleanup()
        super().closeEvent(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("display", choices=["radar", "pfd"])
    parser.add_argument("--port-base", type=int, default=51000)
    args = parser.parse_args(argv)
    bootstrap()
    app = QApplication(sys.argv[:1])
    terminal = radar_terminal(args.port_base) if args.display == "radar" else pfd_terminal(args.port_base)
    status = terminal.start()
    if status != Status.OK:
        print(f"display: terminal init failed: {status.name} (port {args.port_base + terminal.rt} in use?)",
              file=sys.stderr)
        return 1
    window = RadarWindow(terminal) if args.display == "radar" else PfdWindow(terminal)
    window.show()
    started = time.time()
    try:
        code = app.exec()
    finally:
        terminal.stop()
    print(f"display: RT {terminal.rt} ran {time.time() - started:.0f} s, {terminal.syncs} frames, "
          f"{terminal.blocks_completed} blocks, {terminal.resets} resets")
    return code


if __name__ == "__main__":
    sys.exit(main())
