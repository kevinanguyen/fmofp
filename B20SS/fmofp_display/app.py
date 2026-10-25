"""The display windows and the entry point: py -m fmofp_display radar|pfd|mfd [--port-base N].

The windows own a DisplayTerminal (the bus, in a thread) and a feed, and paint
with the Python display classes from FMOFP/Interfaces/userInterface/displays,
unchanged. A timer applies what the feed makes of the latest bus data; the
display classes draw it exactly as they drew the bridge's data before.
"""

import argparse
import functools
import importlib
import importlib.machinery
import importlib.util
import os
import sys
import time
import types
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
from fmofp_display.systems_feed import SystemsFeed, TerrainClearance
from fmofp_display.terminal import DisplayTerminal

B20SS = Path(__file__).resolve().parent.parent
RADAR_RT = m.DspRadarStatus.RT
PFD_RT = m.DspPfdStatus.RT
MFD_RT = m.DspMfdStatus.RT
TICK_MS = 50
DEFAULT_RANGE_KM = 200.0


def bootstrap():
    """The FMOFP display code expects B20SS as the working directory (its config paths are relative) and on sys.path."""
    os.chdir(B20SS)
    if str(B20SS) not in sys.path:
        sys.path.insert(0, str(B20SS))


def radar_terminal(port_base):
    return DisplayTerminal(RADAR_RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd,
                           receives=(m.BcastRdrWxStatus, m.BcastRdrWxScan, m.BcastRdrTfrStatus, m.BcastRdrTfrGuidance),
                           blocks=(b.BcastRdrWxProducts, b.BcastRdrTfrElevation))


def pfd_terminal(port_base):
    return DisplayTerminal(PFD_RT, port_base, m.DspPfdStatus, m.DspPfdModeCmd,
                           receives=(m.BcastFmsFlightData, m.BcastFmsStatus, m.BcastFmsAttitudeStatus))


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


# ---- the MFD, RT 13 --------------------------------------------------------------------

def mfd_terminal(port_base):
    return DisplayTerminal(MFD_RT, port_base, m.DspMfdStatus, m.DspMfdModeCmd,
                           receives=(m.BcastRdrWxStatus, m.BcastRdrWxScan, m.BcastRdrTfrStatus, m.BcastRdrTfrGuidance,
                                     m.BcastFcsGcasAlerts, m.BcastFcsPerfStatus, m.BcastFmsFlightData),
                           blocks=(b.BcastRdrWxProducts, b.BcastRdrTfrElevation))


# The getters mfd.py imports inside its page functions, and the feed attribute each answers with.
SYSTEM_GETTERS = {
    "FMOFP.Systems.engineManagement.ecu.engineControlUnit": ("get_engine_control_unit", "engine"),
    "FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys": ("get_gcas", "gcas"),
    "FMOFP.Systems.flightControlSys.performaneMonitoring.performaneMonitoring": ("get_performance_monitor",
                                                                                 "performance"),
    "FMOFP.Systems.comms.messaging_service": ("get_comms_service", "comms"),
    "FMOFP.Systems.missionPlanning.missionService": ("get_mission_service", "mission"),
    "FMOFP.Systems.radarManagement.radarControl": ("get_radar_management_system", "radar_management"),
}
TFR_RADAR_MODULE = "FMOFP.Systems.radarManagement.terrainFollowing.tfr_radar"
_installed = []  # what install_system_sources() changed, for uninstall_system_sources()


def _load_real_attribute(module, name):
    """A stand-in module's answer for anything but the getter: the real module's attribute, loaded on first need
    (from its file, since the stand-in holds its place in sys.modules), so other code in the same process still
    gets the real classes."""
    if name.startswith("__"):
        raise AttributeError(name)
    real = module.__dict__.get("_fmofp_real_module")
    if real is None:
        parent = importlib.import_module(module.__name__.rpartition(".")[0])
        spec = importlib.machinery.PathFinder.find_spec(module.__name__, parent.__path__)
        if spec is None:
            raise AttributeError(name)
        real = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(real)
        module.__dict__["_fmofp_real_module"] = real
    return getattr(real, name)


def _install_attribute(module_name, attribute, value):
    module = sys.modules.get(module_name)
    if module is None:
        module = types.ModuleType(module_name)
        module._fmofp_stand_in = True
        module.__getattr__ = functools.partial(_load_real_attribute, module)  # PEP 562
        sys.modules[module_name] = module
    _installed.append((module_name, module, attribute, module.__dict__.get(attribute)))
    setattr(module, attribute, value)


def install_system_sources(feed):
    """Points the getters mfd.py imports at the feed's stand-ins, without importing the aircraft's systems.

    The pages do `from FMOFP.Systems... import get_x` when they paint, which
    reads the attribute off whatever module is registered under that name. A
    stand-in module is registered for each, so the display process never loads
    the engine, GCAS, performance, comms, mission or radar management code,
    which belongs to the flight computers; the terrain picture's lookup finds
    TerrainClearance where it expects the tfr_radar class.
    """
    for module_name, (getter, attribute) in SYSTEM_GETTERS.items():
        _install_attribute(module_name, getter, lambda feed=feed, attribute=attribute: getattr(feed, attribute))
    _install_attribute(TFR_RADAR_MODULE, "tfr_radar", TerrainClearance)


def uninstall_system_sources():
    """Undoes install_system_sources() (the tests share one process with code that wants the real modules)."""
    while _installed:
        module_name, module, attribute, previous = _installed.pop()
        if module.__dict__.get("_fmofp_stand_in"):
            if sys.modules.get(module_name) is module:
                del sys.modules[module_name]
        elif previous is None:
            delattr(module, attribute)
        else:
            setattr(module, attribute, previous)


class _NoDisplayTree:
    """What the MFD's radar-type switch asks of the display tree manager, when there is none."""

    async def reset_radar_branch(self, radar_type):
        return None


def install_plain_radar_displays():
    """The MFD's radar page gets its pictures from RadarDisplayFactory, whose weather path starts the in-process
    weather widget (event bus, display tree). On the bus the pictures are the plain display classes from the
    factory's own table, one per radar, fed by the feeds; the factory's methods and the display tree are replaced
    by that, in this process only."""
    from FMOFP.Interfaces.userInterface.displays.display_nodes import display_tree_manager
    from FMOFP.Interfaces.userInterface.displays.radar.radar_display_factory import RadarDisplayFactory
    classes = dict(RadarDisplayFactory._display_classes)
    cache = {}

    def create_display(cls, mode):
        kind = type(mode)
        if kind not in cache:
            cache[kind] = classes[kind]()
        return cache[kind]

    def clear_cache(cls, mode_type=None):
        if mode_type is None:
            cache.clear()
        else:
            cache.pop(mode_type, None)

    RadarDisplayFactory.create_display = classmethod(create_display)
    RadarDisplayFactory.clear_cache = classmethod(clear_cache)
    RadarDisplayFactory.invalidate_cache = classmethod(clear_cache)
    display_tree_manager.get_display_tree_manager = lambda: _NoDisplayTree()


def make_mfd(systems_feed):
    """A MultiFunctionDisplay whose in-process sources are the bus feeds: the old-bus radar handler, the display
    tree and the weather widget machinery are not started."""
    bootstrap()
    install_system_sources(systems_feed)
    import FMOFP.Interfaces.userInterface.displays.mfd as mfd_module
    mfd_module.get_radar_message_handler = lambda: None
    install_plain_radar_displays()
    mfd = mfd_module.MultiFunctionDisplay()
    mfd.start()
    return mfd


class MfdWindow(QWidget):
    """The multi-function display, RT 13: the unchanged MFD, its radar page fed like the radar display's."""

    def __init__(self, terminal, radar_feed=None, systems_feed=None):
        super().__init__()
        bootstrap()
        from FMOFP.Interfaces.userInterface.displays.radar.radar_display_data_coordinator import (
            get_radar_display_data_coordinator)
        from FMOFP.Systems.radarManagement.radar_enums import tfr_radarMode, weather_radarMode
        self.terminal = terminal
        self.systems = systems_feed if systems_feed is not None else SystemsFeed(terminal)
        self.radar = radar_feed if radar_feed is not None else RadarFeed(terminal, get_radar_display_data_coordinator())
        self.setWindowTitle(f"FMOFP multi-function display - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.mfd = make_mfd(self.systems)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayMode, DisplayPage
        from FMOFP.Interfaces.userInterface.displays.mfd import RadarType
        self._display_modes = DisplayMode
        self._pages = DisplayPage
        self._radar_types = RadarType
        self._weather_modes = weather_radarMode
        self._terrain_modes = tfr_radarMode
        self._radar_display_seen = None
        self._weather_mode_name = None
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.mfd, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(TICK_MS)

    def tick(self):
        self.radar.update()
        self._apply_radar_page()
        requested = self.terminal.take_mode_request()
        if requested is not None and requested.name in self._display_modes.__members__:
            self.mfd.set_mode(self._display_modes[requested.name])
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.systems.status_text()}  |  {self.radar.weather_status_text()}  |  "
                            f"{self.radar.tfr_status_text()}")

    def _apply_radar_page(self):
        """Keeps the embedded radar picture in step with the bus: the weather display's mode and range, the terrain
        display's profile, mode and clearance."""
        terrain = self.radar.tfr_data()
        self.systems.terrain = (terrain["terrain_clearance"], terrain["level"])
        display = self.mfd._current_radar_display
        if display is not self._radar_display_seen:
            # The MFD switched radar type: it emptied the coordinator and built a new picture.
            self._radar_display_seen = display
            self._weather_mode_name = None
            self.radar.invalidate()
            self.radar.update()
        if self.mfd.current_page != self._pages.RADAR or display is None:
            return
        kind = self.mfd.current_radar_type
        if kind == self._radar_types.WEATHER:
            name = self.radar.weather_draw_mode
            if name != self._weather_mode_name:
                display._current_mode = self._weather_modes[name]
                display._settings_manager.apply_mode_settings(name)
                display._visual_elements = display._settings_manager.get_settings()
                self._weather_mode_name = name
            display.range_scale = max(10.0, self.radar.weather_range_nm)
            self.mfd.radar_data.mode = self._weather_modes[name]
        elif kind == self._radar_types.TFR:
            self.mfd.radar_data.mode = self._terrain_modes[terrain["mode_name"]]
            self.mfd.radar_data.tfr_data = terrain["tfr_data"]

    def closeEvent(self, event):
        self.mfd.stop()
        super().closeEvent(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("display", choices=["radar", "pfd", "mfd"])
    parser.add_argument("--port-base", type=int, default=51000)
    parser.add_argument("--seconds", type=float, default=0.0, help="close after this long (0 = until the window is closed)")
    args = parser.parse_args(argv)
    bootstrap()
    app = QApplication(sys.argv[:1])
    terminals = {"radar": radar_terminal, "pfd": pfd_terminal, "mfd": mfd_terminal}
    terminal = terminals[args.display](args.port_base)
    status = terminal.start()
    if status != Status.OK:
        print(f"display: terminal init failed: {status.name} (port {args.port_base + terminal.rt} in use?)",
              file=sys.stderr)
        return 1
    windows = {"radar": RadarWindow, "pfd": PfdWindow, "mfd": MfdWindow}
    window = windows[args.display](terminal)
    window.show()
    if args.seconds > 0.0:
        QTimer.singleShot(int(args.seconds * 1000.0), app.quit)
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
