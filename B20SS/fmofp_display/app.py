"""The display windows and the entry point: py -m fmofp_display radar|pfd|mfd|eicas|tsd|sms [--port-base N].

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
from fmofp_bus.icd.enums import (AewcMode, SarMode, TargetingMode, TerminalState, TfrMode, TrackFilter,
                                 WeatherMode)
from fmofp_bus.status import Status
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed
from fmofp_display.systems_feed import SystemsFeed, TerrainClearance, WeatherSeverity
from fmofp_display.terminal_process import TerminalProcess

B20SS = Path(__file__).resolve().parent.parent
RADAR_RT = m.DspRadarStatus.RT
PFD_RT = m.DspPfdStatus.RT
MFD_RT = m.DspMfdStatus.RT
EICAS_RT = m.DspEicasStatus.RT
TSD_RT = m.DspTsdStatus.RT
SMS_RT = m.DspSmsStatus.RT
TICK_MS = 50
DEFAULT_RANGE_KM = 200.0


def bootstrap():
    """The FMOFP display code expects B20SS as the working directory (its config paths are relative) and on sys.path."""
    os.chdir(B20SS)
    if str(B20SS) not in sys.path:
        sys.path.insert(0, str(B20SS))


def radar_terminal(port_base):
    return TerminalProcess(RADAR_RT, port_base, m.DspRadarStatus, m.DspRadarModeCmd,
                           receives=(m.BcastRdrWxStatus, m.BcastRdrWxScan, m.BcastRdrTfrStatus, m.BcastRdrTfrGuidance,
                                     m.BcastRdrTgtStatus, m.BcastRdrAewcStatus, m.BcastRdrSarStatus),
                           blocks=(b.BcastRdrWxProducts, b.BcastRdrTfrElevation))


def pfd_terminal(port_base):
    return TerminalProcess(PFD_RT, port_base, m.DspPfdStatus, m.DspPfdModeCmd,
                           receives=(m.BcastFmsFlightData, m.BcastFmsStatus, m.BcastFmsAttitudeStatus))


def weather_command(terminal, state, mode, range_km=DEFAULT_RANGE_KM):
    cmd = m.RdrWxModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                         range_km=range_km, tilt=0.0)
    return m.DspRdrWxCmd(payload=cmd)


def tfr_command(terminal, state, mode, set_clearance_ft):
    cmd = m.RdrTfrModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                          set_clearance=float(set_clearance_ft))
    return m.DspRdrTfrCmd(payload=cmd)


def targeting_command(terminal, state, mode, lock_track_id=0, track_filter=TrackFilter.ALL):
    cmd = m.RdrTgtModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                          track_filter=track_filter, lock_track_id=int(lock_track_id))
    return m.DspRdrTgtCmd(payload=cmd)


def aewc_command(terminal, state, mode, azimuth_start=0.0, azimuth_end=90.0, elevation=0.0):
    cmd = m.RdrAewcModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                           azimuth_start=float(azimuth_start), azimuth_end=float(azimuth_end),
                           elevation=float(elevation))
    return m.DspRdrAewcCmd(payload=cmd)


def sar_command(terminal, state, mode, resolution_m=1.0):
    cmd = m.RdrSarModeCmd(cmd_seq=terminal.next_command_seq(), requested_state=state, requested_mode=mode,
                          resolution=float(resolution_m))
    return m.DspRdrSarCmd(payload=cmd)


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
        layout.addLayout(self._targeting_controls())
        layout.addLayout(self._aewc_controls())
        layout.addLayout(self._sar_controls())
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

    def _targeting_controls(self):
        """The targeting radar's buttons; its picture is the MFD's radar page, as it was in the Python."""
        row = QHBoxLayout()
        row.addWidget(QLabel("Targeting radar:"))
        self.lock_track = QSpinBox()
        self.lock_track.setRange(0, 65535)
        self.lock_track.setValue(1)
        self.lock_track.setSpecialValueText("none")
        standby = QPushButton("STANDBY")
        standby.clicked.connect(lambda: self.terminal.send(targeting_command(self.terminal, TerminalState.STANDBY,
                                                                             TargetingMode.NONE)))
        row.addWidget(standby)
        search = QPushButton("TARGET_SEARCH")
        search.clicked.connect(lambda: self.terminal.send(targeting_command(self.terminal, TerminalState.NORMAL,
                                                                            TargetingMode.TARGET_SEARCH)))
        row.addWidget(search)
        follow = QPushButton("TARGET_TRACK")
        follow.clicked.connect(lambda: self.terminal.send(targeting_command(self.terminal, TerminalState.NORMAL,
                                                                            TargetingMode.TARGET_TRACK,
                                                                            self.lock_track.value())))
        row.addWidget(follow)
        row.addWidget(QLabel("lock track"))
        row.addWidget(self.lock_track)
        row.addWidget(QLabel("(picture: the MFD's radar page)"))
        row.addStretch(1)
        return row

    def _aewc_controls(self):
        """The AEWC radar's buttons; its picture is the MFD's radar page too."""
        row = QHBoxLayout()
        row.addWidget(QLabel("AEWC radar:"))
        standby = QPushButton("STANDBY")
        standby.clicked.connect(lambda: self.terminal.send(aewc_command(self.terminal, TerminalState.STANDBY,
                                                                        AewcMode.NONE)))
        row.addWidget(standby)
        for mode in (AewcMode.AEWC_SEARCH, AewcMode.AEWC_SURVEILLANCE, AewcMode.AEWC_TRACK):
            button = QPushButton(mode.name)
            button.clicked.connect(lambda _=False, mode=mode: self.terminal.send(
                aewc_command(self.terminal, TerminalState.NORMAL, mode)))
            row.addWidget(button)
        row.addStretch(1)
        return row

    def _sar_controls(self):
        """The synthetic aperture radar's buttons and its resolution; its picture is the MFD's radar page."""
        from PyQt6.QtWidgets import QDoubleSpinBox
        row = QHBoxLayout()
        row.addWidget(QLabel("SAR:"))
        self.sar_resolution = QDoubleSpinBox()
        self.sar_resolution.setRange(0.1, 10.0)
        self.sar_resolution.setSingleStep(0.1)
        self.sar_resolution.setDecimals(1)
        self.sar_resolution.setValue(1.0)
        self.sar_resolution.setSuffix(" m")
        standby = QPushButton("STANDBY")
        standby.clicked.connect(lambda: self.terminal.send(sar_command(self.terminal, TerminalState.STANDBY,
                                                                       SarMode.NONE, self.sar_resolution.value())))
        row.addWidget(standby)
        for mode in (SarMode.STRIPMAP, SarMode.SPOTLIGHT, SarMode.SCANSAR):
            button = QPushButton(mode.name)
            button.clicked.connect(lambda _=False, mode=mode: self.terminal.send(
                sar_command(self.terminal, TerminalState.NORMAL, mode, self.sar_resolution.value())))
            row.addWidget(button)
        row.addWidget(QLabel("resolution"))
        row.addWidget(self.sar_resolution)
        row.addStretch(1)
        return row

    def tick(self):
        self.feed.update()
        self.picture.apply()
        self.terminal.take_mode_request()  # the radar picture has no lighting modes; the status reports it
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.feed.weather_status_text()}  |  {self.feed.tfr_status_text()}  |  "
                            f"{self.feed.targeting_status_text()}  |  {self.feed.aewc_status_text()}  |  "
                            f"{self.feed.sar_status_text()}")


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
    return TerminalProcess(MFD_RT, port_base, m.DspMfdStatus, m.DspMfdModeCmd,
                           receives=(m.BcastRdrWxStatus, m.BcastRdrWxScan, m.BcastRdrTfrStatus, m.BcastRdrTfrGuidance,
                                     m.BcastFcsGcasAlerts, m.BcastFcsPerfStatus, m.BcastFmsFlightData,
                                     m.BcastEngEngineStatus, m.BcastEngFuelStatus, m.BcastRdrTgtStatus,
                                     m.BcastRdrAewcStatus, m.BcastRdrSarStatus),
                           blocks=(b.BcastRdrWxProducts, b.BcastRdrTfrElevation, b.MfdRdrTgtTracks,
                                   b.MfdRdrAewcTracks, b.MfdRdrSarImage))


# The getters mfd.py imports inside its page functions and eicas.py inside its poll, and the feed attribute each
# answers with (BuiltInTestController is a class the EICAS constructs; the stand-in returns the feed's object).
SYSTEM_GETTERS = {
    "FMOFP.Systems.engineManagement.ecu.engineControlUnit": ("get_engine_control_unit", "engine"),
    "FMOFP.Systems.flightControlSys.groundCollisionAvoidanceSys.groundCollisionAvoidanceSys": ("get_gcas", "gcas"),
    "FMOFP.Systems.flightControlSys.performaneMonitoring.performaneMonitoring": ("get_performance_monitor",
                                                                                 "performance"),
    "FMOFP.Systems.comms.messaging_service": ("get_comms_service", "comms"),
    "FMOFP.Systems.missionPlanning.missionService": ("get_mission_service", "mission"),
    "FMOFP.Systems.radarManagement.radarControl": ("get_radar_management_system", "radar_management"),
    "FMOFP.Systems.flightManagementSys.flightManagementSystem": ("get_flightManagementSystem", "fms"),
    "FMOFP.Systems.flightManagementSys.fmsControl": ("get_fms_control", "fms_control"),
    "FMOFP.Systems.builtInTestSystems.bitControl": ("BuiltInTestController", "bit"),
    "FMOFP.Systems.defensiveSys.defensiveService": ("get_defensive_service", "defensive"),
    "FMOFP.Systems.sensorManagement.sensorService": ("get_sensor_service", "sensors"),
    "FMOFP.Systems.radarManagement.radar_data_fusion": ("get_radar_data_fusion", "fusion"),
}
TFR_RADAR_MODULE = "FMOFP.Systems.radarManagement.terrainFollowing.tfr_radar"
WEATHER_RADAR_MODULE = "FMOFP.Systems.radarManagement.weather.weather_radar"
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
    """Points the getters mfd.py and eicas.py import at the feed's stand-ins, without importing the aircraft's
    systems.

    The pages (and the EICAS's poll) do `from FMOFP.Systems... import get_x`
    when they run, which reads the attribute off whatever module is registered
    under that name. A stand-in module is registered for each, so the display
    process never loads the engine, GCAS, performance, comms, mission, FMS,
    built-in test, defensive, sensor, fusion or radar management code, which
    belongs to the flight computers; the terrain picture's lookup finds
    TerrainClearance where it expects the tfr_radar class, and the EICAS's
    weather lookup finds WeatherSeverity where it expects the weather_radar class.
    """
    for module_name, (getter, attribute) in SYSTEM_GETTERS.items():
        _install_attribute(module_name, getter, lambda feed=feed, attribute=attribute: getattr(feed, attribute))
    _install_attribute(TFR_RADAR_MODULE, "tfr_radar", TerrainClearance)
    _install_attribute(WEATHER_RADAR_MODULE, "weather_radar", WeatherSeverity)


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
        from FMOFP.Systems.radarManagement.radar_enums import (aewc_radarMode, sar_radarMode, targeting_radarMode,
                                                               tfr_radarMode, weather_radarMode)
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
        self._targeting_modes = targeting_radarMode
        self._aewc_modes = aewc_radarMode
        self._sar_modes = sar_radarMode
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
        elif kind == self._radar_types.TARGETING:
            targeting = self.radar.targeting_data()
            display.range_scale = targeting["range_nm"]
            self.mfd.radar_data.mode = self._targeting_modes[targeting["mode_name"]]
            self.mfd.radar_data.targets = targeting["targets"]
        elif kind == self._radar_types.AEWC:
            aewc = self.radar.aewc_data()
            display.range_scale = aewc["range_nm"]
            self.mfd.radar_data.mode = self._aewc_modes[aewc["mode_name"]]
            self.mfd.radar_data.aewc_tracks = aewc["targets"]
        elif kind == self._radar_types.SAR:
            sar = self.radar.sar_data()
            display.range_scale = sar["range_nm"]
            self.mfd.radar_data.mode = self._sar_modes[sar["mode_name"]]
            # _get_radar_data merges this dictionary into the picture's data when it is truthy.
            self.mfd.radar_data.sar_data = ({"image_data": sar["image_data"], "corner_points": sar["corner_points"],
                                             "resolution": sar["resolution"], "scan_progress": sar["scan_progress"]}
                                            if sar["image_data"] else None)

    def closeEvent(self, event):
        self.mfd.stop()
        super().closeEvent(event)


# ---- the EICAS, RT 14 ------------------------------------------------------------------

def eicas_terminal(port_base):
    return TerminalProcess(EICAS_RT, port_base, m.DspEicasStatus, m.DspEicasModeCmd,
                           receives=(m.BcastFmsFlightData, m.BcastFmsStatus, m.BcastFcsGcasAlerts, m.BcastFcsPerfStatus,
                                     m.BcastEngEngineStatus, m.BcastEngFuelStatus, m.BcastRdrWxScan),
                           blocks=(b.BcastRdrFusedTracks,))


def make_eicas(systems_feed):
    """An EICASDisplay whose sources are the bus feeds: the FMS and the FMS control, the GCAS, the performance
    monitor, the engine control unit, the built-in test controller, the defensive, sensor and fusion services and
    the weather radar it polls ten times a second all answer from the feed's stand-ins."""
    bootstrap()
    install_system_sources(systems_feed)
    import FMOFP.Interfaces.userInterface.displays.eicas as eicas_module
    eicas = eicas_module.EICASDisplay()
    eicas.start()
    return eicas


class EicasWindow(QWidget):
    """The engine indicating and crew alerting display, RT 14: the unchanged EICAS over the bus feeds."""

    def __init__(self, terminal, systems_feed=None):
        super().__init__()
        bootstrap()
        self.terminal = terminal
        self.systems = systems_feed if systems_feed is not None else SystemsFeed(terminal)
        self.setWindowTitle(f"FMOFP EICAS - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.eicas = make_eicas(self.systems)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayMode
        self._display_modes = DisplayMode
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.eicas, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(TICK_MS)

    def tick(self):
        # The oil readings, the fuel quantity and the balance have no getter the EICAS polls: it takes them through
        # set_engine_readings(), which marks them valid, so they age out like everything else when the bus goes quiet.
        readings = self.systems.engine_readings()
        if readings:
            self.eicas.set_engine_readings(**readings)
        requested = self.terminal.take_mode_request()
        if requested is not None and requested.name in self._display_modes.__members__:
            self.eicas.set_mode(self._display_modes[requested.name])
        alerts = self.eicas._alerts
        warnings = sum(1 for a in alerts if a["severity"] == "WARNING")
        cautions = sum(1 for a in alerts if a["severity"] == "CAUTION")
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{warnings} warnings {cautions} cautions  |  {self.systems.status_text()}")

    def closeEvent(self, event):
        self.eicas.stop()
        super().closeEvent(event)


# ---- the TSD, RT 16 --------------------------------------------------------------------

def tsd_terminal(port_base):
    return TerminalProcess(TSD_RT, port_base, m.DspTsdStatus, m.DspTsdModeCmd,
                           receives=(m.BcastFmsFlightData, m.BcastFmsStatus, m.BcastFmsAttitudeStatus, m.TsdFmsEnergy),
                           blocks=(b.BcastRdrFusedTracks,))


def make_tsd(systems_feed):
    """A TacticalSituationDisplay whose sources are the bus feeds: the FMS and the FMS control (flight data, tactical
    systems, envelope warnings, energy metrics), the radar data fusion (the fused tracks), and the mission and
    defensive services answering as they did with their terminals not yet on the bus (no route, no order of battle,
    no radar warning contacts). Its own 10 Hz poll reads them."""
    bootstrap()
    install_system_sources(systems_feed)
    import FMOFP.Interfaces.userInterface.displays.tsd as tsd_module
    tsd = tsd_module.TacticalSituationDisplay()
    tsd.start()
    return tsd


class TsdWindow(QWidget):
    """The tactical situation display, RT 16: the unchanged TSD over the bus feeds."""

    def __init__(self, terminal, systems_feed=None):
        super().__init__()
        bootstrap()
        self.terminal = terminal
        self.systems = systems_feed if systems_feed is not None else SystemsFeed(terminal)
        self.setWindowTitle(f"FMOFP tactical situation display - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.tsd = make_tsd(self.systems)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayMode
        self._display_modes = DisplayMode
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.tsd, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(250)

    def tick(self):
        requested = self.terminal.take_mode_request()
        if requested is not None and requested.name in self._display_modes.__members__:
            self.tsd.set_mode(self._display_modes[requested.name])
        fused = self.systems.fused_summary()
        tracks = "fusion: no data" if fused is None else f"fused {fused[0]} ({fused[1]} hostile), epoch {fused[2]}"
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.systems.flight.status_text()}  |  {tracks}")

    def closeEvent(self, event):
        self.tsd.stop()
        super().closeEvent(event)


# ---- the SMS, RT 17 --------------------------------------------------------------------

def sms_terminal(port_base):
    return TerminalProcess(SMS_RT, port_base, m.DspSmsStatus, m.DspSmsModeCmd, receives=(m.BcastFmsStatus,))


def make_sms(systems_feed):
    """A StoresManagementDisplay whose sources are the bus feeds: the FMS and the FMS control it polls five times a
    second for the flight mode and the tactical systems (its master arm is the weapons state, its ECM cell the
    countermeasures state, its signature the stealth bit). Its weapon stations are its own table, as in the Python:
    no stores management computer exists to report them."""
    bootstrap()
    install_system_sources(systems_feed)
    import FMOFP.Interfaces.userInterface.displays.sms as sms_module
    sms = sms_module.StoresManagementDisplay()
    sms.start()
    return sms


class SmsWindow(QWidget):
    """The stores management display, RT 17: the unchanged SMS over the bus feeds."""

    def __init__(self, terminal, systems_feed=None):
        super().__init__()
        bootstrap()
        self.terminal = terminal
        self.systems = systems_feed if systems_feed is not None else SystemsFeed(terminal)
        self.setWindowTitle(f"FMOFP stores management display - RT {terminal.rt}")
        self.status = QLabel("waiting for the bus")
        self.sms = make_sms(self.systems)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayMode
        self._display_modes = DisplayMode
        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(self.sms, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(250)

    def tick(self):
        requested = self.terminal.take_mode_request()
        if requested is not None and requested.name in self._display_modes.__members__:
            self.sms.set_mode(self._display_modes[requested.name])
        sms = self.sms
        self.status.setText(f"RT {self.terminal.rt} {self.terminal.state.name}  frames {self.terminal.syncs}  |  "
                            f"{self.systems.flight.tactical_text()}  |  MASTER ARM {sms._master_arm}, "
                            f"{sms._armed_count()} stations armed, {sms._total_weight_kg():.0f} kg")

    def closeEvent(self, event):
        self.sms.stop()
        super().closeEvent(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("display", choices=["radar", "pfd", "mfd", "eicas", "tsd", "sms"])
    parser.add_argument("--port-base", type=int, default=51000)
    parser.add_argument("--seconds", type=float, default=0.0, help="close after this long (0 = until the window is closed)")
    args = parser.parse_args(argv)
    bootstrap()
    app = QApplication(sys.argv[:1])
    terminals = {"radar": radar_terminal, "pfd": pfd_terminal, "mfd": mfd_terminal, "eicas": eicas_terminal,
                 "tsd": tsd_terminal, "sms": sms_terminal}
    terminal = terminals[args.display](args.port_base)
    status = terminal.start()
    if status != Status.OK:
        print(f"display: terminal init failed: {status.name} (port {args.port_base + terminal.rt} in use?)",
              file=sys.stderr)
        return 1
    windows = {"radar": RadarWindow, "pfd": PfdWindow, "mfd": MfdWindow, "eicas": EicasWindow, "tsd": TsdWindow,
               "sms": SmsWindow}
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
