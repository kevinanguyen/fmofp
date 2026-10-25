"""What the radar display draws, made from the radar terminal's bus data.

The weather radar display (Interfaces/userInterface/displays/radar) draws from
the RadarDisplayDataCoordinator's stores: lists of dictionaries with a
`position` in nautical miles from the aircraft (x east, y north) and the fields
the bridge used to put there. The terrain display draws a dictionary with the
profile points, the clearance and the altitudes. RadarFeed builds both from
RDR_WX_PRODUCTS, RDR_WX_SCAN, RDR_WX_STATUS, RDR_TFR_GUIDANCE, RDR_TFR_STATUS
and RDR_TFR_ELEVATION as the display terminal receives them. It knows nothing
of Qt or of the display classes: the window applies what it returns.

Positions are true: the radar's azimuth and range become east and north in
nautical miles, and the weather display's range scale follows the radar's range
in effect, so the range rings read what the radar covers. The Python had scaled
every product into a fixed 40 nm picture.
"""

import math

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import Ssm, TerminalState, TfrMode, WeatherMode
from fmofp_display.terminal import STALE_AFTER_S

NM_PER_KM = 1.0 / 1.852
M_PER_FT = 0.3048
SHEAR_FULL_SCALE_KT = 30.0  # shear_severity SEVERE threshold: the intensity the display shades by

# The weather display draws SURVEILLANCE, MAPPING, WINDSHEAR and TURBULENCE and
# shows STANDBY for anything else; PRECIPITATION is a mapping picture.
WEATHER_DRAW_MODE = {
    WeatherMode.NONE: "STANDBY",
    WeatherMode.SURVEILLANCE: "SURVEILLANCE",
    WeatherMode.MAPPING: "MAPPING",
    WeatherMode.PRECIPITATION: "MAPPING",
    WeatherMode.WINDSHEAR: "WINDSHEAR",
    WeatherMode.TURBULENCE: "TURBULENCE",
}
# The terrain display draws SEARCH, TRACK and GROUND_MAPPING; the terrain-following
# modes show the tracked profile with its guidance.
TFR_DRAW_MODE = {
    TfrMode.NONE: "STANDBY",
    TfrMode.TFR_SEARCH: "SEARCH",
    TfrMode.TFR_TRACK: "TRACK",
    TfrMode.TFR_ACTIVE: "TRACK",
    TfrMode.TERRAIN_FOLLOWING: "TRACK",
    TfrMode.OBSTACLE_AVOIDANCE: "TRACK",
    TfrMode.GROUND_MAPPING: "GROUND_MAPPING",
}
STORES = ("vil", "precipitation", "cells", "windshear", "turbulence")


def polar_to_nm(azimuth_deg, range_km):
    """East and north in nautical miles of a point at a true azimuth and range."""
    r = range_km * NM_PER_KM
    a = math.radians(azimuth_deg)
    return (r * math.sin(a), r * math.cos(a))


def cell_velocity_nm_s(azimuth_deg, range_km, azimuth_rate_dps, range_rate_kms):
    """A storm cell's motion as east and north in nautical miles per second."""
    a = math.radians(azimuth_deg)
    radial = range_rate_kms * NM_PER_KM
    tangential = range_km * NM_PER_KM * math.radians(azimuth_rate_dps)
    return (radial * math.sin(a) + tangential * math.cos(a), radial * math.cos(a) - tangential * math.sin(a))


def weather_items(products, request_id, timestamp):
    """The coordinator items of one RDR_WX_PRODUCTS block: {store: [item, ...]}, the shapes the bridge stored."""
    items = {store: [] for store in STORES}
    for i, p in enumerate(products.vil):
        items["vil"].append({
            "position": polar_to_nm(p.azimuth, p.range), "value": p.value, "layer_count": p.layer_count,
            "intensity": p.intensity, "show_values": p.show_values, "id": f"{request_id}_vil_{i}", "timestamp": timestamp})
    for i, p in enumerate(products.precip):
        kind = p.type.name.lower()
        items["precipitation"].append({
            "position": polar_to_nm(p.azimuth, p.range), "type": kind, "precip_type": kind, "rate": p.rate,
            "intensity": p.intensity, "show_values": p.show_values, "id": f"{request_id}_precip_{i}",
            "timestamp": timestamp})
    for i, c in enumerate(products.cells):
        x, y = polar_to_nm(c.azimuth, c.range)
        items["cells"].append({
            "position": (x, y), "x": x, "y": y,
            "velocity": cell_velocity_nm_s(c.azimuth, c.range, c.azimuth_rate, c.range_rate),
            "cell_id": c.cell_id, "reflectivity": c.reflectivity, "intensity": c.intensity, "size": c.size,
            "altitude": c.altitude, "vertical_development": c.vertical_development,
            "id": f"{request_id}_cell_{c.cell_id}", "timestamp": timestamp})
    for i, e in enumerate(products.windshear):
        items["windshear"].append({
            "position": polar_to_nm(e.azimuth, e.range), "intensity": min(1.0, e.shear / SHEAR_FULL_SCALE_KT),
            "shear_knots": e.shear, "severity": e.severity.name, "is_microburst": e.microburst,
            "altitude_ft": e.altitude, "event_id": i, "id": f"{request_id}_ws_{i}", "timestamp": timestamp})
    for i, t in enumerate(products.turbulence):
        items["turbulence"].append({
            "position": polar_to_nm(t.azimuth, t.range), "intensity": t.intensity, "category": t.category.name,
            "altitude_ft": t.altitude, "cell_id": i, "id": f"{request_id}_turb_{i}", "timestamp": timestamp})
    return items


class RadarFeed:
    """Keeps a RadarDisplayDataCoordinator (anything with store_data and reset_data) in step with the bus.

    update() is called from the display's timer. It stores each new weather
    products block once, empties the stores when the radar stops publishing,
    and reports the weather display's drawing mode and range. tfr_data() is
    the terrain display's dictionary.
    """

    def __init__(self, terminal, coordinator, clock=None):
        self.terminal = terminal
        self.coordinator = coordinator
        self.clock = clock
        self.weather_draw_mode = "STANDBY"
        self.weather_range_nm = 40.0  # BaseRadarDisplay.range_scale until the radar says
        self.weather_version = None
        self.stores_cleared = True
        self.blocks_applied = 0

    def _now(self):
        import time
        return time.time() if self.clock is None else self.clock()

    def update(self):
        """One pass: the weather display's mode and range, and the products block if a new one arrived."""
        status = self.terminal.fresh(m.DspRdrWxStatus)
        if status is None or status.payload.state != TerminalState.NORMAL:
            self.weather_draw_mode = "STANDBY"
        else:
            self.weather_draw_mode = WEATHER_DRAW_MODE.get(status.payload.mode, "STANDBY")
        scan = self.terminal.fresh(m.DspRdrWxScan)
        if scan is not None and scan.payload.range_km > 0.0:
            self.weather_range_nm = scan.payload.range_km * NM_PER_KM
        products, version, age = self.terminal.latest_block(b.DspRdrWxProducts)
        if products is not None and version != self.weather_version and age is not None and age <= STALE_AFTER_S:
            self.weather_version = version
            self._apply_products(products, version)
        elif age is not None and age > STALE_AFTER_S and not self.stores_cleared:
            self._clear_stores()

    def _apply_products(self, products, version):
        items = weather_items(products, f"wx{products.scan_id}v{version}", self._now())
        for store in STORES:
            if items[store]:
                self.coordinator.store_data(store, items[store], f"wx{products.scan_id}v{version}")
            else:
                self.coordinator.reset_data(store)
        self.stores_cleared = not any(items[store] for store in STORES)
        self.blocks_applied += 1

    def _clear_stores(self):
        for store in STORES:
            self.coordinator.reset_data(store)
        self.stores_cleared = True

    def weather_status_text(self):
        status = self.terminal.fresh(m.DspRdrWxStatus)
        scan = self.terminal.fresh(m.DspRdrWxScan)
        if status is None:
            return "WX: no data"
        text = f"WX {status.payload.state.name} {status.payload.mode.name}"
        if scan is not None:
            text += f"  scan {scan.payload.scan_id}  {scan.payload.range_km:.0f} km  max {scan.payload.max_reflectivity:.0f} dBZ"
        return text

    def tfr_data(self):
        """The terrain display's dictionary: draw mode name, profile points (metres), clearance and altitudes (metres)."""
        status = self.terminal.fresh(m.DspRdrTfrStatus)
        guidance = self.terminal.fresh(m.DspRdrTfrGuidance)
        elevation, _, age = self.terminal.latest_block(b.DspRdrTfrElevation)
        data = {"mode_name": "STANDBY", "tfr_data": [], "terrain_clearance": None, "current_altitude": None,
                "clearance_altitude": None, "system_health": "UNKNOWN", "level": "NONE", "pull_up": False}
        if status is not None:
            s = status.payload
            data["system_health"] = s.state.name if s.state in (TerminalState.NORMAL, TerminalState.DEGRADED) else "UNKNOWN"
            if s.state == TerminalState.NORMAL:
                data["mode_name"] = TFR_DRAW_MODE.get(s.mode, "STANDBY")
        if guidance is not None and guidance.payload.clearance_ssm == Ssm.NORMAL:
            g = guidance.payload
            data["terrain_clearance"] = g.min_clearance * M_PER_FT
            data["level"] = g.level.name
            data["pull_up"] = g.pull_up
            if g.commands_valid:
                data["clearance_altitude"] = g.commanded_altitude * M_PER_FT
        if elevation is not None and age is not None and age <= STALE_AFTER_S and elevation.mode != TfrMode.NONE:
            n = len(elevation.profile)
            step = elevation.range_m / (n - 1) if n > 1 else 0.0
            data["tfr_data"] = [{"distance": elevation.range_m if i == n - 1 else i * step, "elevation": p.elevation}
                                for i, p in enumerate(elevation.profile)]
            data["current_altitude"] = elevation.altitude * M_PER_FT
        return data

    def tfr_status_text(self):
        status = self.terminal.fresh(m.DspRdrTfrStatus)
        guidance = self.terminal.fresh(m.DspRdrTfrGuidance)
        if status is None:
            return "TFR: no data"
        text = f"TFR {status.payload.state.name} {status.payload.mode.name}"
        if guidance is not None and guidance.payload.clearance_ssm == Ssm.NORMAL:
            g = guidance.payload
            text += f"  {g.level.name}  min clearance {g.min_clearance:.0f} ft  cmd {g.commanded_altitude:.0f} ft"
        return text
