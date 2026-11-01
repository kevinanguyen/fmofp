"""What the MFD's pages read from the aircraft's systems, made from the MFD terminal's bus data.

mfd.py's systems, communications and mission pages import a getter inside the
page function and read a dictionary or a list from the object it returns:
get_gcas().get_alerts(), get_performance_monitor().get_exceedances(),
get_engine_control_unit().get_data(), get_comms_service().get_data(),
get_mission_service().get_data() and .get_oob(); its terrain picture reads the
terrain-following radar's clearance manager through
get_radar_management_system(). SystemsFeed answers all of them from
BCAST_FCS_GCAS_ALERTS, BCAST_FCS_PERF_STATUS and BCAST_FMS_FLIGHT_DATA, the
flight computer's and the FMS's messages broadcast to the displays (ICD v1.7;
the MFD had copies of its own in v1.6), from BCAST_ENG_ENGINE_STATUS and
BCAST_ENG_FUEL_STATUS, the engine computer's (v1.8), and from the clearance the
radar feed hands it. A page whose producer is not on the bus yet
(communications, mission) gets what the Python gave it when that service was
not running: an empty dictionary. Nothing here touches Qt or imports FMOFP: the
window points the getters at the feed's stand-ins.

eicas.py polls more of them (v1.9): the FMS and the FMS control
(get_flight_data(), get_tactical_status()['profile_limits']['engine_power']),
the built-in test controller, the defensive, sensor and radar data fusion
services, and the weather radar's wind shear and turbulence processors through
the radar management system. The same feed answers those: the FMS getters are
the PFD's FlightDataFeed, the weather severities come from BCAST_RDR_WX_SCAN,
and the services whose terminals do not exist yet answer as they did when they
were not running. The oil readings, the fuel quantity and the left-right
balance have no getter: the EICAS takes them through set_engine_readings(),
and engine_readings() is what the window hands it each tick.

tsd._poll_data() (v1.11) polls the FMS getters too, and the radar data fusion's
get_fused_tracks(), whose tracks it draws from their to_tsd_dict(); the mission
service's get_nav_data() and the defensive service's get_rwr_contacts() answer
empty until those terminals exist. The fused tracks are the BCAST_RDR_FUSED_TRACKS
block's, and the EICAS's hostile count comes from the same block.
"""

import math
import time
from types import SimpleNamespace

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import Ssm
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.terminal import STALE_AFTER_S

# performaneMonitoring.PerformanceMonitor.LIMITS, in the order of FCS_PERF_STATUS's exceedance bits:
# (bit, the parameter name the Python reported, the limit).
LIMITS = (
    ("exceed_g", "g_force", 9.0),
    ("exceed_aoa", "angle_of_attack", 25.0),
    ("exceed_roll", "roll", 60.0),
    ("exceed_pitch", "pitch", 45.0),
    ("exceed_vertical_speed", "vertical_speed", 6000.0),
)


def _valid(ssm):
    return ssm == Ssm.NORMAL


class TerrainClearance:
    """Stands in for the terrain-following radar object mfd._read_tfr_clearance() looks for.

    It walks the radar management system's radars for an instance of the
    tfr_radar class and reads clearance_manager.min_clearance_m and
    .master_level. The window installs this class as that class, and an
    instance of it as the one radar, so the lookup finds the feed's clearance.

    While the radar provides no clearance the stand-in reports 0 m with the
    level UNKNOWN rather than None: the MFD hands the value to the terrain
    display, which compares it with its bands and cannot draw a None (the
    Python's own defect; the radar window makes the same substitution).
    """

    def __init__(self, feed):
        self.feed = feed

    @property
    def clearance_manager(self):
        return self

    @property
    def min_clearance_m(self):
        clearance_m = self.feed.terrain[0]
        return 0.0 if clearance_m is None else clearance_m

    @property
    def master_level(self):
        clearance_m, level = self.feed.terrain
        return "UNKNOWN" if clearance_m is None else level


class WeatherSeverity:
    """Stands in for the weather radar object eicas._poll_data() looks for among the radar management system's radars.

    It reads radar.windshear_proc.get_last_result() (a list of events with
    .severity and .severity_level()) and .has_microburst(), and
    radar.turbulence_proc.max_category(). The scan summary the radar broadcasts
    (BCAST_RDR_WX_SCAN) carries the strongest severity, the strongest category
    and the microburst flag of the last scan, so one event of the strongest
    severity stands for the result list. The window installs this class as the
    weather_radar class and an instance of it as one of the radars, so the
    isinstance lookup finds it.
    """

    def __init__(self, feed):
        self.feed = feed

    @property
    def windshear_proc(self):
        return self

    @property
    def turbulence_proc(self):
        return self

    def get_last_result(self):
        shear, _, _ = self.feed.weather_severity()
        return [] if shear == "NONE" else [ShearEvent(shear)]

    def has_microburst(self):
        return self.feed.weather_severity()[2]

    def max_category(self):
        return self.feed.weather_severity()[1]


class ShearEvent:
    """As much of weather_processor's wind shear event as the EICAS reads."""

    LEVELS = {"NONE": 0, "LOW": 1, "MODERATE": 2, "SEVERE": 3}

    def __init__(self, severity):
        self.severity = severity

    def severity_level(self):
        return self.LEVELS.get(self.severity, 0)


class FusedTrackView:
    """One track of the fused tracks block, as much of radar_data_fusion.FusedTrack as the TSD and the EICAS read:
    to_tsd_dict() (bearing and range from the aircraft, the type, hostile or not, the confidence, the sources, the
    id and the identity) and the threat for get_threat_tracks(). Positions are metres east, north and up of the
    aircraft, as the Python's were. The dictionary carries no velocity, as the Python's did not, so the TSD draws
    no vector on a fused track."""

    def __init__(self, record):
        self.track_id = record.track_id
        self.position_m = (record.x, record.y, record.z)
        self.velocity_ms = (record.velocity_east, record.velocity_north, record.velocity_up)
        self.confidence = record.confidence
        self.classification = record.classification.name
        self.identity = record.identity.name
        self.threat = record.threat.name
        self.sources = [name for flag, name in ((record.from_targeting, "targeting_radar"),
                                                 (record.from_aewc, "aewc_radar")) if flag]

    @property
    def range_m(self):
        x, y, z = self.position_m
        return math.sqrt(x * x + y * y + z * z)

    @property
    def range_nm(self):
        return self.range_m / 1852.0

    @property
    def bearing_deg(self):
        x, y, _ = self.position_m
        return math.degrees(math.atan2(x, y)) % 360.0

    def is_hostile(self):
        return self.threat == "HOSTILE"

    def to_tsd_dict(self):
        return {"bearing": self.bearing_deg, "range_nm": self.range_nm, "type": self.classification,
                "hostile": self.is_hostile(), "confidence": self.confidence, "sources": list(self.sources),
                "track_id": self.track_id, "identity": self.identity}


class SystemsFeed:
    """Stands in for the systems the MFD's pages, the EICAS and the TSD poll."""

    def __init__(self, terminal, stale_after_s=STALE_AFTER_S, clock=time.time):
        self.terminal = terminal
        self.stale_after_s = stale_after_s
        self.clock = clock
        # (metres, level name) for the terrain picture; the window sets it from the radar feed's guidance.
        self.terrain = (None, "UNKNOWN")
        # The objects the pages' getters return.
        self.gcas = SimpleNamespace(get_alerts=self.get_alerts)
        self.performance = SimpleNamespace(get_exceedances=self.get_exceedances)
        self.engine = SimpleNamespace(get_data=self.engine_data)
        self.comms = SimpleNamespace(get_data=self.comms_data)
        self.mission = SimpleNamespace(get_data=self.mission_data, get_oob=self.order_of_battle,
                                       get_nav_data=self.nav_data)
        self.radar_management = SimpleNamespace(radars={"tfr_radar": TerrainClearance(self),
                                                        "weather_radar": WeatherSeverity(self)})
        # What eicas.py polls beyond the MFD's getters.
        self.flight = FlightDataFeed(terminal, stale_after_s)
        self.fms = self.flight            # get_flightManagementSystem(): get_flight_data()
        self.fms_control = self.flight    # get_fms_control(): get_tactical_status()
        self.bit = SimpleNamespace(self_tests=[])  # BuiltInTestController(): no results until the avionics terminal
        self.defensive = SimpleNamespace(get_data=self.defensive_data, get_rwr_contacts=self.rwr_contacts)
        self.sensors = SimpleNamespace(get_data=self.sensor_data)
        self.fusion = SimpleNamespace(get_threat_tracks=self.threat_tracks, get_fused_tracks=self.fused_tracks)

    def get_alerts(self):
        """GCAS.get_alerts(): the Python's alert dictionaries, rebuilt from the bits and values of MFD_GCAS_ALERTS."""
        message = self.terminal.fresh(m.BcastFcsGcasAlerts, self.stale_after_s)
        if message is None:
            return []
        p = message.payload
        now = self.clock()
        alerts = []
        if p.pullup:
            alerts.append({"severity": 1, "code": "PULLUP", "message": f"PULL UP  VS {p.vertical_speed:.0f} fpm", "ts": now})
        elif p.sink_rate:
            alerts.append({"severity": 1, "code": "SINK RATE", "message": f"SINK RATE  {p.vertical_speed:.0f} fpm",
                           "ts": now})
        if p.terrain:
            alerts.append({"severity": 1, "code": "TERRAIN", "message": f"TERRAIN  {p.terrain_clearance:.0f} ft clearance",
                           "ts": now})
        if p.alt_low:
            alerts.append({"severity": 2, "code": "ALT LOW", "message": f"ALT LOW  {p.altitude:.0f} ft", "ts": now})
        return alerts

    def get_exceedances(self):
        """PerformanceMonitor.get_exceedances(): one row per exceedance bit, with the magnitude the bus carries."""
        message = self.terminal.fresh(m.BcastFcsPerfStatus, self.stale_after_s)
        if message is None:
            return []
        p = message.payload
        values = {"g_force": p.g_force if _valid(p.g_ssm) else None,
                  "angle_of_attack": p.angle_of_attack if _valid(p.aoa_ssm) else None}
        flight = self.terminal.fresh(m.BcastFmsFlightData, self.stale_after_s)
        if flight is not None:
            f = flight.payload
            values["roll"] = f.roll if _valid(f.roll_ssm) else None
            values["pitch"] = f.pitch if _valid(f.pitch_ssm) else None
            values["vertical_speed"] = f.vertical_speed if _valid(f.vertical_speed_ssm) else None
        now = self.clock()
        out = []
        for bit, parameter, limit in LIMITS:
            if getattr(p, bit):
                value = values.get(parameter)
                out.append({"parameter": parameter, "value": round(abs(value), 2) if value is not None else float("nan"),
                            "limit": limit, "ts": now})
        return out

    def engine_data(self):
        """EngineControlUnit.get_data(): the ECU's snapshot dictionary, from engine 1 of BCAST_ENG_ENGINE_STATUS.

        The Python simulated one engine and its pages show one; the first
        engine stands for it. Empty while the engine computer is silent or
        reports the engine as NO_COMPUTED_DATA, as the dictionary was empty
        before the ECU's first pass.
        """
        message = self.terminal.fresh(m.BcastEngEngineStatus, self.stale_after_s)
        if message is None or not _valid(message.payload.engine_1_ssm):
            return {}
        p = message.payload
        return {"thrust_pct": round(p.thrust_1, 2), "n1_pct": round(p.n1_1, 2), "n2_pct": round(p.n2_1, 2),
                "egt_c": round(p.egt_1, 1), "ff_kgh": round(p.fuel_flow_1, 1), "oil_psi": round(p.oil_pressure_1, 2),
                "oil_temp_c": round(p.oil_temperature_1, 2), "vibration": round(p.vibration_1, 3),
                "compressor_eff": round(p.compressor_efficiency_1, 2), "timestamp": self.clock()}

    def fuel_data(self):
        """The tanks of BCAST_ENG_FUEL_STATUS (FuelSystem.get_fuel_status()'s name-to-kg dictionary plus the monitor's
        flags), or empty while the engine computer is silent."""
        message = self.terminal.fresh(m.BcastEngFuelStatus, self.stale_after_s)
        if message is None:
            return {}
        p = message.payload
        return {"Main Left": p.main_left, "Main Right": p.main_right, "Center": p.center, "total": p.total,
                "burn_rate_kgh": p.burn_rate, "transferred_kg": p.transferred,
                "low": [name for name, bit in (("Main Left", p.low_left), ("Main Right", p.low_right),
                                               ("Center", p.low_center)) if bit],
                "critical": [name for name, bit in (("Main Left", p.critical_left), ("Main Right", p.critical_right),
                                                    ("Center", p.critical_center)) if bit],
                "leak_suspected": p.leak_suspected}

    def comms_data(self):
        """CommsService.get_data(): nothing until the communications terminal (RT 2) is on the bus."""
        return {}

    def defensive_data(self):
        """DefensiveService.get_data(): nothing until the defensive systems terminal (RT 22) is on the bus; the
        EICAS then shows its own defaults (full countermeasures, ECM standby, no contacts), as it did with the service
        not running."""
        return {}

    def sensor_data(self):
        """SensorService.get_data(): nothing until the sensor management terminal (RT 10) is on the bus; the EICAS
        reads the health as UNKNOWN and no fused tracks."""
        return {}

    def fused_tracks(self):
        """RadarDataFusion.get_fused_tracks(): the tracks of the last fused tracks block the bus delivered whole, while
        it is fresh; empty when the fusion is silent, as the Python's table was with the fusion not running."""
        block, _, age = self.terminal.latest_block(b.BcastRdrFusedTracks)
        if block is None or age is None or age > self.stale_after_s:
            return []
        return [FusedTrackView(record) for record in block.track]

    def threat_tracks(self):
        """RadarDataFusion.get_threat_tracks(): the fused tracks assessed HOSTILE (the EICAS counts them)."""
        return [track for track in self.fused_tracks() if track.is_hostile()]

    def fused_summary(self):
        """(tracks, hostile, epoch) of the last fused tracks block, or None while there is none."""
        block, _, age = self.terminal.latest_block(b.BcastRdrFusedTracks)
        if block is None or age is None or age > self.stale_after_s:
            return None
        return len(block.track), block.hostile_count, block.epoch

    def weather_severity(self):
        """(wind shear severity, turbulence category, microburst) of the last weather scan, as the names eicas.py
        grades, from BCAST_RDR_WX_SCAN; NONE, NONE, False while the radar is silent."""
        scan = self.terminal.fresh(m.BcastRdrWxScan, self.stale_after_s)
        if scan is None:
            return "NONE", "NONE", False
        p = scan.payload
        return p.max_shear.name, p.max_turbulence.name, bool(p.microburst)

    def engine_readings(self):
        """What eicas.set_engine_readings() takes: the oil pressure, oil temperature and vibration of engine 1 (the
        EICAS's ECU override copies them but never marks them valid: eicas.py left them UNKNOWN for want of a feed)
        and the fuel quantity and left-right balance of the tanks, which have no getter at all. Only what the bus
        vouches for is present; set_engine_readings marks each key valid, so they age out like everything else."""
        readings = {}
        engine = self.engine_data()
        if engine:
            readings.update(oil_psi=engine["oil_psi"], oil_temp_c=engine["oil_temp_c"], vib=engine["vibration"])
        fuel = self.fuel_data()
        if fuel:
            readings.update(total_kg=fuel["total"], balance_kg=fuel["Main Left"] - fuel["Main Right"])
        return readings

    def mission_data(self):
        """MissionService.get_data(): the position from the FMS flight data; the route and targets wait for RT 6."""
        flight = self.terminal.fresh(m.BcastFmsFlightData, self.stale_after_s)
        if flight is None or not _valid(flight.payload.position_ssm):
            return {}
        f = flight.payload
        position = {"lat": f.latitude, "lon": f.longitude}
        if _valid(f.altitude_ssm):
            position["alt_ft"] = f.altitude
        return {"current_position": position}

    def order_of_battle(self):
        """MissionService.get_oob(): empty until the mission terminal is on the bus."""
        return {"summary": {}, "friendly": [], "enemy": [], "neutral": []}

    def nav_data(self):
        """MissionService.get_nav_data(): no route until the mission terminal is on the bus (the TSD then draws no
        waypoints, as with the service not running)."""
        return {"route": []}

    def rwr_contacts(self):
        """DefensiveService.get_rwr_contacts(): none until the defensive systems terminal (RT 22) is on the bus."""
        return []

    def status_text(self):
        parts = []
        alerts = self.terminal.fresh(m.BcastFcsGcasAlerts, self.stale_after_s)
        if alerts is not None:
            text = f"GCAS {alerts.payload.status.name}"
            codes = [a["code"] for a in self.get_alerts()]
            if codes:
                text += " " + ",".join(codes)
            parts.append(text)
        exceedances = self.get_exceedances()
        if exceedances:
            parts.append("exceed " + ",".join(e["parameter"] for e in exceedances))
        position = self.mission_data().get("current_position")
        if position is not None:
            parts.append(f"pos {position['lat']:.4f} {position['lon']:.4f}")
        engine = self.engine_data()
        if engine:
            parts.append(f"ENG N1 {engine['n1_pct']:.1f}% EGT {engine['egt_c']:.0f}C")
        fuel = self.fuel_data()
        if fuel:
            text = f"fuel {fuel['total']:.0f} kg"
            if fuel["critical"]:
                text += " CRITICAL " + ",".join(fuel["critical"])
            elif fuel["low"]:
                text += " LOW " + ",".join(fuel["low"])
            if fuel["leak_suspected"]:
                text += " LEAK?"
            parts.append(text)
        fused = self.fused_summary()
        if fused is not None:
            parts.append(f"fused {fused[0]} ({fused[1]} hostile)")
        return "  ".join(parts) if parts else "FCS: no data"
