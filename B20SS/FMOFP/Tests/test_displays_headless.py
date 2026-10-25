"""
Test suite: EICAS / TSD / SMS display smoke tests (headless)

Each display is instantiated, its internal state is populated with
representative data, and paint_display() is called through a real
QPainter on an off-screen QPixmap.  The test passes if no exception
is raised and the pixmap is not null.

Qt is initialised in offscreen mode so no X server / compositor is needed.
"""

import os
import sys

# Force Qt to use the offscreen platform before ANY Qt library is loaded.
os.environ["QT_QPA_PLATFORM"] = "offscreen"

_B20SS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_B20SS, os.path.join(_B20SS, 'FMOFP')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# QApplication MUST exist before any QWidget subclass is instantiated.
# Create it here at module level, before any FMOFP imports that may
# trigger Qt widget construction during module initialisation.
from PyQt6.QtWidgets import QApplication as _QApp
_APP = _QApp.instance() or _QApp(sys.argv)

import time
import traceback

from FMOFP.Utils.logger.sys_logger import get_logger
logger = get_logger()


# ─────────────────────────────────────────────────────── framework ───────

class _Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self._failures = []

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print(f"  ✓  {name}")
        else:
            self.failed += 1
            msg = f"  ✗  {name}" + (f"  [{detail}]" if detail else "")
            print(msg)
            self._failures.append(msg)

    def summary(self):
        total = self.passed + self.failed
        print(f"\n  {self.passed}/{total} passed")
        if self._failures:
            print("\n  Failures:")
            for f in self._failures:
                print(f"    {f}")
        return self.failed == 0


def _qt_app():
    """Return the QApplication singleton (created at module level)."""
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or _APP


def _paint_widget(widget) -> bool:
    """
    Call paint_display() via a real QPainter on a 800×600 QPixmap.
    Returns True if no exception was raised.
    """
    from PyQt6.QtGui import QPainter, QPixmap
    from PyQt6.QtCore import Qt

    # Stop background timers so they cannot fire during the sync paint call
    for attr in ("_poll_timer", "_update_timer"):
        timer = getattr(widget, attr, None)
        if timer is not None and hasattr(timer, "stop"):
            try:
                timer.stop()
            except Exception:
                pass

    pixmap = QPixmap(800, 600)
    pixmap.fill(Qt.GlobalColor.black)
    painter = QPainter(pixmap)
    try:
        widget._running = True
        widget.resize(800, 600)
        widget.paint_display(painter)
        return True
    except Exception as exc:
        print(f"    paint_display raised: {exc}")
        traceback.print_exc()
        return False
    finally:
        painter.end()


# ───────────────────────────── EICAS tests ───────────────────────────────

def test_eicas_instantiation(r: _Results) -> None:
    print("\n  ── EICAS: instantiation ──")
    _qt_app()
    try:
        from FMOFP.Interfaces.userInterface.displays.eicas import EICASDisplay
        disp = EICASDisplay()
        r.check("EICASDisplay instantiates without error", True)
        r.check("has _engine dict",  hasattr(disp, "_engine"))
        r.check("has _fuel dict",    hasattr(disp, "_fuel"))
        r.check("has _hydraulic dict", hasattr(disp, "_hydraulic"))
        r.check("has _electrical dict", hasattr(disp, "_electrical"))
        r.check("has _alerts list",  hasattr(disp, "_alerts"))
        disp.stop()
    except Exception as exc:
        r.check("EICASDisplay instantiates without error", False, str(exc))


def test_eicas_paint_normal(r: _Results) -> None:
    print("\n  ── EICAS: paint in normal state ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.eicas import EICASDisplay
    disp = EICASDisplay()
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds in normal state", ok)
    disp.stop()


def test_eicas_paint_warnings(r: _Results) -> None:
    print("\n  ── EICAS: paint with active warnings ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.eicas import EICASDisplay
    disp = EICASDisplay()
    # Force alert-generating conditions. These must go through the setters,
    # which mark the fields valid: writing straight into the private dicts
    # leaves them UNKNOWN, and an UNKNOWN reading is reported as unknown
    # (CAUTION) rather than run against its threshold, so no WARNING would be
    # produced at all.
    disp.set_engine_readings(egt_c=850.0,      # above WARNING threshold
                             oil_psi=35.0,     # below LOW threshold
                             total_kg=300.0)   # below WARN threshold
    disp.set_system_readings(sys_a_psi=1800.0)  # below WARN threshold
    disp._alerts = disp._compute_alerts(disp._engine["thrust_pct"])
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds with active warnings", ok)
    r.check("alerts list is non-empty", len(disp._alerts) > 0,
            f"got {len(disp._alerts)}")
    has_warn = any(a["severity"] == "WARNING" for a in disp._alerts)
    r.check("at least one WARNING alert generated", has_warn)
    disp.stop()


class _TextRecordingPainter:
    """Forwards to a real QPainter, recording every string drawn."""

    def __init__(self, painter):
        self._painter = painter
        self.texts = []

    def drawText(self, *args):
        for a in args:
            if isinstance(a, str):
                self.texts.append(a)
        return self._painter.drawText(*args)

    def __getattr__(self, name):
        return getattr(self._painter, name)


def _painted_texts(widget):
    """Paint the widget once and return every string it drew."""
    from PyQt6.QtGui import QPainter, QPixmap
    from PyQt6.QtCore import Qt

    for attr in ("_poll_timer", "_update_timer"):
        t = getattr(widget, attr, None)
        if t is not None and hasattr(t, "stop"):
            try:
                t.stop()
            except Exception:
                pass

    pixmap = QPixmap(900, 700)
    pixmap.fill(Qt.GlobalColor.black)
    raw = QPainter(pixmap)
    rec = _TextRecordingPainter(raw)
    try:
        widget._running = True
        widget.resize(900, 700)
        widget.paint_display(rec)
    finally:
        raw.end()
    return rec.texts


def test_eicas_unread_parameters_show_as_unknown(r: _Results) -> None:
    """With nothing feeding it, EICAS must not draw engine or fuel numbers.

    This was found by rendering the display to a PNG and looking at it, not by
    a failing test: the suite was green while the panel showed THRUST 70.0 %,
    N1 78.5 %, EGT 620 °C and the rest -- the seed values from __init__ --
    in normal green, and FUEL TOTAL as `0 kg`, which is drawn RED and reads as
    a fuel emergency rather than as an absent reading.
    """
    from FMOFP.Interfaces.userInterface.displays.eicas import (
        EICASDisplay, _UNKNOWN_TEXT,
    )

    print("\n  \u2500\u2500 EICAS: unread parameters \u2500\u2500")

    disp = EICASDisplay()
    texts = _painted_texts(disp)
    joined = " ".join(texts)

    # The eight engine rows plus FUEL TOTAL and FLOW.
    r.check("every unread parameter is drawn as dashes",
            texts.count(_UNKNOWN_TEXT) >= 10,
            f"count={texts.count(_UNKNOWN_TEXT)}")

    for seed in ("70.0 %", "78.5 %", "84.2 %", "620 \u00b0C", "2400 kg/h",
                 "62.0", "95.0", "0.30"):
        r.check(f"the seed value {seed!r} is not presented as a reading",
                seed not in joined, f"texts={texts[:40]}")

    r.check("FUEL TOTAL does not read 0 kg (which draws red, i.e. an emergency)",
            "     0 kg" not in joined, f"texts={texts[:40]}")

    # NON-TAUTOLOGICAL: the seeds really are still in place, so the assertions
    # above are about the *rendering* and not about an empty object.
    r.check("NON-TAUTOLOGICAL: the engine seeds really are still there",
            disp._engine["thrust_pct"] == 70.0 and disp._engine["n1_pct"] == 78.5)
    r.check("NON-TAUTOLOGICAL: the pre-fix format really did produce '70.0 %'",
            f"{disp._engine['thrust_pct']:5.1f} %".strip() == "70.0 %")
    r.check("NON-TAUTOLOGICAL: fuel total really is 0.0, which is < 500 (red)",
            disp._fuel["total_kg"] == 0.0 and disp._fuel["total_kg"] < 500)

    # A supplied reading must appear, so "unknown" is not simply always drawn.
    disp.set_engine_readings(oil_psi=48.0, vib=0.9, total_kg=4200.0)
    texts2 = " ".join(_painted_texts(disp))
    r.check("a supplied oil pressure is shown as a number", "48.0" in texts2)
    r.check("a supplied vibration is shown as a number", "0.90" in texts2)
    r.check("a supplied fuel quantity is shown as a number", "4200 kg" in texts2)
    r.check("parameters still unread stay as dashes", _UNKNOWN_TEXT in texts2)

    try:
        disp.cleanup()
    except Exception:
        pass


def test_eicas_compute_alerts(r: _Results) -> None:
    print("\n  ── EICAS: _compute_alerts logic ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.eicas import EICASDisplay
    disp = EICASDisplay()

    # Readings must arrive through set_engine_readings(), which marks them
    # valid. Writing straight into disp._engine leaves them UNKNOWN -- and an
    # UNKNOWN reading is now reported as UNKNOWN rather than assumed in-limits,
    # which is the point of the change: silence used to mean "nominal".
    r.check("a fresh display reports UNKNOWN, not nominal",
            any("UNKNOWN" in a["text"] for a in disp._compute_alerts(70.0)))

    # All in-limits, supplied properly. Supplying only the engine and fuel
    # fields is no longer "all parameters": the hydraulic, electrical and fuel
    # balance fields report UNKNOWN until something publishes them, which is the
    # point -- nothing in this project ever has, and the panel used to show a
    # nominal bus voltage, two healthy generators and 3000 psi regardless.
    disp.set_engine_readings(egt_c=600.0, oil_psi=65.0, oil_temp_c=95.0,
                             vib=0.3, total_kg=5000.0)
    engine_ok = disp._compute_alerts(70.0)
    r.check("no engine or fuel-quantity alert when those are in limits",
            not any(t.startswith(("ENG", "FUEL  QUANTITY", "FUEL QTY"))
                    for t in [a["text"] for a in engine_ok]),
            f"got {[a['text'] for a in engine_ok]}")
    r.check("the unpublished systems are reported as unknown, not as nominal",
            {"HYD A  PRESSURE UNKNOWN", "ELEC  BUS VOLTS UNKNOWN"}
            <= {a["text"] for a in engine_ok},
            f"got {[a['text'] for a in engine_ok]}")

    # With every field supplied, the list really does go quiet -- so the
    # UNKNOWN cautions above are about absent data, not a permanent nag.
    disp.set_system_readings(sys_a_psi=3000, sys_b_psi=3000, sys_c_psi=2950,
                             main_bus_v=115.0, ess_bus_v=115.0,
                             gen1_ok=True, gen2_ok=True,
                             cabin_alt_ft=8000, cabin_temp_c=22.0,
                             oxy_psi=1800)
    disp.set_engine_readings(thrust_pct=70.0, n1_pct=78.5, n2_pct=84.2,
                             ff_kgh=2400.0, balance_kg=0.0, flow_kgh=2400.0)
    alerts_ok = disp._compute_alerts(70.0)
    r.check("no alerts when every parameter is supplied and in limits",
            len(alerts_ok) == 0, f"got {len(alerts_ok)}: "
            f"{[a['text'] for a in alerts_ok]}")

    # NON-TAUTOLOGICAL: these alarms were unreachable, because nothing ever
    # wrote the values they test.
    r.check("NON-TAUTOLOGICAL: a bus undervolt alarm is now reachable",
            any("MAIN BUS LO" in a["text"] for a in
                (disp.set_system_readings(main_bus_v=94.0)
                 or disp._compute_alerts(70.0))))
    r.check("NON-TAUTOLOGICAL: a hydraulic low-pressure alarm is now reachable",
            any("HYD C  PRESSURE LOW" in a["text"] for a in
                (disp.set_system_readings(sys_c_psi=1800)
                 or disp._compute_alerts(70.0))))
    r.check("NON-TAUTOLOGICAL: a fuel-imbalance alarm is now reachable",
            any("FUEL  IMBALANCE" in a["text"] for a in
                (disp.set_engine_readings(balance_kg=350.0)
                 or disp._compute_alerts(70.0))))

    # Put them back in limits so the checks below see a quiet baseline.
    disp.set_system_readings(main_bus_v=115.0, sys_c_psi=2950)
    disp.set_engine_readings(balance_kg=0.0)

    # EGT above warning threshold
    disp.set_engine_readings(egt_c=820.0)
    alerts_egt = disp._compute_alerts(70.0)
    r.check("EGT > 800 → WARNING alert",
            any(a["severity"] == "WARNING" and "EGT" in a["text"]
                for a in alerts_egt))

    # The three alarms that were mathematically unreachable before, because
    # their inputs were sine waves bounded inside their own limits:
    #   oil_psi    60 + 4*sin()       -> [56, 64]     vs <50 caution
    #   oil_temp_c 92 + 8*sin()       -> [84, 100]    vs >130 warning
    #   vib        0.2 + 0.15*|sin()| -> [0.20, 0.35] vs >0.8 caution
    disp.set_engine_readings(oil_psi=38.0, oil_temp_c=140.0, vib=0.95)
    reachable = [a["text"] for a in disp._compute_alerts(70.0)]
    r.check("oil-pressure alarm is reachable",
            any("OIL PRESSURE LO" in t for t in reachable), str(reachable))
    r.check("oil-temperature alarm is reachable",
            any("OIL TEMP HIGH" in t for t in reachable), str(reachable))
    r.check("vibration alarm is reachable",
            any("VIBRATION HIGH" in t for t in reachable), str(reachable))

    # NON-TAUTOLOGICAL: the pre-fix generators could not reach those limits.
    import math as _m
    psi_range  = (60 - 4, 60 + 4)
    temp_range = (92 - 8, 92 + 8)
    vib_range  = (0.2, 0.2 + 0.15)
    r.check("pre-fix oil pressure could never fall below 50 "
            "(proves these bite)", psi_range[0] > 50)
    r.check("pre-fix oil temp could never exceed 130", temp_range[1] < 130)
    r.check("pre-fix vibration could never exceed 0.8", vib_range[1] < 0.8)

    # Fuel is no longer owned by the display: it must not start pre-loaded.
    fresh = EICASDisplay()
    r.check("fuel quantity is not seeded to 6800 kg",
            fresh._fuel["total_kg"] == 0.0, f"got {fresh._fuel['total_kg']}")
    r.check("fuel quantity starts UNKNOWN", not fresh.has_valid("total_kg"))
    fresh.stop()
    disp.stop()


# ───────────────────────────── TSD tests ─────────────────────────────────

def test_tsd_instantiation(r: _Results) -> None:
    print("\n  ── TSD: instantiation ──")
    _qt_app()
    try:
        from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
        disp = TacticalSituationDisplay()
        r.check("TacticalSituationDisplay instantiates", True)
        r.check("has _threats list", hasattr(disp, "_threats"))
        r.check("has _heading",      hasattr(disp, "_heading"))
        r.check("has _g_force",      hasattr(disp, "_g_force"))
        disp.stop()
    except Exception as exc:
        r.check("TacticalSituationDisplay instantiates", False, str(exc))


def test_tsd_paint_normal(r: _Results) -> None:
    print("\n  ── TSD: paint in normal state ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
    disp = TacticalSituationDisplay()
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds in normal state", ok)
    disp.stop()


def test_tsd_paint_with_threats(r: _Results) -> None:
    print("\n  ── TSD: paint with threat contacts ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
    disp = TacticalSituationDisplay()
    disp._heading  = 180.0
    disp._airspeed = 420.0
    disp._altitude = 28000.0
    disp._g_force  = 3.5
    disp._threats  = [
        {"bearing": 45.0, "range_nm": 15.0, "type": "FIGHTER", "hostile": True},
        {"bearing": 270.0, "range_nm": 40.0, "type": "SAM",    "hostile": True},
        {"bearing": 120.0, "range_nm": 55.0, "type": "UNKNOWN","hostile": False},
    ]
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds with threat contacts", ok)
    disp.stop()


def test_tfr_warning_bands_are_on_screen(r: _Results) -> None:
    """TFR clearance bands must render inside the display, not below it.

    Every band was built as QRectF(left, rect.bottom(), width, rect.bottom()-y),
    putting its TOP at the display's BOTTOM edge -- so all three extended below
    the visible area and none was ever seen. For an 800x600 rect with
    max_elevation 2000, the critical band spanned y 600.0..645.0 on a display of
    0..600. These are the bands that make a terrain-following profile
    actionable.
    """
    print("\n  ── TFR: warning bands land inside the display ──")
    _qt_app()
    from PyQt6.QtCore import QRectF
    from FMOFP.Interfaces.userInterface.displays.radar.tfr_mode_handler import (
        TFRModeHandler as T)
    from FMOFP.Systems.radarManagement.terrainFollowing import tfr_processor as P

    rect = QRectF(0, 0, 800, 600)
    for name, elev in T._warning_zones.items():
        y = T._elevation_to_y(elev, rect)
        top, bottom_edge = y, y + (rect.bottom() - y)
        r.check(f"{name} band top is on screen",
                0 <= top <= rect.bottom(), f"top={top}")
        r.check(f"{name} band bottom is on screen",
                0 <= bottom_edge <= rect.bottom(), f"bottom={bottom_edge}")

    # One definition of the thresholds, shared with the advisory logic.
    r.check("critical band matches CAUTION_M", T._warning_zones['critical'] == P.CAUTION_M)
    r.check("warning band matches LOW_M",      T._warning_zones['warning']  == P.LOW_M)
    r.check("caution band matches CLEAR_M",    T._warning_zones['caution']  == P.CLEAR_M)

    # NON-TAUTOLOGICAL: the pre-fix geometry really was off-screen.
    y_crit = T._elevation_to_y(P.CAUTION_M, rect)
    pre_fix_top = rect.bottom()
    pre_fix_bottom = pre_fix_top + (rect.bottom() - y_crit)
    r.check("pre-fix band started at the bottom edge and ran past it "
            "(proves this assertion bites)",
            pre_fix_top == 600.0 and pre_fix_bottom > 600.0,
            f"spanned {pre_fix_top}..{pre_fix_bottom}")


def test_mfd_terrain_clearance_not_invented(r: _Results) -> None:
    """Terrain clearance must come from the ClearanceManager or read UNKNOWN.

    It was the literal 500 with system_health 'NORMAL', carrying the author's
    own comment that both should come from real data -- so the most
    safety-relevant number on a terrain-following display showed a constant
    500 m in green on every frame.
    """
    print("\n  ── MFD: terrain clearance is real or UNKNOWN ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.mfd import MultiFunctionDisplay
    disp = MultiFunctionDisplay()

    clearance, health = disp._read_tfr_clearance()
    r.check("no TFR radar -> clearance is None, not 500",
            clearance is None, f"got {clearance}")
    r.check("no TFR radar -> health is UNKNOWN, not NORMAL",
            health == "UNKNOWN", f"got {health}")

    # A real manager is read faithfully, including its level.
    class _FakeMgr:
        min_clearance_m = 120.0
        master_level = "CAUTION"

    class _FakeRadar:
        clearance_manager = _FakeMgr()

    import FMOFP.Systems.radarManagement.terrainFollowing.tfr_radar as TR
    original = TR.tfr_radar
    try:
        TR.tfr_radar = _FakeRadar          # make isinstance match the fake
        from FMOFP.Systems.radarManagement import radarControl as RC
        rms = RC.get_radar_management_system()
        saved = getattr(rms, "radars", None)
        rms.radars = {"tfr": _FakeRadar()}
        clearance, health = disp._read_tfr_clearance()
        r.check("a real clearance is reported", clearance == 120.0, f"got {clearance}")
        r.check("its level is reported", health == "CAUTION", f"got {health}")
        if saved is not None:
            rms.radars = saved
    finally:
        TR.tfr_radar = original


def test_pfd_factory_refuses_unfed_display(r: _Results) -> None:
    """A PFD with no flight-data feed must never be handed to an operator.

    HolographicPFD inherits HolographicDisplay rather than PrimaryFlightDisplay,
    so it never polls the FMS: its altitude/airspeed/mach/heading are assigned
    only in __init__ and read everywhere else, while a scan-line animation makes
    it look live. theme_config.json selects it for the "Modern" theme, so
    choosing Modern silently replaced the PFD with a disconnected one showing a
    fixed 30,000 ft / 450 kt picture.
    """
    print("\n  ── PFD factory: refuses a display with no feed ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays import pfd_display_factory as F
    from FMOFP.Interfaces.userInterface.displays.holographic_pfd import HolographicPFD
    from FMOFP.Interfaces.userInterface.displays.pfd import PrimaryFlightDisplay
    from FMOFP.Interfaces.userInterface.displays.visual import theme_manager as TM

    r.check("HolographicPFD declares it has no feed",
            HolographicPFD.PROVIDES_FLIGHT_DATA is False)
    r.check("PrimaryFlightDisplay declares it has one",
            PrimaryFlightDisplay.PROVIDES_FLIGHT_DATA is True)

    tm = TM.get_theme_manager()
    original = tm.get_display_type
    try:
        tm.get_display_type = lambda kind, default="standard": "holographic"
        F.PFDDisplayFactory._display_instance = None
        F.PFDDisplayFactory._current_display_type = None
        display = F.PFDDisplayFactory.create_display()
        r.check("theme asking for holographic still yields a fed PFD",
                getattr(type(display), "PROVIDES_FLIGHT_DATA", False),
                f"got {type(display).__name__}")
        r.check("the substitute polls the FMS", hasattr(display, "update_flight_data"))
        r.check("the substitute tracks field validity", hasattr(display, "validity"))
    finally:
        tm.get_display_type = original
        F.PFDDisplayFactory._display_instance = None
        F.PFDDisplayFactory._current_display_type = None

    # NON-TAUTOLOGICAL: the unfed display really does hold fixed values with no
    # way to update them.
    holo = HolographicPFD()
    r.check("the unfed display has no FMS poll at all",
            not hasattr(holo, "update_flight_data"))
    r.check("its seeds are neutral, not a cruise picture "
            "(proves this assertion bites)",
            holo.altitude == 0 and holo.airspeed == 0,
            f"alt={holo.altitude} ias={holo.airspeed}")


def test_pfd_secondary_readouts_show_as_unknown(r: _Results) -> None:
    """The small readouts must not assert values beside four X'd-out regions.

    The airspeed, altitude, heading and attitude regions get an amber X when
    their feed is stale, but G-FORCE, AOA and the flight-mode band kept drawing
    from their seeds -- 1.0, 0 and "NORMAL" -- in normal colours. A display that
    flags four instruments invalid while reporting a nominal G, AOA and mode
    beside them contradicts itself, and "NORMAL" is a claim about the aircraft
    rather than about the display. flight_mode had no validity marking at all,
    so it could never go stale.
    """
    from FMOFP.Interfaces.userInterface.displays.pfd import (
        PrimaryFlightDisplay, UNKNOWN_READING_TEXT,
    )

    print("\n  \u2500\u2500 PFD: secondary readouts \u2500\u2500")

    disp = PrimaryFlightDisplay()
    texts = _painted_texts(disp)
    joined = " ".join(texts)

    r.check("G-FORCE, AOA and the mode band all read as unknown",
            texts.count(UNKNOWN_READING_TEXT) >= 3,
            f"count={texts.count(UNKNOWN_READING_TEXT)} texts={texts[:30]}")
    r.check("no G-force value is asserted", "1.0" not in texts, f"texts={texts[:30]}")
    r.check("no AOA value is asserted",
            not any(t.startswith("0.0\u00b0") for t in texts), f"texts={texts[:30]}")
    r.check("the mode band does not claim NORMAL",
            "NORMAL" not in joined, f"texts={texts[:30]}")

    # NON-TAUTOLOGICAL: the seeds are still in place, so the checks above are
    # about the rendering rather than about an emptied object.
    r.check("NON-TAUTOLOGICAL: the seeds really are still there",
            disp.g_force == 1.0 and disp.aoa == 0 and disp.flight_mode == "NORMAL")
    r.check("NON-TAUTOLOGICAL: a seeded g_force of 1.0 really is inside the "
            "normal band, so it drew in the nominal colour",
            disp.g_force <= 2.0)

    # A real reading must still appear, so "unknown" is not simply always drawn.
    disp.g_force = 5.5
    disp.aoa = 12.0
    disp.flight_mode = "COMBAT"
    disp.validity.mark('g_force', 'aoa', 'flight_mode')
    joined2 = " ".join(_painted_texts(disp))
    r.check("a marked g_force is shown", "5.5" in joined2, f"texts={joined2[:200]}")
    r.check("a marked AOA is shown", "12.0" in joined2, f"texts={joined2[:200]}")
    r.check("a marked mode is shown", "COMBAT" in joined2, f"texts={joined2[:200]}")

    try:
        disp.cleanup()
    except Exception:
        pass


def test_pfd_flags_invalid_fields(r: _Results) -> None:
    """The PFD must flag missing data rather than showing seeded values (H9)."""
    print("\n  ── PFD: invalid fields are flagged, not invented ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.pfd import PrimaryFlightDisplay
    disp = PrimaryFlightDisplay()

    for field in ("altitude", "airspeed", "heading", "pitch", "roll"):
        r.check(f"{field} starts INVALID", not disp.validity.is_valid(field))
    r.check("no seeded cruise altitude", disp.altitude == 0, f"got {disp.altitude}")
    r.check("no seeded cruise airspeed", disp.airspeed == 0, f"got {disp.airspeed}")

    r.check("paints with nothing valid", _paint_widget(disp))

    disp.validity.mark('altitude', 'airspeed', 'heading', 'pitch', 'roll')
    r.check("paints with everything valid", _paint_widget(disp))

    # a field the publisher omits must NOT be refreshed by its own last value
    disp.validity.invalidate('altitude')
    disp.validity.mark_from({'heading': 10}, {'heading': 'heading',
                                              'altitude': 'altitude'})
    r.check("an omitted key stays invalid", not disp.validity.is_valid('altitude'))
    r.check("a present key is marked valid", disp.validity.is_valid('heading'))


def test_tsd_no_synthetic_threats(r: _Results) -> None:
    """No fusion must mean no contacts -- not invented ones.

    This test previously asserted the OPPOSITE: that the fallback "produces at
    least one threat". That locked in the defect. _simulate_threats() returned a
    FIGHTER closing inside 20 nm and a SAM at ~32 nm, drawn with the same
    hostile symbology, threat rings and history trails as real tracks, so clean
    airspace displayed as two inbound hostiles with nothing to distinguish them.
    """
    print("\n  ── TSD: empty fusion yields NO contacts ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
    disp = TacticalSituationDisplay()

    disp._fusion = None
    threats = disp._get_fused_threats()
    r.check("_get_fused_threats returns a list", isinstance(threats, list))
    r.check("no fusion -> no contacts", threats == [], f"got {len(threats)}")

    class _EmptyFusion:
        def get_fused_tracks(self):
            return []

    disp._fusion = _EmptyFusion()
    r.check("fusion with zero tracks -> no contacts",
            disp._get_fused_threats() == [])

    class _BrokenFusion:
        def get_fused_tracks(self):
            raise RuntimeError("fusion unavailable")

    disp._fusion = _BrokenFusion()
    r.check("fusion that raises -> no contacts, no exception",
            disp._get_fused_threats() == [])

    r.check("the synthetic generator is gone",
            not hasattr(disp, "_simulate_threats"))

    # NON-TAUTOLOGICAL: the pre-fix shape really did invent contacts.
    import math as _m, time as _t
    pre_fix = [
        {"bearing": (0 + 45 + 10 * _m.sin(_t.time() * 0.2)) % 360,
         "range_nm": 18 - 5 * abs(_m.sin(_t.time() * 0.1)),
         "type": "FIGHTER", "hostile": True},
    ]
    r.check("pre-fix fallback returned a hostile inside 20 nm "
            "(proves this assertion bites)",
            pre_fix and pre_fix[0]["hostile"] and pre_fix[0]["range_nm"] < 20)
    disp.stop()


def test_theme_colours_are_not_shared_objects(r: _Results) -> None:
    """get_color() must hand out a copy, not the theme's own QColor.

    QColor is mutable. Returning the dictionary's own object meant a caller
    that adjusted the alpha for its own drawing rewrote the theme for
    everyone. Three callers do exactly that -- holographic_display.py's
    draw_holographic_background() and draw_startup_animation() each take a
    colour and immediately setAlpha() it.

    Measured against the live theme manager before the fix:

        two calls return the SAME object: True
        alpha before: 200
        after one caller setAlpha(30), a later caller sees: 30
        after a second caller setAlpha(200), the theme reads: 200
        the first caller's own variable also changed to: 200

    The last line is the sharp one: two routines that each set their own alpha
    overwrite each other, so what an element renders at depends on the order
    the drawing code ran in that frame.
    """
    print("\n  ── theme: get_color() hands out copies ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.visual.theme_manager import (
        get_theme_manager)
    tm = get_theme_manager()

    a = tm.get_color("hud")
    b = tm.get_color("hud")
    r.check("two calls return distinct objects", a is not b)
    r.check("...with equal values", a == b, f"{a.getRgb()} vs {b.getRgb()}")

    original = tm.get_color("hud").alpha()
    mine = tm.get_color("hud")
    mine.setAlpha(17)
    r.check("a caller's setAlpha does not reach the theme",
            tm.get_color("hud").alpha() == original,
            f"theme alpha became {tm.get_color('hud').alpha()}, was {original}")

    other = tm.get_color("hud")
    other.setAlpha(211)
    r.check("...nor another caller's copy", mine.alpha() == 17,
            f"the first caller's own colour became {mine.alpha()}")

    # The fallback path must obey the same contract.
    f1 = tm.get_color("definitely_not_a_theme_colour")
    f2 = tm.get_color("definitely_not_a_theme_colour")
    r.check("the unknown-colour fallback also returns distinct objects",
            f1 is not f2)

    # End to end: the real caller that mutates must leave the theme alone.
    before = tm.get_color("hud").alpha()
    try:
        from PyQt6.QtGui import QPainter, QPixmap
        from PyQt6.QtCore import QRectF
        from FMOFP.Interfaces.userInterface.displays.holographic_display import (
            HolographicDisplay)
        from FMOFP.Interfaces.userInterface.displays.base_display import DisplayType
        disp = HolographicDisplay(DisplayType.MFD)
        pm = QPixmap(400, 300)
        painter = QPainter(pm)
        disp.draw_holographic_background(painter, QRectF(0, 0, 400, 300))
        painter.end()
        drew = True
    except Exception as exc:  # noqa: BLE001
        drew = f"{type(exc).__name__}: {exc}"
    r.check("draw_holographic_background() runs", drew is True, str(drew))
    r.check("...and leaves the theme's hud alpha unchanged",
            tm.get_color("hud").alpha() == before,
            f"{tm.get_color('hud').alpha()}, was {before}")

    print("\n  ── NON-TAUTOLOGICAL: the pre-fix shape really does alias ──")
    from PyQt6.QtGui import QColor
    shared = {"colors": {"hud": QColor(0, 255, 0, 200)}}

    def pre_fix_get_color(name):
        return shared["colors"][name]          # the old body

    p1 = pre_fix_get_color("hud")
    p1.setAlpha(30)
    p2 = pre_fix_get_color("hud")
    r.check("returning the dict's own object leaks the mutation",
            p2.alpha() == 30 and p1 is p2,
            f"alpha={p2.alpha()}, same object={p1 is p2}")


def test_tsd_oob_needs_a_real_ownship_position(r: _Results) -> None:
    """The OOB overlay is positioned entirely from ownship, so no fix means no overlay.

    _oob_to_threat_vectors turns each unit's lat/lon into a bearing and range
    relative to us, and the 120 nm inclusion cut is measured from there too.
    The caller used to read ownship as
        nav.get("latitude", 35.4147) / nav.get("longitude", -97.3866)
    -- Tinker AFB. With no navigation fix the whole overlay was drawn relative
    to a position nobody had measured, in the same symbology as real fused
    tracks. Rendered and looked at before the fix: a hostile SAM at 017/30nm,
    a hostile FIGHTER at 233/31nm, a friendly TANKER at 065/26nm.

    Same defect as test_tsd_no_synthetic_threats guards against, reaching the
    same layer by a different route.
    """
    print("\n  ── TSD: no nav fix yields NO order-of-battle overlay ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
    from FMOFP.Utils.common.optional_reads import UNKNOWN, optional_float

    disp = TacticalSituationDisplay()
    oob = {
        "enemy": [
            {"id": "SAM-1", "type": "SAM_SITE", "position": [35.9, -97.2]},
            {"id": "FTR-2", "type": "FIGHTER", "position": [35.1, -97.9]},
        ],
        "friendly": [{"id": "TKR-9", "type": "TANKER", "position": [35.6, -96.9]}],
        "neutral": [],
    }

    r.check("unknown ownship -> no contacts",
            disp._oob_to_threat_vectors(oob, UNKNOWN, UNKNOWN) == [])
    r.check("a known latitude alone is not enough",
            disp._oob_to_threat_vectors(oob, 35.4147, UNKNOWN) == [])
    r.check("nor a known longitude alone",
            disp._oob_to_threat_vectors(oob, UNKNOWN, -97.3866) == [])

    with_fix = disp._oob_to_threat_vectors(oob, 35.4147, -97.3866)
    r.check("a real fix still produces the overlay", len(with_fix) == 3,
            f"got {len(with_fix)}")
    r.check("...with the bearings and ranges unchanged by this fix",
            {c["name"]: (c["bearing"], c["range_nm"]) for c in with_fix} ==
            {"SAM-1": (17.4, 30.5), "FTR-2": (233.1, 31.4),
             "TKR-9": (65.0, 26.3)},
            str({c["name"]: (c["bearing"], c["range_nm"]) for c in with_fix}))

    r.check("the heading argument is gone, since bearings here are true",
            "own_hdg" not in
            __import__("inspect").signature(
                disp._oob_to_threat_vectors).parameters)

    # The reads the caller performs, against a nav dict with no fix in it.
    nav_no_fix = {"heading": 0.0, "altitude": 0.0}
    r.check("latitude reads as UNKNOWN rather than Tinker AFB",
            optional_float(nav_no_fix, "latitude",
                           minimum=-90.0, maximum=90.0) is UNKNOWN)
    r.check("longitude too",
            optional_float(nav_no_fix, "longitude",
                           minimum=-180.0, maximum=180.0) is UNKNOWN)
    r.check("an out-of-range latitude is refused, not clamped",
            optional_float({"latitude": 181.0}, "latitude",
                           minimum=-90.0, maximum=90.0) is UNKNOWN)

    # NON-TAUTOLOGICAL: the pre-fix read really did invent a position, and it
    # really did place contacts from it.
    pre_lat = nav_no_fix.get("latitude", 35.4147)
    pre_lon = nav_no_fix.get("longitude", -97.3866)
    r.check("the pre-fix default was a real place, not a sentinel",
            (pre_lat, pre_lon) == (35.4147, -97.3866))
    invented = disp._oob_to_threat_vectors(oob, pre_lat, pre_lon)
    r.check("and projecting from it yields plausible hostile contacts "
            "(proves this assertion bites)",
            len(invented) == 3 and any(c["hostile"] for c in invented),
            f"{[(c['name'], c['bearing'], c['range_nm']) for c in invented]}")

    r.check("the display tracks whether ownship position is known",
            hasattr(disp, "_ownship_position_known"))
    r.check("and starts out saying it is not", disp._ownship_position_known is False)

    # The withheld overlay has to be visible, not silent: an empty picture and
    # an unplottable one look identical otherwise.
    disp.resize(900, 700)
    disp._ownship_position_known = False
    text = _painted_texts(disp)
    r.check("an unplottable overlay is annunciated on the display",
            any("OOB" in t and "WITHHELD" in t for t in text),
            f"drawn text: {text}")

    disp._ownship_position_known = True
    text_ok = _painted_texts(disp)
    r.check("and the annunciation is absent once position is known",
            not any("WITHHELD" in t for t in text_ok))
    disp.stop()


def test_tsd_paint_combat_mode(r: _Results) -> None:
    print("\n  ── TSD: paint in COMBAT mode ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.tsd import TacticalSituationDisplay
    disp = TacticalSituationDisplay()
    disp._mode = "COMBAT"
    disp._g_force = 7.2
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds in COMBAT mode", ok)
    disp.stop()


# ───────────────────────────── SMS tests ─────────────────────────────────

def test_sms_instantiation(r: _Results) -> None:
    print("\n  ── SMS: instantiation ──")
    _qt_app()
    try:
        from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
        disp = StoresManagementDisplay()
        r.check("StoresManagementDisplay instantiates", True)
        r.check("has _stations list",    hasattr(disp, "_stations"))
        r.check("has _master_arm attr",  hasattr(disp, "_master_arm"))
        r.check("11 stations in loadout", len(disp._stations) == 11,
                f"got {len(disp._stations)}")
        disp.stop()
    except Exception as exc:
        r.check("StoresManagementDisplay instantiates", False, str(exc))


def test_sms_paint_safe(r: _Results) -> None:
    print("\n  ── SMS: paint with master arm SAFE ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
    disp = StoresManagementDisplay()
    disp._master_arm = "SAFE"
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds with master arm SAFE", ok)
    disp.stop()


def test_sms_paint_armed(r: _Results) -> None:
    print("\n  ── SMS: paint with master arm ARMED ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
    disp = StoresManagementDisplay()
    disp._master_arm = "ARMED"
    # Arm a few stations
    for sta in disp._stations[:3]:
        if sta.store:
            sta.store.state = "ARMED"
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds with master arm ARMED", ok)
    disp.stop()


def test_sms_total_weight(r: _Results) -> None:
    print("\n  ── SMS: total weight calculation ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
    disp = StoresManagementDisplay()
    weight = disp._total_weight_kg()
    r.check("total weight is positive", weight > 0, f"got {weight:.0f} kg")
    # Expend all stores
    for sta in disp._stations:
        if sta.store:
            sta.store.state = "EXPENDED"
    weight_after = disp._total_weight_kg()
    r.check("total weight is 0 after all expended",
            weight_after == 0.0, f"got {weight_after}")
    disp.stop()


def test_sms_armed_count(r: _Results) -> None:
    print("\n  ── SMS: armed station count ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
    # Fresh instance so state from previous tests does not leak
    disp = StoresManagementDisplay()
    # Reset all to SAFE first
    for sta in disp._stations:
        if sta.store:
            sta.store.state = "SAFE"
    # Arm exactly 2 stations
    armed = 0
    for sta in disp._stations:
        if sta.store and armed < 2:
            sta.store.state = "ARMED"
            armed += 1
    r.check("armed_count() returns 2", disp._armed_count() == 2,
            f"got {disp._armed_count()}")
    disp.stop()


def test_sms_signature_strip_paint(r: _Results) -> None:
    print("\n  ── SMS: stealth mode affects paint ──")
    _qt_app()
    from FMOFP.Interfaces.userInterface.displays.sms import StoresManagementDisplay
    disp = StoresManagementDisplay()
    disp._stealth = "ON"
    disp._rcs     = 0.001
    disp._ir_sig  = 0.05
    disp._ecm     = "ACTIVE"
    ok = _paint_widget(disp)
    r.check("paint_display() succeeds in stealth mode", ok)
    disp.stop()


# ──────────────────────────────────────────── runner ─────────────────────

def run_all() -> bool:
    print("=" * 60)
    print(" Display Headless Smoke Test Suite")
    print("=" * 60)

    r = _Results()

    tests = [
        # EICAS
        test_eicas_instantiation,
        test_eicas_paint_normal,
        test_eicas_paint_warnings,
        test_eicas_unread_parameters_show_as_unknown,
        test_eicas_compute_alerts,
        # TSD
        test_tsd_instantiation,
        test_tsd_paint_normal,
        test_tsd_paint_with_threats,
        test_tfr_warning_bands_are_on_screen,
        test_mfd_terrain_clearance_not_invented,
        test_pfd_factory_refuses_unfed_display,
        test_pfd_flags_invalid_fields,
        test_pfd_secondary_readouts_show_as_unknown,
        test_tsd_no_synthetic_threats,
        test_tsd_oob_needs_a_real_ownship_position,
        test_theme_colours_are_not_shared_objects,
        test_tsd_paint_combat_mode,
        # SMS
        test_sms_instantiation,
        test_sms_paint_safe,
        test_sms_paint_armed,
        test_sms_total_weight,
        test_sms_armed_count,
        test_sms_signature_strip_paint,
    ]

    for test_fn in tests:
        try:
            test_fn(r)
        except Exception as exc:
            r.failed += 1
            print(f"  ✗  {test_fn.__name__} raised: {exc}")
            traceback.print_exc()

    print("\n" + "=" * 60)
    passed = r.summary()
    print("=" * 60)
    return passed


if __name__ == "__main__":
    ok = run_all()
    sys.exit(0 if ok else 1)
