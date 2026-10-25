"""
Navigation Service

Owns GPS, INS, and NavDataFusion update loops:
  - GPS generates position fixes at ~1 Hz and feeds NavDataFusion
  - INS propagates attitude/position at 10 Hz and applies GPS corrections
  - Fused position is written back into the FMS navigation dict
  - Singleton: get_nav_service()
"""

import threading
import time
from typing import Optional

from FMOFP.Utils.logger.sys_logger import get_logger

logger = get_logger()

_nav_service = None


class NavService:
    def __init__(self):
        self._running  = threading.Event()
        self._thread   = None
        # Failure-streak tracking, read by health_report(): the monotonic
        # time of the first failure in the current unbroken streak, or
        # None when the last pass succeeded.
        self._failing_since = None
        self._last_error = ""
        self._last_gps = 0.0
        self._start_lock = threading.Lock()

        # Lazy-import to avoid circular dependencies at module load
        self._gps     = None
        self._ins     = None
        self._fusion  = None
        self._fms     = None

    def _lazy_init(self):
        if self._gps is None:
            try:
                # Use the GPSSystem already owned and started by FMS.
                # Creating a second GPS instance would produce different positions.
                from FMOFP.Systems.flightManagementSys.flightManagementSystem import get_flightManagementSystem
                _fms_temp = get_flightManagementSystem()
                self._gps = _fms_temp.gps_system   # GPSSystem instance
                logger.info("[NAV] GPS attached via FMS gps_system")
            except Exception as e:
                logger.warning(f"[NAV] GPS unavailable: {e}")

        if self._ins is None:
            try:
                from FMOFP.Systems.nav.ins.inertialNavigationSystem import get_ins
                self._ins = get_ins()
                logger.info("[NAV] INS attached")
            except Exception as e:
                logger.warning(f"[NAV] INS unavailable: {e}")

        if self._fusion is None:
            try:
                from FMOFP.Systems.flightManagementSys.flightManagementSystem import get_flightManagementSystem
                self._fms = get_flightManagementSystem()
                self._fusion = self._fms.nav_fusion
                logger.info("[NAV] NavDataFusion attached via FMS")
            except Exception as e:
                logger.warning(f"[NAV] NavDataFusion unavailable: {e}")

    def _tick(self, dt: float):
        now = time.time()

        # GPS fix at ~1 Hz
        if self._gps and self._fusion and (now - self._last_gps >= 1.0):
            try:
                # get_position_wgs84() returns (lat_deg, lon_deg, alt_ft) or None
                fix = self._gps.get_position_wgs84()
                if fix:
                    lat, lon, alt_ft = fix   # already in feet
                    self._fusion.update_gps(lat, lon, alt_ft)
                    # Correct INS with GPS fix
                    if self._ins:
                        self._ins.correct(lat, lon, alt_ft)
                    self._last_gps = now
            except Exception as e:
                logger.debug(f"[NAV] GPS tick error: {e}")

        # INS propagation at every tick
        if self._ins:
            try:
                self._ins.update(dt)
            except Exception as e:
                logger.debug(f"[NAV] INS tick error: {e}")

        # Write fused position back into FMS
        if self._fusion and self._fms:
            try:
                lat, lon, alt_ft, heading = self._fusion.get_fused_position()
                ins_pos = self._ins.get_position() if self._ins else {}
                self._fms.update_navigation(
                    latitude=lat,
                    longitude=lon,
                    altitude=alt_ft,
                    heading=heading,
                    pitch=ins_pos.get('pitch', 0.0),
                    roll=ins_pos.get('roll', 0.0),
                )
            except Exception as e:
                logger.debug(f"[NAV] FMS navigation update error: {e}")

    def _update_loop(self):
        logger.info("[NAV] Navigation service loop started")
        self._lazy_init()
        last = time.time()
        while not self._running.is_set():
            now = time.time()
            dt  = now - last
            last = now
            try:
                self._tick(dt)
                # A pass completed, so the failure streak (if any) is over.
                self._failing_since = None
                self._last_error = ""
            except Exception as e:
                # Stamp the START of the streak, once. health_report()
                # reports UNHEALTHY when a streak outlives the grace
                # period, so that a worker kept alive by this very
                # handler cannot go on reporting HEALTHY while every
                # pass fails.
                if self._failing_since is None:
                    self._failing_since = time.monotonic()
                self._last_error = f"{type(e).__name__}: {e}"
                logger.error(f"[NAV] Loop error: {e}")
            time.sleep(0.1)   # 10 Hz INS update

    def start(self):
        # Guarded against the same TOCTOU race documented and fixed on the
        # newer subsystem singletons -- see PowerManagementSystem.start()
        # (Systems/powerManagement/elec/powerManagementSystem.py).
        with self._start_lock:
            if self._thread and self._thread.is_alive():
                return
            self._running.clear()
            self._thread = threading.Thread(target=self._update_loop, daemon=True, name="NAV_Service")
            self._thread.start()
            logger.info("[NAV] Navigation service started")

    def stop(self):
        self._running.set()
        if self._thread:
            self._thread.join(timeout=2)
        logger.info("[NAV] Navigation service stopped")

    def get_status(self):
        return {
            'running':      self._thread is not None and self._thread.is_alive(),
            'gps_available': self._gps is not None and self._gps.is_fix_valid(),
            'ins_available': self._ins is not None,
            'fusion_active': self._fusion is not None,
        }


    # ------------------------------------------------------------------ health
    def health_report(self):
        """Report this subsystem's health through the C2.1 contract.

        Story C2.2/C2.3: this subsystem exposed none of health_report(),
        check_health() or is_healthy(), so the sweep could only answer
        UNKNOWN for it -- "we never asked", indistinguishable from "we
        asked and it said yes" for any caller that treated UNKNOWN as
        benign. Measured before this change: 33 of 51 registered
        components, this one among them, reported UNKNOWN.

        The fact reported is the one this class already publishes from
        get_status(): whether its single daemon worker thread is alive.
        Nothing new is inferred about the aircraft.

        Note the polarity: self._running is a STOP event -- start()
        clears it and stop() sets it -- so a stop has been requested
        when it IS set. Reading the name as "is running" would invert
        this check.
        """
        from FMOFP.Utils.common.health import worker_health

        return worker_health("NAV", self._thread,
                             stop_requested=self._running.is_set(),
                             failing_since=self._failing_since,
                             last_error=self._last_error)


def get_nav_service() -> NavService:
    global _nav_service
    if _nav_service is None:
        _nav_service = NavService()
    return _nav_service
