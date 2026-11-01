"""The displays on the bus, end to end, headless.

The plant, the real fmofp_fcs, fmofp_nav, fmofp_fms, fmofp_radar and
fmofp_engine, and five DisplayTerminals in this process (the radar display,
RT 15, the PFD, RT 11, the MFD, RT 13, the EICAS, RT 14, and the TSD, RT 16) under the real fmofp_bc. The radar
display commands the weather, terrain-following and targeting radars from its own
transmit subaddresses, exactly as the window's buttons do; the bus controller
broadcasts the radars' status, scan summary, guidance and product blocks, the
flight computer's alerts and exceedances, the FMS's flight data and the engine
computer's engine and fuel status once each to address 31, and each display
accepts the subaddresses it draws (ICD v1.7, v1.8). The feeds then make of it
what the Qt windows would draw.

Needs FMOFP_BC, FMOFP_FCS, FMOFP_NAV, FMOFP_FMS, FMOFP_RADAR and FMOFP_ENGINE (CI sets them).
"""

import math
import os
import subprocess
import threading
import time
import unittest

from fmofp_bus.icd import blocks as b
from fmofp_bus.icd import messages as m
from fmofp_bus.command_word import BROADCAST_RT
from fmofp_bus.icd.enums import FmsFlightMode, GcasStatus, Ssm, TargetingMode, TerminalState, TfrMode, WeatherMode
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status
from fmofp_bus.tests._support import binary_env, needs
from fmofp_bus.tests.test_display_feeds import FakeCoordinator
from fmofp_display.app import (eicas_terminal, mfd_terminal, pfd_terminal, radar_terminal, targeting_command, tfr_command,
                               tsd_terminal, weather_command)
from fmofp_display.flight_feed import FlightDataFeed
from fmofp_display.radar_feed import RadarFeed
from fmofp_display.systems_feed import SystemsFeed
from fmofp_plant.plant import PlantModel, PlantTerminal

PORT_BASE = 61000
FRAMES = 260
SYNC_MODE_CODE = 17
BINARIES = ("FMOFP_FCS", "FMOFP_NAV", "FMOFP_FMS", "FMOFP_RADAR", "FMOFP_ENGINE")


class PlantThread(threading.Thread):
    def __init__(self, plant):
        super().__init__(daemon=True)
        self.plant = plant
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            self.plant.poll(0.002)


@needs("FMOFP_BC", *BINARIES)
class DisplaysEndToEndTest(unittest.TestCase):
    processes = []
    plant_thread = None
    plant = None
    monitor = None
    radar_display = None
    pfd = None
    mfd = None
    eicas = None
    tsd = None

    @classmethod
    def setUpClass(cls):
        try:
            cls.model = PlantModel(seed=5)
            cls.model.hold = dict(speed=450.0, heading=0.0, altitude=5000.0, vertical_speed=0.0)
            cls.plant = PlantTerminal(cls.model, PORT_BASE)
            assert cls.plant.init() == Status.OK, "plant port in use?"
            cls.plant_thread = PlantThread(cls.plant)
            cls.plant_thread.start()
            cls.monitor = Monitor(PORT_BASE)
            assert cls.monitor.open() == Status.OK, "monitor port in use?"
            cls.radar_display = radar_terminal(PORT_BASE)
            assert cls.radar_display.start() == Status.OK, "radar display port in use?"
            cls.pfd = pfd_terminal(PORT_BASE)
            assert cls.pfd.start() == Status.OK, "PFD port in use?"
            cls.mfd = mfd_terminal(PORT_BASE)
            assert cls.mfd.start() == Status.OK, "MFD port in use?"
            cls.eicas = eicas_terminal(PORT_BASE)
            assert cls.eicas.start() == Status.OK, "EICAS port in use?"
            cls.tsd = tsd_terminal(PORT_BASE)
            assert cls.tsd.start() == Status.OK, "TSD port in use?"
            for binary in BINARIES:
                cls.processes.append(subprocess.Popen(
                    [os.environ[binary], "--port-base", str(PORT_BASE), "--quiet"], stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, env=binary_env()))
            time.sleep(0.3)
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        for p in cls.processes:
            p.kill()
            p.wait(timeout=10)
            p.stdout.close()
        cls.processes = []
        for terminal in (cls.radar_display, cls.pfd, cls.mfd, cls.eicas, cls.tsd):
            if terminal is not None:
                terminal.stop()
        if cls.plant_thread is not None:
            cls.plant_thread.stop.set()
            cls.plant_thread.join(timeout=2)
        if cls.plant is not None:
            cls.plant.shutdown()
        if cls.monitor is not None:
            cls.monitor.close()

    def run_bus(self):
        terminals = ["27", str(m.FcsStatus.RT), str(m.NavSolution.RT), str(m.FmsStatus.RT), str(m.RdrWxStatus.RT),
                     str(m.EngStatus.RT), str(m.DspPfdStatus.RT), str(m.DspRadarStatus.RT), str(m.DspMfdStatus.RT),
                     str(m.DspEicasStatus.RT), str(m.DspTsdStatus.RT)]
        args = [os.environ["FMOFP_BC"]]
        for rt in terminals:
            args += ["--rt", rt]
        args += ["--port-base", str(PORT_BASE), "--frames", str(FRAMES), "--free-run", "--strict", "--timeout-ms", "20"]
        bc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=binary_env())
        records = self.monitor.read_until(lambda: bc.poll() is not None)
        output = bc.stdout.read()
        bc.stdout.close()
        self.assertEqual(0, bc.wait(timeout=60), output)
        return records, output

    @staticmethod
    def by_frame(records):
        frame = -1
        out = []
        for record in records:
            cw = record.command
            if cw.is_mode_code and cw.count_or_mode == SYNC_MODE_CODE:
                frame += 1
                continue
            out.append((frame, record))
        return out

    def transmitted(self, framed, cls, first, last, rt=None):
        rt = cls.RT if rt is None else rt
        out = []
        for frame, record in framed:
            cw = record.command
            if (first <= frame < last and cw.transmit and not cw.is_mode_code and cw.rt == rt
                    and cw.subaddress == cls.SUBADDRESS and record.answered):
                status, msg = cls.decode(record.received)
                self.assertEqual(Status.OK, status)
                out.append((frame, msg))
        return out

    def test_the_displays_see_the_radars_and_the_fms_and_command_the_radars(self):
        self.monitor.drain()
        # The control panel: both radars on, before the bus starts.
        self.radar_display.send(weather_command(self.radar_display, TerminalState.NORMAL, WeatherMode.SURVEILLANCE))
        self.radar_display.send(tfr_command(self.radar_display, TerminalState.NORMAL, TfrMode.TERRAIN_FOLLOWING, 984))
        self.radar_display.send(targeting_command(self.radar_display, TerminalState.NORMAL, TargetingMode.TARGET_SEARCH))
        records, output = self.run_bus()
        framed = self.by_frame(records)
        self.assertGreaterEqual(framed[-1][0], FRAMES - 2)

        # The commands reached the radars: their status says so, relayed back to the display.
        wx_status = self.radar_display.fresh(m.BcastRdrWxStatus, stale_after_s=60.0)
        self.assertIsNotNone(wx_status)
        self.assertEqual((TerminalState.NORMAL, WeatherMode.SURVEILLANCE, 1, False),
                         (wx_status.payload.state, wx_status.payload.mode, wx_status.payload.last_cmd_seq,
                          wx_status.payload.cmd_rejected))
        tfr_status = self.radar_display.fresh(m.BcastRdrTfrStatus, stale_after_s=60.0)
        self.assertEqual((TerminalState.NORMAL, TfrMode.TERRAIN_FOLLOWING, 2),
                         (tfr_status.payload.state, tfr_status.payload.mode, tfr_status.payload.last_cmd_seq))

        # The weather products block arrived whole and is the picture of a surveillance scan.
        products, version, age = self.radar_display.latest_block(b.BcastRdrWxProducts)
        self.assertIsNotNone(products, "no complete weather products block at the display")
        self.assertEqual(WeatherMode.SURVEILLANCE, products.mode)
        self.assertGreater(products.scan_id, 0)
        self.assertGreater(len(products.vil), 0)
        scan = self.radar_display.fresh(m.BcastRdrWxScan, stale_after_s=60.0)
        self.assertEqual(200.0, scan.payload.range_km)
        elevation, _, _ = self.radar_display.latest_block(b.BcastRdrTfrElevation)
        self.assertIsNotNone(elevation, "no complete terrain block at the display")
        self.assertEqual(TfrMode.TERRAIN_FOLLOWING, elevation.mode)
        self.assertEqual(100, len(elevation.profile))
        guidance = self.radar_display.fresh(m.BcastRdrTfrGuidance, stale_after_s=60.0)
        self.assertEqual(Ssm.NORMAL, guidance.payload.clearance_ssm)
        self.assertGreaterEqual(self.radar_display.blocks_completed, 4)

        # What the window would draw, from the same terminal.
        coordinator = FakeCoordinator()
        feed = RadarFeed(self.radar_display, coordinator)
        feed.update()
        self.assertEqual("SURVEILLANCE", feed.weather_draw_mode)
        self.assertAlmostEqual(200.0 / 1.852, feed.weather_range_nm)
        self.assertGreater(len(coordinator.stores.get("vil", [])), 0)
        self.assertGreater(len(coordinator.stores.get("cells", [])), 0)
        for item in coordinator.stores["vil"]:
            self.assertLessEqual(abs(item["position"][0]), feed.weather_range_nm + 1e-6)
        terrain = feed.tfr_data()
        self.assertEqual("TRACK", terrain["mode_name"])
        self.assertEqual(100, len(terrain["tfr_data"]))
        self.assertIsNotNone(terrain["terrain_clearance"])
        self.assertIsNotNone(terrain["current_altitude"])
        self.assertEqual("NORMAL", terrain["system_health"])

        # The PFD: the FMS's flight data, status and attitude status, relayed.
        flight = FlightDataFeed(self.pfd, stale_after_s=60.0)
        data = flight.get_flight_data()
        self.assertEqual({"attitude", "velocity", "navigation", "tactical", "status"}, set(data))
        self.assertAlmostEqual(5000.0, data["navigation"]["altitude"], delta=1.0)
        self.assertAlmostEqual(450.0, data["velocity"]["airspeed"], delta=0.2)
        self.assertEqual("NORMAL", data["status"]["mode"])
        tactical = flight.get_tactical_status()
        self.assertEqual("NORMAL", tactical["mode"])
        self.assertEqual({"max_g_force": 2.5, "max_aoa": 15.0, "engine_power": 0.7}, tactical["profile_limits"])
        self.assertIsNotNone(self.pfd.fresh(m.BcastFmsAttitudeStatus, stale_after_s=60.0))
        self.assertEqual(FmsFlightMode.NORMAL, self.pfd.fresh(m.BcastFmsStatus, stale_after_s=60.0).payload.flight_mode)

        # The displays answer the bus as terminals: their status messages and TERM_STATUS.
        radar_status = self.transmitted(framed, m.DspRadarStatus, 100, FRAMES)
        self.assertTrue(radar_status)
        self.assertEqual(TerminalState.NORMAL, radar_status[-1][1].state)
        pfd_status = self.transmitted(framed, m.DspPfdStatus, 100, FRAMES)
        self.assertEqual(TerminalState.NORMAL, pfd_status[-1][1].state)
        term = self.transmitted(framed, m.TermStatus, 100, FRAMES, rt=m.DspRadarStatus.RT)[-1][1]
        self.assertEqual((m.ICD_MAJOR, m.ICD_MINOR, TerminalState.NORMAL), (term.icd_major, term.icd_minor, term.state))
        # Display data goes out once, by broadcast (ICD v1.7): the broadcast map's subaddresses, and no unicast
        # copy written into any display.
        broadcast = {r.command.subaddress for _, r in framed
                     if not r.command.transmit and not r.command.is_mode_code and r.command.rt == BROADCAST_RT}
        self.assertEqual({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28}, broadcast)
        for rt in (m.DspRadarStatus.RT, m.DspPfdStatus.RT, m.DspMfdStatus.RT, m.DspEicasStatus.RT, m.DspTsdStatus.RT):
            unicast = {r.command.subaddress for _, r in framed
                       if not r.command.transmit and not r.command.is_mode_code and r.command.rt == rt}
            self.assertEqual(set(), unicast, f"RT {rt}")
        # A terminal that did not accept a subaddress ignored the broadcasts on it: the PFD heard none of the radar's.
        self.assertIsNone(self.pfd.fresh(m.BcastRdrWxStatus, stale_after_s=60.0))
        self.assertEqual(0, self.pfd.blocks_completed)

        # The MFD: the radars' pictures from its own copies, the flight computer by relay, the position from the FMS.
        mfd_feed = RadarFeed(self.mfd, FakeCoordinator())
        mfd_feed.update()
        self.assertEqual("SURVEILLANCE", mfd_feed.weather_draw_mode)
        self.assertEqual(1, mfd_feed.blocks_applied, "no complete weather products block at the MFD")
        self.assertEqual("TRACK", mfd_feed.tfr_data()["mode_name"])
        self.assertEqual(100, len(mfd_feed.tfr_data()["tfr_data"]))
        systems = SystemsFeed(self.mfd, stale_after_s=60.0)
        gcas = self.mfd.fresh(m.BcastFcsGcasAlerts, stale_after_s=60.0).payload
        alerts = systems.get_alerts()
        bits = (("pullup", "PULLUP"), ("sink_rate", "SINK RATE"), ("terrain", "TERRAIN"), ("alt_low", "ALT LOW"))
        self.assertEqual([code for bit, code in bits if getattr(gcas, bit)], [a["code"] for a in alerts])
        self.assertEqual(gcas.status != GcasStatus.ARMED, bool(alerts))
        # Level at 5000 ft over hills that reach 1500 m: the radar's clearance ahead is under 500 ft.
        self.assertIn("TERRAIN", [a["code"] for a in alerts])
        self.assertTrue(alerts[-1]["message"].startswith("TERRAIN  "), alerts)
        perf = self.mfd.fresh(m.BcastFcsPerfStatus, stale_after_s=60.0)
        self.assertIsNotNone(perf)
        self.assertEqual([], systems.get_exceedances())
        # The engine computer: its engines at the FMS's lever (50 %), their fuel, and the fuel efficiency the
        # flight computer makes of the broadcast engine status.
        engine = systems.engine.get_data()
        self.assertAlmostEqual(50.0, engine["thrust_pct"], delta=0.01, msg="the FMS lever starts at 50")
        self.assertAlmostEqual(58.0, engine["n1_pct"], delta=0.6)
        self.assertAlmostEqual(2100.0, engine["ff_kgh"], delta=25.0)
        fuel = systems.fuel_data()
        self.assertEqual(([], [], False), (fuel["low"], fuel["critical"], fuel["leak_suspected"]))
        self.assertLess(fuel["total"], 40000.0)
        self.assertGreater(fuel["total"], 39900.0)
        self.assertEqual(Ssm.NORMAL, perf.payload.fuel_ssm, "the FCS heard the engine status broadcast")
        self.assertAlmostEqual(450.0 / (4200.0 / 60.0), perf.payload.fuel_efficiency, delta=0.1)
        self.assertIn("ENG N1", systems.status_text())
        position = systems.mission_data()["current_position"]
        self.assertAlmostEqual(5000.0, position["alt_ft"], delta=1.0)
        self.assertLess(abs(position["lat"]) + abs(position["lon"]), 0.1, "the plant flies from 0 N 0 E")
        self.assertGreaterEqual(self.mfd.blocks_completed, 4)
        mfd_status = self.transmitted(framed, m.DspMfdStatus, 100, FRAMES)
        self.assertEqual(TerminalState.NORMAL, mfd_status[-1][1].state)

        # The EICAS: the engine computer's readings and tanks, the flight computer's alerts, the FMS's power
        # setting and the weather scan's severities, all by broadcast; it ignores the subaddresses it did not accept.
        eicas = SystemsFeed(self.eicas, stale_after_s=60.0)
        readings = eicas.engine_readings()
        self.assertEqual({"oil_psi", "oil_temp_c", "vib", "total_kg", "balance_kg"}, set(readings))
        self.assertAlmostEqual(fuel["total"], readings["total_kg"], delta=20.0)
        self.assertAlmostEqual(0.0, readings["balance_kg"], delta=20.0, msg="both mains burn alike")
        self.assertEqual(0.7, eicas.fms_control.get_tactical_status()["profile_limits"]["engine_power"])
        self.assertEqual(("NONE", "NONE", False), eicas.weather_severity(), "a surveillance scan finds no shear")
        self.assertIsNotNone(self.eicas.fresh(m.BcastRdrWxScan, stale_after_s=60.0))
        self.assertIsNone(self.eicas.fresh(m.BcastRdrWxStatus, stale_after_s=60.0), "not among its receives: ignored")
        self.assertGreaterEqual(self.eicas.blocks_completed, 1, "the fused tracks block, which it accepts")
        self.assertEqual(len(eicas.threat_tracks()), eicas.fused_summary()[1], "the hostile count the EICAS alarms on")
        self.assertEqual([a["code"] for a in alerts], [a["code"] for a in eicas.get_alerts()])
        eicas_status = self.transmitted(framed, m.DspEicasStatus, 100, FRAMES)
        self.assertEqual(TerminalState.NORMAL, eicas_status[-1][1].state)
        # The targeting radar, commanded from the radar display: its status and tracks reach the MFD's targeting page,
        # the fusion's tracks reach the EICAS and the TSD, and the TSD hears the FMS's energy metrics and position.
        tgt_status = self.radar_display.fresh(m.BcastRdrTgtStatus, stale_after_s=60.0)
        self.assertIsNotNone(tgt_status)
        self.assertEqual((TerminalState.NORMAL, TargetingMode.TARGET_SEARCH, 3, False),
                         (tgt_status.payload.state, tgt_status.payload.mode, tgt_status.payload.last_cmd_seq,
                          tgt_status.payload.cmd_rejected))
        self.assertGreaterEqual(tgt_status.payload.track_count, 1)
        targeting = mfd_feed.targeting_data()
        self.assertEqual("TARGET_SEARCH", targeting["mode_name"])
        self.assertGreaterEqual(len(targeting["targets"]), 1, "no tracks block at the MFD")
        for target in targeting["targets"]:
            self.assertLessEqual(math.hypot(target["position"][0], target["position"][1]), targeting["range_nm"] + 1e-6)
            self.assertIn(target["classification"], ("FIXED_WING", "ROTARY_WING", None))
        self.assertIn("TGT NORMAL TARGET_SEARCH", mfd_feed.targeting_status_text())
        tsd = SystemsFeed(self.tsd, stale_after_s=60.0)
        fused = tsd.fusion.get_fused_tracks()
        self.assertGreaterEqual(len(fused), 1, "no fused tracks block at the TSD")
        threats = [track.to_tsd_dict() for track in fused]
        self.assertTrue(all(0.0 <= t["bearing"] < 360.0 and 0.0 < t["range_nm"] <= 60.0 for t in threats), threats)
        self.assertEqual(tsd.fused_summary()[1], len([t for t in threats if t["hostile"]]))
        tactical = tsd.fms_control.get_tactical_status()
        self.assertIn("energy_metrics", tactical, "BCAST_FMS_ENERGY reached the TSD")
        self.assertEqual({"countermeasures", "targeting", "weapons", "stealth_mode"}, set(tactical["tactical_systems"]))
        self.assertIn("latitude", tsd.fms.get_flight_data()["navigation"])
        self.assertIsNone(self.tsd.fresh(m.BcastRdrWxScan, stale_after_s=60.0), "not among its receives: ignored")
        self.assertEqual(({"route": []}, []), (tsd.mission.get_nav_data(), tsd.defensive.get_rwr_contacts()))
        tsd_status = self.transmitted(framed, m.DspTsdStatus, 100, FRAMES)
        self.assertEqual(TerminalState.NORMAL, tsd_status[-1][1].state)
        self.assertIn("overruns=0", output)
        self.assertGreaterEqual(self.radar_display.syncs, FRAMES - 2)


if __name__ == "__main__":
    unittest.main()
