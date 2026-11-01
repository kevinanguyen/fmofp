"""One command that runs the whole aircraft and shows what crosses the bus.

    py -m fmofp_demo [--seconds S] [--port-base N] [--build DIR] [--headless] [--no-displays]
                     [--no-events] [--hold name=value,...] [--seed N] [--log-dir DIR] [--settle S]

It starts, in this order: the plant (the simulated aircraft with its weather,
terrain and engines, RT 27), the ported partitions (flight control RT 5,
navigation RT 7, flight management RT 12, the radar computer RT 9, engine
management RT 3), the three display programs (the radar display RT 15, the PFD
RT 11 and the MFD RT 13, each in its own window), and finally the bus
controller, which drives the bus in real time. Every program writes its own
log under logs/demo/. The console shows one line a second with what is
crossing the bus, read from the controller's monitor port: altitude, airspeed
and heading from the FMS flight data, the ground collision avoidance state,
the engines' N1 and the fuel, the radars' modes and the terrain clearance, the
navigation fix, and which terminals are answering.

Unless --no-events is given, a short scripted flight runs: the radars are
switched on, the thrust lever moves, the autothrottle holds 500 kt, the flight
mode changes, one engine is throttled back, fuel is transferred. Each event is
announced as it happens. The radar display window's own buttons keep working
throughout: it is the radars' control panel.

Stop with Ctrl-C, or let --seconds run out. Closing a display window stops
only that display.
"""

import argparse
import os
import subprocess
import sys
import time
from collections import namedtuple

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import (EngineCommand, EngineSelect, FmsFlightMode, FuelTank, Ssm, TerminalState, TfrMode,
                                 WeatherMode)
from fmofp_bus.monitor import Monitor
from fmofp_bus.status import Status

SYNC_HZ = 50
SYNC_MODE_CODE = 17  # SYNCHRONIZE_WITH_DATA, broadcast by the controller once per minor frame
MSYS2_BIN = r"C:\msys64\ucrt64\bin"  # the MinGW runtime the Windows binaries need on PATH
B20SS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ANSWER_WINDOW_S = 2.5  # a terminal is "answering" if it answered a poll this recently (1 Hz polls included)

PLANT_RT = m.PlantSensors.RT
# (executable name, RT) of each ported partition.
PARTITIONS = (("fcs", m.FcsStatus.RT), ("nav", m.NavSolution.RT), ("fms", m.FmsStatus.RT),
              ("radar", m.RdrWxStatus.RT), ("engine", m.EngStatus.RT))
# (fmofp_display argument, RT) of each display program.
DISPLAYS = (("radar", m.DspRadarStatus.RT), ("pfd", m.DspPfdStatus.RT), ("mfd", m.DspMfdStatus.RT))
NAMES = {PLANT_RT: "plant", m.FcsStatus.RT: "FCS", m.NavSolution.RT: "nav", m.FmsStatus.RT: "FMS",
         m.RdrWxStatus.RT: "radar", m.EngStatus.RT: "engine", m.DspRadarStatus.RT: "radar display",
         m.DspPfdStatus.RT: "PFD", m.DspMfdStatus.RT: "MFD"}

Event = namedtuple("Event", "seconds what message")


def default_events():
    """The scripted flight. The cmd_seq values start at 201 so they never collide with the radar display
    window's own numbering, which starts at 1 (a command with the same cmd_seq as the last one is ignored)."""
    return [
        Event(2.0, "weather radar on: SURVEILLANCE at 200 km",
              m.RdrWxModeCmd(cmd_seq=201, requested_state=TerminalState.NORMAL,
                             requested_mode=WeatherMode.SURVEILLANCE, range_km=200.0, tilt=0.0)),
        Event(3.0, "terrain-following radar on, set clearance 1000 ft",
              m.RdrTfrModeCmd(cmd_seq=201, requested_state=TerminalState.NORMAL,
                              requested_mode=TfrMode.TERRAIN_FOLLOWING, set_clearance=1000.0)),
        Event(10.0, "FMS thrust lever to 80 %", m.FmsControlCmd(cmd_seq=201, thrust_present=True, thrust=0.8)),
        Event(20.0, "autothrottle engaged: hold 500 kt",
              m.EngCmd(cmd_seq=201, command=EngineCommand.AUTOTHROTTLE_ENGAGE, target_airspeed=500.0)),
        Event(30.0, "flight mode COMBAT", m.FmsFlightModeCmd(cmd_seq=201, flight_mode=FmsFlightMode.COMBAT)),
        Event(38.0, "autothrottle disengaged", m.EngCmd(cmd_seq=202, command=EngineCommand.AUTOTHROTTLE_DISENGAGE)),
        Event(40.0, "engine 2 alone to 40 %: asymmetric thrust",
              m.EngCmd(cmd_seq=203, command=EngineCommand.SET_THRUST, engine=EngineSelect.ENGINE_2, thrust=40.0)),
        Event(50.0, "1000 kg from the Center tank to Main Left (it takes what fits)",
              m.EngCmd(cmd_seq=204, command=EngineCommand.FUEL_TRANSFER, source=FuelTank.CENTER,
                       destination=FuelTank.MAIN_LEFT, amount=1000.0)),
        Event(60.0, "weather radar to MAPPING",
              m.RdrWxModeCmd(cmd_seq=202, requested_state=TerminalState.NORMAL,
                             requested_mode=WeatherMode.MAPPING, range_km=200.0, tilt=0.0)),
        Event(75.0, "both engines back to 70 %",
              m.EngCmd(cmd_seq=205, command=EngineCommand.SET_THRUST, engine=EngineSelect.ALL, thrust=70.0)),
    ]


def event_frame(event):
    return int(round(event.seconds * SYNC_HZ))


def script_lines(events):
    """The bus controller's --script file: 'frame rt sa hexwords' per event."""
    lines = ["# frame rt sa words (hex) - the scripted flight of fmofp_demo"]
    for event in events:
        status, words = event.message.encode()
        if status != Status.OK:
            raise ValueError(f"event {event.what!r} does not encode: {status.name}")
        lines.append(f"{event_frame(event)} {event.message.RT} {event.message.SUBADDRESS} "
                     + " ".join(f"{w:04X}" for w in words))
    return lines


def write_script(path, events):
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(script_lines(events)) + "\n")


# ---- the binaries -------------------------------------------------------------------------

def shown(path):
    """A path relative to B20SS when it is under it, else as given (relpath fails across Windows drives)."""
    try:
        relative = os.path.relpath(path, B20SS)
    except ValueError:
        return path
    return path if relative.startswith("..") else relative


def default_build_dir():
    preset = "windows-ucrt64" if sys.platform == "win32" else "linux-gcc"
    return os.path.join(B20SS, "ofp", "build", preset)


def binaries(build_dir):
    """name -> path of every C++ program the demo runs."""
    exe = ".exe" if sys.platform == "win32" else ""
    paths = {"bc": os.path.join(build_dir, "bc", "fmofp_bc" + exe)}
    for name, _ in PARTITIONS:
        paths[name] = os.path.join(build_dir, "partitions", f"fmofp_{name}{exe}")
    return paths


def missing_binaries(paths):
    return [path for path in paths.values() if not os.path.isfile(path)]


def source_newer_than_binaries(paths):
    """A C++ or ICD source file newer than the oldest binary (the build is stale), or None."""
    oldest = min(os.path.getmtime(path) for path in paths.values())
    newest, newest_path = oldest, None
    for root in (os.path.join(B20SS, "ofp"), os.path.join(B20SS, "icd")):
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in ("build", "tests") and not d.startswith(".")]
            for name in filenames:
                if name.endswith((".cpp", ".hpp", ".txt", ".cmake", ".xml", ".py")):
                    path = os.path.join(dirpath, name)
                    modified = os.path.getmtime(path)
                    if modified > newest:
                        newest, newest_path = modified, path
    return newest_path


def child_env(headless):
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    if sys.platform == "win32" and os.path.isdir(MSYS2_BIN):
        env["PATH"] = MSYS2_BIN + os.pathsep + env.get("PATH", "")
    if headless:
        env["QT_QPA_PLATFORM"] = "offscreen"
    return env


def spawn_kwargs():
    """Children in their own process group, so a Ctrl-C in this console reaches only the launcher, which then
    stops them in order."""
    if sys.platform == "win32":
        return {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
    return {"start_new_session": True}


class Programs:
    """The child processes, each with its own log file."""

    def __init__(self, log_dir, env):
        self.log_dir = log_dir
        self.env = env
        self.children = []  # (name, Popen, log file, log path)
        self.stopped = set()  # the names this launcher terminated itself

    def start(self, name, argv):
        path = os.path.join(self.log_dir, f"{name}.log")
        log = open(path, "w", encoding="utf-8")
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, cwd=B20SS, env=self.env,
                                **spawn_kwargs())
        self.children.append((name, proc, log, path))
        return proc

    def exited(self):
        """[(name, exit code)] of the children that are no longer running."""
        return [(name, proc.returncode) for name, proc, _, _ in self.children if proc.poll() is not None]

    def stop_all(self):
        for name, proc, _, _ in reversed(self.children):
            if proc.poll() is None:
                self.stopped.add(name)
                proc.terminate()
        for name, proc, log, _ in self.children:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
            log.close()

    def tail(self, name, lines=4):
        for child, _, _, path in self.children:
            if child == name:
                with open(path, encoding="utf-8", errors="replace") as f:
                    return [line.rstrip() for line in f.readlines()[-lines:]]
        return []


# ---- the dashboard ------------------------------------------------------------------------

class Dashboard:
    """What the monitor port shows, summarised one line at a time."""

    def __init__(self, terminals):
        self.terminals = list(terminals)
        self.frame = -1
        self.latest = {}       # message class -> the newest decoded message
        self.last_answer = {}  # rt -> monotonic time of its last answered poll
        self.last_poll = {}    # rt -> monotonic time of its last poll
        self.classes = {(info.rt, info.subaddress): info.cls for info in m.MESSAGES
                        if info.direction == "T" and info.rt != m.EVERY_TERMINAL}

    def take(self, record, now):
        cw = record.command
        if cw.is_mode_code:
            if cw.count_or_mode == SYNC_MODE_CODE:
                self.frame += 1
            return
        if not cw.transmit:
            return
        self.last_poll[cw.rt] = now
        if not record.answered:
            return
        self.last_answer[cw.rt] = now
        cls = m.TermStatus if cw.subaddress == m.TermStatus.SUBADDRESS else self.classes.get((cw.rt, cw.subaddress))
        if cls is None or len(record.received) != cls.WORDS:
            return
        status, message = cls.decode(record.received)
        if status == Status.OK:
            self.latest[(cls, cw.rt) if cls is m.TermStatus else cls] = message

    def answering(self, now):
        return [rt for rt in self.terminals if now - self.last_answer.get(rt, -1e9) <= ANSWER_WINDOW_S]

    def line(self, elapsed_s, now):
        parts = [f"t={elapsed_s:4.0f}s", f"frame {max(self.frame, 0):5d}"]
        fd = self.latest.get(m.FmsFlightData)
        if fd is not None:
            parts.append(f"alt {fd.altitude:5.0f} ft  ias {fd.airspeed:3.0f} kt  hdg {fd.heading:03.0f}  {fd.flight_mode.name}")
        gcas = self.latest.get(m.FcsGcasAlerts)
        if gcas is not None:
            codes = [code for bit, code in (("pullup", "PULLUP"), ("sink_rate", "SINK RATE"), ("terrain", "TERRAIN"),
                                            ("alt_low", "ALT LOW")) if getattr(gcas, bit)]
            parts.append("GCAS " + gcas.status.name + (" " + ",".join(codes) if codes else ""))
        engines = self.latest.get(m.EngEngineStatus)
        if engines is not None:
            text = f"N1 {engines.n1_1:3.0f}/{engines.n1_2:3.0f} %"
            status = self.latest.get(m.EngStatus)
            if status is not None and status.autothrottle_engaged:
                text += " AT"
            if status is not None and status.asymmetric_thrust:
                text += " ASYM"
            fuel = self.latest.get(m.EngFuelStatus)
            if fuel is not None:
                text += f"  fuel {fuel.total:5.0f} kg"
            parts.append(text)
        wx = self.latest.get(m.RdrWxStatus)
        if wx is not None:
            text = "WX " + (wx.mode.name if wx.state == TerminalState.NORMAL else wx.state.name)
            scan = self.latest.get(m.RdrWxScan)
            if scan is not None and scan.scan_id:
                text += f" scan {scan.scan_id}"
            parts.append(text)
        tfr = self.latest.get(m.RdrTfrGuidance)
        if tfr is not None:
            if tfr.clearance_ssm == Ssm.NORMAL:
                parts.append(f"TFR {tfr.mode.name} clr {tfr.min_clearance:5.0f} ft {tfr.level.name}")
            else:
                parts.append("TFR " + ("off" if tfr.mode == TfrMode.NONE else tfr.mode.name))
        nav = self.latest.get(m.NavSolution)
        if nav is not None and nav.position_ssm == Ssm.NORMAL:
            parts.append(f"nav {nav.latitude:+.4f} {nav.longitude:+.4f} ({nav.sats_used} sats)")
        answering = self.answering(now)
        text = f"{len(answering)}/{len(self.terminals)} answering"
        silent = [NAMES.get(rt, str(rt)) for rt in self.terminals if rt not in answering and rt in self.last_poll]
        if silent:
            text += " (silent: " + ", ".join(silent) + ")"
        parts.append(text)
        return " | ".join(parts)


# ---- the run ------------------------------------------------------------------------------

def banner(args, terminals, events, log_dir, paths):
    print(f"fmofp_demo: {len(terminals)} terminals on port base {args.port_base}, binaries from "
          f"{shown(os.path.dirname(paths['bc']))}, logs in {shown(log_dir)}")
    if not args.no_displays and not args.headless:
        print("  radar display window: weather picture left, terrain picture right; its WX and TFR buttons set the\n"
              "    radars' modes and the spinbox is the terrain-following set clearance.\n"
              "  PFD window: attitude, airspeed, altitude and heading from the FMS flight data.\n"
              "  MFD window: RADAR shows the radars' pictures, SYSTEMS the GCAS alerts, exceedances and engine\n"
              "    rows, MISSION the position.")
    if events:
        print("  scripted flight: " + "; ".join(f"{e.seconds:.0f} s {e.what}" for e in events))
    print("  stop with Ctrl-C" + (f"; runs for {args.seconds:.0f} s" if args.seconds > 0 else ""))
    print()


def run(monitor, bc, programs, terminals, events):
    """The dashboard loop until the controller exits (a timed run) or a core program dies. Returns the exit code."""
    dash = Dashboard(terminals)
    start = time.monotonic()
    next_line = start + 1.0
    fired = set()
    closed = set()
    while True:
        record = monitor.read(0.02)
        now = time.monotonic()
        if record is not None:
            dash.take(record, now)
        for index, event in enumerate(events):
            if index not in fired and dash.frame >= event_frame(event):
                fired.add(index)
                print(f"  >> t={event.seconds:.0f} s: {event.what}")
        if now >= next_line:
            print(dash.line(now - start, now), flush=True)
            next_line += 1.0
        if bc.poll() is not None:
            return 0 if bc.returncode == 0 else 1
        for name, code in programs.exited():
            if name == "bc" or name in closed:
                continue
            closed.add(name)
            if name.startswith("display_"):
                print(f"  (the {name[len('display_'):]} display window closed)")
            else:
                print(f"\n{name} exited with code {code}:")
                for line in programs.tail(name):
                    print("    " + line)
                return 1


def summary(programs, log_dir):
    print(f"\nlogs: {log_dir}")
    bc_log = os.path.join(log_dir, "bc.log")
    if os.path.isfile(bc_log):
        with open(bc_log, encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith(("bc: frames=", "  rt ", "bc: script")):
                    print("  " + line.rstrip())
    print("  " + ", ".join(f"{name} {'stopped' if name in programs.stopped else f'exit {proc.returncode}'}"
                           for name, proc, _, _ in programs.children))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=0.0, help="run this long (0 = until Ctrl-C)")
    parser.add_argument("--port-base", type=int, default=51000)
    parser.add_argument("--build", default=None, help="the C++ build directory (default: ofp/build/<preset>)")
    parser.add_argument("--headless", action="store_true", help="displays without windows (offscreen Qt)")
    parser.add_argument("--no-displays", action="store_true", help="only the plant, the partitions and the bus")
    parser.add_argument("--no-events", action="store_true", help="no scripted flight")
    parser.add_argument("--hold", default=None, help="plant quantities held constant, e.g. altitude=5000,speed=450")
    parser.add_argument("--seed", type=int, default=None, help="the plant's random seed")
    parser.add_argument("--log-dir", default=None, help="where the programs' logs go (default: logs/demo)")
    parser.add_argument("--settle", type=float, default=4.0, help="seconds to let the programs start before the bus")
    args = parser.parse_args(argv)

    paths = binaries(args.build or default_build_dir())
    missing = missing_binaries(paths)
    if missing:
        print("fmofp_demo: the C++ programs are not built:", file=sys.stderr)
        for path in missing:
            print("  " + path, file=sys.stderr)
        print("  build them first: see ofp/README.md (cmake --preset ... && cmake --build --preset ...)", file=sys.stderr)
        return 2
    stale = source_newer_than_binaries(paths)
    if stale is not None:
        print(f"fmofp_demo: note: {shown(stale)} is newer than the binaries; rebuild if the C++ or the ICD changed")
    log_dir = args.log_dir or os.path.join(B20SS, "logs", "demo")
    os.makedirs(log_dir, exist_ok=True)
    events = [] if args.no_events else default_events()
    if args.seconds > 0.0:
        events = [event for event in events if event.seconds < args.seconds]
    script = os.path.join(log_dir, "events.txt")
    if events:
        write_script(script, events)
    terminals = [PLANT_RT] + [rt for _, rt in PARTITIONS]
    if not args.no_displays:
        terminals += [rt for _, rt in DISPLAYS]

    monitor = Monitor(args.port_base)
    if monitor.open() != Status.OK:
        print(f"fmofp_demo: the monitor port {args.port_base + 33} is in use: is another bus running? "
              f"(try --port-base)", file=sys.stderr)
        return 2
    programs = Programs(log_dir, child_env(args.headless))
    banner(args, terminals, events, log_dir, paths)
    code = 1
    try:
        plant = [sys.executable, "-m", "fmofp_plant", "--port-base", str(args.port_base), "--quiet"]
        if args.seed is not None:
            plant += ["--seed", str(args.seed)]
        if args.hold:
            plant += ["--hold", args.hold]
        programs.start("plant", plant)
        for name, _ in PARTITIONS:
            programs.start(name, [paths[name], "--port-base", str(args.port_base), "--quiet"])
        if not args.no_displays:
            for name, _ in DISPLAYS:
                programs.start(f"display_{name}",
                               [sys.executable, "-m", "fmofp_display", name, "--port-base", str(args.port_base)])
        time.sleep(args.settle)
        dead = programs.exited()
        if dead:
            for name, exit_code in dead:
                print(f"{name} exited with code {exit_code} before the bus started:")
                for line in programs.tail(name):
                    print("    " + line)
            return 1
        bc = [paths["bc"]]
        for rt in terminals:
            bc += ["--rt", str(rt)]
        bc += ["--port-base", str(args.port_base), "--report", "--timeout-ms", "20"]
        if args.seconds > 0.0:
            bc += ["--frames", str(int(args.seconds * SYNC_HZ))]
        if events:
            bc += ["--script", script]
        code = run(monitor, programs.start("bc", bc), programs, terminals, events)
    except KeyboardInterrupt:
        print("\nstopping")
        code = 0
    finally:
        programs.stop_all()
        monitor.close()
        summary(programs, log_dir)
    return code


if __name__ == "__main__":
    sys.exit(main())
