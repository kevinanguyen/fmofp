# FMOFP OFP — the C++ flight software

The C++ port of FMOFP's flight logic. It talks to the Python displays and tools only
through the simulated MIL-STD-1553B bus defined in [`../icd/`](../icd/README.md).
Rules for the code are in [`CLAUDE.md`](CLAUDE.md).

## Layout

| Directory | Contents | Target |
|---|---|---|
| `bus/` | Bus word primitives (bit fields, two's complement, scaling), command and status words, the datagram framing, `RemoteTerminal`: what every terminal owes the bus, and the block transfer (descriptor and segments) for payloads over 32 words, with `BlockPublisher` and `BlockCollector` to put a block on a terminal's subaddresses | `ofp::bus` |
| `osal/` | The only code that talks to the OS: monotonic clock with absolute-deadline sleep, UDP on 127.0.0.1. One implementation each for Windows and POSIX | `ofp::osal` |
| `icd/` | Builds the generated headers (`fmofp/icd/enums.hpp`, `messages.hpp`, `blocks.hpp`, `version.hpp`) from `../icd/fmofp_icd.xml`. Nothing here is checked in | `ofp::icd` |
| `bc/` | The bus controller: the 20 ms schedule, the frame loop with sync and timing, the transaction engine, and the `fmofp_bc` process, which can also send commands to terminals at given frames from a script | `ofp::bc`, `fmofp_bc` |
| `partitions/` | `common/`: the socket loop every partition runs around its terminal. `rt_stub/`: a stand-in terminal that answers `TERM_STATUS`. `fcs/`: flight control, the first ported partition (GCAS alerts, envelope monitoring, mode and surface commands). `nav/`: navigation (the GPS position solver, the GPS / dead-reckoning fusion), plus `fmofp_nav_tool`, which puts its arithmetic on stdin/stdout for the Python oracle tests. `fms/`: flight management (flight modes and their envelopes, attitude commands and maneuvers, autopilot targets, thrust and gear, the flight data the displays read, energy maneuverability). `radar/`: the radar computer with the weather radar and the terrain-following radar ported: the sensor model that renders the plant's weather, the signal processing (VIL, rain rate, storm cells, precipitation areas, wind shear, turbulence), the 1.2 s scan and the products block; the clearance assessment, path optimiser and terrain warnings over the plant's terrain profile, the guidance and the elevation block; plus `fmofp_radar_tool` for the oracle tests. Each is a pure library plus a process | `ofp::rt_process`, `fmofp_rt_stub`, `ofp::fcs`, `fmofp_fcs`, `ofp::nav`, `fmofp_nav`, `fmofp_nav_tool`, `ofp::fms`, `fmofp_fms`, `ofp::radar`, `fmofp_radar`, `fmofp_radar_tool` |
| `tests/` | GoogleTest unit tests for the hand-written code, including the Phase 1 gate run in-process | `ofp_tests` |
| `cmake/` | The compiler flags every target gets | `ofp::warnings`, `ofp::flight` |

## Running the bus

The Phase 1 gate: a bus controller and two terminals in separate processes,
exchanging `TERM_STATUS` in lockstep for 150 frames (3 s) with no overruns.
From `build/<preset>/`, in three shells (or background the first two):

```bash
partitions/fmofp_rt_stub --rt 1 --port-base 53000
partitions/fmofp_rt_stub --rt 2 --port-base 53000
bc/fmofp_bc --rt 1 --rt 2 --port-base 53000 --frames 150 --strict
```

`--strict` makes `fmofp_bc` exit 1 if any terminal failed to answer or any frame
overran. Leave out `--frames` to run until killed; add `--report` for a line per
major frame and `--free-run` to go as fast as the terminals answer. Default ports
follow the ICD (base 51000: terminal *n* at 51000+*n*, the controller at 51032,
the monitor at 51033); a different `--port-base` lets two buses share a machine.

## Running the partitions

Flight control, navigation, flight management, the weather radar and the
terrain-following radar on the bus, fed by the Python plant model (the
simulated aircraft, its weather and its terrain, RT 27). From `B20SS/`:

```bash
py -m fmofp_plant --port-base 53000                 # the aircraft; add --hold altitude=150 to test ALT LOW
ofp/build/<preset>/partitions/fmofp_fcs --port-base 53000
ofp/build/<preset>/partitions/fmofp_nav --port-base 53000
ofp/build/<preset>/partitions/fmofp_fms --port-base 53000
ofp/build/<preset>/partitions/fmofp_radar --port-base 53000
ofp/build/<preset>/bc/fmofp_bc --rt 27 --rt 5 --rt 7 --rt 12 --rt 9 --port-base 53000 --report
```

The controller polls the plant's `PLANT_SENSORS` 50 times a second and relays it
to every consumer that is present (`FCS_SENSORS`, `FMS_SENSORS`,
`NAV_SENSORS`, `RDR_TFR_SENSORS`), polls the plant's GPS pseudoranges and its
weather once a second and relays them (`NAV_GPS`, `RDR_WX_WEATHER`), polls the
plant's terrain block every frame and relays its descriptor and segments to the
radar (`RDR_TFR_TERRAIN`, the first block relay), relays `FCS_ATTITUDE` and
`NAV_SOLUTION` to the FMS and the radar's `RDR_TFR_GUIDANCE` to the FCS
(`FCS_TERRAIN`, where the ground collision avoidance TERRAIN alert now gets its
clearance), and collects everything the partitions publish, including the
radar's `RDR_WX_PRODUCTS` and `RDR_TFR_ELEVATION` blocks a segment per frame.
Anything listening on the monitor port sees every transaction
(`fmofp_bus.monitor` reads it from Python; `fmofp_bus.block.blocks_from_records`
reassembles the blocks). The radars do nothing until they are told a mode: a
`RDR_WX_MODE_CMD` (RT 9, SA 1) for NORMAL and SURVEILLANCE starts the weather
radar scanning, and `RDR_WX_SCAN` then reports what each 1.2 s sweep found; a
`RDR_TFR_MODE_CMD` (RT 9, SA 2) for NORMAL and TERRAIN_FOLLOWING with a set
clearance starts the terrain-following radar assessing each profile the plant
publishes, and `RDR_TFR_GUIDANCE` reports the clearance and the first altitude
command ten times a second.

To command a partition, give the controller a script: one command per line,
`frame rt sa word word ...` with the data words in hex, sent in that minor
frame. For example, a `FMS_MANEUVER_CMD` (RT 12, SA 18) for BREAK_RIGHT with
`cmd_seq` 1 in frame 10, and a `FMS_FLIGHT_MODE_CMD` (SA 20) for COMBAT in
frame 40:

```text
# frame rt sa words
10 12 18 0001 0001 0000
40 12 20 1001
```

```bash
ofp/build/<preset>/bc/fmofp_bc --rt 27 --rt 5 --rt 7 --rt 12 --port-base 53000 --script commands.txt
```

The Python message classes encode the words (`FmsManeuverCmd(...).encode()`),
which is how the tests write their scripts. `test_fcs_e2e.py`, `test_nav_e2e.py`,
`test_fms_e2e.py`, `test_radar_e2e.py` and `test_tfr_e2e.py` run exactly these
processes: the first checks the FCS's alerts against the original Python GCAS
on the same numbers, with the real radar measuring the clearance over flat
terrain, the second that navigation lands on the plant and moves with it, the
third that the FMS answers a scripted sequence of commands as the original
`fmsControl` does, the fourth that the weather radar scans, answers mode
commands and publishes its block on cadence, the fifth that the plant's terrain
reaches the radar through the block relay, that the guidance and the elevation
block are the assessment of that terrain, and that the FCS's TERRAIN alert
follows the radar. `test_nav_oracle.py`, `test_weather_oracle.py` and
`test_tfr_oracle.py` check the arithmetic against the original Python through
`fmofp_nav_tool` and `fmofp_radar_tool`: the rendered volume, the VIL and
rain-rate fields, the point selection, the storm cell detection, the
precipitation areas, the wind shear events and the turbulence cells; the
clearance levels and their latch, the path commands and the terrain warnings.

## Building on Windows

Install MSYS2 (`winget install MSYS2.MSYS2`), open the **UCRT64** shell
(`C:\msys64\ucrt64.exe`), and install the toolchain once:

```bash
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake \
    mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-gtest \
    mingw-w64-ucrt-x86_64-cppcheck mingw-w64-ucrt-x86_64-gcovr
```

Then, in this directory:

```bash
cmake --preset windows-ucrt64
cmake --build --preset windows-ucrt64
ctest --preset windows-ucrt64
```

The ICD tool runs under the MSYS2 Python that comes with those packages; it needs
nothing outside the standard library.

## Building on Linux

```bash
sudo apt-get install g++ cmake ninja-build libgtest-dev cppcheck gcovr python3
cmake --preset linux-gcc
cmake --build --preset linux-gcc
ctest --preset linux-gcc
```

`linux-asan` is the same build with AddressSanitizer and UndefinedBehaviorSanitizer;
CI runs both. MinGW GCC has no sanitizers, so on Windows use `linux-asan` under WSL
if you need it locally.

## Other targets

```bash
cmake --build build/<preset> --target cppcheck     # static analysis, fails on any finding
cmake --preset coverage && cmake --build --preset coverage   # tests + gcovr summary
```

## Generated code

`cmake --build` regenerates the ICD headers whenever `fmofp_icd.xml` or
`icd_tool.py` changes. To read them, look in `build/<preset>/icd/generated/include/`,
or produce them anywhere with:

```bash
py ../icd/tools/icd_tool.py gen-cpp --out some/dir
```
