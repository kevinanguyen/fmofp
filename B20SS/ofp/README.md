# FMOFP OFP — the C++ flight software

The C++ port of FMOFP's flight logic. It talks to the Python displays and tools only
through the simulated MIL-STD-1553B bus defined in [`../icd/`](../icd/README.md).
Rules for the code are in [`CLAUDE.md`](CLAUDE.md).

## Layout

| Directory | Contents | Target |
|---|---|---|
| `bus/` | Bus word primitives (bit fields, two's complement, scaling), command and status words, the datagram framing, and `RemoteTerminal`: what every terminal owes the bus | `ofp::bus` |
| `osal/` | The only code that talks to the OS: monotonic clock with absolute-deadline sleep, UDP on 127.0.0.1. One implementation each for Windows and POSIX | `ofp::osal` |
| `icd/` | Builds the generated headers (`fmofp/icd/enums.hpp`, `messages.hpp`, `version.hpp`) from `../icd/fmofp_icd.xml`. Nothing here is checked in | `ofp::icd` |
| `bc/` | The bus controller: the 20 ms schedule, the frame loop with sync and timing, the transaction engine, and the `fmofp_bc` process | `ofp::bc`, `fmofp_bc` |
| `partitions/` | `common/`: the socket loop every partition runs around its terminal. `rt_stub/`: a stand-in terminal that answers `TERM_STATUS`. Real subsystems go here as they are ported | `ofp::rt_process`, `fmofp_rt_stub` |
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
