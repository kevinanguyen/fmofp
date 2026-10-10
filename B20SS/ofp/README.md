# FMOFP OFP — the C++ flight software

The C++ port of FMOFP's flight logic. It talks to the Python displays and tools only
through the simulated MIL-STD-1553B bus defined in [`../icd/`](../icd/README.md).
Rules for the code are in [`CLAUDE.md`](CLAUDE.md).

## Layout

| Directory | Contents | Target |
|---|---|---|
| `bus/` | Bus word primitives: bit fields, two's complement, scaling, command and status words | `ofp::bus` |
| `icd/` | Builds the generated headers (`fmofp/icd/enums.hpp`, `messages.hpp`, `version.hpp`) from `../icd/fmofp_icd.xml`. Nothing here is checked in | `ofp::icd` |
| `tests/` | GoogleTest unit tests for the hand-written code | `ofp_tests` |
| `cmake/` | The compiler flags every target gets | `ofp::warnings`, `ofp::flight` |

Coming next: `osal/` (sockets and clocks for Windows and Linux), the RT library,
the bus controller, then one `partitions/<subsystem>/` per flight subsystem.

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
