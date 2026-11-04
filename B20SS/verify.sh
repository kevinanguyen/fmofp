#!/usr/bin/env bash
# verify.sh - the gate every change passes before it is pushed to port/cpp.
#
#   bash B20SS/verify.sh            # everything: icd cpp py load
#   bash B20SS/verify.sh icd py     # a subset, in this order
#
# Steps
#   icd   icd_tool.py check, then regenerate fmofp_bus/icd and ICD_TABLES.md and
#         say whether that changed anything (CI fails when generated files are stale,
#         so commit what this regenerates)
#   cpp   configure if needed, build under -Werror, ctest, the cppcheck target
#   py    the full Python suite in fmofp_bus/tests against the binaries just built
#         (unit tests, the oracle tests against the frozen Python, the end-to-end
#         runs over the real bus, the display feeds/render/e2e, the headless demo)
#   load  the bus controller's schedule for the full fourteen-terminal aircraft; the
#         worst minor frame must stay under the 10000 us budget
#
# Works from Git Bash or an MSYS2 UCRT64 shell on Windows (MSYS2 at C:\msys64) and
# from any shell on Linux. PRESET=... overrides the CMake preset (windows-ucrt64 on
# Windows, linux-gcc elsewhere; linux-asan is what CI runs in addition).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../B20SS
OFP="$HERE/ofp"
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) WIN=1;; *) WIN=0;; esac
if command -v py >/dev/null 2>&1; then PY=py; else PY=python3; fi
if [ "$WIN" = 1 ]; then PRESET="${PRESET:-windows-ucrt64}"; else PRESET="${PRESET:-linux-gcc}"; fi
BUILD="$OFP/build/$PRESET"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8 QT_QPA_PLATFORM=offscreen
[ "$WIN" = 1 ] && [ -d /c/msys64/ucrt64/bin ] && export PATH="/c/msys64/ucrt64/bin:$PATH"

steps="$*"
[ -z "$steps" ] && steps="all"
case " $steps " in *" all "*) steps="icd cpp py load";; esac

say()  { printf '\n=== %s ===\n' "$1"; }
fail() { printf '\nverify: FAILED at %s\n' "$1"; exit 1; }

# Run a command line with the C++ toolchain available. On Windows outside an MSYS2
# shell that means the UCRT64 login shell (cmake, ninja, g++ and the DLLs live there).
tool() {
    if [ "$WIN" = 1 ] && [ "${MSYSTEM:-}" != "UCRT64" ]; then
        MSYSTEM=UCRT64 /c/msys64/usr/bin/bash.exe -lc "$1"
    else
        bash -c "$1"
    fi
}

step_icd() {
    say "icd check"
    (cd "$HERE" && $PY icd/tools/icd_tool.py check) || fail icd
    (cd "$HERE" && $PY icd/tools/icd_tool.py gen-py >/dev/null && $PY icd/tools/icd_tool.py render >/dev/null) || fail "icd generators"
    changed="$(cd "$HERE" && git status --short -- icd/ICD_TABLES.md fmofp_bus/icd)"
    if [ -n "$changed" ]; then
        printf 'generated files changed when regenerated - commit them with your change:\n%s\n' "$changed"
    else
        echo "generated Python and ICD_TABLES.md are current"
    fi
}

step_cpp() {
    if [ ! -d "$BUILD" ]; then
        say "cmake configure ($PRESET)"
        tool "cd \"$OFP\" && cmake --preset $PRESET" || fail configure
    fi
    say "build ($PRESET)"
    tool "cd \"$OFP\" && cmake --build --preset $PRESET 2>&1 | grep -v '^\[' | tail -n 30; exit \${PIPESTATUS[0]}" || fail build
    say "ctest"
    tool "cd \"$OFP\" && ctest --preset $PRESET 2>&1 | tail -n 6; exit \${PIPESTATUS[0]}" || fail ctest
    say "cppcheck"
    tool "cd \"$OFP\" && cmake --build build/$PRESET --target cppcheck 2>&1 | tail -n 6; exit \${PIPESTATUS[0]}" || fail cppcheck
}

export_binaries() {
    exe=""
    [ -f "$BUILD/bc/fmofp_bc.exe" ] && exe=".exe"
    [ -f "$BUILD/bc/fmofp_bc$exe" ] || fail "no binaries under $BUILD (run the cpp step first)"
    # Python's subprocess wants native paths on Windows (K:/..., not /k/...).
    WB="$(cygpath -m "$BUILD" 2>/dev/null || echo "$BUILD")"
    export FMOFP_BC="$WB/bc/fmofp_bc$exe"
    for p in fcs nav nav_tool fms radar radar_tool engine engine_tool mission mission_tool comms; do
        export "FMOFP_$(echo "$p" | tr '[:lower:]' '[:upper:]')=$WB/partitions/fmofp_$p$exe"
    done
}

step_py() {
    export_binaries
    say "python suite (fmofp_bus/tests)"
    (cd "$HERE" && $PY -m unittest discover -s fmofp_bus/tests -t . 2>&1 \
        | grep -v ' - INFO - \| - DEBUG - \| - WARNING - \| - ERROR - \[WEATHER\]\|DeprecationWarning\|Root directory' \
        | tail -n 40; exit "${PIPESTATUS[0]}") || fail "python suite"
}

step_load() {
    export_binaries
    say "bus load, fourteen terminals"
    "$FMOFP_BC" --schedule --rt 27 --rt 5 --rt 7 --rt 12 --rt 9 --rt 3 --rt 6 --rt 2 \
        --rt 11 --rt 13 --rt 14 --rt 15 --rt 16 --rt 17 2>&1 | grep -i 'scheduled messages\|budget\|refus\|error' | tail -n 4
    [ "${PIPESTATUS[0]}" -eq 0 ] || fail "bus load (the schedule does not fit the 10 ms budget)"
}

for s in $steps; do
    case "$s" in
        icd)  step_icd;;
        cpp)  step_cpp;;
        py)   step_py;;
        load) step_load;;
        *) echo "verify.sh: unknown step '$s' (icd cpp py load all)"; exit 2;;
    esac
done
printf '\nverify: OK (%s)\n' "$steps"
