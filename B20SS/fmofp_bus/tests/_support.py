"""Shared bits for the tests that run the C++ binaries.

The binaries are named by environment variables (CI sets them):
    FMOFP_BC        bc/fmofp_bc
    FMOFP_FCS       partitions/fmofp_fcs
    FMOFP_NAV       partitions/fmofp_nav
    FMOFP_NAV_TOOL  partitions/fmofp_nav_tool
    FMOFP_FMS       partitions/fmofp_fms
    FMOFP_RADAR     partitions/fmofp_radar
    FMOFP_RADAR_TOOL partitions/fmofp_radar_tool
    FMOFP_ENGINE    partitions/fmofp_engine
    FMOFP_ENGINE_TOOL partitions/fmofp_engine_tool
"""

import os
import subprocess
import sys
import unittest


def binary_env():
    """The environment to run a MinGW binary in: its DLLs on PATH on Windows."""
    env = dict(os.environ)
    if sys.platform == "win32" and os.path.isdir(r"C:\msys64\ucrt64\bin"):
        env["PATH"] = r"C:\msys64\ucrt64\bin;" + env.get("PATH", "")
    return env


def needs(*names):
    """Skip the test class unless every named binary is configured."""
    missing = [n for n in names if not os.environ.get(n)]
    return unittest.skipIf(missing, f"set {', '.join(missing)} to the binaries to run")


class RadarTool:
    """One fmofp_radar_tool run; answers are read back command by command."""

    def __init__(self):
        self.lines = []
        self.out = []
        self.pos = 0

    def send(self, *commands):
        self.lines += commands

    def run(self):
        result = subprocess.run([os.environ["FMOFP_RADAR_TOOL"]], input="\n".join(self.lines) + "\n",
                                capture_output=True, text=True, timeout=600, env=binary_env())
        assert result.returncode == 0, result.stderr
        self.out = result.stdout.splitlines()
        self.pos = 0
        return self

    def line(self):
        self.pos += 1
        return self.out[self.pos - 1].split()

    def block(self):
        rows = []
        while True:
            row = self.line()
            if row == ["end"]:
                return rows
            rows.append(row)
