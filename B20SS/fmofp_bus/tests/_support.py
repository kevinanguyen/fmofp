"""Shared bits for the tests that run the C++ binaries.

The binaries are named by environment variables (CI sets them):
    FMOFP_BC        bc/fmofp_bc
    FMOFP_FCS       partitions/fmofp_fcs
    FMOFP_NAV       partitions/fmofp_nav
    FMOFP_NAV_TOOL  partitions/fmofp_nav_tool
    FMOFP_FMS       partitions/fmofp_fms
"""

import os
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
