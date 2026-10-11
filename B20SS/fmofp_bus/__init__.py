"""The Python side of the FMOFP MIL-STD-1553B bus.

A line-for-line mirror of the C++ flight code in B20SS/ofp (bus words, framing,
the remote terminal), so Python processes -- the plant model, the displays, the
bus monitor -- can be terminals on the same bus as the C++ partitions. The
message layouts in fmofp_bus.icd are generated from B20SS/icd/fmofp_icd.xml by
`icd_tool.py gen-py`; never edit them by hand.

Rules are the ICD's (B20SS/icd/README.md): nothing here invents a layout, and
every encode reports clamping and bad values through Status rather than
raising, exactly as the C++ does, so both sides behave identically on the wire.
"""
