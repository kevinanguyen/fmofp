"""The displays on the bus.

    py -m fmofp_display radar [--port-base 51000]    the weather and terrain-following radar pictures, RT 15
    py -m fmofp_display pfd   [--port-base 51000]    the primary flight display, RT 11
    py -m fmofp_display mfd   [--port-base 51000]    the multi-function display, RT 13

Each display is its own process and its own terminal (ICD README section 2, "one
process per display"). The bus controller relays what the display needs into
its receive subaddresses and receive blocks; the display's own commands sit on
its transmit subaddresses for the controller to relay on. The PyQt6 display
classes under FMOFP/Interfaces are unchanged: a feed turns the bus words into
the dictionaries they always drew.

    terminal.py     DisplayTerminal: the terminal's socket loop in a thread, the latest copy of every watched message and block
    radar_feed.py   RadarFeed: RDR_WX_PRODUCTS and the TFR's guidance and profile into the radar displays' data
    flight_feed.py  FlightDataFeed: FMS_FLIGHT_DATA into what the PFD reads from the FMS
    systems_feed.py SystemsFeed: the GCAS alerts, exceedances and position the MFD's pages read from the systems
    app.py          the Qt windows and the entry point

The terminal and the feeds have no Qt dependency, so the tests drive them headless.
"""
