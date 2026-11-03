"""The RF environment the communications terminal measures: CommsService._simulate's draws, as the world.

The Python's CommsService drew, once a second, a radio signal strength that random-walked from 0 by up
to 3 % a step, a satellite connection that appeared with a 1 % chance a second (5 % while acquiring) and
then walked its signal, data rate and latency, and a data link that counted random packets and walked an
error rate while the satcom was connected. Here those numbers are the world's, published as PLANT_RF once a
second: the field strength at the aircraft, whether a satellite is in view (it comes into view with the
1 % draw and stays, as the Python's connection never dropped) with its number, signal, rate and latency,
the link's error rate and the packets the network offers in and out. The communications terminal (RT 2)
applies the equipment's rules to them: active above 20 %, connected when a satellite is in view, the data
link up while connected.
"""

from fmofp_bus.icd import messages as m

RADIO_WALK_PCT = 3.0            # _simulate: signal_strength += uniform(-3, 3), held to 0..100
SATELLITE_CHANCE_PER_S = 0.01   # random.random() < 0.01: the satellite appears
SATELLITE_SIGNAL = (40.0, 80.0)  # uniform on appearance, then += uniform(-1, 1), held to 0..100
SATELLITE_RATE_KBPS = (50.0, 200.0)  # then += uniform(-5, 5), never below 0
SATELLITE_LATENCY_MS = (200.0, 600.0)  # then += uniform(-10, 10), never below 200
SATELLITE_IDS = (1, 12)          # randint(1, 12)
PACKETS_PER_S = (0, 2)           # randint(0, 2) sent and received
ERROR_WALK_PCT = 0.1             # error_rate += uniform(-0.1, 0.1), never below 0


class RfWorld:
    """The field, the satellite and the link, stepped once a second from its own random generator."""

    def __init__(self, rng, satellite_in_view=False, satellite_after_s=None):
        self.rng = rng
        self.radio_field = 0.0
        self.satellite_available = False
        self.satellite_id = 0
        self.satellite_signal = 0.0
        self.satellite_data_rate = 0.0
        self.satellite_latency = 0.0
        self.link_error_rate = 0.0
        self.packets_in = 0
        self.packets_out = 0
        self.seconds = 0
        # For scripted runs: a satellite in view from the start, or after so many seconds, instead of the draw.
        self.satellite_after_s = satellite_after_s
        if satellite_in_view:
            self._satellite_appears()

    def _satellite_appears(self):
        u = self.rng.uniform
        self.satellite_available = True
        self.satellite_signal = u(*SATELLITE_SIGNAL)
        self.satellite_data_rate = u(*SATELLITE_RATE_KBPS)
        self.satellite_latency = u(*SATELLITE_LATENCY_MS)
        self.satellite_id = self.rng.randint(*SATELLITE_IDS)

    def step(self):
        """One second, in the Python's draw order: the radio, the satcom, the data link."""
        u = self.rng.uniform
        self.seconds += 1
        self.radio_field = max(0.0, min(100.0, self.radio_field + u(-RADIO_WALK_PCT, RADIO_WALK_PCT)))
        if self.satellite_available:
            self.satellite_signal = max(0.0, min(100.0, self.satellite_signal + u(-1.0, 1.0)))
            self.satellite_data_rate = max(0.0, self.satellite_data_rate + u(-5.0, 5.0))
            self.satellite_latency = max(SATELLITE_LATENCY_MS[0], self.satellite_latency + u(-10.0, 10.0))
        elif self.satellite_after_s is not None:
            if self.seconds >= self.satellite_after_s:
                self._satellite_appears()
        elif self.rng.random() < SATELLITE_CHANCE_PER_S:
            self._satellite_appears()
        self.packets_out = self.rng.randint(*PACKETS_PER_S)
        self.packets_in = self.rng.randint(*PACKETS_PER_S)
        self.link_error_rate = max(0.0, self.link_error_rate + u(-ERROR_WALK_PCT, ERROR_WALK_PCT))

    def message(self):
        return m.PlantRf(radio_field=self.radio_field, satellite_available=self.satellite_available,
                         satellite_id=self.satellite_id if self.satellite_available else 0,
                         satellite_signal=self.satellite_signal if self.satellite_available else 0.0,
                         satellite_data_rate=self.satellite_data_rate if self.satellite_available else 0.0,
                         satellite_latency=self.satellite_latency if self.satellite_available else 0.0,
                         link_error_rate=self.link_error_rate, packets_in=self.packets_in,
                         packets_out=self.packets_out)
