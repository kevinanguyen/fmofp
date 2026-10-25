"""The plant's terrain and the block it publishes."""

import math
import unittest

import numpy as np

from fmofp_bus.block import BlockCollector, BlockPublisher, BlockReceiver
from fmofp_bus.icd.blocks import PlantTerrain
from fmofp_bus.status import Status
from fmofp_plant.terrain import (PROFILE_RANGE_M, PROFILE_SAMPLES, FlatTerrain, HillTerrain, local_metres, profile,
                                 profile_message, sample_distance)


class TerrainTest(unittest.TestCase):
    def test_sample_distances_are_numpy_linspace(self):
        expected = np.linspace(0.0, PROFILE_RANGE_M, PROFILE_SAMPLES)
        for i in range(PROFILE_SAMPLES):
            self.assertEqual(expected[i], sample_distance(i), i)
        self.assertEqual(PROFILE_RANGE_M, sample_distance(PROFILE_SAMPLES - 1))
        self.assertEqual(0.0, sample_distance(0, samples=1))

    def test_northbound_from_home_is_the_pythons_profile(self):
        # tfr_radar._initialize_terrain_data: 1000 + 500 sin(d/1000) + 200 sin(d/300), plus noise of sigma 50 m;
        # the port's two 35 m ripples stand in for the noise.
        heights = profile(HillTerrain(), 0.0, 0.0, 0.0)
        self.assertEqual(PROFILE_SAMPLES, len(heights))
        for i, h in enumerate(heights):
            d = sample_distance(i)
            python = 1000.0 + 500.0 * math.sin(d / 1000.0) + 200.0 * math.sin(d / 300.0)
            self.assertLessEqual(abs(h - python), 70.0 + 1e-9, i)
        self.assertGreater(max(heights), 1500.0, "there is high terrain to warn about")
        self.assertLess(min(heights), 700.0, "and steep terrain")

    def test_the_hills_are_a_place(self):
        hills = HillTerrain()
        here = profile(hills, 0.0, 0.0, 0.0)
        again = profile(hills, 0.0, 0.0, 0.0)
        self.assertEqual(here, again, "the same place has the same height")
        east = profile(hills, 0.0, 0.0, 90.0)
        self.assertNotEqual(here[1:], east[1:], "another heading sees other ground")
        self.assertEqual(here[0], east[0], "but the ground under the aircraft is the same")
        # A degree north is 111 km: well past the hills' first period, a different profile.
        north = profile(hills, 1.0, 0.0, 0.0)
        self.assertNotEqual(here, north)
        self.assertEqual((0.0, 111_000.0), local_metres(1.0, 0.0))
        e, n = local_metres(60.0, 1.0)
        self.assertAlmostEqual(111_000.0 * math.cos(math.radians(60.0)), e)
        self.assertAlmostEqual(60.0 * 111_000.0, n)

    def test_flat_terrain(self):
        heights = profile(FlatTerrain(2600.0), 0.3, -0.2, 45.0)
        self.assertEqual([2600.0 * 0.3048] * PROFILE_SAMPLES, heights)

    def test_profile_message_round_trips(self):
        m = profile_message(HillTerrain(), 0.0, 0.0, 0.0, seq=300)
        self.assertEqual(44, m.terrain_seq, "sequence numbers wrap at 256")
        self.assertEqual(PROFILE_SAMPLES, len(m.elevation))
        status, words = m.encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual(PlantTerrain.MAX_PAYLOAD_WORDS, len(words))
        status, back = PlantTerrain.decode(words)
        self.assertEqual(Status.OK, status)
        self.assertEqual(PROFILE_RANGE_M, back.range_m)
        truth = profile(HillTerrain(), 0.0, 0.0, 0.0)
        for i, record in enumerate(back.elevation):
            self.assertLessEqual(abs(record.elevation - truth[i]), 0.125, "quarter-metre quantisation")


class FakeTerminal:
    """Just enough of a RemoteTerminal for the publisher and collector."""

    def __init__(self):
        self.tx = {}
        self.reads = {}
        self.rx = {}

    def transmit_count(self, sa):
        return self.reads.get(sa, 0)

    def set_transmit(self, sa, words):
        self.tx[sa] = list(words)
        return Status.OK

    def bus_reads(self, sa):
        """The bus controller polled `sa`: what it got, and the terminal counts the read."""
        self.reads[sa] = self.reads.get(sa, 0) + 1
        return self.tx[sa]

    def deliver(self, sa, words):
        self.rx[sa] = (list(words), True)

    def get_received(self, sa):
        if sa not in self.rx:
            return Status.NOT_READY, [], False
        words, fresh = self.rx[sa]
        self.rx[sa] = (words, False)
        return Status.OK, words, fresh


class BlockPublisherTest(unittest.TestCase):
    def test_segments_advance_as_the_bus_reads_them_and_a_new_payload_restarts(self):
        terminal = FakeTerminal()
        publisher = BlockPublisher(PlantTerrain.MAX_PAYLOAD_WORDS)
        payload = list(range(1, 100))  # four segments
        self.assertEqual(Status.OK, publisher.publish(terminal, 5, payload))
        self.assertEqual(1, publisher.version)
        receiver = BlockReceiver()
        for frame in range(4):
            self.assertEqual(Status.OK, publisher.serve(terminal, 4, 5))
            self.assertEqual(frame, terminal.tx[5][0] & 0xFF, "segment index on offer")
            receiver.feed_descriptor(terminal.bus_reads(4))
            done = receiver.feed_segment(terminal.bus_reads(5))
        self.assertTrue(done)
        self.assertEqual(payload, receiver.payload)
        # Not read this frame: the same segment stays on offer.
        publisher.serve(terminal, 4, 5)
        self.assertEqual(0, terminal.tx[5][0] & 0xFF, "wrapped to the first segment after the last")
        publisher.serve(terminal, 4, 5)
        self.assertEqual(0, terminal.tx[5][0] & 0xFF)
        # A new payload starts at segment 0 even though the bus read in between.
        terminal.bus_reads(5)
        self.assertEqual(Status.OK, publisher.publish(terminal, 5, [7, 8, 9]))
        publisher.serve(terminal, 4, 5)
        self.assertEqual(2, publisher.version)
        self.assertEqual((2 << 8) | 0, terminal.tx[5][0])
        self.assertEqual([7, 8, 9], terminal.tx[5][1:4])

    def test_collector_completes_a_relayed_block(self):
        source = FakeTerminal()
        publisher = BlockPublisher()
        self.assertEqual(Status.OK, publisher.publish(source, 5, list(range(50))))
        consumer = FakeTerminal()
        collector = BlockCollector()
        self.assertFalse(collector.collect(consumer, 21, 22), "nothing delivered yet")
        completed = []
        for _ in range(2):
            publisher.serve(source, 4, 5)
            consumer.deliver(21, source.bus_reads(4))
            consumer.deliver(22, source.bus_reads(5))
            completed.append(collector.collect(consumer, 21, 22))
        self.assertEqual([False, True], completed)
        self.assertEqual(list(range(50)), collector.receiver.payload)
        self.assertFalse(collector.collect(consumer, 21, 22), "the same words again complete nothing new")


if __name__ == "__main__":
    unittest.main()
