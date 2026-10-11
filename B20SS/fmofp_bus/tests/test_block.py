"""Block transfers: CRC, transmitter paging, receiver reassembly, and the monitor helper."""

import unittest

from fmofp_bus import block
from fmofp_bus.command_word import CommandWord, encode_command_word
from fmofp_bus.monitor import MonitorRecord, parse_record
from fmofp_bus.status import Status
from fmofp_bus.transport import Frame, FrameType


def payload_of(length):
    return [(i * 3 + 1) & 0xFFFF for i in range(length)]


class CrcTest(unittest.TestCase):
    def test_check_values(self):
        self.assertEqual(0x29B1, block.crc16_ccitt_false_bytes(b"123456789"))  # the standard check value
        self.assertEqual(0xFFFF, block.crc16_ccitt_false([]))
        self.assertEqual(0x1D0F, block.crc16_ccitt_false([0]))  # the C++ unit test pins the same two
        self.assertEqual(block.crc16_ccitt_false_bytes(b"\x31\x32\x33\x34"), block.crc16_ccitt_false([0x3132, 0x3334]))


class TransmitterTest(unittest.TestCase):
    def test_descriptor_segments_and_versions(self):
        tx = block.BlockTransmitter(100)
        self.assertFalse(tx.has_payload)
        self.assertEqual(0, tx.descriptor()[0])
        p = payload_of(70)
        self.assertEqual(Status.OK, tx.set_payload(p))
        self.assertEqual((1, 70, 3, 0), (tx.version, tx.length, tx.segment_count, tx.next_index))
        self.assertEqual([0x0100, 70, 3, block.crc16_ccitt_false(p)], tx.descriptor())
        self.assertEqual([0x0100] + p[:31], tx.segment(0))
        self.assertEqual([0x0102] + p[62:] + [0] * 23, tx.segment(2))
        tx.advance()
        tx.advance()
        tx.advance()
        self.assertEqual(0, tx.next_index)
        self.assertEqual(Status.BAD_LENGTH, tx.set_payload(payload_of(101)))
        for _ in range(254):
            tx.set_payload(p[:3])
        self.assertEqual(255, tx.version)
        tx.set_payload(p[:3])
        self.assertEqual(1, tx.version, "the version never returns to 0")


class ReceiverTest(unittest.TestCase):
    def setUp(self):
        self.tx = block.BlockTransmitter(200)
        self.rx = block.BlockReceiver(200)

    def test_complete_in_any_order_then_repeats_change_nothing(self):
        p = payload_of(100)
        self.tx.set_payload(p)
        self.assertFalse(self.rx.feed_descriptor(self.tx.descriptor()))
        for i in (0, 3, 1):
            self.assertFalse(self.rx.feed_segment(self.tx.segment(i)))
        self.assertTrue(self.rx.feed_segment(self.tx.segment(2)))
        self.assertEqual(p, self.rx.payload)
        self.assertEqual((1, 100, 1), (self.rx.version, self.rx.length, self.rx.completed_blocks))
        self.assertFalse(self.rx.feed_segment(self.tx.segment(1)))
        self.assertTrue(self.rx.complete)

    def test_descriptor_last_new_version_and_corruption(self):
        p = payload_of(40)
        self.tx.set_payload(p)
        self.assertFalse(self.rx.feed_segment(self.tx.segment(0)))
        self.assertFalse(self.rx.feed_segment(self.tx.segment(1)))
        self.assertTrue(self.rx.feed_descriptor(self.tx.descriptor()))
        q = list(p)
        q[35] = 0xBEEF
        self.tx.set_payload(q)
        self.assertFalse(self.rx.feed_segment(self.tx.segment(0)), "version 2 starts a new collection")
        self.assertFalse(self.rx.complete)
        self.assertFalse(self.rx.feed_descriptor(self.tx.descriptor()))
        bad = self.tx.segment(1)
        bad[5] ^= 1
        self.assertFalse(self.rx.feed_segment(bad), "the CRC catches a flipped bit")
        self.assertTrue(self.rx.feed_segment(self.tx.segment(1)))
        self.assertEqual(0xBEEF, self.rx.payload[35])
        self.assertFalse(self.rx.feed_segment([0] * 32), "an unloaded subaddress disturbs nothing")
        self.assertFalse(self.rx.feed_descriptor([0] * 4))
        self.assertTrue(self.rx.complete)

    def test_blocks_from_records(self):
        """The monitor helper feeds only the stream's own transmits and reports each completion once."""
        p = payload_of(50)
        self.tx.set_payload(p)
        records = []

        def transmit(rt, sa, words):
            _, cw = encode_command_word(CommandWord(rt, True, sa, len(words)))
            status = (rt << 11)
            records.append(parse_record(Frame(FrameType.MONITOR, 0, len(records), [cw, status] + list(words))))

        transmit(9, 6, self.tx.descriptor())
        transmit(9, 7, self.tx.segment(0))
        transmit(9, 1, [0x1000, 0])  # another subaddress, ignored
        transmit(5, 7, self.tx.segment(1))  # another terminal, ignored
        transmit(9, 7, self.tx.segment(1))
        transmit(9, 7, self.tx.segment(0))  # a repeat
        self.tx.set_payload(p[:10])
        transmit(9, 7, self.tx.segment(0))
        transmit(9, 6, self.tx.descriptor())
        done = block.blocks_from_records(records, 9, 6, 7)
        self.assertEqual([(4, 1, p), (7, 2, p[:10])], done)
        self.assertIsInstance(records[0], MonitorRecord)


if __name__ == "__main__":
    unittest.main()
