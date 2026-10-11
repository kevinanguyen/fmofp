import unittest

from fmofp_bus.status import Status
from fmofp_bus.transport import FLAG_PARITY_ERROR, MAX_FRAME_WORDS, Frame, FrameType, decode_frame, encode_frame


def sample():
    return Frame(FrameType.BC_TO_RT, 0, 0x1234, [0x4C22, 0xBEEF, 0x0001])


class FrameTest(unittest.TestCase):
    def test_bytes_are_big_endian_in_the_documented_order(self):
        status, data = encode_frame(sample())
        self.assertEqual(Status.OK, status)
        self.assertEqual(bytes([0x15, 0x53, 0x01, 0x00, 0x12, 0x34, 0x00, 0x03, 0x4C, 0x22, 0xBE, 0xEF, 0x00, 0x01]),
                         data)

    def test_round_trips_with_flags_and_every_type(self):
        for frame_type in FrameType:
            f = sample()
            f.type = frame_type
            f.flags = FLAG_PARITY_ERROR
            status, data = encode_frame(f)
            self.assertEqual(Status.OK, status)
            status, back = decode_frame(data)
            self.assertEqual(Status.OK, status)
            self.assertEqual(f, back)

    def test_largest_frame(self):
        f = Frame(FrameType.MONITOR, 0, 1, [i * 0x0101 for i in range(MAX_FRAME_WORDS)])
        status, data = encode_frame(f)
        self.assertEqual(Status.OK, status)
        self.assertEqual(2 * (4 + MAX_FRAME_WORDS), len(data))
        self.assertEqual((Status.OK, f), decode_frame(data))

    def test_encode_rejects_what_cannot_be_sent(self):
        self.assertEqual((Status.INVALID_ARGUMENT, b""), encode_frame(Frame(words=[0] * (MAX_FRAME_WORDS + 1))))
        self.assertEqual(Status.INVALID_ARGUMENT, encode_frame(Frame(type=9))[0])

    def test_decode_rejects_damaged_bytes(self):
        _, data = encode_frame(sample())
        self.assertEqual((Status.BAD_LENGTH, None), decode_frame(data[:-1]))
        self.assertEqual((Status.BAD_LENGTH, None), decode_frame(data[:-2]))
        self.assertEqual((Status.BAD_LENGTH, None), decode_frame(data[:6]))
        self.assertEqual((Status.BAD_LENGTH, None), decode_frame(None))
        self.assertEqual((Status.BAD_MAGIC, None), decode_frame(b"\x00" + data[1:]))
        self.assertEqual((Status.MALFORMED, None), decode_frame(data[:2] + b"\x07" + data[3:]))
        self.assertEqual((Status.BAD_LENGTH, None), decode_frame(data[:7] + bytes([MAX_FRAME_WORDS + 1]) + data[8:]))


if __name__ == "__main__":
    unittest.main()
