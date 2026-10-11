"""The same bit-level facts the C++ word tests pin, so both sides agree on the wire."""

import math
import unittest

from fmofp_bus import word as w
from fmofp_bus.status import Status

ANGLE_NUM, ANGLE_DEN = 45, 8192  # 180/32768 deg per count, reduced


class FieldTest(unittest.TestCase):
    def test_masks(self):
        self.assertEqual(0xFFFF, w.field_mask(15, 0))
        self.assertEqual(0x000F, w.field_mask(3, 0))
        self.assertEqual(0xF000, w.field_mask(15, 12))
        self.assertEqual(0x0080, w.field_mask(7, 7))

    def test_set_get(self):
        word = w.set_field(0, 15, 12, 0xA)
        word = w.set_field(word, 11, 8, 0x5)
        word = w.set_field(word, 7, 0, 0x3C)
        self.assertEqual(0xA53C, word)
        self.assertEqual(0xA, w.get_field(word, 15, 12))
        self.assertEqual(0x3C, w.get_field(word, 7, 0))
        self.assertEqual(0x00F0, w.set_field(0, 7, 4, 0x1F))  # excess bits dropped
        self.assertEqual(0xFF0F, w.set_field(0xFFFF, 7, 4, 0))

    def test_bits(self):
        word = w.set_bit(w.set_bit(0, 15, True), 0, True)
        self.assertEqual(0x8001, word)
        self.assertTrue(w.get_bit(word, 15))
        self.assertFalse(w.get_bit(word, 14))

    def test_twos_complement(self):
        self.assertEqual(0xFFFF, w.to_twos(-1, 16))
        self.assertEqual(0x8000, w.to_twos(-32768, 16))
        self.assertEqual(0xF, w.to_twos(-1, 4))
        self.assertEqual(-1, w.from_twos(0xFFFF, 16))
        self.assertEqual(-32768, w.from_twos(0x8000, 16))
        self.assertEqual(32767, w.from_twos(0x7FFF, 16))
        self.assertEqual(-8, w.from_twos(0x8, 4))
        self.assertEqual(7, w.from_twos(0xFFF7, 4))
        for v in range(-128, 128):
            self.assertEqual(v, w.from_twos(w.to_twos(v, 8), 8))


class ScaledTest(unittest.TestCase):
    def test_exact_angles(self):
        self.assertEqual((Status.OK, 8192), w.encode_scaled(45.0, ANGLE_NUM, ANGLE_DEN, -32768, 32767))
        self.assertEqual((Status.OK, -32768), w.encode_scaled(-180.0, ANGLE_NUM, ANGLE_DEN, -32768, 32767))
        self.assertEqual((Status.OK, 32767), w.encode_scaled(179.9945, ANGLE_NUM, ANGLE_DEN, -32768, 32767))
        self.assertEqual(45.0, w.decode_scaled(8192, ANGLE_NUM, ANGLE_DEN))
        self.assertEqual(-180.0, w.decode_scaled(-32768, ANGLE_NUM, ANGLE_DEN))

    def test_rounds_half_away_from_zero_like_llround(self):
        # Python's round() goes to even; the C++ uses llround. Pin the C++ rule.
        self.assertEqual((Status.OK, 2), w.encode_scaled(1.5, 1, 1, -100, 100))
        self.assertEqual((Status.OK, 3), w.encode_scaled(2.5, 1, 1, -100, 100))
        self.assertEqual((Status.OK, -2), w.encode_scaled(-1.5, 1, 1, -100, 100))
        self.assertEqual((Status.OK, 1), w.encode_scaled(1.4, 1, 1, 0, 100))
        self.assertEqual((Status.OK, 2), w.encode_scaled(1.6, 1, 1, 0, 100))
        self.assertEqual((Status.OK, 0), w.encode_scaled(-0.3, 1, 1, 0, 100))

    def test_clamps_and_says_so(self):
        self.assertEqual((Status.CLAMPED, 32767), w.encode_scaled(200.0, ANGLE_NUM, ANGLE_DEN, -32768, 32767))
        self.assertEqual((Status.CLAMPED, -32768), w.encode_scaled(-200.0, ANGLE_NUM, ANGLE_DEN, -32768, 32767))
        self.assertEqual((Status.CLAMPED, 0), w.encode_scaled(-0.6, 1, 1, 0, 100))
        self.assertEqual((Status.CLAMPED, 100), w.encode_scaled(100.6, 1, 1, 0, 100))
        self.assertEqual((Status.OK, 100), w.encode_scaled(100.4, 1, 1, 0, 100))
        self.assertEqual((Status.CLAMPED, 100), w.encode_scaled(1e300, 1, 1, 0, 100))

    def test_rejects_unusable_input(self):
        self.assertEqual((Status.INVALID_ARGUMENT, 0), w.encode_scaled(math.nan, 1, 1, 0, 100))
        self.assertEqual((Status.INVALID_ARGUMENT, 0), w.encode_scaled(math.inf, 1, 1, 0, 100))
        self.assertEqual((Status.INVALID_ARGUMENT, 0), w.encode_scaled(1.0, 0, 1, 0, 100))
        self.assertEqual((Status.INVALID_ARGUMENT, 0), w.encode_scaled(1.0, 1, 1, 10, 0))
        self.assertEqual((Status.INVALID_ARGUMENT, 0), w.encode_scaled("x", 1, 1, 0, 100))

    def test_round_trips_across_the_range(self):
        for raw in range(-32768, 32768, 997):
            value = w.decode_scaled(raw, ANGLE_NUM, ANGLE_DEN)
            self.assertEqual((Status.OK, raw), w.encode_scaled(value, ANGLE_NUM, ANGLE_DEN, -32768, 32767))

    def test_raw_field_checks(self):
        self.assertEqual((Status.OK, 255), w.check_uint(255, 8))
        self.assertEqual((Status.CLAMPED, 255), w.check_uint(256, 8))
        self.assertEqual((Status.CLAMPED, 0), w.check_uint(-1, 8))
        self.assertEqual((Status.OK, -128), w.check_sint(-128, 8))
        self.assertEqual((Status.CLAMPED, 127), w.check_sint(128, 8))


if __name__ == "__main__":
    unittest.main()
