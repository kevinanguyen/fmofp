import unittest

from fmofp_bus.command_word import (BROADCAST_RT, CommandWord, StatusWord, decode_command_word, decode_status_word,
                                    encode_command_word, encode_status_word)
from fmofp_bus.status import Status


class CommandWordTest(unittest.TestCase):
    def test_icd_layout(self):
        self.assertEqual((Status.OK, 0x4C22), encode_command_word(CommandWord(9, True, 1, 2)))
        self.assertEqual((Status.OK, 0xFBF1), encode_command_word(CommandWord(BROADCAST_RT, False, 31, 17)))

    def test_thirty_two_words_is_sent_as_zero(self):
        status, word = encode_command_word(CommandWord(5, False, 11, 32))
        self.assertEqual(Status.OK, status)
        self.assertEqual(0, word & 0x1F)
        self.assertEqual(32, decode_command_word(word).count_or_mode)
        # Mode code 0 stays 0.
        _, word = encode_command_word(CommandWord(5, True, 0, 0))
        self.assertEqual(0, decode_command_word(word).count_or_mode)

    def test_rejects_out_of_range(self):
        for cw in (CommandWord(32, False, 1, 1), CommandWord(1, False, 32, 1), CommandWord(1, False, 1, 0),
                   CommandWord(1, False, 1, 33), CommandWord(1, True, 31, 32)):
            self.assertEqual((Status.INVALID_ARGUMENT, 0), encode_command_word(cw))

    def test_every_pattern_round_trips(self):
        for pattern in range(0x10000):
            self.assertEqual((Status.OK, pattern), encode_command_word(decode_command_word(pattern)))


class StatusWordTest(unittest.TestCase):
    def test_icd_layout(self):
        self.assertEqual((Status.OK, 0x4C01), encode_status_word(StatusWord(9, message_error=True, terminal_flag=True)))
        self.assertEqual((Status.OK, 0xF91C), encode_status_word(
            StatusWord(31, service_request=True, broadcast_received=True, busy=True, subsystem_flag=True)))
        self.assertEqual((Status.INVALID_ARGUMENT, 0), encode_status_word(StatusWord(32)))

    def test_reserved_bits_are_malformed_but_decoded(self):
        status, sw = decode_status_word(0x4A00)
        self.assertEqual(Status.MALFORMED, status)
        self.assertEqual(9, sw.rt)
        self.assertEqual(Status.MALFORMED, decode_status_word(0x0020)[0])
        self.assertEqual(Status.MALFORMED, decode_status_word(0x0002)[0])
        status, sw = decode_status_word(0x4C1D)
        self.assertEqual(Status.OK, status)
        self.assertTrue(sw.message_error and sw.broadcast_received and sw.busy and sw.subsystem_flag and sw.terminal_flag)
        self.assertFalse(sw.service_request)


if __name__ == "__main__":
    unittest.main()
