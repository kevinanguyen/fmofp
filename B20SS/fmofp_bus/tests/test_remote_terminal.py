"""The Python terminal behaves exactly like the C++ one (same cases as remote_terminal_test.cpp)."""

import unittest

from fmofp_bus.command_word import BROADCAST_RT, CommandWord, decode_status_word, encode_command_word
from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import TerminalState
from fmofp_bus.remote_terminal import (MODE_CODE_INITIATE_SELF_TEST, MODE_CODE_RESET_REMOTE_TERMINAL,
                                       MODE_CODE_SYNCHRONIZE_WITH_DATA, MODE_CODE_TRANSMIT_BIT_WORD,
                                       MODE_CODE_TRANSMIT_LAST_COMMAND, MODE_CODE_TRANSMIT_STATUS_WORD,
                                       MODE_CODE_TRANSMIT_VECTOR_WORD, BitWord, CommandSequence, RemoteTerminal,
                                       decode_bit_word, encode_bit_word)
from fmofp_bus.status import Status
from fmofp_bus.transport import FLAG_PARITY_ERROR, Frame, FrameType

RT = 9


def command(rt, transmit, sa, count_or_mode, data=(), flags=0):
    _, word = encode_command_word(CommandWord(rt, transmit, sa, count_or_mode))
    return Frame(FrameType.BC_TO_RT, flags, 0x0101, [word] + list(data))


def mode_code(rt, transmit, code, data=()):
    return command(rt, transmit, 31, code, data)


def status_of(reply):
    status, sw = decode_status_word(reply.words[0])
    assert status == Status.OK
    return sw


class RemoteTerminalTest(unittest.TestCase):
    def setUp(self):
        self.rt = RemoteTerminal()
        self.assertEqual(Status.OK, self.rt.init(RT))

    def run_cmd(self, frame):
        status, reply = self.rt.handle(frame)
        self.assertEqual(Status.OK, status)
        return reply

    def test_init_rejects_broadcast_address(self):
        self.assertEqual(Status.INVALID_ARGUMENT, RemoteTerminal().init(31))

    def test_receive_and_answer(self):
        reply = self.run_cmd(command(RT, False, 5, 3, [0x1111, 0x2222, 0x3333]))
        self.assertEqual(FrameType.RT_TO_BC, reply.type)
        self.assertEqual(1, reply.word_count)
        self.assertEqual(0x0101, reply.transaction)
        self.assertEqual(RT, status_of(reply).rt)
        self.assertEqual((Status.OK, [0x1111, 0x2222, 0x3333], True), self.rt.get_received(5))
        self.assertEqual((Status.OK, [0x1111, 0x2222, 0x3333], False), self.rt.get_received(5))
        self.assertEqual((Status.NOT_READY, [], False), self.rt.get_received(6))

    def test_transmit_loaded_or_zeros(self):
        self.assertEqual(Status.OK, self.rt.set_transmit(29, [0xA1, 0xA2, 0xA3]))
        reply = self.run_cmd(command(RT, True, 29, 3))
        self.assertEqual([0xA1, 0xA2, 0xA3], reply.words[1:])
        reply = self.run_cmd(command(RT, True, 12, 2))
        self.assertEqual([0, 0], reply.words[1:])
        self.assertEqual(Status.INVALID_ARGUMENT, self.rt.set_transmit(0, [1]))
        self.assertEqual(Status.INVALID_ARGUMENT, self.rt.set_transmit(5, []))
        self.assertEqual(Status.INVALID_ARGUMENT, self.rt.set_transmit(5, [0] * 33))

    def test_ignores_other_terminals(self):
        self.assertIsNone(self.run_cmd(command(RT + 1, False, 5, 1, [7])))
        self.assertEqual(1, self.rt.counters.ignored)

    def test_broadcast_sets_the_bit_once(self):
        self.assertEqual(Status.OK, self.rt.accept_broadcast(5))
        self.assertIsNone(self.run_cmd(command(BROADCAST_RT, False, 5, 1, [0xB0])))
        self.assertEqual((Status.OK, [0xB0], True), self.rt.get_received(5))
        self.assertTrue(status_of(self.run_cmd(command(RT, True, 5, 1))).broadcast_received)
        self.assertFalse(status_of(self.run_cmd(command(RT, True, 5, 1))).broadcast_received)

    def test_broadcast_data_on_an_unaccepted_subaddress_is_ignored(self):
        # ICD v1.7: a terminal stores a broadcast only on a subaddress of its broadcast map.
        self.assertIsNone(self.run_cmd(command(BROADCAST_RT, False, 5, 1, [0xB0])))
        self.assertEqual(Status.NOT_READY, self.rt.get_received(5)[0], "nothing stored")
        self.assertEqual((1, 0, 0), (self.rt.counters.broadcasts_ignored, self.rt.counters.broadcasts,
                                     self.rt.counters.message_errors))
        self.assertFalse(status_of(self.run_cmd(command(RT, True, 5, 1))).broadcast_received)
        self.assertEqual(Status.INVALID_ARGUMENT, self.rt.accept_broadcast(0))
        self.assertEqual(Status.INVALID_ARGUMENT, self.rt.accept_broadcast(31))
        self.assertEqual(Status.OK, self.rt.accept_broadcast(6))
        self.assertTrue(self.rt.accepts_broadcast(6))
        self.assertFalse(self.rt.accepts_broadcast(5))
        # The map survives a reset; the sync never needed it.
        self.assertIsNone(self.run_cmd(mode_code(BROADCAST_RT, True, MODE_CODE_RESET_REMOTE_TERMINAL)))
        self.assertTrue(self.rt.take_reset_request())
        self.assertIsNone(self.run_cmd(command(BROADCAST_RT, False, 6, 2, [0xC0, 0xC1])))
        self.assertEqual((Status.OK, [0xC0, 0xC1], True), self.rt.get_received(6))
        self.assertIsNone(self.run_cmd(mode_code(BROADCAST_RT, False, MODE_CODE_SYNCHRONIZE_WITH_DATA, [0x0C07])))
        self.assertEqual(1, self.rt.counters.syncs)

    def test_parity_error_means_silence_then_message_error(self):
        self.assertIsNone(self.run_cmd(command(RT, False, 5, 1, [1], flags=FLAG_PARITY_ERROR)))
        self.assertEqual(Status.NOT_READY, self.rt.get_received(5)[0])
        self.assertTrue(status_of(self.run_cmd(command(RT, True, 5, 1))).message_error)
        self.assertFalse(status_of(self.run_cmd(command(RT, True, 5, 1))).message_error)

    def test_bad_commands_are_message_errors(self):
        self.assertIsNone(self.run_cmd(command(RT, False, 5, 4, [1, 2, 3])))
        self.assertIsNone(self.run_cmd(command(BROADCAST_RT, True, 5, 1)))
        self.assertIsNone(self.run_cmd(mode_code(RT, True, 5)))
        self.assertIsNone(self.run_cmd(mode_code(RT, False, MODE_CODE_TRANSMIT_STATUS_WORD)))
        self.assertIsNone(self.run_cmd(mode_code(RT, False, MODE_CODE_SYNCHRONIZE_WITH_DATA)))
        self.assertEqual(5, self.rt.counters.message_errors)

    def test_sync(self):
        self.assertIsNone(self.run_cmd(mode_code(BROADCAST_RT, False, MODE_CODE_SYNCHRONIZE_WITH_DATA, [0x0C07])))
        self.assertEqual((True, 0x0C07), self.rt.take_sync())
        self.assertEqual((False, 0), self.rt.take_sync())
        reply = self.run_cmd(mode_code(RT, False, MODE_CODE_SYNCHRONIZE_WITH_DATA, [0x0C08]))
        self.assertEqual(1, reply.word_count)
        self.assertEqual((True, 0x0C08), self.rt.take_sync())
        self.assertEqual(2, self.rt.counters.syncs)

    def test_transmit_status_word_does_not_clear(self):
        self.assertIsNone(self.run_cmd(command(RT, False, 5, 1, [1], flags=FLAG_PARITY_ERROR)))
        self.assertTrue(status_of(self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_STATUS_WORD))).message_error)
        self.assertTrue(status_of(self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_STATUS_WORD))).message_error)
        self.assertTrue(status_of(self.run_cmd(command(RT, True, 5, 1))).message_error)
        self.assertFalse(status_of(self.run_cmd(command(RT, True, 5, 1))).message_error)

    def test_self_test_bit_vector_last_command_reset(self):
        self.assertIsNotNone(self.run_cmd(mode_code(RT, True, MODE_CODE_INITIATE_SELF_TEST)))
        self.assertTrue(self.rt.take_self_test_request())
        self.assertFalse(self.rt.take_self_test_request())
        self.assertTrue(decode_bit_word(self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_BIT_WORD)).words[1]).in_progress)
        self.rt.set_bit(BitWord(False, True, 3))
        self.assertEqual(0x4003, self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_BIT_WORD)).words[1])

        self.rt.request_service(12)
        reply = self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_VECTOR_WORD))
        self.assertTrue(status_of(reply).service_request)
        self.assertEqual(12, reply.words[1])
        self.assertFalse(status_of(self.run_cmd(command(RT, True, 5, 1))).service_request)

        first = command(RT, False, 5, 1, [1])
        self.run_cmd(first)
        self.assertEqual(first.words[0], self.run_cmd(mode_code(RT, True, MODE_CODE_TRANSMIT_LAST_COMMAND)).words[1])

        self.rt.set_terminal_flag(True)
        reply = self.run_cmd(mode_code(RT, True, MODE_CODE_RESET_REMOTE_TERMINAL))
        self.assertFalse(status_of(reply).terminal_flag)
        self.assertTrue(self.rt.take_reset_request())
        self.assertFalse(self.rt.take_reset_request())
        self.assertEqual(Status.NOT_READY, self.rt.get_received(5)[0])

    def test_transport_faults(self):
        self.assertEqual((Status.INVALID_ARGUMENT, None), self.rt.handle(Frame(FrameType.RT_TO_BC, 0, 1, [1])))
        self.assertEqual((Status.BAD_LENGTH, None), self.rt.handle(Frame(FrameType.BC_TO_RT, 0, 1, [])))

    def test_carries_a_generated_message(self):
        status, words = m.TermStatus(state=TerminalState.NORMAL, healthy=True, last_cmd_seq=0x2A, icd_major=1).encode()
        self.assertEqual(Status.OK, status)
        self.assertEqual(Status.OK, self.rt.set_transmit(m.TermStatus.SUBADDRESS, words))
        reply = self.run_cmd(command(RT, True, m.TermStatus.SUBADDRESS, m.TermStatus.WORDS))
        status, decoded = m.TermStatus.decode(reply.words[1:])
        self.assertEqual(Status.OK, status)
        self.assertEqual(TerminalState.NORMAL, decoded.state)
        self.assertEqual(0x2A, decoded.last_cmd_seq)


class HelpersTest(unittest.TestCase):
    def test_command_sequence(self):
        seq = CommandSequence()
        self.assertFalse(seq.accept(0), "0 is no command: an unloaded subaddress reads as zeros")
        self.assertTrue(seq.accept(1))
        self.assertFalse(seq.accept(1))
        self.assertTrue(seq.accept(2))
        self.assertFalse(seq.accept(0))
        self.assertFalse(seq.accept(2))
        self.assertTrue(seq.accept(255))
        self.assertTrue(seq.accept(1), "wrap-around is just another change")

    def test_bit_word(self):
        self.assertEqual(0x80FF, encode_bit_word(BitWord(True, False, 255)))
        self.assertEqual(BitWord(False, True, 200), decode_bit_word(encode_bit_word(BitWord(False, True, 200))))


if __name__ == "__main__":
    unittest.main()
