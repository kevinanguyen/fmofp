"""What every terminal owes the bus, mirroring fmofp::bus::RemoteTerminal.

Fixed buffers per subaddress, the status word with its latched bits, and the
nine mode codes of the ICD. A bad command is answered with silence and the
message-error bit in the next status word, as a real RT does. The behaviour is
kept identical to the C++ so a Python terminal and a C++ terminal are
indistinguishable to the bus controller.
"""

from dataclasses import dataclass

from fmofp_bus import word as w
from fmofp_bus.command_word import (BROADCAST_RT, MAX_DATA_WORDS, MAX_SUBADDRESS, StatusWord,
                                    decode_command_word, encode_status_word)
from fmofp_bus.status import Status
from fmofp_bus.transport import FLAG_PARITY_ERROR, Frame, FrameType

MODE_CODE_TRANSMIT_STATUS_WORD = 2
MODE_CODE_INITIATE_SELF_TEST = 3
MODE_CODE_INHIBIT_TERMINAL_FLAG = 6
MODE_CODE_OVERRIDE_INHIBIT_TERMINAL_FLAG = 7
MODE_CODE_RESET_REMOTE_TERMINAL = 8
MODE_CODE_TRANSMIT_VECTOR_WORD = 16
MODE_CODE_SYNCHRONIZE_WITH_DATA = 17
MODE_CODE_TRANSMIT_LAST_COMMAND = 18
MODE_CODE_TRANSMIT_BIT_WORD = 19

FIRST_DATA_SUBADDRESS = 1
LAST_DATA_SUBADDRESS = 30

# code -> (transmit expected, data word in, data word out, broadcast allowed)
_MODE_CODE_RULES = {
    MODE_CODE_TRANSMIT_STATUS_WORD: (True, False, False, False),
    MODE_CODE_INITIATE_SELF_TEST: (True, False, False, True),
    MODE_CODE_INHIBIT_TERMINAL_FLAG: (True, False, False, True),
    MODE_CODE_OVERRIDE_INHIBIT_TERMINAL_FLAG: (True, False, False, True),
    MODE_CODE_RESET_REMOTE_TERMINAL: (True, False, False, True),
    MODE_CODE_TRANSMIT_VECTOR_WORD: (True, False, True, False),
    MODE_CODE_SYNCHRONIZE_WITH_DATA: (False, True, False, True),
    MODE_CODE_TRANSMIT_LAST_COMMAND: (True, False, True, False),
    MODE_CODE_TRANSMIT_BIT_WORD: (True, False, True, False),
}


@dataclass
class BitWord:
    """TERM_STATUS word 2 and the data word of mode code 19."""
    in_progress: bool = False
    failed: bool = False
    fault_count: int = 0


def encode_bit_word(bit):
    word = 0
    word = w.set_bit(word, 15, bit.in_progress)
    word = w.set_bit(word, 14, bit.failed)
    return w.set_field(word, 7, 0, bit.fault_count)


def decode_bit_word(word):
    return BitWord(w.get_bit(word, 15), w.get_bit(word, 14), w.get_field(word, 7, 0))


class CommandSequence:
    """ICD README section 5: act only when cmd_seq differs from the last one processed; 0 is no command."""

    def __init__(self):
        self.have_last = False
        self.last = 0

    def accept(self, seq):
        if seq == 0 or (self.have_last and seq == self.last):
            return False
        self.have_last = True
        self.last = seq
        return True


@dataclass
class RtCounters:
    commands: int = 0
    ignored: int = 0
    receives: int = 0
    transmits: int = 0
    broadcasts: int = 0
    syncs: int = 0
    message_errors: int = 0


class RemoteTerminal:
    def __init__(self):
        self.rt = 0
        self.counters = RtCounters()
        self._reset_pending = False
        self._reset()

    # ---- the partition's side -------------------------------------------

    def init(self, rt):
        if not 0 <= rt < BROADCAST_RT:
            return Status.INVALID_ARGUMENT
        self.rt = rt
        self._reset()
        self._reset_pending = False
        self.counters = RtCounters()
        return Status.OK

    def set_transmit(self, sa, words):
        """Loads what the terminal sends the next time the BC asks for `sa`."""
        words = list(words)
        if not FIRST_DATA_SUBADDRESS <= sa <= LAST_DATA_SUBADDRESS or not 1 <= len(words) <= MAX_DATA_WORDS:
            return Status.INVALID_ARGUMENT
        self._tx[sa] = [x & 0xFFFF for x in words] + [0] * (MAX_DATA_WORDS - len(words))
        return Status.OK

    def get_received(self, sa):
        """(Status, words, fresh): NOT_READY until something has arrived on `sa`."""
        if not FIRST_DATA_SUBADDRESS <= sa <= LAST_DATA_SUBADDRESS:
            return Status.INVALID_ARGUMENT, [], False
        if not self._rx_valid[sa]:
            return Status.NOT_READY, [], False
        fresh = self._rx_fresh[sa]
        self._rx_fresh[sa] = False
        return Status.OK, list(self._rx[sa][:self._rx_count[sa]]), fresh

    def transmit_count(self, sa):
        """How many times the bus controller has read subaddress `sa` since init or reset."""
        return self._tx_count[sa]

    def request_service(self, sa):
        self._service_requested = True
        self._service_sa = sa & MAX_SUBADDRESS

    def set_subsystem_flag(self, value):
        self._subsystem_flag = bool(value)

    def set_terminal_flag(self, value):
        self._terminal_flag = bool(value)

    def set_bit(self, bit):
        self.bit = bit

    def take_sync(self):
        """(True, sync word) once per SYNCHRONIZE_WITH_DATA; (False, 0) otherwise."""
        if not self._sync_pending:
            return False, 0
        self._sync_pending = False
        return True, self._sync_word

    def take_reset_request(self):
        pending, self._reset_pending = self._reset_pending, False
        return pending

    def take_self_test_request(self):
        pending, self._self_test_pending = self._self_test_pending, False
        return pending

    # ---- the bus side ---------------------------------------------------

    def handle(self, frame):
        """(Status, reply Frame or None) for one BC-to-RT datagram.

        None means nothing is sent back: a broadcast, a parity-flagged frame, an
        invalid command (message error latched) or a command for someone else.
        INVALID_ARGUMENT / BAD_LENGTH are transport faults, not bus messages.
        """
        if frame.type != FrameType.BC_TO_RT:
            return Status.INVALID_ARGUMENT, None
        if frame.word_count < 1:
            return Status.BAD_LENGTH, None
        cw = decode_command_word(frame.words[0])
        if cw.rt != self.rt and not cw.is_broadcast:
            self.counters.ignored += 1
            return Status.OK, None
        self.counters.commands += 1
        self._previous_command = self._last_command
        self._last_command = frame.words[0]
        if frame.flags & FLAG_PARITY_ERROR:
            self._message_error()
            return Status.OK, None
        if cw.is_mode_code:
            return Status.OK, self._handle_mode_code(cw, frame)
        if cw.transmit:
            return Status.OK, self._handle_transmit(cw, frame)
        return Status.OK, self._handle_receive(cw, frame)

    # ---- internals ------------------------------------------------------

    def _reset(self):
        n = MAX_SUBADDRESS + 1
        self._rx = [[0] * MAX_DATA_WORDS for _ in range(n)]
        self._tx = [[0] * MAX_DATA_WORDS for _ in range(n)]
        self._rx_count = [0] * n
        self._rx_valid = [False] * n
        self._rx_fresh = [False] * n
        self._tx_count = [0] * n
        self._previous_command = 0
        self._last_command = 0
        self._sync_word = 0
        self._service_sa = 0
        self._message_error_latched = False
        self._broadcast_received = False
        self._service_requested = False
        self._subsystem_flag = False
        self._terminal_flag = False
        self._terminal_flag_inhibited = False
        self._sync_pending = False
        self._self_test_pending = False
        self.bit = BitWord()

    def _message_error(self):
        self._message_error_latched = True
        self.counters.message_errors += 1

    def _status_word(self, clear_latches):
        sw = StatusWord(self.rt, self._message_error_latched, self._service_requested, self._broadcast_received,
                        False, self._subsystem_flag, self._terminal_flag and not self._terminal_flag_inhibited)
        _, word = encode_status_word(sw)
        if clear_latches:
            self._message_error_latched = False
            self._broadcast_received = False
        return word

    def _reply(self, frame, clear_latches=True):
        return Frame(FrameType.RT_TO_BC, 0, frame.transaction, [self._status_word(clear_latches)])

    def _handle_receive(self, cw, frame):
        if frame.word_count != 1 + cw.count_or_mode:
            self._message_error()
            return None
        data = frame.words[1:1 + cw.count_or_mode]
        self._rx[cw.subaddress] = list(data) + [0] * (MAX_DATA_WORDS - len(data))
        self._rx_count[cw.subaddress] = cw.count_or_mode
        self._rx_valid[cw.subaddress] = True
        self._rx_fresh[cw.subaddress] = True
        self.counters.receives += 1
        if cw.is_broadcast:
            self._broadcast_received = True
            self.counters.broadcasts += 1
            return None
        return self._reply(frame)

    def _handle_transmit(self, cw, frame):
        if cw.is_broadcast or frame.word_count != 1:
            self._message_error()
            return None
        reply = self._reply(frame)
        reply.words += self._tx[cw.subaddress][:cw.count_or_mode]
        self.counters.transmits += 1
        self._tx_count[cw.subaddress] += 1
        return reply

    def _handle_mode_code(self, cw, frame):
        rule = _MODE_CODE_RULES.get(cw.count_or_mode)
        has_data = frame.word_count == 2
        if (rule is None or cw.transmit != rule[0] or has_data != rule[1] or frame.word_count > 2
                or (cw.is_broadcast and not rule[3])):
            self._message_error()
            return None
        code = cw.count_or_mode
        if code == MODE_CODE_INITIATE_SELF_TEST:
            self._self_test_pending = True
            self.bit.in_progress = True
        elif code == MODE_CODE_INHIBIT_TERMINAL_FLAG:
            self._terminal_flag_inhibited = True
        elif code == MODE_CODE_OVERRIDE_INHIBIT_TERMINAL_FLAG:
            self._terminal_flag_inhibited = False
        elif code == MODE_CODE_RESET_REMOTE_TERMINAL:
            self._reset()
            self._last_command = frame.words[0]
            self._reset_pending = True
        elif code == MODE_CODE_SYNCHRONIZE_WITH_DATA:
            self._sync_word = frame.words[1]
            self._sync_pending = True
            self.counters.syncs += 1

        if cw.is_broadcast:
            self._broadcast_received = True
            self.counters.broadcasts += 1
            return None

        reply = self._reply(frame, clear_latches=code != MODE_CODE_TRANSMIT_STATUS_WORD)
        if rule[2]:
            if code == MODE_CODE_TRANSMIT_VECTOR_WORD:
                data = self._service_sa
                self._service_requested = False
            elif code == MODE_CODE_TRANSMIT_LAST_COMMAND:
                data = self._previous_command
            else:
                data = encode_bit_word(self.bit)
            reply.words.append(data)
        return reply
