#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <initializer_list>

#include <fmofp/bus/command_word.hpp>
#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status_word.hpp>
#include <fmofp/icd/messages.hpp>

namespace {

using fmofp::Status;
using fmofp::bus::BitWord;
using fmofp::bus::CommandSequence;
using fmofp::bus::CommandWord;
using fmofp::bus::decode_bit_word;
using fmofp::bus::decode_status_word;
using fmofp::bus::encode_bit_word;
using fmofp::bus::encode_command_word;
using fmofp::bus::Frame;
using fmofp::bus::FrameType;
using fmofp::bus::kBroadcastRt;
using fmofp::bus::kFlagParityError;
using fmofp::bus::kModeCodeInhibitTerminalFlag;
using fmofp::bus::kModeCodeInitiateSelfTest;
using fmofp::bus::kModeCodeOverrideInhibitTerminalFlag;
using fmofp::bus::kModeCodeResetRemoteTerminal;
using fmofp::bus::kModeCodeSynchronizeWithData;
using fmofp::bus::kModeCodeTransmitBitWord;
using fmofp::bus::kModeCodeTransmitLastCommand;
using fmofp::bus::kModeCodeTransmitStatusWord;
using fmofp::bus::kModeCodeTransmitVectorWord;
using fmofp::bus::RemoteTerminal;
using fmofp::bus::StatusWord;
using fmofp::bus::Word;

constexpr std::uint8_t kRt = 9;

// A BC-to-RT datagram: command word plus optional data words.
Frame command(std::uint8_t rt, bool transmit, std::uint8_t sa, std::uint8_t count_or_mode,
              std::initializer_list<Word> data = {}) {
    CommandWord cw;
    cw.rt = rt;
    cw.transmit = transmit;
    cw.subaddress = sa;
    cw.count_or_mode = count_or_mode;
    Frame f;
    f.type = FrameType::kBcToRt;
    f.transaction = 0x0101;
    EXPECT_EQ(Status::kOk, encode_command_word(cw, f.words[0]));
    std::uint8_t n = 1;
    for (const Word w : data) {
        f.words[n++] = w;
    }
    f.word_count = n;
    return f;
}

Frame mode_code(std::uint8_t rt, bool transmit, std::uint8_t code, std::initializer_list<Word> data = {}) {
    return command(rt, transmit, 31, code, data);
}

StatusWord status_of(const Frame& reply) {
    StatusWord sw;
    EXPECT_EQ(Status::kOk, decode_status_word(reply.words[0], sw));
    return sw;
}

class Rt : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_EQ(Status::kOk, rt.init(kRt)); }

    // Runs one command and returns whether a reply was produced.
    bool run(const Frame& in) {
        reply = Frame{};
        bool needed = false;
        EXPECT_EQ(Status::kOk, rt.handle(in, reply, needed));
        return needed;
    }

    RemoteTerminal rt;
    Frame reply;
};

TEST(RtInit, RejectsTheBroadcastAddress) {
    RemoteTerminal t;
    EXPECT_EQ(Status::kInvalidArgument, t.init(31));
    EXPECT_EQ(Status::kOk, t.init(30));
    EXPECT_EQ(30, t.address());
}

TEST_F(Rt, ReceivesDataAndAnswersWithItsStatusWord) {
    ASSERT_TRUE(run(command(kRt, false, 5, 3, {0x1111, 0x2222, 0x3333})));
    EXPECT_EQ(FrameType::kRtToBc, reply.type);
    EXPECT_EQ(1, reply.word_count);
    EXPECT_EQ(Word{0x0101}, reply.transaction);
    const StatusWord sw = status_of(reply);
    EXPECT_EQ(kRt, sw.rt);
    EXPECT_FALSE(sw.message_error);

    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    ASSERT_EQ(Status::kOk, rt.get_received(5, got.data(), 32, count, fresh));
    EXPECT_EQ(3, count);
    EXPECT_TRUE(fresh);
    EXPECT_EQ(Word{0x1111}, got[0]);
    EXPECT_EQ(Word{0x3333}, got[2]);
    // Reading again without new data: same words, no longer fresh.
    ASSERT_EQ(Status::kOk, rt.get_received(5, got.data(), 32, count, fresh));
    EXPECT_FALSE(fresh);
    EXPECT_EQ(1U, rt.counters().receives);
    EXPECT_EQ(1U, rt.counters().commands);
}

TEST_F(Rt, ReceiveBufferReportsNotReadyAndTooSmall) {
    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    EXPECT_EQ(Status::kNotReady, rt.get_received(7, got.data(), 32, count, fresh));
    ASSERT_TRUE(run(command(kRt, false, 7, 4, {1, 2, 3, 4})));
    EXPECT_EQ(Status::kBadLength, rt.get_received(7, got.data(), 3, count, fresh));
    EXPECT_EQ(Status::kInvalidArgument, rt.get_received(0, got.data(), 32, count, fresh));
    EXPECT_EQ(Status::kInvalidArgument, rt.get_received(31, got.data(), 32, count, fresh));
}

TEST_F(Rt, TransmitsWhatThePartitionLoadedAndZerosOtherwise) {
    const std::array<Word, 5> data{0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
    ASSERT_EQ(Status::kOk, rt.set_transmit(29, data.data(), 5));
    ASSERT_TRUE(run(command(kRt, true, 29, 5)));
    EXPECT_EQ(6, reply.word_count);
    EXPECT_EQ(kRt, status_of(reply).rt);
    for (std::uint8_t i = 0; i < 5; ++i) {
        EXPECT_EQ(data[i], reply.words[1 + i]);
    }
    // An unloaded subaddress transmits zeros, the fail-safe pattern.
    ASSERT_TRUE(run(command(kRt, true, 12, 2)));
    EXPECT_EQ(3, reply.word_count);
    EXPECT_EQ(Word{0}, reply.words[1]);
    EXPECT_EQ(Word{0}, reply.words[2]);
    EXPECT_EQ(2U, rt.counters().transmits);

    EXPECT_EQ(Status::kInvalidArgument, rt.set_transmit(0, data.data(), 1));
    EXPECT_EQ(Status::kInvalidArgument, rt.set_transmit(31, data.data(), 1));
    EXPECT_EQ(Status::kInvalidArgument, rt.set_transmit(5, data.data(), 0));
    EXPECT_EQ(Status::kInvalidArgument, rt.set_transmit(5, data.data(), 33));
    EXPECT_EQ(Status::kInvalidArgument, rt.set_transmit(5, nullptr, 1));
}

TEST_F(Rt, IgnoresCommandsForOtherTerminals) {
    EXPECT_FALSE(run(command(kRt + 1, false, 5, 1, {7})));
    EXPECT_EQ(1U, rt.counters().ignored);
    EXPECT_EQ(0U, rt.counters().commands);
}

TEST_F(Rt, BroadcastGetsNoReplyAndSetsTheBitOnce) {
    EXPECT_FALSE(run(command(kBroadcastRt, false, 5, 1, {0xB0})));
    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    ASSERT_EQ(Status::kOk, rt.get_received(5, got.data(), 32, count, fresh));
    EXPECT_EQ(Word{0xB0}, got[0]);
    EXPECT_EQ(1U, rt.counters().broadcasts);

    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).broadcast_received);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_FALSE(status_of(reply).broadcast_received);
}

TEST_F(Rt, ParityErrorMeansSilenceThenMessageError) {
    Frame bad = command(kRt, false, 5, 1, {1});
    bad.flags = kFlagParityError;
    EXPECT_FALSE(run(bad));
    EXPECT_EQ(1U, rt.counters().message_errors);
    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    EXPECT_EQ(Status::kNotReady, rt.get_received(5, got.data(), 32, count, fresh)) << "bad data is not stored";

    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).message_error);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_FALSE(status_of(reply).message_error) << "the bit clears once reported";
}

TEST_F(Rt, WrongWordCountAndTransmitToBroadcastAreMessageErrors) {
    EXPECT_FALSE(run(command(kRt, false, 5, 4, {1, 2, 3})));  // says 4, carries 3
    EXPECT_FALSE(run(command(kBroadcastRt, true, 5, 1)));     // nobody may answer a broadcast
    Frame extra = command(kRt, true, 5, 1);
    extra.word_count = 2;                                     // transmit command with a data word
    EXPECT_FALSE(run(extra));
    EXPECT_EQ(3U, rt.counters().message_errors);
}

TEST_F(Rt, SynchronizeWithDataDrivesTheFrame) {
    EXPECT_FALSE(run(mode_code(kBroadcastRt, false, kModeCodeSynchronizeWithData, {0x0C07})));
    Word sync = 0;
    EXPECT_TRUE(rt.take_sync(sync));
    EXPECT_EQ(Word{0x0C07}, sync);
    EXPECT_FALSE(rt.take_sync(sync)) << "one frame per sync";
    EXPECT_EQ(1U, rt.counters().syncs);

    // Addressed (non-broadcast) sync is answered.
    EXPECT_TRUE(run(mode_code(kRt, false, kModeCodeSynchronizeWithData, {0x0C08})));
    EXPECT_EQ(1, reply.word_count);
    EXPECT_TRUE(rt.take_sync(sync));
    EXPECT_EQ(Word{0x0C08}, sync);

    // Without its data word it is not a valid command.
    EXPECT_FALSE(run(mode_code(kRt, false, kModeCodeSynchronizeWithData)));
    EXPECT_EQ(1U, rt.counters().message_errors);
}

TEST_F(Rt, TransmitStatusWordRereadsWithoutClearing) {
    Frame bad = command(kRt, false, 5, 1, {1});
    bad.flags = kFlagParityError;
    EXPECT_FALSE(run(bad));
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitStatusWord)));
    EXPECT_EQ(1, reply.word_count);
    EXPECT_TRUE(status_of(reply).message_error);
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitStatusWord)));
    EXPECT_TRUE(status_of(reply).message_error) << "mode code 2 does not consume the latch";
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).message_error);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_FALSE(status_of(reply).message_error);
}

TEST_F(Rt, SelfTestAndBitWord) {
    EXPECT_FALSE(rt.take_self_test_request());
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeInitiateSelfTest)));
    EXPECT_TRUE(rt.take_self_test_request());
    EXPECT_FALSE(rt.take_self_test_request());
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitBitWord)));
    EXPECT_EQ(2, reply.word_count);
    EXPECT_TRUE(decode_bit_word(reply.words[1]).in_progress);

    BitWord done;
    done.in_progress = false;
    done.failed = true;
    done.fault_count = 3;
    rt.set_bit(done);
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitBitWord)));
    EXPECT_EQ(Word{0x4003}, reply.words[1]);

    // Broadcast self test: everyone starts, nobody answers.
    EXPECT_FALSE(run(mode_code(kBroadcastRt, true, kModeCodeInitiateSelfTest)));
    EXPECT_TRUE(rt.take_self_test_request());
}

TEST_F(Rt, TerminalFlagCanBeInhibitedAndRestored) {
    rt.set_terminal_flag(true);
    rt.set_subsystem_flag(true);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).terminal_flag);
    EXPECT_TRUE(status_of(reply).subsystem_flag);
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeInhibitTerminalFlag)));
    EXPECT_FALSE(status_of(reply).terminal_flag);
    EXPECT_TRUE(status_of(reply).subsystem_flag) << "inhibit touches only the terminal flag";
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeOverrideInhibitTerminalFlag)));
    EXPECT_TRUE(status_of(reply).terminal_flag);
}

TEST_F(Rt, ResetClearsEverythingAndTellsThePartitionOnce) {
    ASSERT_TRUE(run(command(kRt, false, 5, 1, {0x55})));
    rt.set_terminal_flag(true);
    rt.request_service(5);
    EXPECT_FALSE(rt.take_reset_request());
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeResetRemoteTerminal)));
    const StatusWord sw = status_of(reply);
    EXPECT_FALSE(sw.terminal_flag);
    EXPECT_FALSE(sw.service_request);
    EXPECT_TRUE(rt.take_reset_request());
    EXPECT_FALSE(rt.take_reset_request());
    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    EXPECT_EQ(Status::kNotReady, rt.get_received(5, got.data(), 32, count, fresh));
    EXPECT_EQ(kRt, rt.address());
}

TEST_F(Rt, ServiceRequestAndVectorWord) {
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_FALSE(status_of(reply).service_request);
    rt.request_service(12);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).service_request);
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitVectorWord)));
    EXPECT_EQ(2, reply.word_count);
    EXPECT_TRUE(status_of(reply).service_request) << "still requested in the status that carries the vector";
    EXPECT_EQ(Word{12}, reply.words[1]);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_FALSE(status_of(reply).service_request) << "satisfied once the vector word went out";
}

TEST_F(Rt, TransmitLastCommandReturnsThePreviousOne) {
    const Frame first = command(kRt, false, 5, 1, {1});
    ASSERT_TRUE(run(first));
    ASSERT_TRUE(run(mode_code(kRt, true, kModeCodeTransmitLastCommand)));
    EXPECT_EQ(2, reply.word_count);
    EXPECT_EQ(first.words[0], reply.words[1]);
}

TEST_F(Rt, UnknownOrMisdirectedModeCodesAreMessageErrors) {
    EXPECT_FALSE(run(mode_code(kRt, true, 5)));                               // not implemented
    EXPECT_FALSE(run(mode_code(kRt, false, kModeCodeTransmitStatusWord)));     // wrong direction
    EXPECT_FALSE(run(mode_code(kRt, true, kModeCodeTransmitBitWord, {1})));   // data where none belongs
    EXPECT_FALSE(run(mode_code(kBroadcastRt, true, kModeCodeTransmitVectorWord)));  // cannot broadcast
    EXPECT_EQ(4U, rt.counters().message_errors);
    ASSERT_TRUE(run(command(kRt, true, 5, 1)));
    EXPECT_TRUE(status_of(reply).message_error);
}

TEST_F(Rt, TransportFaultsAreReportedNotAnswered) {
    Frame wrong_type = command(kRt, false, 5, 1, {1});
    wrong_type.type = FrameType::kRtToBc;
    bool needed = true;
    EXPECT_EQ(Status::kInvalidArgument, rt.handle(wrong_type, reply, needed));
    EXPECT_FALSE(needed);
    Frame empty;
    empty.type = FrameType::kBcToRt;
    empty.word_count = 0;
    needed = true;
    EXPECT_EQ(Status::kBadLength, rt.handle(empty, reply, needed));
    EXPECT_FALSE(needed);
    EXPECT_EQ(0U, rt.counters().commands);
}

TEST_F(Rt, CarriesAGeneratedMessageEndToEnd) {
    // The partition fills TERM_STATUS and loads it at its subaddress; the bus
    // controller asks for it; the words come back and decode.
    using fmofp::icd::TermStatus;
    TermStatus status{};
    status.state = fmofp::icd::TerminalState::NORMAL;
    status.healthy = true;
    status.last_cmd_seq = 0x2A;
    status.icd_major = 1;
    TermStatus::Words words{};
    ASSERT_EQ(Status::kOk, status.encode(words));
    ASSERT_EQ(Status::kOk, rt.set_transmit(TermStatus::kSubaddress, words.data(), TermStatus::kWords));

    ASSERT_TRUE(run(command(kRt, true, TermStatus::kSubaddress, TermStatus::kWords)));
    ASSERT_EQ(1 + TermStatus::kWords, reply.word_count);
    TermStatus::Words received{};
    for (std::uint8_t i = 0; i < TermStatus::kWords; ++i) {
        received[i] = reply.words[1 + i];
    }
    TermStatus decoded{};
    ASSERT_EQ(Status::kOk, TermStatus::decode(received, decoded));
    EXPECT_EQ(fmofp::icd::TerminalState::NORMAL, decoded.state);
    EXPECT_TRUE(decoded.healthy);
    EXPECT_EQ(0x2A, decoded.last_cmd_seq);
}

TEST(CommandSequenceRule, ActsOnlyOnAChangedNumber) {
    CommandSequence seq;
    EXPECT_FALSE(seq.accept(0)) << "0 is no command: an unloaded subaddress reads as zeros";
    EXPECT_TRUE(seq.accept(1));
    EXPECT_FALSE(seq.accept(1));
    EXPECT_TRUE(seq.accept(2));
    EXPECT_FALSE(seq.accept(0)) << "still no command, and it does not disturb the sequence";
    EXPECT_FALSE(seq.accept(2));
    EXPECT_TRUE(seq.accept(255));
    EXPECT_TRUE(seq.accept(1)) << "wrap-around is just another change";
}

TEST(BitWordCodec, RoundTrips) {
    for (const bool in_progress : {false, true}) {
        for (const bool failed : {false, true}) {
            BitWord b;
            b.in_progress = in_progress;
            b.failed = failed;
            b.fault_count = 200;
            const BitWord back = decode_bit_word(encode_bit_word(b));
            EXPECT_EQ(in_progress, back.in_progress);
            EXPECT_EQ(failed, back.failed);
            EXPECT_EQ(200, back.fault_count);
        }
    }
    EXPECT_EQ(Word{0x80FF}, encode_bit_word(BitWord{true, false, 255}));
}

}  // namespace
