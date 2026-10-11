// The Phase 1 gate: a bus controller and stub terminals, each a real
// RtProcess on its own socket, exchanging TERM_STATUS in lockstep. The
// terminals run on threads here so the test is one process; fmofp_bc and
// fmofp_rt_stub do the same from separate processes.
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>

#include <fmofp/bc/bus_controller.hpp>
#include <fmofp/bc/icd_schedule.hpp>
#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/transport.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/osal/clock.hpp>
#include <fmofp/osal/udp.hpp>
#include <fmofp/rt/rt_process.hpp>

namespace {

using fmofp::Status;
using fmofp::bc::BcConfig;
using fmofp::bc::BcMode;
using fmofp::bc::BusController;
using fmofp::bc::encode_sync_word;
using fmofp::bc::kMinorFramesPerMajor;
using fmofp::bc::Schedule;
using fmofp::bus::Word;
using fmofp::icd::TermStatus;
using fmofp::osal::kNsPerMs;
using fmofp::osal::Nanoseconds;
using fmofp::osal::now;

constexpr std::uint16_t kPortBase = 52000;

// A stub terminal on its own thread: answers the bus, and on every sync
// refreshes its TERM_STATUS with the sync word it just received.
class StubTerminal {
public:
    // `preload` runs on the fresh terminal before its thread starts, for
    // loading transmit subaddresses other than TERM_STATUS.
    Status start(std::uint8_t rt, void (*preload)(fmofp::bus::RemoteTerminal&) = nullptr) {
        fmofp::rt::RtProcessConfig cfg;
        cfg.rt = rt;
        cfg.port_base = kPortBase;
        const Status s = process_.init(cfg);
        if (s != Status::kOk) {
            return s;
        }
        if (preload != nullptr) {
            preload(process_.terminal());
        }
        thread_ = std::thread([this] { run(); });
        return Status::kOk;
    }

    void stop() {
        stop_ = true;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    ~StubTerminal() { stop(); }

    std::uint32_t syncs() const { return syncs_.load(); }
    // Only valid after stop(): the terminal is then no longer touched by its thread.
    fmofp::rt::RtProcess& process() { return process_; }

private:
    void run() {
        while (!stop_) {
            (void)process_.poll(2 * kNsPerMs);
            Word sync = 0;
            if (process_.terminal().take_sync(sync)) {
                last_sync_ = sync;
                syncs_.fetch_add(1);
                TermStatus status{};
                status.state = fmofp::icd::TerminalState::NORMAL;
                status.healthy = true;
                status.last_sync = sync;
                status.icd_major = 1;
                TermStatus::Words words{};
                if (status.encode(words) == Status::kOk) {
                    (void)process_.terminal().set_transmit(TermStatus::kSubaddress, words.data(), TermStatus::kWords);
                }
            }
        }
    }

    fmofp::rt::RtProcess process_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<std::uint32_t> syncs_{0};
    Word last_sync_{0};
};

BcConfig config(std::initializer_list<std::uint8_t> rts, BcMode mode, bool monitor = false) {
    BcConfig c;
    c.port_base = kPortBase;
    c.mode = mode;
    c.monitor_enabled = monitor;
    c.response_timeout = 20 * kNsPerMs;  // generous: the stubs are threads sharing a CI core
    for (const std::uint8_t rt : rts) {
        c.terminals[c.terminal_count++] = rt;
    }
    return c;
}

Schedule schedule_for(const BcConfig& c) {
    Schedule s;
    EXPECT_EQ(Status::kOk, fmofp::bc::add_icd_messages(s, c.terminals.data(), c.terminal_count));
    EXPECT_EQ(Status::kOk, s.build());
    return s;
}

void wait_for_syncs(const StubTerminal& t, std::uint32_t expected) {
    const Nanoseconds deadline = now() + 500 * kNsPerMs;
    while (t.syncs() < expected && now() < deadline) {
        (void)fmofp::osal::sleep_for(kNsPerMs);
    }
}

// The controller holds ~80 KB of buffers; keep it off the test's stack.
std::unique_ptr<BusController> make_bc() { return std::make_unique<BusController>(); }

TEST(BusControllerGate, TwoTerminalsFollowTheBusControllerFrameForFrame) {
    StubTerminal a;
    StubTerminal b;
    ASSERT_EQ(Status::kOk, a.start(1));
    ASSERT_EQ(Status::kOk, b.start(2));

    const BcConfig cfg = config({1, 2}, BcMode::kFreeRun);
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));

    const std::uint32_t frames = 2 * kMinorFramesPerMajor;  // two major frames
    EXPECT_EQ(Status::kOk, bc->run_frames(frames));
    wait_for_syncs(a, frames);
    wait_for_syncs(b, frames);
    a.stop();
    b.stop();

    EXPECT_EQ(frames, a.syncs());
    EXPECT_EQ(frames, b.syncs());
    EXPECT_EQ(frames, bc->stats().frames);
    for (const std::uint8_t rt : {std::uint8_t{1}, std::uint8_t{2}}) {
        const fmofp::bc::TerminalStats& t = bc->terminal(rt);
        EXPECT_TRUE(t.responded);
        EXPECT_EQ(0U, t.no_response) << "RT " << static_cast<int>(rt);
        EXPECT_EQ(0U, t.malformed);
        EXPECT_EQ(0U, t.message_errors);
        EXPECT_EQ(2U, t.transactions) << "TERM_STATUS is 1 Hz: once per major frame";
    }

    // The last TERM_STATUS poll of RT 1 happened in major frame 1, minor 0,
    // after that frame's sync, so it carries exactly that sync word.
    TermStatus::Words words{};
    std::uint8_t count = 0;
    std::uint32_t frame = 0;
    ASSERT_EQ(Status::kOk, bc->get_latest(1, TermStatus::kSubaddress, words.data(), TermStatus::kWords, count, frame));
    ASSERT_EQ(TermStatus::kWords, count);
    TermStatus status{};
    ASSERT_EQ(Status::kOk, TermStatus::decode(words, status));
    EXPECT_EQ(fmofp::icd::TerminalState::NORMAL, status.state);
    EXPECT_TRUE(status.healthy);
    EXPECT_EQ(encode_sync_word(1, 0), status.last_sync);
    EXPECT_EQ(2U, a.process().terminal().counters().transmits);
    EXPECT_EQ(frames, a.process().terminal().counters().syncs);
    bc->shutdown();
}

TEST(BusControllerGate, RealTimeFramesHoldTwentyMillisecondsWithoutOverruns) {
    StubTerminal a;
    StubTerminal b;
    ASSERT_EQ(Status::kOk, a.start(1));
    ASSERT_EQ(Status::kOk, b.start(2));
    const BcConfig cfg = config({1, 2}, BcMode::kRealTime);
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));

    const std::uint32_t frames = kMinorFramesPerMajor;  // one second
    const Nanoseconds start = now();
    EXPECT_EQ(Status::kOk, bc->run_frames(frames));
    const Nanoseconds elapsed = now() - start;
    wait_for_syncs(a, frames);
    a.stop();
    b.stop();

    EXPECT_EQ(0U, bc->stats().overruns);
    EXPECT_LT(bc->stats().worst_frame_ns, fmofp::bc::kMinorFramePeriodNs);
    // 50 frames of 20 ms: the first runs at once, so 49 periods must have passed.
    EXPECT_GE(elapsed, 49 * fmofp::bc::kMinorFramePeriodNs);
    EXPECT_LT(elapsed, 2 * frames * fmofp::bc::kMinorFramePeriodNs);
    EXPECT_EQ(frames, a.syncs());
    EXPECT_EQ(0U, bc->terminal(1).no_response);
    EXPECT_EQ(0U, bc->terminal(2).no_response);
}

TEST(BusController, AMissingTerminalIsCountedAndTheBusKeepsGoing) {
    StubTerminal a;
    ASSERT_EQ(Status::kOk, a.start(1));
    BcConfig cfg = config({1, 7}, BcMode::kFreeRun);  // nobody is listening at RT 7
    cfg.response_timeout = 2 * kNsPerMs;
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));

    const Status worst = bc->run_frames(kMinorFramesPerMajor + 2);
    EXPECT_EQ(Status::kTimeout, worst) << "the worst outcome of the run is reported";
    a.stop();

    EXPECT_TRUE(bc->terminal(1).responded);
    EXPECT_EQ(0U, bc->terminal(1).no_response);
    EXPECT_FALSE(bc->terminal(7).responded);
    EXPECT_GE(bc->terminal(7).no_response, 1U);
    EXPECT_EQ(bc->terminal(7).transactions, bc->terminal(7).no_response);
    EXPECT_EQ(kMinorFramesPerMajor + 2, bc->stats().frames);
}

TEST(BusController, AQueuedCommandReachesTheTerminalNextFrame) {
    StubTerminal a;
    ASSERT_EQ(Status::kOk, a.start(1));
    const BcConfig cfg = config({1}, BcMode::kFreeRun);
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));

    const std::array<Word, 3> data{0xCAFE, 0x0001, 0xBEEF};
    ASSERT_EQ(Status::kOk, bc->queue_receive(1, 20, data.data(), 3));
    EXPECT_EQ(Status::kOk, bc->run_frames(2));
    EXPECT_EQ(1U, bc->stats().aperiodic_sent);
    wait_for_syncs(a, 2);
    a.stop();

    std::array<Word, 32> got{};
    std::uint8_t count = 0;
    bool fresh = false;
    ASSERT_EQ(Status::kOk, a.process().terminal().get_received(20, got.data(), 32, count, fresh));
    EXPECT_EQ(3, count);
    EXPECT_TRUE(fresh);
    EXPECT_EQ(Word{0xCAFE}, got[0]);
    EXPECT_EQ(Word{0xBEEF}, got[2]);

    // The queue is bounded; overflow is reported, not silently grown.
    for (std::size_t i = 0; i < fmofp::bc::kAperiodicQueueDepth; ++i) {
        ASSERT_EQ(Status::kOk, bc->queue_receive(1, 20, data.data(), 1));
    }
    EXPECT_EQ(Status::kNotReady, bc->queue_receive(1, 20, data.data(), 1));
    EXPECT_EQ(1U, bc->stats().aperiodic_dropped);
}

TEST(BusController, TheMonitorSeesEveryTransaction) {
    ASSERT_EQ(Status::kOk, fmofp::osal::init());
    fmofp::osal::UdpSocket monitor;
    ASSERT_EQ(Status::kOk, monitor.open(static_cast<std::uint16_t>(kPortBase + 33)));

    StubTerminal a;
    ASSERT_EQ(Status::kOk, a.start(1));
    const BcConfig cfg = config({1}, BcMode::kFreeRun, true);
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));
    const std::uint32_t frames = 10;
    EXPECT_EQ(Status::kOk, bc->run_frames(frames));
    a.stop();
    EXPECT_EQ(0U, bc->stats().monitor_failures);

    // 10 syncs plus the one TERM_STATUS poll in minor frame 0.
    std::uint32_t records = 0;
    std::uint32_t syncs = 0;
    std::uint32_t polls = 0;
    for (;;) {
        std::array<std::uint8_t, fmofp::osal::kMaxDatagramBytes> bytes{};
        std::size_t length = 0;
        if (monitor.receive(bytes.data(), bytes.size(), length, 100 * kNsPerMs, nullptr) != Status::kOk) {
            break;
        }
        fmofp::bus::Frame f;
        ASSERT_EQ(Status::kOk, fmofp::bus::decode_frame(bytes.data(), length, f));
        EXPECT_EQ(fmofp::bus::FrameType::kMonitor, f.type);
        ++records;
        if (f.word_count == 2) {
            ++syncs;  // command word + sync data word, no reply to a broadcast
        } else if (f.word_count == 1 + 1 + TermStatus::kWords) {
            ++polls;  // command, then status word + 5 data words
        }
    }
    EXPECT_EQ(frames + 1, records);
    EXPECT_EQ(frames, syncs);
    EXPECT_EQ(1U, polls);
}

void load_plant_sensors(fmofp::bus::RemoteTerminal& terminal) {
    fmofp::icd::PlantSensors s{};
    s.altitude_ssm = fmofp::icd::Ssm::NORMAL;
    s.vertical_speed_ssm = fmofp::icd::Ssm::NORMAL;
    s.altitude = 1500.0;
    s.vertical_speed = -4500.0;
    s.gear = fmofp::icd::GearPosition::UP;
    fmofp::icd::PlantSensors::Words words{};
    ASSERT_EQ(Status::kOk, s.encode(words));
    ASSERT_EQ(Status::kOk, terminal.set_transmit(fmofp::icd::PlantSensors::kSubaddress, words.data(),
                                                 fmofp::icd::PlantSensors::kWords));
}

TEST(BusController, APlantMessageIsRelayedToTheFlightControlComputer) {
    StubTerminal plant;
    StubTerminal fcs;
    ASSERT_EQ(Status::kOk, plant.start(27, load_plant_sensors));
    ASSERT_EQ(Status::kOk, fcs.start(5));
    const BcConfig cfg = config({27, 5}, BcMode::kFreeRun);
    auto bc = make_bc();
    ASSERT_EQ(Status::kOk, bc->init(cfg, schedule_for(cfg)));

    EXPECT_EQ(Status::kOk, bc->run_frames(3));
    wait_for_syncs(fcs, 3);
    plant.stop();
    fcs.stop();
    EXPECT_EQ(3U, bc->stats().relays);
    EXPECT_EQ(0U, bc->terminal(27).no_response);
    EXPECT_EQ(0U, bc->terminal(5).no_response);

    // The FCS terminal holds the plant's words at FCS_SENSORS, decodable as such.
    using fmofp::icd::FcsSensors;
    FcsSensors::Words got{};
    std::uint8_t count = 0;
    bool fresh = false;
    ASSERT_EQ(Status::kOk, fcs.process().terminal().get_received(FcsSensors::kSubaddress, got.data(),
                                                                   FcsSensors::kWords, count, fresh));
    ASSERT_EQ(FcsSensors::kWords, count);
    FcsSensors decoded{};
    ASSERT_EQ(Status::kOk, FcsSensors::decode(got, decoded));
    EXPECT_EQ(fmofp::icd::Ssm::NORMAL, decoded.payload.altitude_ssm);
    EXPECT_DOUBLE_EQ(1500.0, decoded.payload.altitude);
    EXPECT_DOUBLE_EQ(-4500.0, decoded.payload.vertical_speed);
    EXPECT_EQ(fmofp::icd::GearPosition::UP, decoded.payload.gear);
    EXPECT_EQ(3U, fcs.process().terminal().counters().receives);
}

TEST(BusController, RefusesABadConfiguration) {
    auto bc = make_bc();
    Schedule unbuilt;
    BcConfig cfg = config({1}, BcMode::kFreeRun);
    EXPECT_EQ(Status::kInvalidArgument, bc->init(cfg, unbuilt));
    Schedule built;
    ASSERT_EQ(Status::kOk, built.build());
    cfg.terminals[0] = 31;
    EXPECT_EQ(Status::kInvalidArgument, bc->init(cfg, built));
    cfg = config({1}, BcMode::kFreeRun);
    cfg.response_timeout = 0;
    EXPECT_EQ(Status::kInvalidArgument, bc->init(cfg, built));
    EXPECT_EQ(Status::kNotReady, bc->run_minor_frame()) << "not initialised";
}

}  // namespace
