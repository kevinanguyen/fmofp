#ifndef FMOFP_BC_BUS_CONTROLLER_HPP
#define FMOFP_BC_BUS_CONTROLLER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bc/schedule.hpp>
#include <fmofp/bus/command_word.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/bus/transport.hpp>
#include <fmofp/bus/word.hpp>
#include <fmofp/osal/clock.hpp>
#include <fmofp/osal/udp.hpp>

// The bus controller: the one process that starts every transfer on the bus
// and the master clock of the whole system (ICD README sections 6 and 7).
// Each minor frame it broadcasts SYNCHRONIZE_WITH_DATA, runs the frame's
// scheduled transactions, then any queued aperiodic commands, and sleeps
// until the next frame's absolute deadline.

namespace fmofp::bc {

enum class BcMode : std::uint8_t {
    kRealTime,  // one minor frame every 20 ms, timed against absolute deadlines
    kFreeRun,   // frames back to back, as fast as the terminals answer (tests)
};

constexpr std::size_t kMaxTerminals = 31;
constexpr std::size_t kAperiodicQueueDepth = 16;

struct BcConfig {
    std::uint16_t port_base{51000};
    std::uint8_t bc_port_offset{32};
    std::uint8_t monitor_port_offset{33};
    bool monitor_enabled{true};
    osal::Nanoseconds response_timeout{5 * osal::kNsPerMs};
    BcMode mode{BcMode::kRealTime};
    std::array<std::uint8_t, kMaxTerminals> terminals{};
    std::uint8_t terminal_count{0};
};

struct TerminalStats {
    std::uint32_t transactions{0};
    std::uint32_t no_response{0};
    std::uint32_t message_errors{0};  // the terminal's status word said so
    std::uint32_t malformed{0};       // a reply that did not fit the command
    std::uint32_t busy{0};
    bool responded{false};  // at least once since init
};

struct BcStats {
    std::uint32_t frames{0};
    std::uint32_t overruns{0};  // a frame's work ran past the next deadline
    std::uint32_t relays{0};    // RT-to-RT transfers performed
    std::uint32_t aperiodic_sent{0};
    std::uint32_t aperiodic_dropped{0};
    std::uint32_t monitor_failures{0};
    osal::Nanoseconds worst_frame_ns{0};
};

// The most recent data a terminal transmitted on a subaddress.
struct LatestData {
    std::array<bus::Word, bus::kMaxDataWords> words{};
    std::uint8_t count{0};
    std::uint32_t frame{0};  // stats().frames when it arrived
    bool valid{false};
};

class BusController {
public:
    // Copies the schedule (it must be built) and opens the BC socket.
    [[nodiscard]] Status init(const BcConfig& cfg, const Schedule& schedule) noexcept;
    void shutdown() noexcept;

    // One minor frame. In real-time mode it first waits for the frame's
    // deadline; the first call sets the timeline's origin.
    [[nodiscard]] Status run_minor_frame() noexcept;
    [[nodiscard]] Status run_frames(std::uint32_t count) noexcept;

    // Queues a receive transaction (data to a terminal) for the next frame's
    // aperiodic slot. kNotReady when the queue is full.
    [[nodiscard]] Status queue_receive(std::uint8_t rt, std::uint8_t sa, const bus::Word* words,
                                       std::uint8_t count) noexcept;

    // Last data received from rt/sa. kNotReady if nothing has arrived yet.
    [[nodiscard]] Status get_latest(std::uint8_t rt, std::uint8_t sa, bus::Word* out,
                                    std::uint8_t capacity, std::uint8_t& count,
                                    std::uint32_t& frame) const noexcept;

    // One transaction, start to finish: send the command (and data), wait for
    // the reply unless `expect_reply` is false (broadcasts), validate it,
    // record the statistics and copy it to the monitor. Exposed for the
    // schedule runner, start-up probes and tests.
    [[nodiscard]] Status transact(std::uint8_t rt, bool transmit, std::uint8_t sa,
                                  std::uint8_t count_or_mode, const bus::Word* data,
                                  std::uint8_t data_count, bool expect_reply,
                                  bus::Frame& reply) noexcept;

    [[nodiscard]] const TerminalStats& terminal(std::uint8_t rt) const noexcept;
    [[nodiscard]] const BcStats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::uint16_t major_frame() const noexcept { return major_; }
    [[nodiscard]] std::uint8_t minor_frame() const noexcept { return minor_; }
    [[nodiscard]] const BcConfig& config() const noexcept { return config_; }

private:
    struct Aperiodic {
        std::uint8_t rt{0};
        std::uint8_t sa{0};
        std::uint8_t count{0};
        std::array<bus::Word, bus::kMaxDataWords> words{};
    };

    [[nodiscard]] Status send_sync() noexcept;
    [[nodiscard]] Status await_reply(bus::Word transaction, bus::Frame& reply) noexcept;
    void record_transmit_data(std::uint8_t rt, std::uint8_t sa, const bus::Frame& reply) noexcept;
    void monitor(const bus::Frame& command, const bus::Frame* reply) noexcept;

    BcConfig config_{};
    Schedule schedule_{};
    osal::UdpSocket socket_{};
    bus::Word transaction_{0};
    std::uint16_t major_{0};
    std::uint8_t minor_{0};
    osal::Nanoseconds next_deadline_{0};
    bool started_{false};
    std::array<TerminalStats, bus::kBroadcastRt + 1> terminals_{};
    BcStats stats_{};
    std::array<std::array<LatestData, bus::kMaxSubaddress + 1>, bus::kBroadcastRt> latest_{};
    std::array<Aperiodic, kAperiodicQueueDepth> queue_{};
    std::uint8_t queued_{0};
};

}  // namespace fmofp::bc

#endif
