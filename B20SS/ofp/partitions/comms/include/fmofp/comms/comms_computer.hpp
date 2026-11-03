#ifndef FMOFP_COMMS_COMMS_COMPUTER_HPP
#define FMOFP_COMMS_COMMS_COMPUTER_HPP

#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>

// The communications terminal's logic (RT 2), ported from
// Systems/comms/messaging_service.py (CommsService): the radio's settings and
// whether it is active, the satcom's connection and the satellite it holds, the
// data link's state, settings and counters, and the manual controls. What the
// links measure (the field strength, the satellite in view, the link's traffic
// and errors) is the plant's RF environment (PLANT_RF, relayed as COM_RF); the
// Python drew those numbers inside the service. Pure: ICD messages in, ICD
// messages out.

namespace fmofp::comms {

constexpr double kRadioActiveAbovePct = 20.0;  // _simulate: active = signal_strength > 20
constexpr double kDefaultFrequencyMhz = 118.0;
constexpr double kMaxFrequencyMhz = 2000.0;    // the ICD field's range
constexpr std::uint8_t kDefaultVolume = 50;
constexpr std::uint8_t kMaxVolume = 100;       // set_radio_volume: max(0, min(100, volume))
constexpr std::uint8_t kDefaultSquelch = 3;
constexpr std::uint8_t kMaxSquelch = 9;        // set_radio_squelch: max(0, min(9, squelch))
constexpr std::uint8_t kMinChannel = 1;        // set_datalink_channel: max(1, min(20, channel))
constexpr std::uint8_t kMaxChannel = 20;
constexpr std::uint8_t kDefaultChannel = 1;
constexpr std::uint32_t kRfStaleFrames = 150;  // PLANT_RF is 1 Hz: three periods
constexpr std::uint32_t kSelfTestFrames = 10;

class CommsComputer {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame: the RF environment received this frame (COM_RF), or null; the frame count.
    void step(const icd::ComRf* rf, std::uint32_t frame) noexcept;

    // Commands, each applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::ComModeCmd& cmd) noexcept;
    void command(const icd::ComCmd& cmd) noexcept;

    void reset() noexcept;
    void start_self_test() noexcept;

    [[nodiscard]] const icd::ComStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::ComLinkStatus& link_status() const noexcept { return link_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }

private:
    void apply_rf(const icd::PlantRf& rf) noexcept;
    void fill_status() noexcept;

    icd::ComStatus status_{};
    icd::ComLinkStatus link_{};
    icd::CmdStatus cmd_status_{icd::CmdStatus::NONE};
    bool have_rf_{false};
    std::uint32_t last_rf_frame_{0};
    std::uint8_t radio_tx_{0};
    std::uint8_t satcom_tx_{0};
    std::uint32_t packets_sent_{0};
    std::uint32_t packets_received_{0};
    std::uint32_t self_test_started_{0};
    bool self_test_running_{false};
    bus::BitWord bit_{};
    bus::CommandSequence mode_seq_{};
    bus::CommandSequence cmd_seq_{};
};

}  // namespace fmofp::comms

#endif
