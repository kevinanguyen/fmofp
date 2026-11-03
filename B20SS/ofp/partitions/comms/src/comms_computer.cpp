#include <fmofp/comms/comms_computer.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::comms {

using icd::TerminalState;

namespace {

bool commandable(TerminalState s) noexcept {
    return s == TerminalState::STANDBY || s == TerminalState::NORMAL || s == TerminalState::TEST ||
           s == TerminalState::MAINTENANCE || s == TerminalState::CALIBRATION;
}

template <typename T>
T clamp_to(T v, T lo, T hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

Status CommsComputer::init() noexcept {
    status_ = icd::ComStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    link_ = icd::ComLinkStatus{};
    link_.frequency = kDefaultFrequencyMhz;
    link_.radio_mode = icd::RadioMode::AM;
    link_.volume = kDefaultVolume;
    link_.squelch = kDefaultSquelch;
    link_.satcom_state = icd::SatcomState::DISCONNECTED;
    link_.datalink_state = icd::LinkState::INACTIVE;
    link_.datalink_mode = icd::DatalinkMode::LOS;
    link_.channel = kDefaultChannel;
    cmd_status_ = icd::CmdStatus::NONE;
    have_rf_ = false;
    last_rf_frame_ = 0;
    radio_tx_ = 0;
    satcom_tx_ = 0;
    packets_sent_ = 0;
    packets_received_ = 0;
    self_test_started_ = 0;
    self_test_running_ = false;
    bit_ = bus::BitWord{};
    mode_seq_ = bus::CommandSequence{};
    cmd_seq_ = bus::CommandSequence{};
    fill_status();
    return Status::kOk;
}

void CommsComputer::reset() noexcept { (void)init(); }

void CommsComputer::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

// CommsService._simulate on the plant's numbers: the radio is active above 20 %; the satcom connects when a
// satellite is in view (from DISCONNECTED or ACQUIRING) and holds its numbers; the data link is up while the
// satcom is connected, counting the network's packets and taking its error rate.
void CommsComputer::apply_rf(const icd::PlantRf& rf) noexcept {
    link_.radio_signal = std::isfinite(rf.radio_field) ? rf.radio_field : 0.0;
    link_.radio_active = link_.radio_signal > kRadioActiveAbovePct;
    if (rf.satellite_available) {
        link_.satcom_state = icd::SatcomState::CONNECTED;
        link_.satellite_id = rf.satellite_id;
        link_.satcom_signal = rf.satellite_signal;
        link_.data_rate = rf.satellite_data_rate;
        link_.latency = rf.satellite_latency;
    } else if (link_.satcom_state == icd::SatcomState::CONNECTED) {
        // The Python never dropped a connection; the world's satellite may still leave.
        link_.satcom_state = icd::SatcomState::DISCONNECTED;
        link_.satellite_id = 0;
        link_.satcom_signal = 0.0;
        link_.data_rate = 0.0;
        link_.latency = 0.0;
    }
    if (link_.satcom_state == icd::SatcomState::CONNECTED) {
        link_.datalink_state = icd::LinkState::ACTIVE;
        packets_sent_ += rf.packets_out;
        packets_received_ += rf.packets_in;
        link_.error_rate = rf.link_error_rate;
    } else {
        link_.datalink_state = icd::LinkState::INACTIVE;
    }
    link_.packets_sent = static_cast<std::uint16_t>(packets_sent_ & 0xFFFFU);
    link_.packets_received = static_cast<std::uint16_t>(packets_received_ & 0xFFFFU);
}

void CommsComputer::step(const icd::ComRf* rf, std::uint32_t frame) noexcept {
    if (rf != nullptr) {
        apply_rf(rf->payload);
        have_rf_ = true;
        last_rf_frame_ = frame;
    }
    const bool rf_fresh = have_rf_ && frame - last_rf_frame_ < kRfStaleFrames;

    if (self_test_running_) {
        if (self_test_started_ == 0) {
            self_test_started_ = frame == 0 ? 1 : frame;
        } else if (frame - self_test_started_ >= kSelfTestFrames) {
            self_test_running_ = false;
            self_test_started_ = 0;
            bit_.in_progress = false;
        }
    }

    if (status_.state == TerminalState::INITIALIZING && rf_fresh) {
        status_.state = TerminalState::NORMAL;
    } else if (status_.state == TerminalState::NORMAL && !rf_fresh) {
        status_.state = TerminalState::DEGRADED;
    } else if (status_.state == TerminalState::DEGRADED && rf_fresh) {
        status_.state = TerminalState::NORMAL;
    }
    status_.healthy = !bit_.failed;
    status_.mode_transition = false;
    fill_status();
}

void CommsComputer::command_mode(const icd::ComModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    status_.mode_transition = false;
}

void CommsComputer::command(const icd::ComCmd& cmd) noexcept {
    if (!cmd_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    cmd_status_ = icd::CmdStatus::REJECTED_INVALID;
    switch (cmd.command) {
        case icd::CommsCommand::SET_RADIO_FREQUENCY:
            // set_radio_frequency: round(float(freq), 3); the bus carries kHz steps already.
            if (std::isfinite(cmd.frequency) && cmd.frequency > 0.0 && cmd.frequency <= kMaxFrequencyMhz) {
                link_.frequency = std::round(cmd.frequency * 1000.0) / 1000.0;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            }
            break;
        case icd::CommsCommand::SET_RADIO_MODE:
            if (icd::is_valid(cmd.radio_mode)) {
                link_.radio_mode = cmd.radio_mode;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            }
            break;
        case icd::CommsCommand::SET_RADIO_VOLUME:
            link_.volume = clamp_to<std::uint8_t>(static_cast<std::uint8_t>(cmd.volume), 0U, kMaxVolume);
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        case icd::CommsCommand::SET_RADIO_SQUELCH:
            link_.squelch = clamp_to<std::uint8_t>(static_cast<std::uint8_t>(cmd.squelch), 0U, kMaxSquelch);
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        case icd::CommsCommand::RADIO_TRANSMIT:
            // transmit_radio: only with a signal.
            if (link_.radio_active) {
                ++radio_tx_;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            } else {
                cmd_status_ = icd::CmdStatus::FAILED;
            }
            break;
        case icd::CommsCommand::ACQUIRE_SATELLITE:
            // acquire_satellite: 'acquiring' unless already connected.
            if (link_.satcom_state != icd::SatcomState::CONNECTED) {
                link_.satcom_state = icd::SatcomState::ACQUIRING;
            }
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        case icd::CommsCommand::SATCOM_SEND:
            if (link_.satcom_state == icd::SatcomState::CONNECTED) {
                ++satcom_tx_;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            } else {
                cmd_status_ = icd::CmdStatus::FAILED;
            }
            break;
        case icd::CommsCommand::SET_DATALINK_MODE:
            if (icd::is_valid(cmd.datalink_mode)) {
                link_.datalink_mode = cmd.datalink_mode;
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            }
            break;
        case icd::CommsCommand::SET_DATALINK_CHANNEL:
            link_.channel = clamp_to<std::uint8_t>(static_cast<std::uint8_t>(cmd.channel), kMinChannel, kMaxChannel);
            cmd_status_ = icd::CmdStatus::ACCEPTED;
            break;
        case icd::CommsCommand::DATALINK_SEND:
            // send_datalink: one packet, only while the link is up.
            if (link_.datalink_state == icd::LinkState::ACTIVE) {
                ++packets_sent_;
                link_.packets_sent = static_cast<std::uint16_t>(packets_sent_ & 0xFFFFU);
                cmd_status_ = icd::CmdStatus::ACCEPTED;
            } else {
                cmd_status_ = icd::CmdStatus::FAILED;
            }
            break;
        case icd::CommsCommand::NONE:
            break;
    }
    fill_status();
}

void CommsComputer::fill_status() noexcept {
    status_.radio_active = link_.radio_active;
    status_.satcom_connected = link_.satcom_state == icd::SatcomState::CONNECTED;
    status_.datalink_active = link_.datalink_state == icd::LinkState::ACTIVE;
    status_.comms_cmd_status = cmd_status_;
    status_.radio_transmissions = radio_tx_;
    status_.satcom_transmissions = satcom_tx_;
}

}  // namespace fmofp::comms
