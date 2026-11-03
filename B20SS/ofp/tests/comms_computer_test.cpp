// The communications terminal against CommsService: the radio's activity
// threshold, the satcom's connection on the plant's satellite, the data link
// riding on it, the manual controls and their outcomes, and the partition's state.
#include <gtest/gtest.h>

#include <cstdint>

#include <fmofp/comms/comms_computer.hpp>

namespace {

using fmofp::Status;
using fmofp::comms::CommsComputer;
using fmofp::comms::kRfStaleFrames;
using namespace fmofp::icd;  // NOLINT: test

ComRf rf(double field, bool satellite = false, std::uint8_t packets_in = 0, std::uint8_t packets_out = 0) {
    ComRf r{};
    r.payload.radio_field = field;
    r.payload.satellite_available = satellite;
    if (satellite) {
        r.payload.satellite_id = 7;
        r.payload.satellite_signal = 65.0;
        r.payload.satellite_data_rate = 120.0;
        r.payload.satellite_latency = 350.0;
    }
    r.payload.link_error_rate = 0.3;
    r.payload.packets_in = packets_in;
    r.payload.packets_out = packets_out;
    return r;
}

ComCmd cmd(std::uint8_t seq, CommsCommand command) {
    ComCmd c{};
    c.cmd_seq = seq;
    c.command = command;
    return c;
}

TEST(CommsComputerTest, StartsAsCommsServiceDidAndFollowsTheField) {
    CommsComputer cc;
    ASSERT_EQ(Status::kOk, cc.init());
    EXPECT_EQ(TerminalState::INITIALIZING, cc.state());
    const ComLinkStatus& l = cc.link_status();
    EXPECT_DOUBLE_EQ(118.0, l.frequency);
    EXPECT_EQ(RadioMode::AM, l.radio_mode);
    EXPECT_EQ(50U, l.volume);
    EXPECT_EQ(3U, l.squelch);
    EXPECT_EQ(SatcomState::DISCONNECTED, l.satcom_state);
    EXPECT_EQ(LinkState::INACTIVE, l.datalink_state);
    EXPECT_EQ(DatalinkMode::LOS, l.datalink_mode);
    EXPECT_EQ(1U, l.channel);
    ComRf r = rf(15.0);
    cc.step(&r, 1);
    EXPECT_EQ(TerminalState::NORMAL, cc.state());
    EXPECT_FALSE(cc.status().radio_active) << "15 % is squelched";
    r = rf(20.0);
    cc.step(&r, 51);
    EXPECT_FALSE(cc.status().radio_active) << "active needs more than 20";
    r = rf(20.5);
    cc.step(&r, 101);
    EXPECT_TRUE(cc.status().radio_active);
    EXPECT_DOUBLE_EQ(20.5, cc.link_status().radio_signal);
    // No RF for three periods: DEGRADED at the 150th frame without one; then back.
    std::uint32_t frame = 102;
    for (; frame < 101 + kRfStaleFrames; ++frame) {
        cc.step(nullptr, frame);
    }
    EXPECT_EQ(TerminalState::NORMAL, cc.state()) << "149 frames old is still fresh";
    cc.step(nullptr, frame);
    ++frame;
    EXPECT_EQ(TerminalState::DEGRADED, cc.state());
    cc.step(&r, frame);
    EXPECT_EQ(TerminalState::NORMAL, cc.state());
}

TEST(CommsComputerTest, SatcomConnectsOnTheSatelliteAndTheDataLinkRidesOnIt) {
    CommsComputer cc;
    ASSERT_EQ(Status::kOk, cc.init());
    ComRf r = rf(50.0, false, 2, 1);
    cc.step(&r, 1);
    EXPECT_EQ(SatcomState::DISCONNECTED, cc.link_status().satcom_state);
    EXPECT_EQ(0U, cc.link_status().packets_received) << "no traffic while the link is down";
    cc.command(cmd(1, CommsCommand::ACQUIRE_SATELLITE));
    EXPECT_EQ(CmdStatus::ACCEPTED, cc.status().comms_cmd_status);
    EXPECT_EQ(SatcomState::ACQUIRING, cc.link_status().satcom_state);
    cc.command(cmd(2, CommsCommand::SATCOM_SEND));
    EXPECT_EQ(CmdStatus::FAILED, cc.status().comms_cmd_status) << "not connected";
    cc.command(cmd(3, CommsCommand::DATALINK_SEND));
    EXPECT_EQ(CmdStatus::FAILED, cc.status().comms_cmd_status) << "link inactive";
    r = rf(50.0, true, 2, 1);
    cc.step(&r, 51);
    const ComLinkStatus& l = cc.link_status();
    EXPECT_EQ(SatcomState::CONNECTED, l.satcom_state);
    EXPECT_EQ(7U, l.satellite_id);
    EXPECT_DOUBLE_EQ(65.0, l.satcom_signal);
    EXPECT_DOUBLE_EQ(120.0, l.data_rate);
    EXPECT_DOUBLE_EQ(350.0, l.latency);
    EXPECT_EQ(LinkState::ACTIVE, l.datalink_state);
    EXPECT_EQ(2U, l.packets_received);
    EXPECT_EQ(1U, l.packets_sent);
    EXPECT_DOUBLE_EQ(0.3, l.error_rate);
    EXPECT_TRUE(cc.status().satcom_connected);
    EXPECT_TRUE(cc.status().datalink_active);
    cc.command(cmd(4, CommsCommand::SATCOM_SEND));
    EXPECT_EQ(CmdStatus::ACCEPTED, cc.status().comms_cmd_status);
    EXPECT_EQ(1U, cc.status().satcom_transmissions);
    cc.command(cmd(5, CommsCommand::DATALINK_SEND));
    EXPECT_EQ(CmdStatus::ACCEPTED, cc.status().comms_cmd_status);
    EXPECT_EQ(2U, cc.link_status().packets_sent) << "one more than the background traffic";
    cc.command(cmd(6, CommsCommand::ACQUIRE_SATELLITE));
    EXPECT_EQ(SatcomState::CONNECTED, cc.link_status().satcom_state) << "already connected: unchanged";
    // The satellite drops out of view: disconnected, link down, counters kept.
    r = rf(50.0, false, 2, 1);
    cc.step(&r, 101);
    EXPECT_EQ(SatcomState::DISCONNECTED, cc.link_status().satcom_state);
    EXPECT_EQ(0U, cc.link_status().satellite_id);
    EXPECT_EQ(LinkState::INACTIVE, cc.link_status().datalink_state);
    EXPECT_EQ(2U, cc.link_status().packets_sent);
}

TEST(CommsComputerTest, TheManualControlsClampAndTheRadioTransmitsOnlyWithASignal) {
    CommsComputer cc;
    ASSERT_EQ(Status::kOk, cc.init());
    ComCmd c = cmd(1, CommsCommand::SET_RADIO_FREQUENCY);
    c.frequency = 121.5;
    cc.command(c);
    EXPECT_EQ(CmdStatus::ACCEPTED, cc.status().comms_cmd_status);
    EXPECT_DOUBLE_EQ(121.5, cc.link_status().frequency);
    c = cmd(2, CommsCommand::SET_RADIO_FREQUENCY);
    c.frequency = 0.0;
    cc.command(c);
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, cc.status().comms_cmd_status);
    EXPECT_DOUBLE_EQ(121.5, cc.link_status().frequency);
    c = cmd(3, CommsCommand::SET_RADIO_MODE);
    c.radio_mode = RadioMode::USB;
    cc.command(c);
    EXPECT_EQ(RadioMode::USB, cc.link_status().radio_mode);
    c = cmd(4, CommsCommand::SET_RADIO_VOLUME);
    c.volume = 127;
    cc.command(c);
    EXPECT_EQ(100U, cc.link_status().volume) << "max(0, min(100, volume))";
    c = cmd(5, CommsCommand::SET_RADIO_SQUELCH);
    c.squelch = 15;
    cc.command(c);
    EXPECT_EQ(9U, cc.link_status().squelch);
    c = cmd(6, CommsCommand::SET_DATALINK_CHANNEL);
    c.channel = 0;
    cc.command(c);
    EXPECT_EQ(1U, cc.link_status().channel) << "max(1, min(20, channel))";
    c = cmd(7, CommsCommand::SET_DATALINK_CHANNEL);
    c.channel = 31;
    cc.command(c);
    EXPECT_EQ(20U, cc.link_status().channel);
    c = cmd(8, CommsCommand::SET_DATALINK_MODE);
    c.datalink_mode = DatalinkMode::BLOS;
    cc.command(c);
    EXPECT_EQ(DatalinkMode::BLOS, cc.link_status().datalink_mode);
    cc.command(cmd(9, CommsCommand::RADIO_TRANSMIT));
    EXPECT_EQ(CmdStatus::FAILED, cc.status().comms_cmd_status) << "no signal yet";
    ComRf r = rf(60.0);
    cc.step(&r, 1);
    cc.command(cmd(10, CommsCommand::RADIO_TRANSMIT));
    EXPECT_EQ(CmdStatus::ACCEPTED, cc.status().comms_cmd_status);
    EXPECT_EQ(1U, cc.status().radio_transmissions);
    cc.command(cmd(10, CommsCommand::RADIO_TRANSMIT));  // repeated cmd_seq: ignored
    EXPECT_EQ(1U, cc.status().radio_transmissions);
    cc.command(cmd(11, CommsCommand::NONE));
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, cc.status().comms_cmd_status);
    // Mode command: an uncommandable state is rejected and recorded.
    ComModeCmd mode{};
    mode.cmd_seq = 1;
    mode.requested_state = TerminalState::INITIALIZING;
    cc.command_mode(mode);
    EXPECT_TRUE(cc.status().cmd_rejected);
    mode.cmd_seq = 2;
    mode.requested_state = TerminalState::STANDBY;
    cc.command_mode(mode);
    EXPECT_FALSE(cc.status().cmd_rejected);
    EXPECT_EQ(TerminalState::STANDBY, cc.state());
}

}  // namespace
