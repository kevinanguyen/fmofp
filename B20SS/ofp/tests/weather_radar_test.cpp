#include <gtest/gtest.h>

#include <array>
#include <memory>

#include <fmofp/bus/block.hpp>
#include <fmofp/radar/weather_radar.hpp>

namespace {

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using fmofp::Status;
using icd::TerminalState;
using icd::WeatherMode;

// A broad 60 dBZ cell, a small sharp 65 dBZ cell whose edges are steep enough
// to count as wind shear (32 dBZ per degree), and a stratiform area.
icd::RdrWxWeather one_cell() {
    icd::RdrWxWeather w{};
    w.payload.weather_seq = 1;
    w.payload.cell_count = 2;
    w.payload.cell_1_azimuth = 90.0;
    w.payload.cell_1_range = 60.0;
    w.payload.cell_1_size = 15.0;
    w.payload.cell_1_intensity = 60.0;
    w.payload.cell_2_azimuth = 200.0;
    w.payload.cell_2_range = 100.0;
    w.payload.cell_2_size = 4.0;
    w.payload.cell_2_intensity = 65.0;
    w.payload.area_count = 1;
    w.payload.area_azimuth = 270.0;
    w.payload.area_range = 80.0;
    w.payload.area_width = 90.0;
    w.payload.area_depth = 60.0;
    w.payload.area_intensity = 28.0;
    return w;
}

icd::RdrWxModeCmd command(std::uint8_t seq, TerminalState state, WeatherMode mode, double range_km = 200.0) {
    icd::RdrWxModeCmd c{};
    c.cmd_seq = seq;
    c.requested_state = state;
    c.requested_mode = mode;
    c.range_km = range_km;
    c.tilt = 0.0;
    return c;
}

class WeatherRadarTest : public ::testing::Test {
protected:
    void SetUp() override { ASSERT_EQ(Status::kOk, radar->init()); }

    // Runs `frames` frames with the weather delivered on the first.
    void run(std::uint32_t frames, bool with_weather = true) {
        for (std::uint32_t i = 0; i < frames; ++i) {
            ++frame;
            const icd::RdrWxWeather w = one_cell();
            radar->step(with_weather && i == 0 ? &w : nullptr, frame);
        }
    }

    std::unique_ptr<radar::WeatherRadar> radar = std::make_unique<radar::WeatherRadar>();
    std::uint32_t frame{0};
};

TEST_F(WeatherRadarTest, ReadyWithNoModeAndAnEmptyBlock) {
    EXPECT_TRUE(radar->take_products_update()) << "the empty block is published first";
    run(1);
    EXPECT_EQ(TerminalState::NORMAL, radar->state());
    EXPECT_EQ(WeatherMode::NONE, radar->mode());
    EXPECT_FALSE(radar->scanning());
    EXPECT_FALSE(radar->take_products_update());
    EXPECT_EQ(0U, radar->products().vil_count);
    EXPECT_EQ(WeatherMode::NONE, radar->products().mode);
    EXPECT_TRUE(radar->status().healthy);
}

TEST_F(WeatherRadarTest, SurveillanceScansInSixtyFrames) {
    (void)radar->take_products_update();
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    EXPECT_EQ(1U, radar->status().last_cmd_seq);
    EXPECT_FALSE(radar->status().cmd_rejected);
    run(59);
    EXPECT_TRUE(radar->scanning());
    EXPECT_TRUE(radar->status().mode_transition) << "no scan in the new mode has completed yet";
    EXPECT_FALSE(radar->take_products_update());
    EXPECT_EQ(0U, radar->scans_completed());
    run(1);
    EXPECT_FALSE(radar->scanning());
    EXPECT_EQ(1U, radar->scans_completed());
    EXPECT_TRUE(radar->take_products_update());
    EXPECT_FALSE(radar->status().mode_transition);
    const icd::RdrWxProducts& p = radar->products();
    EXPECT_EQ(1U, p.scan_id);
    EXPECT_EQ(WeatherMode::SURVEILLANCE, p.mode);
    EXPECT_GT(p.vil_count, 0U);
    EXPECT_LE(p.vil_count, icd::RdrWxProducts::kMaxVil);
    EXPECT_GT(p.precip_count, 0U);
    EXPECT_GE(p.cells_count, 2U) << "both cells are storm cells";
    EXPECT_EQ(0U, p.windshear_count);
    EXPECT_EQ(0U, p.turbulence_count);
    EXPECT_DOUBLE_EQ(65.0, p.max_reflectivity);
    const icd::RdrWxScan& s = radar->scan_summary();
    EXPECT_EQ(1U, s.scan_id);
    EXPECT_EQ(WeatherMode::SURVEILLANCE, s.mode);
    EXPECT_EQ(4U, s.elevations);
    EXPECT_EQ(p.vil_count, s.vil_points);
    EXPECT_EQ(p.cells_count <= s.storm_cells, true);
    EXPECT_EQ(60U, s.scan_frame);
    EXPECT_GT(s.vil_max, 0.0);
    // The products encode into a block that fits.
    std::array<fmofp::bus::Word, icd::RdrWxProducts::kMaxPayloadWords> payload{};
    std::uint16_t length = 0;
    const Status encoded = p.encode(payload.data(), static_cast<std::uint16_t>(payload.size()), length);
    EXPECT_TRUE(encoded == Status::kOk || encoded == Status::kClamped);
    EXPECT_LE(fmofp::bus::block_segment_count(length), 18U);
    icd::RdrWxProducts again{};
    ASSERT_EQ(Status::kOk, icd::RdrWxProducts::decode(payload.data(), length, again));
    EXPECT_EQ(p.vil_count, again.vil_count);
    EXPECT_EQ(p.vil[0].layer_count, again.vil[0].layer_count);
    // The second scan completes sixty frames later, with the same cell tracked.
    run(60, false);
    EXPECT_EQ(2U, radar->scans_completed());
    EXPECT_TRUE(radar->take_products_update());
    EXPECT_EQ(2U, radar->products().scan_id);
    EXPECT_EQ(p.cells[0].cell_id, radar->products().cells[0].cell_id);
}

TEST_F(WeatherRadarTest, AModeChangeRestartsTheScanAndChangesTheProducts) {
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    run(30);
    radar->command_mode(command(2, TerminalState::NORMAL, WeatherMode::WINDSHEAR));
    run(59);
    EXPECT_EQ(0U, radar->scans_completed()) << "the surveillance scan was abandoned";
    run(1);
    EXPECT_EQ(1U, radar->scans_completed());
    EXPECT_EQ(WeatherMode::WINDSHEAR, radar->products().mode);
    EXPECT_EQ(0U, radar->products().vil_count);
    EXPECT_GT(radar->products().windshear_count, 0U) << "the storm cell's edges are shear";
    EXPECT_EQ(1U, radar->scan_summary().elevations);
    radar->command_mode(command(3, TerminalState::NORMAL, WeatherMode::TURBULENCE));
    run(60);
    EXPECT_EQ(WeatherMode::TURBULENCE, radar->products().mode);
    EXPECT_EQ(0U, radar->products().windshear_count);
    radar->command_mode(command(4, TerminalState::NORMAL, WeatherMode::MAPPING));
    run(60);
    EXPECT_EQ(WeatherMode::MAPPING, radar->products().mode);
    EXPECT_GT(radar->products().areas_count, 0U);
    EXPECT_GT(radar->products().precip_count, 0U);
    EXPECT_EQ(0U, radar->products().cells_count);
    EXPECT_EQ(7U, radar->scan_summary().elevations);
}

TEST_F(WeatherRadarTest, StandbyClearsTheProductsAndRejectsBadStates) {
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    run(60);
    (void)radar->take_products_update();
    radar->command_mode(command(2, TerminalState::STANDBY, WeatherMode::SURVEILLANCE));
    run(1);
    EXPECT_EQ(TerminalState::STANDBY, radar->state());
    EXPECT_EQ(WeatherMode::NONE, radar->mode());
    EXPECT_EQ(WeatherMode::NONE, radar->status().mode);
    EXPECT_TRUE(radar->take_products_update());
    EXPECT_EQ(WeatherMode::NONE, radar->products().mode);
    EXPECT_EQ(0U, radar->products().vil_count);
    EXPECT_EQ(WeatherMode::NONE, radar->scan_summary().mode);
    EXPECT_EQ(1U, radar->scan_summary().scan_id) << "the id of the last scan stays";
    run(10);
    EXPECT_FALSE(radar->take_products_update()) << "cleared once, not every frame";
    radar->command_mode(command(3, TerminalState::DEGRADED, WeatherMode::SURVEILLANCE));
    EXPECT_TRUE(radar->status().cmd_rejected);
    EXPECT_EQ(TerminalState::STANDBY, radar->state());
    radar->command_mode(command(3, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    EXPECT_TRUE(radar->status().cmd_rejected) << "a repeated cmd_seq is ignored";
    radar->command_mode(command(4, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    EXPECT_FALSE(radar->status().cmd_rejected);
    run(60);
    EXPECT_EQ(2U, radar->scans_completed());
}

TEST_F(WeatherRadarTest, TheRangeLimitsWhatIsScanned) {
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE, 50.0));
    run(60);
    EXPECT_DOUBLE_EQ(50.0, radar->scan_summary().range_km);
    const icd::RdrWxProducts& p = radar->products();
    EXPECT_GT(p.vil_count, 0U) << "the clutter alone is a VIL return";
    for (std::size_t i = 0; i < p.vil_count; ++i) {
        EXPECT_LT(p.vil[i].range, 50.0);
    }
    EXPECT_LT(p.max_reflectivity, 60.0) << "both cells are beyond the range";
    radar->command_mode(command(2, TerminalState::NORMAL, WeatherMode::SURVEILLANCE, 5000.0));
    run(60);
    EXPECT_DOUBLE_EQ(200.0, radar->scan_summary().range_km) << "limited to the ICD's range";
    EXPECT_DOUBLE_EQ(65.0, radar->products().max_reflectivity);
}

TEST_F(WeatherRadarTest, SelfTestAndReset) {
    radar->start_self_test();
    EXPECT_TRUE(radar->bit().in_progress);
    run(12);
    EXPECT_FALSE(radar->bit().in_progress);
    EXPECT_FALSE(radar->bit().failed);
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    run(60);
    EXPECT_EQ(1U, radar->scans_completed());
    radar->reset();
    EXPECT_EQ(TerminalState::INITIALIZING, radar->state());
    EXPECT_EQ(0U, radar->scans_completed());
    EXPECT_TRUE(radar->take_products_update());
    run(1);
    EXPECT_EQ(TerminalState::NORMAL, radar->state());
    EXPECT_FALSE(radar->scanning());
}

TEST_F(WeatherRadarTest, ScanNowMatchesStepping) {
    radar->command_mode(command(1, TerminalState::NORMAL, WeatherMode::SURVEILLANCE));
    run(60);
    std::array<fmofp::bus::Word, icd::RdrWxProducts::kMaxPayloadWords> stepped{};
    std::uint16_t stepped_length = 0;
    ASSERT_TRUE(fmofp::ok(radar->products().encode(stepped.data(), static_cast<std::uint16_t>(stepped.size()), stepped_length)));

    auto other = std::make_unique<radar::WeatherRadar>();
    ASSERT_EQ(Status::kOk, other->init());
    other->scan_now(radar::truth_from(one_cell().payload), WeatherMode::SURVEILLANCE, 60);
    std::array<fmofp::bus::Word, icd::RdrWxProducts::kMaxPayloadWords> at_once{};
    std::uint16_t at_once_length = 0;
    ASSERT_TRUE(fmofp::ok(other->products().encode(at_once.data(), static_cast<std::uint16_t>(at_once.size()), at_once_length)));
    EXPECT_EQ(stepped_length, at_once_length);
    EXPECT_EQ(stepped, at_once);
}

}  // namespace
