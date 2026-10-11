#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <fmofp/radar/weather_model.hpp>

namespace {

namespace radar = fmofp::radar;
using fmofp::icd::PlantWeather;
using fmofp::icd::WeatherMode;

radar::Texture no_texture() {
    radar::Texture t;
    t.stratiform_dbz = 0.0;
    t.clutter_dbz = 0.0;
    return t;
}

TEST(WeatherModel, VolumeCoveragePatternsAreThePythons) {
    const radar::Vcp s = radar::vcp_for(WeatherMode::SURVEILLANCE);
    EXPECT_EQ(4U, s.count);
    EXPECT_DOUBLE_EQ(0.5, s.elevations[0]);
    EXPECT_DOUBLE_EQ(3.4, s.elevations[3]);
    const radar::Vcp m = radar::vcp_for(WeatherMode::MAPPING);
    EXPECT_EQ(7U, m.count);
    EXPECT_DOUBLE_EQ(9.0, m.elevations[6]);
    EXPECT_EQ(7U, radar::vcp_for(WeatherMode::PRECIPITATION).count);
    EXPECT_EQ(1U, radar::vcp_for(WeatherMode::WINDSHEAR).count);
    EXPECT_EQ(1U, radar::vcp_for(WeatherMode::TURBULENCE).count);
    EXPECT_EQ(0U, radar::vcp_for(WeatherMode::NONE).count);
}

TEST(WeatherModel, VerticalDevelopmentProfiles) {
    // Initial cells: 1 - 0.1 i below elevation 4, then max(0.2, 1 - 0.2 i).
    EXPECT_DOUBLE_EQ(1.0, radar::vertical_development(false, 0));
    EXPECT_DOUBLE_EQ(0.7, radar::vertical_development(false, 3));
    EXPECT_DOUBLE_EQ(0.2, radar::vertical_development(false, 4));
    EXPECT_DOUBLE_EQ(0.2, radar::vertical_development(false, 9));
    // Cells that entered at the edge: max(0.2, 1 - 0.15 i).
    EXPECT_DOUBLE_EQ(0.85, radar::vertical_development(true, 1));
    EXPECT_DOUBLE_EQ(0.4, radar::vertical_development(true, 4));
    EXPECT_DOUBLE_EQ(0.2, radar::vertical_development(true, 6));
    EXPECT_DOUBLE_EQ(0.1, radar::vertical_development(false, 10)) << "dict.get(el, 0.1)";
}

TEST(WeatherModel, TextureIsRepeatableAndUnit) {
    double sum = 0.0;
    for (std::uint32_t i = 0; i < 1000; ++i) {
        const double u = radar::texture_unit(7, i, 1, 2);
        EXPECT_GE(u, 0.0);
        EXPECT_LT(u, 1.0);
        EXPECT_DOUBLE_EQ(u, radar::texture_unit(7, i, 1, 2));
        sum += u;
    }
    EXPECT_NEAR(0.5, sum / 1000.0, 0.05);
    EXPECT_NE(radar::texture_unit(7, 1, 1, 2), radar::texture_unit(8, 1, 1, 2)) << "the seed matters";
}

TEST(WeatherModel, TruthFromTheMessage) {
    PlantWeather m{};
    m.cell_count = 2;
    m.cell_1_azimuth = 90.0;
    m.cell_1_range = 50.0;
    m.cell_1_size = 10.0;
    m.cell_1_intensity = 60.0;
    m.cell_2_azimuth = 180.0;
    m.cell_2_range = 120.0;
    m.cell_2_size = 7.5;
    m.cell_2_intensity = 45.5;
    m.cell_2_entry_profile = true;
    m.area_count = 1;
    m.area_azimuth = 300.0;
    m.area_range = 60.0;
    m.area_width = 90.0;
    m.area_depth = 40.0;
    m.area_intensity = 25.0;
    const radar::WeatherTruth t = radar::truth_from(m);
    ASSERT_EQ(2U, t.cell_count);
    EXPECT_DOUBLE_EQ(90.0, t.cells[0].azimuth_deg);
    EXPECT_DOUBLE_EQ(60.0, t.cells[0].intensity_dbz);
    EXPECT_FALSE(t.cells[0].entry_profile);
    EXPECT_TRUE(t.cells[1].entry_profile);
    EXPECT_DOUBLE_EQ(7.5, t.cells[1].size_km);
    EXPECT_TRUE(t.has_area);
    EXPECT_DOUBLE_EQ(40.0, t.area.depth_km);
    m.cell_count = 15;
    EXPECT_EQ(radar::kMaxTruthCells, radar::truth_from(m).cell_count);
}

class Render : public ::testing::Test {
protected:
    std::unique_ptr<radar::Volume> volume = std::make_unique<radar::Volume>();
    radar::Geometry geometry{};
    radar::Vcp surveillance = radar::vcp_for(WeatherMode::SURVEILLANCE);

    void render_all(const radar::WeatherTruth& truth, const radar::Texture& texture) {
        for (std::size_t az = 0; az < radar::kAzimuths; ++az) {
            radar::render_azimuth(truth, surveillance, geometry, texture, az, *volume);
        }
    }
};

TEST_F(Render, OneStormCell) {
    radar::WeatherTruth t;
    t.cell_count = 1;
    t.cells[0] = radar::TruthCell{90.0, 50.0, 10.0, 60.0, false};
    render_all(t, no_texture());
    const auto& v = volume->dbz;
    EXPECT_DOUBLE_EQ(60.0, v[90][0][50]) << "the peak";
    EXPECT_DOUBLE_EQ(54.0, v[90][1][50]) << "0.9 at the second elevation";
    EXPECT_DOUBLE_EQ(42.0, v[90][3][50]) << "0.7 at the fourth";
    // Two degrees off and five km out: az_factor 1 - 2/5, r_factor 1 - 5/10.
    EXPECT_DOUBLE_EQ(60.0 * 0.6 * 0.5, v[92][0][55]);
    EXPECT_DOUBLE_EQ(0.0, v[95][0][50]) << "az_factor reaches zero at size/2 degrees";
    EXPECT_DOUBLE_EQ(0.0, v[90][0][60]) << "r_factor reaches zero at size km";
    EXPECT_DOUBLE_EQ(0.0, v[90][4][50]) << "elevations beyond the pattern stay zero";
    EXPECT_DOUBLE_EQ(10.0, v[0][0][3]) << "ground clutter floor without texture";
    EXPECT_DOUBLE_EQ(10.0, v[200][1][9]);
    EXPECT_DOUBLE_EQ(0.0, v[200][2][9]) << "clutter only in the lowest two elevations";
    EXPECT_DOUBLE_EQ(0.0, v[200][0][10]) << "and only in the first ten km";
}

TEST_F(Render, AzimuthWrapsAndIntensityClips) {
    radar::WeatherTruth t;
    t.cell_count = 1;
    t.cells[0] = radar::TruthCell{359.0, 100.0, 10.0, 80.0, false};
    render_all(t, no_texture());
    const auto& v = volume->dbz;
    EXPECT_DOUBLE_EQ(65.0, v[359][0][100]) << "clipped to 65 dBZ";
    EXPECT_DOUBLE_EQ(80.0 * (1.0 - 2.0 / 5.0), v[1][0][100]) << "azimuth 1 is two degrees from 359";
}

TEST_F(Render, StratiformArea) {
    radar::WeatherTruth t;
    t.has_area = true;
    t.area = radar::TruthArea{180.0, 50.0, 60.0, 40.0, 30.0};
    render_all(t, no_texture());
    const auto& v = volume->dbz;
    EXPECT_DOUBLE_EQ(30.0, v[180][0][50]) << "full intensity at the centre";
    EXPECT_DOUBLE_EQ(30.0, v[180][2][50]) << "the lowest three elevations";
    EXPECT_DOUBLE_EQ(0.0, v[180][3][50]);
    // 29 degrees off centre with a half width of 30: factor 1 - 29/30.
    EXPECT_NEAR(30.0 * (1.0 - 29.0 / 30.0), v[209][0][50], 1e-12);
    EXPECT_DOUBLE_EQ(0.0, v[210][0][50]) << "the azimuth span is half-open";
    EXPECT_NEAR(30.0 * (1.0 - std::sqrt(0.25 + 0.25)), v[195][0][60], 1e-12) << "15 of 30 degrees, 10 of 20 km";
    EXPECT_DOUBLE_EQ(0.0, v[180][0][71]) << "beyond the depth";
}

TEST_F(Render, TextureAndRangeGate) {
    radar::WeatherTruth t;
    t.cell_count = 1;
    t.cells[0] = radar::TruthCell{90.0, 150.0, 10.0, 60.0, false};
    radar::Texture texture;
    render_all(t, texture);
    const auto& v = volume->dbz;
    EXPECT_GE(v[0][0][3], 10.0);
    EXPECT_LE(v[0][0][3], 15.0) << "clutter is 10 + 5 u";
    EXPECT_DOUBLE_EQ(60.0, v[90][0][150]) << "cells get no texture";
    geometry.range_bins = 100;
    render_all(t, texture);
    EXPECT_DOUBLE_EQ(0.0, v[90][0][150]) << "beyond the instrumented range nothing is scanned";
    EXPECT_GE(v[0][0][3], 10.0);
}

}  // namespace
