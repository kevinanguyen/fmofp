#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <fmofp/radar/weather_products.hpp>

namespace {

namespace radar = fmofp::radar;
using fmofp::icd::PrecipType;
using fmofp::icd::ShearSeverity;
using fmofp::icd::TurbulenceCategory;
using fmofp::icd::WeatherMode;

TEST(WeatherProducts, RoundToSigfigsIsThePythons) {
    // Values from round_to_sigfigs in vil_data_generator_sync.py, run in Python.
    EXPECT_DOUBLE_EQ(1230.0, radar::round_to_sigfigs(1234.5678));
    EXPECT_DOUBLE_EQ(0.00123, radar::round_to_sigfigs(0.0012345));
    EXPECT_DOUBLE_EQ(2.5, radar::round_to_sigfigs(2.5));
    EXPECT_DOUBLE_EQ(1.0, radar::round_to_sigfigs(1.005));
    EXPECT_DOUBLE_EQ(65.0, radar::round_to_sigfigs(65.0));
    EXPECT_DOUBLE_EQ(100.0, radar::round_to_sigfigs(99.95));
    EXPECT_DOUBLE_EQ(123000.0, radar::round_to_sigfigs(123456.0));
    EXPECT_DOUBLE_EQ(4.44, radar::round_to_sigfigs(4.4445));
    EXPECT_DOUBLE_EQ(1e-7, radar::round_to_sigfigs(1e-7));
    EXPECT_DOUBLE_EQ(0.0, radar::round_to_sigfigs(0.0));
    EXPECT_DOUBLE_EQ(-1230.0, radar::round_to_sigfigs(-1234.5678));
    EXPECT_DOUBLE_EQ(1234.57, radar::round_to_sigfigs(1234.5678, 6));
    EXPECT_TRUE(std::isnan(radar::round_to_sigfigs(NAN)));
}

TEST(WeatherProducts, LayerThicknessesAreAHundredMetresForEveryPattern) {
    // The elevation steps are too small for 1000 (sin e2 - sin e1) to exceed 100 m.
    for (const WeatherMode mode : {WeatherMode::SURVEILLANCE, WeatherMode::MAPPING, WeatherMode::WINDSHEAR}) {
        const radar::Vcp vcp = radar::vcp_for(mode);
        const auto t = radar::layer_thicknesses(vcp);
        for (std::size_t i = 0; i < vcp.count; ++i) {
            EXPECT_DOUBLE_EQ(mode == WeatherMode::WINDSHEAR ? 1000.0 : 100.0, t[i]) << "elevation " << i;
        }
    }
}

class Columns : public ::testing::Test {
protected:
    std::unique_ptr<radar::Volume> volume = std::make_unique<radar::Volume>();
    std::array<double, radar::kMaxElevations> hundred{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};

    void set_column(std::size_t az, std::size_t r, std::vector<double> dbz) {
        for (std::size_t el = 0; el < dbz.size(); ++el) {
            volume->dbz[az][el][r] = dbz[el];
        }
    }
};

TEST_F(Columns, VilAndRainRateMatchThePython) {
    set_column(10, 20, {45.0, 40.5, 36.0, 31.5});
    EXPECT_DOUBLE_EQ(260.0, radar::vil_column(*volume, 4, 10, 20, hundred));
    EXPECT_DOUBLE_EQ(23.7, radar::precip_column(*volume, 4, 10, 20));
    EXPECT_DOUBLE_EQ(45.0, radar::composite_column(*volume, 4, 10, 20));
    set_column(11, 20, {10.0, 12.5});
    EXPECT_DOUBLE_EQ(3.06, radar::vil_column(*volume, 2, 11, 20, hundred));
    EXPECT_DOUBLE_EQ(0.22, radar::precip_column(*volume, 2, 11, 20));
    set_column(12, 20, {65.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0});
    EXPECT_DOUBLE_EQ(12500.0, radar::vil_column(*volume, 7, 12, 20, hundred));
    EXPECT_DOUBLE_EQ(421.0, radar::precip_column(*volume, 7, 12, 20));
    set_column(13, 20, {0.0, -5.0});
    EXPECT_DOUBLE_EQ(0.0, radar::vil_column(*volume, 2, 13, 20, hundred)) << "only positive returns count";
    EXPECT_DOUBLE_EQ(0.0, radar::precip_column(*volume, 2, 13, 20));
}

TEST(WeatherProducts, UniformFilterMatchesScipy) {
    // scipy.ndimage.uniform_filter(f, size=3) on a 3 x 4 array, computed in Python.
    const std::array<double, 12> f{{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}};
    std::array<double, 12> out{};
    std::array<double, 12> scratch{};
    radar::uniform_filter3(f.data(), out.data(), 3, 4, scratch.data());
    const std::array<double, 12> expected{{2.6666666666666665, 3.3333333333333335, 4.333333333333333, 5.0,
                                           5.333333333333333, 6.0, 7.0, 7.666666666666667, 8.0, 8.666666666666666,
                                           9.666666666666666, 10.333333333333334}};
    for (std::size_t i = 0; i < 12; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], out[i]) << "index " << i;
    }
    std::array<double, 12> squares{};
    for (std::size_t i = 0; i < 12; ++i) {
        squares[i] = f[i] * f[i];
    }
    radar::uniform_filter3(squares.data(), out.data(), 3, 4, scratch.data());
    const std::array<double, 12> expected_sq{{10.888888888888888, 15.333333333333334, 23.0, 28.777777777777782,
                                              39.333333333333336, 47.333333333333336, 60.333333333333336,
                                              69.66666666666667, 67.77777777777779, 79.33333333333333,
                                              97.66666666666667, 110.55555555555554}};
    for (std::size_t i = 0; i < 12; ++i) {
        EXPECT_DOUBLE_EQ(expected_sq[i], out[i]) << "square index " << i;
    }
}

TEST(WeatherProducts, GradientsAreNumpys) {
    auto f = std::make_unique<radar::Field>();
    for (std::size_t az = 0; az < radar::kAzimuths; ++az) {
        for (std::size_t r = 0; r < radar::kRanges; ++r) {
            (*f)[az][r] = 4.0 * static_cast<double>(az) + static_cast<double>(r);
        }
    }
    EXPECT_DOUBLE_EQ(4.0, radar::gradient_axis0(*f, 0, 5)) << "one-sided at the first row";
    EXPECT_DOUBLE_EQ(4.0, radar::gradient_axis0(*f, 100, 5)) << "central inside";
    EXPECT_DOUBLE_EQ(4.0, radar::gradient_axis0(*f, 359, 5)) << "one-sided at the last row";
    EXPECT_DOUBLE_EQ(1.0, radar::gradient_axis1(*f, 7, 0));
    EXPECT_DOUBLE_EQ(1.0, radar::gradient_axis1(*f, 7, 100));
    EXPECT_DOUBLE_EQ(1.0, radar::gradient_axis1(*f, 7, 199));
}

TEST(WeatherProducts, RegionsAreFourConnectedInRasterOrder) {
    auto f = std::make_unique<radar::Field>();
    auto work = std::make_unique<radar::Labels>();
    // A 2x2 block at (10..11, 20..21) of 40 dBZ, a diagonal neighbour at (12, 22)
    // that touches it only by a corner, and a far 3-bin line at azimuth 50.
    (*f)[10][20] = 40.0;
    (*f)[10][21] = 42.0;
    (*f)[11][20] = 44.0;
    (*f)[11][21] = 46.0;
    (*f)[12][22] = 50.0;
    (*f)[50][100] = 35.0;
    (*f)[50][101] = 35.0;
    (*f)[50][102] = 35.0;
    std::array<radar::Region, 8> regions{};
    const std::size_t n = radar::find_regions(*f, 30.0, *work, regions.data(), regions.size());
    ASSERT_EQ(3U, n);
    EXPECT_DOUBLE_EQ(10.5, regions[0].center_az);
    EXPECT_DOUBLE_EQ(20.5, regions[0].center_r);
    EXPECT_DOUBLE_EQ(43.0, regions[0].mean_dbz);
    EXPECT_EQ(4U, regions[0].pixels);
    EXPECT_DOUBLE_EQ(12.0, regions[1].center_az) << "a corner neighbour is its own region";
    EXPECT_EQ(1U, regions[1].pixels);
    EXPECT_DOUBLE_EQ(101.0, regions[2].center_r);
    EXPECT_EQ(3U, regions[2].pixels);
    EXPECT_EQ(1U, radar::find_regions(*f, 30.0, *work, regions.data(), 1)) << "the capacity holds";
}

class Points : public ::testing::Test {
protected:
    std::unique_ptr<radar::Field> field = std::make_unique<radar::Field>();
    std::unique_ptr<radar::PointWork> work = std::make_unique<radar::PointWork>();
    radar::PointTracker tracker{};
    radar::PointProducts out{};
};

TEST_F(Points, FewPointsComeThroughAsTheyAre) {
    (*field)[80][40] = 5.0;    // downsampled bin (10, 5), sampled by the [::10, ::10] check
    (*field)[160][80] = 7.0;   // bin (20, 10)
    (*field)[8][0] = 100.0;    // bin (1, 0): at the origin, dropped as the Python did
    (*field)[81][40] = 500.0;  // not on the 8x grid: never a point, but the field maximum
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, tracker, *work, out);
    ASSERT_EQ(2U, out.count);
    EXPECT_EQ(3U, out.significant);
    EXPECT_DOUBLE_EQ(500.0, out.max_value);
    EXPECT_EQ(10U, out.points[0].az_index);
    EXPECT_EQ(5U, out.points[0].r_index);
    EXPECT_DOUBLE_EQ(5.0, out.points[0].value);
    EXPECT_DOUBLE_EQ(radar::round_to_sigfigs(5.0 / 65.0), out.points[0].intensity);
    EXPECT_EQ(2U, out.points[0].layer_count) << "int(5/65 * 15) + 1";
    EXPECT_FALSE(out.points[0].show_values);
    EXPECT_EQ(20U, out.points[1].az_index);
    EXPECT_EQ(0U, tracker.count) << "no selection, no tracking";
    // The same field as precipitation.
    radar::generate_points(*field, radar::PointKind::kPrecip, radar::kMaxPrecipPoints, 0, tracker, *work, out);
    ASSERT_EQ(2U, out.count);
    EXPECT_DOUBLE_EQ(0.1, out.points[0].intensity);
    EXPECT_TRUE(out.points[0].type == PrecipType::RAIN || out.points[0].type == PrecipType::SNOW);
}

TEST_F(Points, NothingSignificantGivesNoPoints) {
    (*field)[85][45] = 50.0;  // significant, but not where the quick check samples
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, tracker, *work, out);
    EXPECT_EQ(0U, out.count);
    EXPECT_EQ(0U, out.significant) << "the Python returned before counting";
    EXPECT_DOUBLE_EQ(50.0, out.max_value);
}

TEST_F(Points, ManyPointsAreSelectedAndTheSelectionPersists) {
    // A broad strong area: every downsampled bin in azimuths 40..200, ranges 32..160.
    for (std::size_t az = 40; az <= 200; ++az) {
        for (std::size_t r = 32; r <= 160; ++r) {
            (*field)[az][r] = 100.0 + static_cast<double>(r % 7) * 10.0;
        }
    }
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, tracker, *work, out);
    EXPECT_EQ(radar::kMaxVilPoints, out.count);
    EXPECT_GT(out.significant, 300U);
    EXPECT_EQ(radar::kMaxVilPoints, tracker.count);
    for (std::size_t i = 1; i < out.count; ++i) {
        const bool ascending = out.points[i - 1].az_index < out.points[i].az_index ||
                               (out.points[i - 1].az_index == out.points[i].az_index &&
                                out.points[i - 1].r_index < out.points[i].r_index);
        EXPECT_TRUE(ascending) << "chosen points come in raster order";
    }
    // The second selection continues from the first (the Python matched the
    // new points to the previous selection greedily, so it is not simply the
    // same set), and the whole sequence is repeatable from a fresh tracker.
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, tracker, *work, out);
    ASSERT_EQ(radar::kMaxVilPoints, out.count);
    const std::array<radar::Point, radar::kMaxSelected> second = out.points;
    radar::PointTracker again{};
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, again, *work, out);
    radar::generate_points(*field, radar::PointKind::kVil, radar::kMaxVilPoints, 0, again, *work, out);
    for (std::size_t i = 0; i < out.count; ++i) {
        EXPECT_EQ(second[i].az_index, out.points[i].az_index) << "the same history gives the same points";
        EXPECT_EQ(second[i].r_index, out.points[i].r_index);
    }
    radar::generate_points(*field, radar::PointKind::kPrecip, radar::kMaxPrecipPoints, 0, tracker, *work, out);
    EXPECT_EQ(radar::kMaxPrecipPoints, out.count);
    for (std::size_t i = 0; i < out.count; ++i) {
        EXPECT_EQ(PrecipType::HAIL, out.points[i].type) << "rates above 50 mm/h";
        EXPECT_DOUBLE_EQ(1.0, out.points[i].intensity);
    }
}

class Processors : public ::testing::Test {
protected:
    std::unique_ptr<radar::Volume> volume = std::make_unique<radar::Volume>();
    std::unique_ptr<radar::Field> composite = std::make_unique<radar::Field>();
    std::unique_ptr<radar::Labels> labels = std::make_unique<radar::Labels>();
    std::unique_ptr<radar::FilterWork> filter = std::make_unique<radar::FilterWork>();

    void block(std::size_t az0, std::size_t az1, std::size_t r0, std::size_t r1, double dbz) {
        for (std::size_t az = az0; az <= az1; ++az) {
            for (std::size_t r = r0; r <= r1; ++r) {
                volume->dbz[az][0][r] = dbz;
                (*composite)[az][r] = dbz;
            }
        }
    }
};

TEST_F(Processors, StormCellsAreTrackedAcrossScans) {
    radar::StormCellTracker tracker;
    tracker.init();
    block(100, 103, 50, 53, 45.0);  // 16 bins: 8 nm
    block(200, 200, 150, 150, 31.0);
    tracker.process(*composite, 0.5, 1.2, 1.2, *labels);
    ASSERT_EQ(2U, tracker.size());
    const radar::TrackedCell& c = tracker.cell_at(0);
    EXPECT_EQ(1U, c.id);
    EXPECT_DOUBLE_EQ(101.5, c.center_az);
    EXPECT_DOUBLE_EQ(51.5, c.center_r);
    EXPECT_DOUBLE_EQ(45.0, c.reflectivity);
    EXPECT_DOUBLE_EQ(8.0, c.size);
    EXPECT_NEAR((45.0 - 30.0) / 35.0, c.intensity, 1e-12);
    EXPECT_DOUBLE_EQ(0.0, c.velocity_az) << "a new cell has no velocity";
    const double top = 8.0 * 1852.0 * std::tan(0.5 * radar::kDegToRad);
    EXPECT_NEAR(top, c.altitude, 1e-9);
    EXPECT_NEAR(top / 1.2, c.vertical_development, 1e-9);
    EXPECT_EQ(2U, tracker.cell_at(1).id);
    // Next scan: the first cell moved two bins in range; the second is gone; a third appears.
    for (auto& row : *composite) {
        row.fill(0.0);
    }
    block(100, 103, 52, 55, 47.0);
    block(300, 301, 20, 21, 50.0);
    tracker.process(*composite, 0.5, 2.4, 1.2, *labels);
    ASSERT_EQ(2U, tracker.size());
    EXPECT_EQ(1U, tracker.cell_at(0).id) << "the moved cell keeps its id";
    EXPECT_NEAR(2.0 / 1.2, tracker.cell_at(0).velocity_r, 1e-12);
    EXPECT_DOUBLE_EQ(0.0, tracker.cell_at(0).velocity_az);
    EXPECT_DOUBLE_EQ(47.0, tracker.cell_at(0).reflectivity);
    EXPECT_NEAR(0.0, tracker.cell_at(0).vertical_development, 1e-9) << "same size, same top";
    EXPECT_EQ(3U, tracker.cell_at(1).id) << "ids are never reused";
    EXPECT_DOUBLE_EQ(300.5, tracker.cell_at(1).center_az);
}

TEST_F(Processors, PrecipitationAreasAreClassified) {
    radar::PrecipitationAnalyzer analyzer;
    analyzer.init();
    block(10, 11, 10, 11, 60.0);
    block(20, 21, 10, 11, 50.0);
    block(30, 31, 10, 11, 40.0);
    analyzer.process(*composite, *labels);
    ASSERT_EQ(3U, analyzer.size());
    EXPECT_EQ(PrecipType::HAIL, analyzer.area_at(0).type);
    EXPECT_EQ(PrecipType::HEAVY_RAIN, analyzer.area_at(1).type);
    EXPECT_EQ(PrecipType::RAIN, analyzer.area_at(2).type);
    EXPECT_NEAR(std::pow(std::pow(10.0, 6.0) / 200.0, 1.0 / 1.6), analyzer.area_at(0).rate, 1e-9);
    EXPECT_NEAR(30.0 / 35.0, analyzer.area_at(0).intensity, 1e-12);
    EXPECT_NEAR(10.0 / 35.0, analyzer.area_at(2).intensity, 1e-12);
    EXPECT_DOUBLE_EQ(10.5, analyzer.area_at(1).center_r);
}

TEST_F(Processors, WindShearAtTheEdgeOfAStorm) {
    radar::WindShearProcessor ws;
    ws.init();
    // A 60 dBZ block with sharp azimuth edges at 100 and 120: the gradient is
    // 30 dBZ per bin at the edge rows, 24 kt/nm after the 0.8 scaling.
    block(100, 120, 50, 90, 60.0);
    ws.process(*volume, 0.5);
    ASSERT_GT(ws.size(), 0U);
    for (std::size_t i = 0; i < ws.size(); ++i) {
        const radar::ShearEvent& e = ws.event_at(i);
        EXPECT_TRUE(e.az == 100 || e.az == 120) << "events only on the edge rows";
        EXPECT_DOUBLE_EQ(24.0, e.shear_kts);
        EXPECT_EQ(ShearSeverity::MODERATE, e.severity);
        EXPECT_NEAR((static_cast<double>(e.r) + 1.0) * 1000.0 * std::tan(0.5 * radar::kDegToRad) / 0.3048, e.altitude_ft,
                    1e-9);
    }
    // One event per 5x5 cluster: the edge rows cover ranges 50..90, nine range clusters each.
    EXPECT_EQ(18U, ws.size());
    // Divergence is the range gradient: at the near edge of the block the
    // return rises by 30 dBZ per km (outflow, 1.5), at the far edge it falls,
    // and inside it is flat. The Python called the near edge a microburst.
    EXPECT_TRUE(ws.has_microburst());
    for (std::size_t i = 0; i < ws.size(); ++i) {
        const radar::ShearEvent& e = ws.event_at(i);
        EXPECT_EQ(e.r == 50, e.microburst) << "range bin " << e.r;
        if (e.r == 50) {
            EXPECT_DOUBLE_EQ(1.5, e.divergence);
        } else if (e.r == 90) {
            EXPECT_DOUBLE_EQ(-1.5, e.divergence);
        } else {
            EXPECT_DOUBLE_EQ(0.0, e.divergence);
        }
    }
    // Severity order: MODERATE events (the 60 dBZ edges, 24 kt/nm) come before
    // LOW ones (a 40 dBZ block's edges, 16 kt/nm). SEVERE needs a jump above
    // 75 dBZ between neighbouring bins, which clipped returns cannot make.
    block(200, 210, 50, 60, 40.0);
    ws.process(*volume, 0.5);
    EXPECT_EQ(24U, ws.size());
    EXPECT_EQ(ShearSeverity::MODERATE, ws.event_at(0).severity);
    EXPECT_EQ(ShearSeverity::LOW, ws.event_at(ws.size() - 1).severity);
    EXPECT_DOUBLE_EQ(16.0, ws.event_at(ws.size() - 1).shear_kts);
    for (std::size_t i = 1; i < ws.size(); ++i) {
        EXPECT_GE(static_cast<int>(ws.event_at(i - 1).severity), static_cast<int>(ws.event_at(i).severity));
    }
}

TEST_F(Processors, TurbulenceWhereTheReturnsFluctuate) {
    radar::TurbulenceProcessor turb;
    turb.init();
    block(50, 70, 2, 20, 40.0);  // flat: no spectrum width inside
    for (std::size_t az = 50; az <= 70; ++az) {
        for (std::size_t r = 2; r <= 20; ++r) {
            volume->dbz[az][0][r] = ((az + r) % 2 == 0) ? 50.0 : 30.0;  // a checkerboard: large local variance
        }
    }
    turb.process(*volume, 0.5, *filter);
    ASSERT_GT(turb.size(), 0U);
    EXPECT_NE(TurbulenceCategory::NONE, turb.max_category());
    for (std::size_t i = 1; i < turb.size(); ++i) {
        EXPECT_GE(turb.cell_at(i - 1).intensity, turb.cell_at(i).intensity) << "strongest first";
    }
    const radar::TurbulenceCell& c = turb.cell_at(0);
    EXPECT_GE(c.spectrum_width, 1.0);
    EXPECT_NEAR(std::fmin(1.0, c.edr / 0.6), c.intensity, 1e-12);
    EXPECT_EQ(c.category, c.edr >= 0.6   ? TurbulenceCategory::EXTREME
                          : c.edr >= 0.4 ? TurbulenceCategory::SEVERE
                          : c.edr >= 0.2 ? TurbulenceCategory::MODERATE
                                         : TurbulenceCategory::LIGHT);
    // A flat block has no width anywhere inside it.
    for (auto& az : volume->dbz) {
        az[0].fill(0.0);
    }
    block(50, 70, 2, 20, 40.0);
    turb.process(*volume, 0.5, *filter);
    for (std::size_t i = 0; i < turb.size(); ++i) {
        const radar::TurbulenceCell& edge = turb.cell_at(i);
        EXPECT_TRUE(edge.az == 50 || edge.az == 70 || edge.r == 2 || edge.r == 20) << "only the edges fluctuate";
    }
}

}  // namespace
