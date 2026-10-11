#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <fmofp/nav/gps_solver.hpp>

namespace {

using fmofp::Status;
using fmofp::nav::bancroft;
using fmofp::nav::build_constellation;
using fmofp::nav::Constellation;
using fmofp::nav::Ecef;
using fmofp::nav::ecef_to_wgs84;
using fmofp::nav::elevation_deg;
using fmofp::nav::Fix;
using fmofp::nav::Geodetic;
using fmofp::nav::kDegToRad;
using fmofp::nav::kSatellites;
using fmofp::nav::kSpeedOfLight;
using fmofp::nav::Matrix4;
using fmofp::nav::Observation;
using fmofp::nav::solve_4x4;
using fmofp::nav::Vector4;
using fmofp::nav::wgs84_to_ecef;
using fmofp::nav::wls_solve;

// The Python INS's starting point (Tinker AFB), at 30 000 ft: a place from
// which the constellation shows exactly four satellites, all coplanar.
Geodetic tinker() {
    Geodetic g;
    g.latitude_deg = 35.4147;
    g.longitude_deg = -97.3866;
    g.altitude_ft = 30000.0;
    return g;
}

double norm(const Ecef& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

double distance(const Ecef& a, const Ecef& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// Noise-free pseudoranges from `truth` with a receiver clock bias, for the
// satellites at least `min_elevation` degrees up. Returns how many.
std::size_t exact_observations(const Constellation& sats, const Ecef& truth, double bias_s, double min_elevation,
                               std::array<Observation, kSatellites>& out) {
    std::size_t n = 0;
    for (const auto& s : sats) {
        if (elevation_deg(truth, s.position) < min_elevation) {
            continue;
        }
        out[n].satellite = s.id;
        out[n].pseudorange_m = distance(truth, s.position) + kSpeedOfLight * (bias_s - s.clock_bias_s);
        ++n;
    }
    return n;
}

TEST(Constellation, TwelveSatellitesOnTheOrbitRadiusWithTheirClockBiases) {
    const Constellation sats = build_constellation();
    for (std::size_t i = 0; i < sats.size(); ++i) {
        EXPECT_EQ(i + 1, sats[i].id);
        EXPECT_NEAR(26560000.0, norm(sats[i].position), 1e-6);
        EXPECT_NEAR(static_cast<double>(i + 1) * 1.3e-9, sats[i].clock_bias_s, 1e-24);
    }
    // Slot 0 of each plane lies in the equatorial plane; slot 1 is inclined 55 degrees.
    EXPECT_DOUBLE_EQ(0.0, sats[0].position.z);
    EXPECT_NEAR(26560000.0 * std::sin(55.0 * kDegToRad), sats[1].position.z, 1e-6);
    EXPECT_DOUBLE_EQ(0.0, sats[4].position.z);
}

TEST(Wgs84, KnownPointsAndRoundTrip) {
    Geodetic g;
    Ecef p = wgs84_to_ecef(g);  // lat 0, lon 0, alt 0: on the equator at the semi-major axis
    EXPECT_NEAR(6378137.0, p.x, 1e-6);
    EXPECT_NEAR(0.0, p.y, 1e-6);
    EXPECT_NEAR(0.0, p.z, 1e-6);
    g.latitude_deg = 90.0;
    p = wgs84_to_ecef(g);  // the pole: the semi-minor axis
    EXPECT_NEAR(0.0, p.x, 1e-6);
    EXPECT_NEAR(6356752.314245, p.z, 1e-6);

    const Geodetic home = tinker();
    const Geodetic back = ecef_to_wgs84(wgs84_to_ecef(home));
    EXPECT_NEAR(home.latitude_deg, back.latitude_deg, 1e-9);
    EXPECT_NEAR(home.longitude_deg, back.longitude_deg, 1e-9);
    EXPECT_NEAR(home.altitude_ft, back.altitude_ft, 1e-5);
}

TEST(Elevation, StraightUpIsNinetyAndTheEarthsCentreCountsAsVisible) {
    const Ecef receiver{6378137.0, 0.0, 0.0};
    EXPECT_NEAR(90.0, elevation_deg(receiver, Ecef{26560000.0, 0.0, 0.0}), 1e-9);
    EXPECT_LT(elevation_deg(receiver, Ecef{0.0, 26560000.0, 0.0}), 0.0);  // on the horizon's far side
    EXPECT_DOUBLE_EQ(90.0, elevation_deg(Ecef{}, Ecef{1.0, 0.0, 0.0}));
    EXPECT_DOUBLE_EQ(-90.0, elevation_deg(receiver, receiver));
}

TEST(Solve4x4, SolvesAKnownSystemAndRejectsASingularOne) {
    const Matrix4 a{{{2.0, 1.0, 0.0, 0.0}, {1.0, 3.0, 1.0, 0.0}, {0.0, 1.0, 4.0, 1.0}, {0.0, 0.0, 1.0, 5.0}}};
    const Vector4 expected{1.0, -1.0, 2.0, 0.5};
    Vector4 b{};
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            b[r] += a[r][c] * expected[c];
        }
    }
    Vector4 x{};
    ASSERT_EQ(Status::kOk, solve_4x4(a, b, x));
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(expected[i], x[i], 1e-12);
    }
    const Matrix4 singular{{{1.0, 2.0, 3.0, 4.0}, {2.0, 4.0, 6.0, 8.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}};
    EXPECT_EQ(Status::kMalformed, solve_4x4(singular, b, x));
}

TEST(WlsSolve, ConvergesFromTheEarthsCentreWhenTheGeometryIsGood) {
    const Constellation sats = build_constellation();
    const Ecef truth = wgs84_to_ecef(tinker());
    const double bias = 1e-6;
    std::array<Observation, kSatellites> obs{};
    const std::size_t n = exact_observations(sats, truth, bias, -90.0, obs);
    ASSERT_EQ(kSatellites, n);
    Fix fix;
    ASSERT_EQ(Status::kOk, wls_solve(sats, obs.data(), n, Ecef{}, 0.0, fix));
    EXPECT_TRUE(fix.converged);
    EXPECT_NEAR(truth.x, fix.position.x, 5e-3);
    EXPECT_NEAR(truth.y, fix.position.y, 5e-3);
    EXPECT_NEAR(truth.z, fix.position.z, 5e-3);
    EXPECT_NEAR(bias, fix.clock_bias_s, 1e-11);
    EXPECT_LE(fix.iterations, 8);
    EXPECT_EQ(12, fix.satellites);
}

TEST(WlsSolve, TheFourSatellitesSeenFromHomeAreSingularFromTheCentreButNotFromNearby) {
    const Constellation sats = build_constellation();
    const Ecef truth = wgs84_to_ecef(tinker());
    std::array<Observation, kSatellites> obs{};
    const std::size_t n = exact_observations(sats, truth, 1e-6, 5.0, obs);
    ASSERT_EQ(4U, n);  // what the home position sees above 5 degrees: a coplanar four
    Fix fix;
    EXPECT_EQ(Status::kMalformed, wls_solve(sats, obs.data(), n, Ecef{}, 0.0, fix));
    // Seeded 100 km away it converges at once, as the Python did from its truth.
    const Ecef nearby{truth.x + 60000.0, truth.y - 50000.0, truth.z + 60000.0};
    ASSERT_EQ(Status::kOk, wls_solve(sats, obs.data(), n, nearby, 0.0, fix));
    EXPECT_TRUE(fix.converged);
    EXPECT_NEAR(0.0, distance(truth, fix.position), 1e-2);
    EXPECT_NEAR(1e-6, fix.clock_bias_s, 1e-11);
    EXPECT_LE(fix.iterations, 8);
}

TEST(Bancroft, AcquiresTheFixInClosedFormAndSeedsTheIteration) {
    const Constellation sats = build_constellation();
    const Ecef truth = wgs84_to_ecef(tinker());
    std::array<Observation, kSatellites> obs{};
    for (const double min_elevation : {5.0, -90.0}) {
        const std::size_t n = exact_observations(sats, truth, 1e-6, min_elevation, obs);
        Fix start;
        ASSERT_EQ(Status::kOk, bancroft(sats, obs.data(), n, start)) << n << " satellites";
        EXPECT_NEAR(0.0, distance(truth, start.position), 1e-2) << n << " satellites";
        EXPECT_NEAR(1e-6, start.clock_bias_s, 1e-10);
        EXPECT_EQ(n, start.satellites);
        Fix fix;
        ASSERT_EQ(Status::kOk, wls_solve(sats, obs.data(), n, start.position, start.clock_bias_s, fix));
        EXPECT_TRUE(fix.converged);
        EXPECT_LE(fix.iterations, 2);
        EXPECT_NEAR(0.0, distance(truth, fix.position), 1e-3);
    }
    Fix few;
    EXPECT_EQ(Status::kInvalidArgument, bancroft(sats, obs.data(), 3, few));
}

TEST(WlsSolve, RejectsTooFewUnknownOrDegenerateObservations) {
    const Constellation sats = build_constellation();
    std::array<Observation, kSatellites> obs{};
    const std::size_t n = exact_observations(sats, wgs84_to_ecef(tinker()), 0.0, -90.0, obs);
    Fix fix;
    EXPECT_EQ(Status::kInvalidArgument, wls_solve(sats, obs.data(), 3, Ecef{}, 0.0, fix));
    EXPECT_EQ(Status::kInvalidArgument, wls_solve(sats, nullptr, n, Ecef{}, 0.0, fix));
    std::array<Observation, kSatellites> unknown = obs;
    unknown[0].satellite = 13;
    EXPECT_EQ(Status::kInvalidArgument, wls_solve(sats, unknown.data(), n, Ecef{}, 0.0, fix));
    EXPECT_EQ(Status::kInvalidArgument, bancroft(sats, unknown.data(), n, fix));
    // Four copies of one satellite: the normal equations are singular.
    std::array<Observation, kSatellites> same{};
    for (std::size_t i = 0; i < 4; ++i) {
        same[i] = obs[0];
    }
    EXPECT_EQ(Status::kMalformed, wls_solve(sats, same.data(), 4, Ecef{}, 0.0, fix));
    EXPECT_EQ(Status::kMalformed, bancroft(sats, same.data(), 4, fix));
}

}  // namespace
