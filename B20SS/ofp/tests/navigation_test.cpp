#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <fmofp/icd/messages.hpp>
#include <fmofp/nav/gps_solver.hpp>
#include <fmofp/nav/navigation.hpp>

namespace {

using fmofp::Status;
using fmofp::icd::NavGps;
using fmofp::icd::NavSensors;
using fmofp::icd::PlantGps;
using fmofp::icd::Ssm;
using fmofp::icd::TermCmd;
using fmofp::icd::TerminalState;
using fmofp::nav::build_constellation;
using fmofp::nav::Constellation;
using fmofp::nav::dead_reckoning_rates;
using fmofp::nav::DeadReckoningRates;
using fmofp::nav::Ecef;
using fmofp::nav::elevation_deg;
using fmofp::nav::Geodetic;
using fmofp::nav::kDegToRad;
using fmofp::nav::kGpsRAlt;
using fmofp::nav::kMetresPerDegree;
using fmofp::nav::kMetresPerFoot;
using fmofp::nav::kSpeedOfLight;
using fmofp::nav::NavFusion;
using fmofp::nav::Navigation;
using fmofp::nav::ScalarKalman;
using fmofp::nav::wgs84_to_ecef;

// The Python INS's starting point. The ranges here are exact, so its poor
// satellite geometry does not matter; the plant's home is elsewhere.
constexpr double kSiteLat = 35.4147;
constexpr double kSiteLon = -97.3866;
constexpr double kSiteAlt = 30000.0;

NavSensors sensors(double airspeed, double heading, double vertical_speed) {
    NavSensors s{};
    s.payload.airspeed_ssm = Ssm::NORMAL;
    s.payload.heading_ssm = Ssm::NORMAL;
    s.payload.vertical_speed_ssm = Ssm::NORMAL;
    s.payload.altitude_ssm = Ssm::NORMAL;
    s.payload.airspeed = airspeed;
    s.payload.heading = heading;
    s.payload.vertical_speed = vertical_speed;
    s.payload.altitude = kSiteAlt;
    return s;
}

void set_pseudorange(PlantGps& g, std::uint8_t satellite, double value) {
    double* slots[] = {&g.pseudorange_1, &g.pseudorange_2,  &g.pseudorange_3,  &g.pseudorange_4,
                       &g.pseudorange_5, &g.pseudorange_6,  &g.pseudorange_7,  &g.pseudorange_8,
                       &g.pseudorange_9, &g.pseudorange_10, &g.pseudorange_11, &g.pseudorange_12};
    *slots[satellite - 1U] = value;
}

// A NAV_GPS epoch with exact pseudoranges from the site, receiver
// clock bias `bias_s`, for the satellites at least 5 degrees up.
NavGps epoch(const Constellation& sats, std::uint8_t number, double bias_s, std::size_t max_satellites = 12) {
    Geodetic home;
    home.latitude_deg = kSiteLat;
    home.longitude_deg = kSiteLon;
    home.altitude_ft = kSiteAlt;
    const Ecef truth = wgs84_to_ecef(home);
    NavGps g{};
    g.payload.epoch = number;
    std::size_t n = 0;
    for (const auto& s : sats) {
        if (n >= max_satellites || elevation_deg(truth, s.position) < 5.0) {
            continue;
        }
        const double dx = s.position.x - truth.x;
        const double dy = s.position.y - truth.y;
        const double dz = s.position.z - truth.z;
        set_pseudorange(g.payload, s.id,
                        std::sqrt(dx * dx + dy * dy + dz * dz) + kSpeedOfLight * (bias_s - s.clock_bias_s));
        g.payload.visible_mask = static_cast<std::uint16_t>(g.payload.visible_mask | (1U << (s.id - 1U)));
        ++n;
    }
    return g;
}

TEST(DeadReckoning, MatchesTheFmsFormula) {
    const double gs_dps_lat = (450.0 * 1.68781) / (kMetresPerDegree / kMetresPerFoot);
    DeadReckoningRates r = dead_reckoning_rates(450.0, 90.0, 600.0, 0.0);
    EXPECT_NEAR(0.0, r.lat_dps, 1e-15);
    EXPECT_NEAR(gs_dps_lat, r.lon_dps, 1e-15);
    EXPECT_DOUBLE_EQ(10.0, r.alt_fps);
    r = dead_reckoning_rates(450.0, 0.0, -1200.0, 60.0);
    EXPECT_NEAR(gs_dps_lat, r.lat_dps, 1e-15);
    EXPECT_NEAR(0.0, r.lon_dps, 1e-15);
    EXPECT_DOUBLE_EQ(-20.0, r.alt_fps);
    // At the pole the cosine is floored at 0.001.
    r = dead_reckoning_rates(450.0, 90.0, 0.0, 90.0);
    EXPECT_NEAR(gs_dps_lat / 0.001, r.lon_dps, 1e-9);
}

TEST(ScalarKalman, PredictsAndUpdatesAsTheOneDimensionalFilter) {
    ScalarKalman k{0.0, 1.0, 0.5, 2.0};
    k.predict(2.0, 3.0);  // x += 6, P += 1
    EXPECT_DOUBLE_EQ(6.0, k.x);
    EXPECT_DOUBLE_EQ(2.0, k.p);
    k.update(10.0);  // K = 2 / 4
    EXPECT_DOUBLE_EQ(8.0, k.x);
    EXPECT_DOUBLE_EQ(1.0, k.p);
}

TEST(NavFusion, StartsAtTheOriginAndTakesTheFirstFixAsThePythonDid) {
    NavFusion f;
    f.init();
    EXPECT_DOUBLE_EQ(0.0, f.latitude());
    EXPECT_DOUBLE_EQ(30000.0, f.altitude_ft());
    f.update_gps(kSiteLat, kSiteLon, 29000.0);
    EXPECT_EQ(1U, f.fix_count());
    // Horizontal: R is tiny against P = 1, so the fix is taken almost whole.
    EXPECT_NEAR(kSiteLat, f.latitude(), 1e-6);
    EXPECT_NEAR(kSiteLon, f.longitude(), 1e-6);
    // Vertical: R = (10 m)^2 in ft^2 dwarfs P = 1, so altitude moves by K = 1 / (1 + R).
    const double k = 1.0 / (1.0 + kGpsRAlt);
    EXPECT_NEAR(30000.0 + k * (29000.0 - 30000.0), f.altitude_ft(), 1e-9);
}

TEST(Navigation, FollowsTheSensorsIntoNormalAndDegraded) {
    Navigation nav;
    ASSERT_EQ(Status::kOk, nav.init());
    EXPECT_EQ(TerminalState::INITIALIZING, nav.state());
    EXPECT_EQ(Ssm::NO_COMPUTED_DATA, nav.solution().position_ssm);
    EXPECT_EQ(255, nav.solution().fix_age);
    const NavSensors s = sensors(450.0, 90.0, 0.0);
    nav.step(&s, nullptr, 1);
    EXPECT_EQ(TerminalState::NORMAL, nav.state());
    nav.step(nullptr, nullptr, 2);
    nav.step(nullptr, nullptr, 3);
    EXPECT_EQ(TerminalState::NORMAL, nav.state());
    nav.step(nullptr, nullptr, 4);
    EXPECT_EQ(TerminalState::DEGRADED, nav.state());
    nav.step(&s, nullptr, 5);
    EXPECT_EQ(TerminalState::NORMAL, nav.state());
}

TEST(Navigation, SolvesEachNewEpochOnceAndKeepsThePositionThroughALostOne) {
    const Constellation sats = build_constellation();
    Navigation nav;
    ASSERT_EQ(Status::kOk, nav.init());
    const NavSensors still = sensors(0.0, 0.0, 0.0);
    std::uint32_t frame = 1;
    NavGps first = epoch(sats, 1, 1e-6);
    nav.step(&still, &first, frame++);
    const auto& sol = nav.solution();
    EXPECT_TRUE(sol.gps_valid);
    EXPECT_EQ(4, sol.sats_used);
    EXPECT_EQ(1, sol.fix_count);
    EXPECT_EQ(0, sol.fix_age);
    EXPECT_EQ(Ssm::NORMAL, sol.position_ssm);
    EXPECT_EQ(Ssm::NORMAL, sol.altitude_ssm);
    EXPECT_NEAR(kSiteLat, sol.latitude, 1e-6);
    EXPECT_NEAR(kSiteLon, sol.longitude, 1e-6);
    EXPECT_NEAR(kSiteAlt, sol.altitude, 1.0);
    EXPECT_NEAR(1.0, sol.clock_bias, 1e-3);  // microseconds

    // The same epoch again is not a new fix.
    nav.step(&still, &first, frame++);
    EXPECT_EQ(1, nav.solution().fix_count);

    // An epoch with three satellites gives no fix: GPS invalid, position kept.
    NavGps thin = epoch(sats, 2, 1e-6, 3);
    nav.step(&still, &thin, frame++);
    EXPECT_FALSE(nav.solution().gps_valid);
    EXPECT_EQ(1, nav.solution().fix_count);
    EXPECT_EQ(Ssm::NORMAL, nav.solution().position_ssm);
    EXPECT_NEAR(kSiteLat, nav.solution().latitude, 1e-6);

    // Fix age counts whole seconds since the last fix.
    while (frame < 60) {
        nav.step(&still, nullptr, frame++);
    }
    EXPECT_EQ(1, nav.solution().fix_age);

    // A good epoch makes it valid again and resets the age.
    NavGps third = epoch(sats, 3, 1e-6);
    nav.step(&still, &third, frame++);
    EXPECT_TRUE(nav.solution().gps_valid);
    EXPECT_EQ(2, nav.solution().fix_count);
    EXPECT_EQ(0, nav.solution().fix_age);
}

TEST(Navigation, DeadReckonsBetweenFixesFromAirspeedAndHeading) {
    const Constellation sats = build_constellation();
    Navigation nav;
    ASSERT_EQ(Status::kOk, nav.init());
    NavGps first = epoch(sats, 1, 0.0);
    const NavSensors east = sensors(450.0, 90.0, 600.0);
    nav.step(&east, &first, 1);
    const double lon0 = nav.solution().longitude;
    const double alt0 = nav.solution().altitude;
    for (std::uint32_t frame = 2; frame <= 51; ++frame) {
        nav.step(&east, nullptr, frame);  // one second of flight
    }
    const DeadReckoningRates r = dead_reckoning_rates(450.0, 90.0, 600.0, nav.solution().latitude);
    EXPECT_NEAR(lon0 + r.lon_dps, nav.solution().longitude, 1e-9);
    EXPECT_NEAR(alt0 + 10.0, nav.solution().altitude, 1e-9);
    EXPECT_NEAR(kSiteLat, nav.solution().latitude, 1e-6);

    // Invalid sensors hold the position instead of integrating garbage.
    NavSensors bad = east;
    bad.payload.heading_ssm = Ssm::NO_COMPUTED_DATA;
    const double lon1 = nav.solution().longitude;
    nav.step(&bad, nullptr, 52);
    EXPECT_DOUBLE_EQ(lon1, nav.solution().longitude);
}

TEST(Navigation, TerminalCommandsFollowTheSequenceRule) {
    Navigation nav;
    ASSERT_EQ(Status::kOk, nav.init());
    const NavSensors s = sensors(0.0, 0.0, 0.0);
    nav.step(&s, nullptr, 1);
    TermCmd cmd{};
    cmd.requested_state = TerminalState::STANDBY;
    cmd.cmd_seq = 1;
    nav.command(cmd);
    EXPECT_EQ(TerminalState::STANDBY, nav.state());
    EXPECT_EQ(1, nav.last_cmd_seq());
    EXPECT_FALSE(nav.cmd_rejected());
    cmd.requested_state = TerminalState::FAILURE;  // not commandable
    nav.command(cmd);                              // same seq: ignored
    EXPECT_EQ(TerminalState::STANDBY, nav.state());
    cmd.cmd_seq = 2;
    nav.command(cmd);
    EXPECT_TRUE(nav.cmd_rejected());
    EXPECT_EQ(TerminalState::STANDBY, nav.state());
    EXPECT_EQ(2, nav.last_cmd_seq());
}

}  // namespace
