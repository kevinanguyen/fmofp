#ifndef FMOFP_NAV_NAVIGATION_HPP
#define FMOFP_NAV_NAVIGATION_HPP

#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/nav/gps_solver.hpp>

// The navigation partition's logic: the GPS / dead-reckoning fusion of
// Systems/nav/dataFusion/fuseData.py (NavDataFusion, one scalar Kalman
// filter per axis), the dead-reckoning rates flightManagementSystem.
// update_flight_data derived, and the GPS epochs solved with gps_solver.
// Pure: inputs are ICD messages, outputs are ICD messages, no bus, no clock.
//
// Not carried from Systems/nav: the INS (a random-walk IMU whose attitude
// nothing used) and the heading creep of 0.01 degrees per cycle; see the ICD
// README, section 2.

namespace fmofp::nav {

// fuseData._ScalarKalman: a one-dimensional constant-velocity filter.
struct ScalarKalman {
    double x{0.0};  // state estimate
    double p{1.0};  // error covariance
    double q{0.0};  // process noise variance per second
    double r{0.0};  // measurement noise variance

    void predict(double dt, double velocity) noexcept {
        x += velocity * dt;
        p += q * dt;
    }
    void update(double measurement) noexcept {
        const double k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1.0 - k) * p;
    }
};

// NavDataFusion's constants: GPS noise 5 m horizontally and 10 m vertically,
// dead-reckoning uncertainty 1 m/s horizontally and 2 m/s vertically, with
// 1 degree of latitude taken as 111 000 m.
constexpr double kMetresPerDegree = 111000.0;
constexpr double kGpsRLat = (5.0 / kMetresPerDegree) * (5.0 / kMetresPerDegree);
constexpr double kGpsRLon = kGpsRLat;
constexpr double kGpsRAlt = (10.0 / kMetresPerFoot) * (10.0 / kMetresPerFoot);
constexpr double kQLat = (1.0 / kMetresPerDegree) * (1.0 / kMetresPerDegree);
constexpr double kQLon = kQLat;
constexpr double kQAlt = (2.0 / kMetresPerFoot) * (2.0 / kMetresPerFoot);
constexpr double kInitialAltitudeFt = 30000.0;

class NavFusion {
public:
    void init() noexcept;
    void predict(double dt, double lat_rate_dps, double lon_rate_dps, double alt_rate_fps) noexcept;
    void update_gps(double lat_deg, double lon_deg, double alt_ft) noexcept;

    [[nodiscard]] double latitude() const noexcept { return lat_.x; }
    [[nodiscard]] double longitude() const noexcept { return lon_.x; }
    [[nodiscard]] double altitude_ft() const noexcept { return alt_.x; }
    [[nodiscard]] std::uint32_t fix_count() const noexcept { return fix_count_; }

private:
    ScalarKalman lat_{};
    ScalarKalman lon_{};
    ScalarKalman alt_{};
    std::uint32_t fix_count_{0};
};

struct DeadReckoningRates {
    double lat_dps{0.0};
    double lon_dps{0.0};
    double alt_fps{0.0};
};

// flightManagementSystem.update_flight_data's rates: knots to feet per
// second to degrees per second, longitude scaled by the cosine of latitude
// (floored at 0.001), altitude from feet per minute.
DeadReckoningRates dead_reckoning_rates(double ground_speed_kt, double heading_deg, double vertical_speed_fpm,
                                        double latitude_deg) noexcept;

constexpr std::uint32_t kSensorStaleFrames = 3;  // ICD README section 4
constexpr double kFrameSeconds = 0.02;
constexpr std::uint32_t kFramesPerSecond = 50;

class Navigation {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `sensors` is the NAV_SENSORS received this frame
    // and `gps` the NAV_GPS received this frame, or null; `frame` is the
    // partition's own frame count.
    void step(const icd::NavSensors* sensors, const icd::NavGps* gps, std::uint32_t frame) noexcept;

    void command(const icd::TermCmd& cmd) noexcept;
    void reset() noexcept;
    void start_self_test() noexcept;

    [[nodiscard]] const icd::NavSolution& solution() const noexcept { return solution_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return state_; }
    [[nodiscard]] bool healthy() const noexcept { return !bit_.failed; }
    [[nodiscard]] std::uint8_t last_cmd_seq() const noexcept { return last_cmd_seq_; }
    [[nodiscard]] bool cmd_rejected() const noexcept { return cmd_rejected_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] const NavFusion& fusion() const noexcept { return fusion_; }
    [[nodiscard]] const Fix& last_fix() const noexcept { return last_fix_; }

private:
    void solve_epoch(const icd::PlantGps& gps) noexcept;
    void produce_solution() noexcept;

    Constellation constellation_{};
    NavFusion fusion_{};
    icd::NavSolution solution_{};
    icd::PlantSensors last_sensors_{};
    bool have_sensors_{false};
    std::uint32_t last_sensor_frame_{0};
    bool have_gps_{false};
    std::uint8_t last_epoch_{0};
    bool have_fix_{false};
    Fix last_fix_{};
    bool gps_valid_{false};
    std::uint8_t sats_used_{0};
    std::uint32_t frames_since_fix_{0};
    icd::TerminalState state_{icd::TerminalState::INITIALIZING};
    std::uint8_t last_cmd_seq_{0};
    bool cmd_rejected_{false};
    bus::CommandSequence cmd_seq_{};
    bus::BitWord bit_{};
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
};

}  // namespace fmofp::nav

#endif
