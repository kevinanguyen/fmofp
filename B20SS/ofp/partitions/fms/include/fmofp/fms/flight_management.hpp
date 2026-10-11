#ifndef FMOFP_FMS_FLIGHT_MANAGEMENT_HPP
#define FMOFP_FMS_FLIGHT_MANAGEMENT_HPP

#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>

// The flight management partition's logic, ported from
// Systems/flightManagementSys: fmsControl.py (flight modes and their profile
// limits, envelope-limited attitude commands, maneuvers, autopilot targets,
// thrust and gear, energy maneuverability) and the parts of
// flightManagementSystem.py that computed rather than simulated (mach,
// energy state, the flight data the displays read). Pure: inputs are ICD
// messages, outputs are ICD messages, no bus, no clock.
//
// What the Python simulated is not here: the FMS-internal flight control
// object whose attitude it copied (the FCS partition's FCS_ATTITUDE arrives
// instead), the sinusoidal airspeed and vertical speed (the sensors arrive
// instead), and the navigation it did in-process (NAV_SOLUTION arrives
// instead). See the ICD README, section 2.

namespace fmofp::fms {

// fmsControl.flight_profiles: the envelope each flight mode allows.
struct FlightProfile {
    double max_bank_angle;   // degrees
    double max_pitch_angle;  // degrees
    double max_g_force;      // g
    double max_aoa;          // degrees
    double engine_power;     // 0..1
    double turn_rate_limit;  // degrees per second
};
const FlightProfile& profile(icd::FmsFlightMode mode) noexcept;

// fmsControl._update_tactical_systems: what each flight mode sets.
struct TacticalSystems {
    icd::CountermeasuresState countermeasures{icd::CountermeasuresState::STANDBY};
    icd::TargetingState targeting{icd::TargetingState::STANDBY};
    icd::WeaponsState weapons{icd::WeaponsState::SAFE};
    bool stealth_mode{false};
};
TacticalSystems tactical_systems(icd::FmsFlightMode mode) noexcept;

struct Attitude {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double roll_rate{0.0};
    double pitch_rate{0.0};
    double yaw_rate{0.0};
};

// What fmsControl.update_attitude takes: a value counts only when present.
struct AttitudeRequest {
    bool roll_present{false};
    bool pitch_present{false};
    bool yaw_present{false};
    bool roll_rate_present{false};
    bool pitch_rate_present{false};
    bool yaw_rate_present{false};
    Attitude value{};
};

struct LimitedAttitude {
    Attitude applied{};
    bool bank_limited{false};        // BANK_ANGLE
    bool pitch_limited{false};       // PITCH_ANGLE
    bool roll_rate_limited{false};   // ROLL_RATE
    bool pitch_rate_limited{false};  // PITCH_RATE
    bool yaw_rate_limited{false};    // YAW_RATE
};

// fmsControl.update_attitude: absent values come from `current`, then roll is
// held to max_bank_angle, pitch to max_pitch_angle, the rates to 3x, 2x and
// 1x turn_rate_limit. Yaw is never limited.
LimitedAttitude limit_attitude(const AttitudeRequest& request, const Attitude& current,
                               const FlightProfile& p) noexcept;

// fmsControl.set_thrust: at least 0.1, at most 0.9 (1.0 with the afterburner
// lit), throttle_position = int(thrust x 100), afterburner lit above 0.9 and
// put out below 0.85. Returns the thrust in force.
struct ThrustState {
    double thrust{0.5};
    std::uint8_t throttle_position{50};
    bool afterburner{false};
};
double apply_thrust(ThrustState& state, double level) noexcept;

// fmsControl.set_autopilot_targets' limits, applied to the present targets.
struct AutopilotTargets {
    double altitude_ft{30000.0};
    double airspeed_kt{450.0};
    double heading_deg{0.0};
    double vertical_speed_fpm{0.0};
};
void apply_autopilot_targets(AutopilotTargets& targets, const icd::FmsNavUpdateCmd& cmd) noexcept;

// fmsControl.execute_maneuver: the attitude request a maneuver makes.
// COMPLETED with the request filled in; NOT_IMPLEMENTED for BARREL_ROLL;
// REJECTED_INVALID for NONE.
icd::CmdStatus maneuver_request(icd::Maneuver maneuver, double dive_angle_deg, const FlightProfile& p,
                                AttitudeRequest& out) noexcept;

// flightManagementSystem.update_flight_data's derived values.
double mach_number(double airspeed_kt) noexcept;
double energy_state(double airspeed_kt, double altitude_ft) noexcept;

// fmsControl.calculate_energy_maneuverability.
struct EnergyMetrics {
    double specific_excess_power{0.0};        // m/s
    double max_sustained_turn_rate{0.0};      // deg/s
    double max_instantaneous_turn_rate{0.0};  // deg/s
    double max_climb_rate_fpm{0.0};
    double energy_advantage{0.0};
};
EnergyMetrics energy_maneuverability(double altitude_ft, double airspeed_kt, double aoa_deg,
                                     const FlightProfile& p) noexcept;

constexpr std::uint32_t kSensorStaleFrames = 3;  // 50 Hz inputs: ICD README section 4
constexpr std::uint32_t kNavStaleFrames = 15;    // NAV_SOLUTION is 10 Hz: three periods

class FlightManagement {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame; each input is the message received this frame or null.
    void step(const icd::FmsSensors* sensors, const icd::FmsAttitudeIn* attitude,
              const icd::FmsNavSolution* nav, std::uint32_t frame) noexcept;

    // Commands, each applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::FmsModeCmd& cmd) noexcept;
    void command_flight_mode(const icd::FmsFlightModeCmd& cmd) noexcept;
    void command_attitude(const icd::FmsAttitudeCmd& cmd) noexcept;
    void command_nav_update(const icd::FmsNavUpdateCmd& cmd) noexcept;
    void command_maneuver(const icd::FmsManeuverCmd& cmd) noexcept;
    void command_control(const icd::FmsControlCmd& cmd) noexcept;

    void reset() noexcept;
    void start_self_test() noexcept;

    [[nodiscard]] const icd::FmsStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::FmsFlightData& flight_data() const noexcept { return flight_data_; }
    [[nodiscard]] const icd::FmsAttitudeStatus& attitude_status() const noexcept { return attitude_status_; }
    [[nodiscard]] const icd::FmsNavUpdateStatus& nav_update_status() const noexcept { return nav_update_status_; }
    [[nodiscard]] const icd::FmsManeuverStatus& maneuver_status() const noexcept { return maneuver_status_; }
    [[nodiscard]] const icd::FmsControlStatus& control_status() const noexcept { return control_status_; }
    [[nodiscard]] const icd::FmsEnergy& energy() const noexcept { return energy_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::FmsFlightMode flight_mode() const noexcept { return status_.flight_mode; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }

private:
    // The attitude an absent command field keeps: the measured one, as the
    // Python's flight-data dict held the FCS attitude; rates 0, as that dict
    // held the Python's idle flight-control object's rates.
    [[nodiscard]] Attitude current_attitude() const noexcept;
    void hold_attitude(const AttitudeRequest& request) noexcept;
    void produce_outputs(std::uint32_t frame) noexcept;

    icd::FmsStatus status_{};
    icd::FmsFlightData flight_data_{};
    icd::FmsAttitudeStatus attitude_status_{};
    icd::FmsNavUpdateStatus nav_update_status_{};
    icd::FmsManeuverStatus maneuver_status_{};
    icd::FmsControlStatus control_status_{};
    icd::FmsEnergy energy_{};
    icd::PlantSensors sensors_{};
    icd::FcsAttitude attitude_{};
    icd::NavSolution nav_{};
    bool have_sensors_{false};
    bool have_attitude_{false};
    bool have_nav_{false};
    std::uint32_t sensor_frame_{0};
    std::uint32_t attitude_frame_{0};
    std::uint32_t nav_frame_{0};
    bool sensors_fresh_{false};
    bool attitude_fresh_{false};
    bool nav_fresh_{false};
    ThrustState thrust_{};
    AutopilotTargets targets_{};
    icd::GearPosition gear_{icd::GearPosition::UP};
    bus::CommandSequence mode_seq_{};
    bus::CommandSequence flight_mode_seq_{};
    bus::CommandSequence attitude_seq_{};
    bus::CommandSequence nav_update_seq_{};
    bus::CommandSequence maneuver_seq_{};
    bus::CommandSequence control_seq_{};
    bus::BitWord bit_{};
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
};

}  // namespace fmofp::fms

#endif
