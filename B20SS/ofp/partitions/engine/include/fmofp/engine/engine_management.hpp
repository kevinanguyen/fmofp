#ifndef FMOFP_ENGINE_ENGINE_MANAGEMENT_HPP
#define FMOFP_ENGINE_ENGINE_MANAGEMENT_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>

// The engine management partition's logic (RT 3), ported from
// Systems/engineManagement: thrustManagement.py (the commanded thrust of each
// engine, the autothrottle, the asymmetry check), fuelControl.py with
// fuelMonitor.py (the tanks, the burn along the fuel lines, the Center-to-Mains
// policy, the low-fuel and unexplained-loss flags), and the assessment the
// MFD's engine page made of engineControlUnit.get_data(). The engine itself
// (engineControlUnit.py's model) is simulation and lives in the plant; this
// partition commands it (ENG_THRUST_CMD) and reports what it measures
// (ENG_ENGINE_STATUS). Pure: ICD messages in, ICD messages out, nothing
// touches the bus or the clock, so the tests drive it frame by frame.

namespace fmofp::engine {

constexpr std::size_t kEngines = 2;  // Engine 1 and Engine 2 (FuelManagementSystem.setup_aircraft)
constexpr std::size_t kTanks = 3;    // fuel_tank MAIN_LEFT, MAIN_RIGHT, CENTER: index = enum value - 1

// thrustManagement.py
constexpr double kMaxSafeThrustAsymmetryPct = 25.0;  // MAX_SAFE_THRUST_ASYMMETRY_PCT
constexpr double kAutothrottleKp = 0.5;
constexpr double kAutothrottleKi = 0.05;
constexpr double kAutothrottleIntegralLimit = 50.0;
constexpr std::uint32_t kAutothrottleFrames = 25;  // its update loop ran at 2 Hz
constexpr double kAutothrottleDt = 0.5;

// fuelControl.py and fuelMonitor.py
constexpr std::array<double, kTanks> kTankCapacityKg = {10000.0, 10000.0, 20000.0};
constexpr std::uint32_t kFuelFrames = 5;  // FuelManagementSystem.update_interval 0.1 s
constexpr double kFuelDt = 0.1;
constexpr double kLowFuelPct = 20.0;
constexpr double kCriticalFuelPct = 10.0;
constexpr double kLeakUnexplainedDropFraction = 0.02;
constexpr double kTransferSourceAboveKg = 5000.0;  // perform_fuel_transfer: while the Center holds more than this
constexpr double kTransferFeedBelowKg = 5000.0;    // a main below this
constexpr double kTransferAmountKg = 1000.0;       // is topped up by this much

// The limits the MFD's engine page colours (mfd.py draw_engine_page): red at
// N1 or N2 100 % and above, EGT above 800 C, oil pressure below 40 psi, oil
// temperature above 130 C; amber for vibration above 0.8.
constexpr double kEgtHighC = 800.0;
constexpr double kOilLowPsi = 40.0;
constexpr double kOilHotC = 130.0;
constexpr double kVibrationHigh = 0.8;
constexpr double kOverspeedPct = 100.0;

// Inputs older than this many frames are absent (ICD README section 4: three periods).
constexpr std::uint32_t kEngineStaleFrames = 15;      // ENG_ENGINE_IN is 10 Hz
constexpr std::uint32_t kFlightDataStaleFrames = 15;  // BCAST_FMS_FLIGHT_DATA, heard by broadcast, is 10 Hz

// ThrustManagementSystem: the commanded thrust of each engine and the autothrottle.
struct ThrustManager {
    std::array<double, kEngines> thrust{};  // percent; register_engine started both at 0.0
    bool autothrottle{false};
    double target_kt{0.0};
    double integral{0.0};
    double last_error{0.0};

    void set_thrust(std::size_t engine, double pct) noexcept;  // set_thrust: clamped to 0..100
    void set_all(double pct) noexcept;                          // set_all_thrust
    void engage(double target) noexcept;                        // engage_autothrottle
    void disengage() noexcept;                                  // disengage_autothrottle
    // update_autothrottle: a PI step on the airspeed error, applied to both
    // engines around their average. Nothing happens unless engaged and dt > 0.
    void update_autothrottle(double airspeed_kt, double dt) noexcept;
    [[nodiscard]] double asymmetry() const noexcept;  // check_asymmetry: max - min, percentage points
    [[nodiscard]] bool asymmetric() const noexcept { return asymmetry() > kMaxSafeThrustAsymmetryPct; }
};

// A fuel line of setup_aircraft: the tank that feeds an engine and the line's weight.
struct FuelLine {
    std::size_t tank;
    double weight;
};
constexpr std::array<std::array<FuelLine, 2>, kEngines> kFuelLines = {{
    {{{0, 100.0}, {2, 50.0}}},  // Engine 1: Main Left 100, Center 50
    {{{1, 100.0}, {2, 50.0}}},  // Engine 2: Main Right 100, Center 50
}};

struct TankFlags {
    bool low{false};       // at or below 20% and not critical (fuelMonitor LOW_FUEL)
    bool critical{false};  // at or below 10% (CRITICAL_FUEL)
    bool leak{false};      // an unexplained drop was seen; latched, as the Python's alert list kept it
};

// FuelSystem, FuelMonitor and FuelManagementSystem's Center-to-Mains policy.
class FuelSystem {
public:
    void init() noexcept;

    // One fuel pass (every 0.1 s in the Python): burn the engines' measured
    // fuel flow along the lines, run the transfer policy, poll the monitor.
    void step(const std::array<double, kEngines>& fuel_flow_kgh, double dt) noexcept;

    // transfer_fuel: moves what the source holds and the destination can take,
    // at most `kg`. Returns the kg moved. (The Python took the amount from the
    // source and let the destination's overflow vanish; nothing ever called
    // it, and this keeps the fuel.)
    double transfer(std::size_t source, std::size_t destination, double kg) noexcept;

    // Test and scenario support: a tank's level, as the Python's tests set them.
    void set_level(std::size_t tank, double kg) noexcept;

    [[nodiscard]] double level(std::size_t tank) const noexcept { return levels_[tank]; }
    [[nodiscard]] double total() const noexcept;
    [[nodiscard]] const std::array<TankFlags, kTanks>& flags() const noexcept { return flags_; }
    [[nodiscard]] bool transferred_left() const noexcept { return transfer_left_; }    // by the policy, last pass
    [[nodiscard]] bool transferred_right() const noexcept { return transfer_right_; }  // by the policy, last pass
    // The monitor's consumption rates summed, per hour: each tank's drop since
    // the last poll less what transfers moved out of it.
    [[nodiscard]] double burn_rate_kgh() const noexcept;
    [[nodiscard]] double transferred_total() const noexcept { return transferred_total_; }

private:
    void burn(const std::array<double, kEngines>& fuel_flow_kgh, double dt) noexcept;
    void policy() noexcept;
    void poll(double dt) noexcept;

    std::array<double, kTanks> levels_{};
    std::array<double, kTanks> last_levels_{};
    std::array<double, kTanks> rates_kg_s_{};
    std::array<double, kTanks> expected_burn_{};  // what the burn should have taken since the last poll
    std::array<double, kTanks> transfers_out_{};  // what transfers moved out since the last poll
    std::array<TankFlags, kTanks> flags_{};
    bool transfer_left_{false};
    bool transfer_right_{false};
    bool have_last_{false};
    double transferred_total_{0.0};
};

class EngineManagement {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. Each input is the message received this frame,
    // or null if none arrived: the FMS thrust lever (ENG_FMS_CONTROL), the
    // FMS flight data heard by broadcast (its airspeed, for the autothrottle)
    // and the engines' measurements (ENG_ENGINE_IN); `frame` is the
    // partition's own frame count.
    void step(const icd::EngFmsControl* fms, const icd::BcastFmsFlightData* flight, const icd::EngEngineIn* engines,
              std::uint32_t frame) noexcept;

    // Commands, each applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::EngModeCmd& cmd) noexcept;
    void command(const icd::EngCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    [[nodiscard]] const icd::EngStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::EngThrustCmd& thrust_cmd() const noexcept { return thrust_cmd_; }
    [[nodiscard]] const icd::EngEngineStatus& engine_status() const noexcept { return engine_status_; }
    [[nodiscard]] const icd::EngFuelStatus& fuel_status() const noexcept { return fuel_status_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] const ThrustManager& thrust() const noexcept { return thrust_; }
    [[nodiscard]] const FuelSystem& fuel() const noexcept { return fuel_; }

private:
    void produce_outputs(bool engines_fresh) noexcept;

    icd::EngStatus status_{};
    icd::EngThrustCmd thrust_cmd_{};
    icd::EngEngineStatus engine_status_{};
    icd::EngFuelStatus fuel_status_{};
    ThrustManager thrust_{};
    FuelSystem fuel_{};
    bool commands_valid_{false};  // something has commanded the thrust (the FMS lever, a SET_THRUST, the autothrottle)
    icd::PlantEngine last_engines_{};
    bool have_engines_{false};
    std::uint32_t last_engine_frame_{0};
    double airspeed_kt_{0.0};
    bool have_airspeed_{false};
    std::uint32_t last_flight_frame_{0};
    bool afterburner_{false};
    bool have_throttle_{false};
    std::uint8_t last_throttle_{0};
    icd::CmdStatus cmd_status_{icd::CmdStatus::NONE};
    std::uint32_t self_test_started_{0};
    bool self_test_running_{false};
    bus::BitWord bit_{};
    bus::CommandSequence mode_seq_{};
    bus::CommandSequence cmd_seq_{};
};

}  // namespace fmofp::engine

#endif
