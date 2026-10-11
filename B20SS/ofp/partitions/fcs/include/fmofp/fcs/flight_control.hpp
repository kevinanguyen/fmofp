#ifndef FMOFP_FCS_FLIGHT_CONTROL_HPP
#define FMOFP_FCS_FLIGHT_CONTROL_HPP

#include <array>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/messages.hpp>

// The flight control partition's logic, ported from
// Systems/flightControlSys (groundCollisionAvoidanceSys.py and
// performaneMonitoring.py) and the FCS command handling the ICD defines.
// Everything here is pure: inputs are ICD messages, outputs are ICD messages,
// and nothing touches the bus or the clock, so the tests can drive it frame
// by frame. The random-walk "flight computer" of the Python is simulation,
// not flight logic; it lives in the plant terminal, not here.

namespace fmofp::fcs {

// groundCollisionAvoidanceSys.py module constants.
struct GcasThresholds {
    double pullup_vs_fpm{-4000.0};   // PULLUP when vs <= this ...
    double pullup_alt_ft{2000.0};    // ... and altitude < this
    double sink_vs_fpm{-2000.0};     // SINK RATE when vs <= this
    double terrain_ft{500.0};        // TERRAIN when clearance < this
    double alt_low_ft{200.0};        // ALT LOW when altitude < this (and no TERRAIN)
};

// performaneMonitoring.PerformanceMonitor.LIMITS: an exceedance while |value| > limit.
struct EnvelopeLimits {
    double g{9.0};
    double aoa_deg{25.0};
    double roll_deg{60.0};
    double pitch_deg{45.0};
    double vs_fpm{6000.0};
};

// Sensor data older than this many frames is treated as absent (ICD README
// section 4: stale after 3 periods; FCS_SENSORS is 50 Hz, one per frame).
constexpr std::uint32_t kSensorStaleFrames = 3;

class FlightControl {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `sensors` is the FCS_SENSORS received this frame,
    // or null if none arrived; `frame` is the partition's own frame count.
    void step(const icd::FcsSensors* sensors, std::uint32_t frame) noexcept;

    // Commands, each applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::FcsModeCmd& cmd) noexcept;
    void command_surface(const icd::FcsSurfaceCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    [[nodiscard]] const icd::FcsStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::FcsAttitude& attitude() const noexcept { return attitude_; }
    [[nodiscard]] const icd::FcsGcasAlerts& gcas() const noexcept { return gcas_; }
    [[nodiscard]] const icd::FcsPerfStatus& perf() const noexcept { return perf_; }
    [[nodiscard]] const icd::FcsSurfaceStatus& surfaces() const noexcept { return surfaces_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }

    // The two decision functions, exposed so tests can table them against the Python.
    static icd::FcsGcasAlerts evaluate_gcas(const icd::PlantSensors& s, const GcasThresholds& t) noexcept;
    static icd::FcsPerfStatus evaluate_envelope(const icd::PlantSensors& s, const EnvelopeLimits& l) noexcept;

private:
    void produce_outputs(bool sensors_valid) noexcept;
    void invalidate_outputs() noexcept;

    icd::FcsStatus status_{};
    icd::FcsAttitude attitude_{};
    icd::FcsGcasAlerts gcas_{};
    icd::FcsPerfStatus perf_{};
    icd::FcsSurfaceStatus surfaces_{};
    icd::PlantSensors last_sensors_{};
    bool have_sensors_{false};
    std::uint32_t last_sensor_frame_{0};
    std::uint32_t self_test_started_{0};
    bool self_test_running_{false};
    bus::BitWord bit_{};
    bus::CommandSequence mode_seq_{};
    bus::CommandSequence surface_seq_{};
    GcasThresholds gcas_thresholds_{};
    EnvelopeLimits limits_{};
};

}  // namespace fmofp::fcs

#endif
