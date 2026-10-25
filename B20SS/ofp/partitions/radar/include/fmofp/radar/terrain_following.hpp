#ifndef FMOFP_RADAR_TERRAIN_FOLLOWING_HPP
#define FMOFP_RADAR_TERRAIN_FOLLOWING_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>

// The terrain-following radar, ported from
// Systems/radarManagement/terrainFollowing: tfr_processor.py's clearance
// manager and path optimiser, decision for decision, and tfr_radar.py's
// terrain warnings. The terrain itself is measured, not invented: the plant
// publishes the elevation profile ahead of the aircraft (PLANT_TERRAIN, relayed
// as RDR_TFR_TERRAIN) and the radar assesses it against the aircraft's altitude
// (RDR_TFR_SENSORS) at the set clearance (RDR_TFR_MODE_CMD). Outputs are the
// ICD's RDR_TFR_STATUS, RDR_TFR_GUIDANCE and the RDR_TFR_ELEVATION block. Pure:
// nothing here touches the bus or the clock.

namespace fmofp::radar {

// tfr_processor.py constants, metres.
constexpr double kClearM = 600.0;     // clearance at or above this: CLEAR
constexpr double kLowM = 300.0;       // ... LOW
constexpr double kCautionM = 150.0;   // ... CAUTION; below: PULL_UP
constexpr double kLookaheadM = 9260.0;  // the path optimiser's window (about 5 nm)
constexpr double kMaxAltRateMps = 25.0;  // altitude change the commands are limited to
constexpr double kCommandStepM = 20.0;   // a command is emitted when it moves by more than this
constexpr double kDefaultSpeedMps = 250.0;  // PathOptimiser's speed until told otherwise
constexpr double kMinSpeedMps = 50.0;       // PathOptimiser.update_speed's floor
constexpr std::size_t kFallbackWindowSamples = 20;  // when no sample lies within the look-ahead
constexpr std::size_t kMinWindowSamples = 5;        // the sliding window is max(5, n / 10) samples
// tfr_radar._check_terrain_warnings
constexpr double kHighTerrainM = 1500.0;
constexpr double kSteepReferenceM = 1000.0;
constexpr double kSteepBandM = 300.0;
// RDR_TFR_MODE_CMD set clearance, feet.
constexpr double kMinSetClearanceFt = 30.0;
constexpr double kMaxSetClearanceFt = 5000.0;
constexpr double kDefaultSetClearanceFt = 1000.0;  // until commanded; tfr_processor's 300 m was 984 ft
// Units, as the Python wrote them.
constexpr double kFtPerM = 3.28084;  // tfr_processor._FT_PER_M
constexpr double kMPerFt = 0.3048;
constexpr double kMpsPerKt = 0.514444;

constexpr std::size_t kMaxProfileSamples = icd::PlantTerrain::kMaxElevation;  // 100
constexpr std::size_t kMaxPathCommands = kMaxProfileSamples + 1;  // one per window sample plus the final one
constexpr std::size_t kMaxReportedCommands = icd::RdrTfrElevation::kMaxCommands;
constexpr std::uint32_t kSensorStaleFrames = 3;    // RDR_TFR_SENSORS is 50 Hz
constexpr std::uint32_t kTerrainStaleFrames = 15;  // a profile arrives about every 5 frames
constexpr std::uint32_t kElevationBlockFrames = 25;  // the block is republished no more often than this: its ten
                                                      // segments take 20 frames at the block's 25 Hz

struct TerrainProfile {
    std::size_t samples{0};
    double range_m{0.0};  // ground distance of the last sample
    std::uint8_t seq{0};
    std::array<double, kMaxProfileSamples> elevation_m{};

    // Sample i's ground distance: i x range / (samples - 1), the last exactly
    // range, which is what numpy.linspace gave the Python.
    [[nodiscard]] double distance_m(std::size_t i) const noexcept;
};

// ClearanceManager.assess_profile's result for one profile.
struct ClearanceAssessment {
    icd::ClearanceLevel master{icd::ClearanceLevel::CLEAR};  // the latched master level, after this profile
    double min_clearance_m{0.0};
    std::size_t min_index{0};
    std::size_t advisories{0};  // samples whose level is not CLEAR
    std::array<icd::ClearanceLevel, kMaxProfileSamples> level{};
};

// The level of one clearance (metres).
[[nodiscard]] icd::ClearanceLevel clearance_level(double clearance_m) noexcept;

// ClearanceManager.assess_profile: every sample's level, the minimum clearance,
// and the master level: it rises to the worst level seen and falls back to
// CLEAR only when a profile carries no advisory at all. `previous_master` is
// the latch before this profile (CLEAR after a reset).
void assess_profile(const TerrainProfile& profile, double altitude_m, icd::ClearanceLevel previous_master,
                    ClearanceAssessment& out) noexcept;

struct PathCommand {
    double distance_m{0.0};
    double altitude_m{0.0};
    icd::PathReason reason{icd::PathReason::NONE};
};

struct PathCommands {
    std::size_t count{0};
    std::array<PathCommand, kMaxPathCommands> command{};
};

// PathOptimiser.compute_path, with the set clearance in place of its fixed
// MIN_CLEARANCE_M: the required altitude is the highest terrain in a sliding
// window of max(5, n / 10) samples plus the clearance, limited to 25 m/s of
// change at `speed_mps` from the previous command, a command emitted when it
// moves by more than 20 m, and a final CLEARANCE command when the last one
// is below the window's highest terrain plus the clearance.
void compute_path(const TerrainProfile& profile, double current_alt_m, double speed_mps, double clearance_m,
                  PathCommands& out) noexcept;

// tfr_radar._check_terrain_warnings for one sample: HIGH_TERRAIN above
// 1500 m, else STEEP_TERRAIN more than 300 m from 1000 m.
[[nodiscard]] icd::TerrainWarning terrain_warning(double elevation_m) noexcept;

class TerrainFollowing {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `sensors` is the RDR_TFR_SENSORS received this
    // frame, or null; `terrain` a profile that completed this frame, or null.
    void step(const icd::RdrTfrSensors* sensors, const icd::PlantTerrain* terrain, std::uint32_t frame) noexcept;

    // Applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::RdrTfrModeCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    // True once after each change of the elevation block's contents.
    [[nodiscard]] bool take_products_update() noexcept;

    [[nodiscard]] const icd::RdrTfrStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::RdrTfrGuidance& guidance() const noexcept { return guidance_; }
    [[nodiscard]] const icd::RdrTfrElevation& products() const noexcept { return products_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::TfrMode mode() const noexcept { return mode_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] double set_clearance_ft() const noexcept { return set_clearance_ft_; }
    [[nodiscard]] std::uint32_t profiles_assessed() const noexcept { return profiles_assessed_; }

    // ---- for the oracle tool and the tests -------------------------------
    [[nodiscard]] const TerrainProfile& profile() const noexcept { return profile_; }
    [[nodiscard]] const ClearanceAssessment& assessment() const noexcept { return assessment_; }
    [[nodiscard]] const PathCommands& path() const noexcept { return path_; }

private:
    [[nodiscard]] bool operating() const noexcept;
    void run_self_test(std::uint32_t frame) noexcept;
    void assess(double altitude_ft, double speed_mps) noexcept;
    void fill_guidance(double altitude_m) noexcept;
    void invalidate_guidance() noexcept;
    void fill_products(double altitude_ft) noexcept;
    void clear_products() noexcept;

    // Outputs
    icd::RdrTfrStatus status_{};
    icd::RdrTfrGuidance guidance_{};
    icd::RdrTfrElevation products_{};
    bool products_dirty_{false};
    bool products_cleared_{false};
    std::uint32_t last_block_frame_{0};
    bool block_published_{false};
    // Settings and inputs
    icd::TfrMode mode_{icd::TfrMode::NONE};
    double set_clearance_ft_{kDefaultSetClearanceFt};
    bus::CommandSequence mode_seq_{};
    icd::PlantSensors sensors_{};
    bool have_sensors_{false};
    std::uint32_t sensor_frame_{0};
    TerrainProfile profile_{};
    bool have_profile_{false};
    bool profile_unassessed_{false};
    std::uint32_t profile_frame_{0};
    // The processors' state
    icd::ClearanceLevel master_{icd::ClearanceLevel::CLEAR};
    ClearanceAssessment assessment_{};
    PathCommands path_{};
    std::uint32_t profiles_assessed_{0};
    bool mode_transition_{false};
    // Self test and health
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
    bus::BitWord bit_{};
};

}  // namespace fmofp::radar

#endif
