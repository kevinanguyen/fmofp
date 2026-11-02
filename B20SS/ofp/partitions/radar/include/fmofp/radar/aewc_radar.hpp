#ifndef FMOFP_RADAR_AEWC_RADAR_HPP
#define FMOFP_RADAR_AEWC_RADAR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>

// The airborne early warning and control radar, ported from
// Systems/radarManagement/aewc/aewc_radar.py and aewc_processor.py. The
// Python radar invented up to ten targets within 400 km and moved them itself;
// that world is the plant's traffic (PLANT_TRAFFIC, relayed as RDR_TRAFFIC),
// which says what the antenna would measure of every aircraft within reach.
// The radar keeps the Python's track table from those detections, its radar
// equation (signal-to-noise ratio, and from it the track confidence and the
// chance of detecting a stealth aircraft), its classification rule, its six
// 60-degree sectors with their scan progress and detected tracks, the sector
// priority manager and the electronic protection assessment of
// aewc_processor.py (RDR_AEWC_STATUS, RDR_AEWC_TRACKS). Pure: inputs are ICD
// messages and blocks, outputs are ICD messages and a block.

namespace fmofp::radar {

constexpr double kAewcMaxRangeM = 400000.0;          // aewc_radar.max_range
constexpr double kAewcStealthRcsM2 = 0.1;            // the generator's stealth band tops at 0.1 m2; conventional starts at 1
constexpr double kAewcStealthProbability = 0.2;      // stealth_probability
constexpr double kAewcSnrReferenceDb = 30.0;         // confidence = snr / 30; stealth detection 0.2 x snr / 30
constexpr double kAewcTxPowerW = 100000.0;           // _calculate_snr: 100 kW
constexpr double kAewcAntennaGain = 10000.0;         // 40 dB
constexpr double kAewcWavelengthM = 0.03;            // 10 GHz
constexpr double kAewcNoiseFloorDbm = -110.0;        // noise_floor (the Python subtracts dBm from dBW: kept)
constexpr double kAewcFighterSpeedMps = 400.0;       // _generate_target's classification rule
constexpr double kAewcHighAltitudeM = 30000.0;
constexpr std::size_t kAewcSectorCount = 6;          // _initialize_sectors
constexpr double kAewcSectorWidthDeg = 60.0;
constexpr double kAewcSectorElevationMinDeg = -15.0;
constexpr double kAewcSectorElevationMaxDeg = 45.0;
constexpr double kAewcScanProgressStep = 0.1;        // _update_targets: a tenth a second, round and round
constexpr std::uint32_t kAewcScanProgressFrames = 50;
constexpr double kAewcTrackGateM = 1000.0;           // a detection within this of a track's predicted position continues it
constexpr std::uint32_t kAewcTrackDropFrames = 15;   // traffic comes every 10 frames
constexpr std::uint32_t kAewcTracksBlockFrames = 35; // six segments polled every fifth frame take 30 frames to deliver
constexpr std::uint32_t kAewcSelfTestFrames = 50;
constexpr double kAewcFrameSeconds = 0.02;
constexpr std::size_t kAewcMaxTracks = icd::RdrAewcTracks::kMaxTrack;
constexpr std::size_t kAewcMaxDetections = icd::PlantTraffic::kMaxTarget;
constexpr std::uint8_t kAewcMaxReportedCount = 31;   // RDR_AEWC_STATUS.track_count saturates
constexpr std::uint8_t kAewcMaxReportedStealth = 15;
// SectorPriorityManager
constexpr double kAewcDensityWeight = 1.0;   // W_DENSITY
constexpr double kAewcThreatWeight = 3.0;    // W_THREAT
constexpr double kAewcDwellAgeWeight = 0.5;  // W_DWELL_AGE
constexpr double kAewcMinDwell = 0.05;       // MIN_DWELL
constexpr double kAewcJammedBonus = 2.0;     // a jammed sector's score bonus
// ElectronicProtection
constexpr double kEcmThresholdDb = 6.0;          // ECM_THRESHOLD_DB
constexpr double kEcmSeverityFullScaleDb = 40.0; // severity = min(1, exceedance / 40)
constexpr double kEcmNoiseSigmaDb = 3.0;         // the radar's own noise draw: gauss(0, 3) over the floor
constexpr double kEcmContinuousDb = 30.0;
constexpr double kEcmSpotDb = 20.0;
constexpr double kEcmSweepDb = 10.0;
constexpr double kEcmBeamNotchSeverity = 0.4;
constexpr double kEcmPowerSeverity = 0.7;
constexpr double kEcmSkipSeverity = 0.9;

// aewc_radar._calculate_snr: the radar equation in watts, read as dBm against
// the -110 dBm floor as the Python read it, never negative.
[[nodiscard]] double aewc_snr_db(double rcs_m2, double range_m) noexcept;

// aewc_radar._get_target_sector: 1..6 by azimuth measured counter-clockwise
// from east (atan2(north, east), the Python's), 0 outside the elevation band.
[[nodiscard]] std::size_t aewc_sector_of(const std::array<double, 3>& position_m) noexcept;

// _generate_target's classification: STEALTH, else FIGHTER above 400 m/s,
// else HIGH_ALT beyond 30 km above or below, else UNKNOWN.
[[nodiscard]] icd::TargetClass aewc_classify(double speed_mps, double altitude_m, bool stealth) noexcept;

struct EcmAssessment {
    bool jammed{false};
    double severity{0.0};
    icd::JamType jam_type{icd::JamType::NONE};
    icd::EcmMitigation mitigation{icd::EcmMitigation::NONE};
};

// ElectronicProtection.assess_sector for a noise exceedance over the thermal floor.
[[nodiscard]] EcmAssessment ecm_assess(double exceedance_db) noexcept;

struct SectorPriorities {
    std::array<double, kAewcSectorCount> score{};
    std::array<double, kAewcSectorCount> dwell{};
    std::array<std::uint8_t, kAewcSectorCount> order{};  // sector indices 0..5, highest score first (ties in sector order)
};

// SectorPriorityManager.update for the six sectors.
[[nodiscard]] SectorPriorities aewc_priorities(const std::array<std::uint8_t, kAewcSectorCount>& tracks,
                                               const std::array<std::uint8_t, kAewcSectorCount>& hostile,
                                               const std::array<double, kAewcSectorCount>& age_s,
                                               std::uint8_t jammed_mask) noexcept;

// One of the Python's current_targets entries.
struct AewcTrack {
    std::uint16_t id{0};
    std::array<double, 3> position{};  // metres east, north and up of the aircraft
    std::array<double, 3> velocity{};  // m/s relative to the aircraft
    double rcs{0.0};
    double snr_db{0.0};
    double confidence{0.0};            // min(1, snr / 30)
    icd::TargetClass classification{icd::TargetClass::UNKNOWN};
    icd::IffIdentity identity{icd::IffIdentity::UNKNOWN};
    icd::Threat threat{icd::Threat::UNKNOWN};
    bool stealth{false};
    bool detected{false};              // the last detection succeeded (always, unless stealth)
    std::uint8_t sector{0};            // 1..6, 0 outside the elevation band (or not detected)
    std::uint32_t last_seen_frame{0};
    bool held{false};
};

// One of the Python's Sector objects with its SectorStatus and JammerContact.
struct AewcSector {
    std::uint8_t active_tracks{0};     // detected tracks in the sector this update
    std::uint8_t all_tracks{0};        // every held track in the sector (what the priority manager counted)
    double scan_progress{0.0};
    double priority_score{0.0};
    double dwell_fraction{1.0 / static_cast<double>(kAewcSectorCount)};
    bool jammed{false};
    EcmAssessment ecm{};
};

class AewcRadar {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `traffic` is an RDR_TRAFFIC block that completed this frame, or null.
    void step(const icd::PlantTraffic* traffic, std::uint32_t frame) noexcept;

    // Applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::RdrAewcModeCmd& cmd) noexcept;

    void reset() noexcept;
    void start_self_test() noexcept;

    // True once after each refresh of the tracks block: the process publishes it.
    [[nodiscard]] bool take_tracks_update() noexcept;

    [[nodiscard]] const icd::RdrAewcStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::RdrAewcTracks& tracks_block() const noexcept { return block_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::AewcMode mode() const noexcept { return mode_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] std::uint32_t updates() const noexcept { return updates_; }
    [[nodiscard]] bool operating() const noexcept;
    [[nodiscard]] std::size_t held_tracks() const noexcept;
    [[nodiscard]] std::size_t stealth_tracks() const noexcept;
    [[nodiscard]] const AewcTrack& slot(std::size_t index) const noexcept { return tracks_[index]; }
    [[nodiscard]] const AewcTrack* find(std::uint16_t id) const noexcept;
    [[nodiscard]] const AewcSector& sector(std::size_t index) const noexcept { return sectors_[index]; }
    [[nodiscard]] std::uint8_t jammed_mask() const noexcept;
    [[nodiscard]] std::uint8_t priority_sector() const noexcept { return priority_sector_; }

    // The seed of the repeatable draws (stealth detection, receiver noise); the tests set it.
    void set_seed(std::uint32_t seed) noexcept { seed_ = seed; }

private:
    void run_self_test(std::uint32_t frame) noexcept;
    void clear_tracks() noexcept;
    void update_tracks(const icd::PlantTraffic& traffic, std::uint32_t frame) noexcept;
    [[nodiscard]] std::size_t associate(const std::array<double, 3>& position, double dt,
                                        const std::array<bool, kAewcMaxTracks>& matched) const noexcept;
    [[nodiscard]] std::size_t new_slot() noexcept;
    void process_track(AewcTrack& track, double range_m) noexcept;
    void process_sectors(std::uint32_t frame) noexcept;
    void advance_scan(std::uint32_t frame) noexcept;
    void drop_stale(std::uint32_t frame) noexcept;
    void fill_status() noexcept;
    void fill_block() noexcept;

    // Outputs
    icd::RdrAewcStatus status_{};
    icd::RdrAewcTracks block_{};
    bool block_dirty_{true};
    bool block_ready_{false};
    bool block_published_{false};
    std::uint32_t last_block_frame_{0};
    // Settings
    icd::AewcMode mode_{icd::AewcMode::NONE};
    bus::CommandSequence mode_seq_{};
    bool mode_transition_{false};
    double scan_start_deg_{0.0};  // RDR_AEWC_MODE_CMD's sector, reported in the block
    double scan_end_deg_{90.0};
    // The track table and the sectors
    std::array<AewcTrack, kAewcMaxTracks> tracks_{};
    std::array<AewcSector, kAewcSectorCount> sectors_{};
    std::uint8_t priority_sector_{0};
    std::uint16_t next_id_{1};
    std::uint32_t updates_{0};
    std::uint8_t update_seq_{0};
    std::uint32_t last_update_frame_{0};
    bool have_update_{false};
    std::uint32_t init_frame_{0};       // the sectors' last scan: never, as in the Python
    std::uint32_t last_progress_frame_{0};
    bool have_progress_{false};
    std::uint32_t seed_{0x414557U};
    // Self test and health
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
    bus::BitWord bit_{};
};

}  // namespace fmofp::radar

#endif
