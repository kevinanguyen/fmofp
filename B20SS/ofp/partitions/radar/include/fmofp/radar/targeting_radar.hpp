#ifndef FMOFP_RADAR_TARGETING_RADAR_HPP
#define FMOFP_RADAR_TARGETING_RADAR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/radar/target_processing.hpp>

// The targeting radar, ported from
// Systems/radarManagement/targeting/targeting_radar.py and target_processor.py.
// The Python radar invented its targets and moved them itself; that world now
// lives in the plant, whose PLANT_TRAFFIC (relayed as RDR_TRAFFIC) says what
// the antenna would measure of each aircraft within reach: slant range, true
// azimuth and elevation, the velocity relative to the aircraft and the radar
// cross-section. The radar keeps the Python's track table from those
// detections (RDR_TGT_TRACKS), its signal-to-noise model, its lock and jamming
// flag, the signature analyser's classification and the stealth detector's
// assessment, and the track filter of RDR_TGT_MODE_CMD. Pure: inputs are ICD
// messages and blocks, outputs are ICD messages and a block; nothing here
// touches the bus or the clock.

namespace fmofp::radar {

constexpr double kTgtMaxRangeM = 100000.0;      // targeting_radar.max_range
constexpr double kTgtSnrReferenceDb = 30.0;     // snr = 30 - 20 log10(range / 1000); lock quality = min(1, snr / 30)
constexpr double kTgtLockedQuality = 0.5;       // LOCKED above this quality, ACQUIRING at or below
constexpr double kTgtJammingProbability = 0.05; // per update while locked
constexpr double kTgtTrackGateM = 1000.0;       // a detection within this of a track's predicted position continues it
constexpr double kTgtFrameSeconds = 0.02;
constexpr std::uint32_t kTgtTrackDropFrames = 15;    // a track not detected for this long is dropped (traffic comes every 10)
constexpr std::uint32_t kTgtTracksBlockFrames = 15;  // the block is republished no more often than this
constexpr std::uint32_t kTgtSelfTestFrames = 50;
constexpr std::size_t kTgtMaxTracks = icd::RdrTgtTracks::kMaxTrack;
constexpr std::size_t kTgtMaxDetections = icd::PlantTraffic::kMaxTarget;
constexpr std::uint8_t kTgtMaxReportedCount = 31;  // RDR_TGT_STATUS.track_count saturates

// One of the Python's current_targets entries, with the processors' fields.
struct TgtTrack {
    std::uint16_t id{0};
    std::array<double, 3> position{};  // metres east, north and up of the aircraft
    std::array<double, 3> velocity{};  // m/s relative to the aircraft
    double rcs{0.0};                   // as measured (the Python's raw 'rcs')
    double snr_db{0.0};
    double confidence{0.0};            // min(1, max(0, snr / 30))
    icd::TargetClass classification{icd::TargetClass::UNKNOWN};
    icd::IffIdentity identity{icd::IffIdentity::UNKNOWN};
    double detection_confidence{0.0};  // the signature analyser's
    double rcs_corrected{0.0};         // after the scintillation draw
    StealthState stealth{};
    double stealth_probability{0.0};
    bool is_stealth{false};
    icd::Threat threat{icd::Threat::UNKNOWN};
    std::uint32_t last_seen_frame{0};
    bool held{false};
};

class TargetingRadar {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `traffic` is an RDR_TRAFFIC block that completed
    // this frame, or null; `frame` is the partition's own frame count.
    void step(const icd::PlantTraffic* traffic, std::uint32_t frame) noexcept;

    // Applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::RdrTgtModeCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    // True once after each refresh of the tracks block: the process publishes it.
    [[nodiscard]] bool take_tracks_update() noexcept;

    [[nodiscard]] const icd::RdrTgtStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::RdrTgtTracks& tracks_block() const noexcept { return block_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::TargetingMode mode() const noexcept { return mode_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] icd::TrackFilter track_filter() const noexcept { return filter_; }
    [[nodiscard]] std::uint16_t locked_track() const noexcept { return locked_id_; }
    [[nodiscard]] double lock_quality() const noexcept { return lock_quality_; }
    [[nodiscard]] bool jamming() const noexcept { return jamming_; }
    [[nodiscard]] std::uint32_t updates() const noexcept { return updates_; }
    [[nodiscard]] bool operating() const noexcept;
    [[nodiscard]] std::size_t held_tracks() const noexcept;
    // Track slots 0..kTgtMaxTracks-1; `held` says whether a slot is in use.
    [[nodiscard]] const TgtTrack& slot(std::size_t index) const noexcept { return tracks_[index]; }
    [[nodiscard]] const TgtTrack* find(std::uint16_t id) const noexcept;
    [[nodiscard]] bool passes_filter(const TgtTrack& track) const noexcept;

    // The seed of the repeatable draws (scintillation, jamming); the tests set it.
    void set_seed(std::uint32_t seed) noexcept { seed_ = seed; }

private:
    void run_self_test(std::uint32_t frame) noexcept;
    void clear_tracks() noexcept;
    void release_lock() noexcept;
    void apply_lock_request(std::uint16_t id) noexcept;
    void update_tracks(const icd::PlantTraffic& traffic, std::uint32_t frame) noexcept;
    [[nodiscard]] std::size_t associate(const std::array<double, 3>& position, double dt,
                                        const std::array<bool, kTgtMaxTracks>& matched) const noexcept;
    [[nodiscard]] std::size_t new_slot() noexcept;
    void process_track(TgtTrack& track) noexcept;
    void drop_stale(std::uint32_t frame) noexcept;
    void update_lock() noexcept;
    void fill_status() noexcept;
    void fill_block() noexcept;

    // Outputs
    icd::RdrTgtStatus status_{};
    icd::RdrTgtTracks block_{};
    bool block_dirty_{true};
    bool block_ready_{false};
    bool block_published_{false};
    std::uint32_t last_block_frame_{0};
    // Settings
    icd::TargetingMode mode_{icd::TargetingMode::NONE};
    icd::TrackFilter filter_{icd::TrackFilter::ALL};
    bus::CommandSequence mode_seq_{};
    bool mode_transition_{false};
    // The track table
    std::array<TgtTrack, kTgtMaxTracks> tracks_{};
    std::uint16_t next_id_{1};
    std::uint16_t locked_id_{0};
    double lock_quality_{0.0};
    bool jamming_{false};
    std::uint32_t updates_{0};
    std::uint8_t update_seq_{0};
    std::uint32_t last_update_frame_{0};
    bool have_update_{false};
    std::uint32_t seed_{0x544754U};
    // Self test and health
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
    bus::BitWord bit_{};
};

}  // namespace fmofp::radar

#endif
