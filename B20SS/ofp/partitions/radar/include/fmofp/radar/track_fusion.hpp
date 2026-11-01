#ifndef FMOFP_RADAR_TRACK_FUSION_HPP
#define FMOFP_RADAR_TRACK_FUSION_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/radar/targeting_radar.hpp>

// The cross-radar track fusion, ported from
// Systems/radarManagement/radar_data_fusion.py: the radars' tracks become
// reports, reports within 500 m of a cluster's centroid join it (heaviest
// sensor first), each cluster merges into one fused track (positions and
// velocities averaged by sensor weight, confidence the probability union of
// the weights, class and identity from the heaviest report, the threat from
// the rule set), and a fused track keeps the id of the previous epoch's track
// nearest within 500 m. An epoch every 25 frames, the Python's 2 Hz. The
// table is rebuilt from the sensors' current tracks every epoch, as the
// Python's was; its 30 s ageing never had a track left to age. Output: the
// RDR_FUSED_TRACKS block. Pure, like the radars.

namespace fmofp::radar {

constexpr double kFusionGateM = 500.0;             // radar_data_fusion.GATE_M
constexpr double kFusionWeightTargeting = 0.8;     // _W_TARGETING
constexpr double kFusionWeightAewc = 0.7;          // _W_AEWC
constexpr std::uint32_t kFusionEpochFrames = 25;   // UPDATE_HZ 2 at 50 frames a second
constexpr std::uint8_t kSensorTargeting = 1U;      // TrackReport::sensor and FusedTrack::sources bits
constexpr std::uint8_t kSensorAewc = 2U;
constexpr std::size_t kFusionMaxReports = 2 * kTgtMaxTracks;  // the targeting radar's tracks and, later, the AEWC's
constexpr std::size_t kFusionMaxTracks = icd::RdrFusedTracks::kMaxTrack;

// RadarDataFusion._collect_reports' entry for one radar track.
struct TrackReport {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    icd::TargetClass classification{icd::TargetClass::UNKNOWN};
    icd::IffIdentity identity{icd::IffIdentity::UNKNOWN};
    double rcs{0.0};
    double weight{0.0};
    std::uint8_t sensor{0};
};

struct FusedTrack {
    std::uint16_t id{0};
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    double confidence{0.0};
    icd::TargetClass classification{icd::TargetClass::UNKNOWN};
    icd::IffIdentity identity{icd::IffIdentity::UNKNOWN};
    icd::Threat threat{icd::Threat::UNKNOWN};
    std::uint8_t sources{0};
};

// The targeting radar's held tracks as reports (weight 0.8). Returns how many were written.
[[nodiscard]] std::size_t targeting_reports(const TargetingRadar& radar, TrackReport* out, std::size_t capacity) noexcept;

class TrackFusion {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame: an epoch every kFusionEpochFrames over the reports given.
    void step(const TrackReport* reports, std::size_t n_reports, bool targeting_present, bool aewc_present,
              std::uint32_t frame) noexcept;

    // One epoch now (RadarDataFusion._update), for the tool and the tests.
    void epoch(const TrackReport* reports, std::size_t n_reports, bool targeting_present, bool aewc_present) noexcept;

    // True once after each epoch: the process publishes the block.
    [[nodiscard]] bool take_update() noexcept;

    [[nodiscard]] const icd::RdrFusedTracks& block() const noexcept { return block_; }
    [[nodiscard]] std::size_t count() const noexcept { return count_; }
    [[nodiscard]] const FusedTrack& track(std::size_t index) const noexcept { return tracks_[index]; }
    [[nodiscard]] std::uint8_t hostile_count() const noexcept;
    [[nodiscard]] std::uint8_t epochs() const noexcept { return epoch_; }

private:
    struct Cluster {
        std::size_t members{0};
        std::array<double, 3> sum_position{};           // for the centroid the gate is measured from
        std::array<double, 3> weighted_position{};      // sums of weight x value
        std::array<double, 3> weighted_velocity{};
        double total_weight{0.0};
        double miss_product{1.0};                       // product of (1 - weight)
        double sum_rcs{0.0};
        double first_speed{0.0};                        // cluster[0]'s speed, which the threat rule uses
        double anchor_weight{-1.0};
        icd::TargetClass anchor_class{icd::TargetClass::UNKNOWN};
        icd::IffIdentity anchor_identity{icd::IffIdentity::UNKNOWN};
        std::uint8_t sources{0};
    };

    void sort_by_weight(const TrackReport* reports, std::size_t n_reports) noexcept;
    [[nodiscard]] std::size_t correlate(const TrackReport* reports, std::size_t n_reports) noexcept;
    void merge(std::size_t clusters) noexcept;
    void carry_ids() noexcept;
    void fill_block(bool targeting_present, bool aewc_present) noexcept;

    std::array<FusedTrack, kFusionMaxTracks> tracks_{};
    std::size_t count_{0};
    std::array<FusedTrack, kFusionMaxTracks> previous_{};
    std::size_t previous_count_{0};
    std::array<std::size_t, kFusionMaxReports> order_{};
    std::array<Cluster, kFusionMaxReports> clusters_{};
    std::uint16_t next_id_{1};
    std::uint8_t epoch_{0};
    std::uint32_t last_epoch_frame_{0};
    bool have_epoch_{false};
    bool update_ready_{false};
    icd::RdrFusedTracks block_{};
};

}  // namespace fmofp::radar

#endif
