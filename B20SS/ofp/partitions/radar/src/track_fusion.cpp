#include <fmofp/radar/track_fusion.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::radar {

namespace {

double distance_m(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double magnitude(const std::array<double, 3>& v) noexcept { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

}  // namespace

std::size_t targeting_reports(const TargetingRadar& radar, TrackReport* out, std::size_t capacity) noexcept {
    std::size_t n = 0;
    for (std::size_t s = 0; s < kTgtMaxTracks && n < capacity; ++s) {
        const TgtTrack& source = radar.slot(s);
        if (!source.held) {
            continue;
        }
        TrackReport& report = out[n++];
        report.position = source.position;
        report.velocity = source.velocity;
        report.classification = source.classification;
        report.identity = source.identity;
        report.rcs = source.rcs;
        report.weight = kFusionWeightTargeting;
        report.sensor = kSensorTargeting;
    }
    return n;
}

std::size_t aewc_reports(const AewcRadar& radar, TrackReport* out, std::size_t capacity) noexcept {
    std::size_t n = 0;
    for (std::size_t s = 0; s < kAewcMaxTracks && n < capacity; ++s) {
        const AewcTrack& source = radar.slot(s);
        if (!source.held) {
            continue;
        }
        TrackReport& report = out[n++];
        report.position = source.position;
        report.velocity = source.velocity;
        report.classification = source.classification;
        report.identity = source.identity;
        report.rcs = source.rcs;
        report.weight = kFusionWeightAewc;
        report.sensor = kSensorAewc;
    }
    return n;
}

Status TrackFusion::init() noexcept {
    std::fill(tracks_.begin(), tracks_.end(), FusedTrack{});
    std::fill(previous_.begin(), previous_.end(), FusedTrack{});
    count_ = 0;
    previous_count_ = 0;
    next_id_ = 1;
    epoch_ = 0;
    last_epoch_frame_ = 0;
    have_epoch_ = false;
    block_ = icd::RdrFusedTracks{};
    update_ready_ = true;  // the empty block is published first
    return Status::kOk;
}

void TrackFusion::step(const TrackReport* reports, std::size_t n_reports, bool targeting_present, bool aewc_present,
                       std::uint32_t frame) noexcept {
    if (have_epoch_ && frame - last_epoch_frame_ < kFusionEpochFrames) {
        return;
    }
    epoch(reports, n_reports, targeting_present, aewc_present);
    have_epoch_ = true;
    last_epoch_frame_ = frame;
}

bool TrackFusion::take_update() noexcept {
    const bool ready = update_ready_;
    update_ready_ = false;
    return ready;
}

std::uint8_t TrackFusion::hostile_count() const noexcept {
    const auto n = std::count_if(tracks_.begin(), tracks_.begin() + static_cast<std::ptrdiff_t>(count_),
                                 [](const FusedTrack& f) { return f.threat == icd::Threat::HOSTILE; });
    return static_cast<std::uint8_t>(std::min<std::ptrdiff_t>(n, 255));
}

// sorted(reports, key=lambda r: -r["weight"]): heaviest first, ties in the
// reports' order (Python's sort is stable; so is an insertion sort).
void TrackFusion::sort_by_weight(const TrackReport* reports, std::size_t n_reports) noexcept {
    for (std::size_t i = 0; i < n_reports; ++i) {
        std::size_t j = i;
        while (j > 0 && reports[order_[j - 1]].weight < reports[i].weight) {
            order_[j] = order_[j - 1];
            --j;
        }
        order_[j] = i;
    }
}

// Greedy nearest-centroid clustering within the gate. Returns the cluster count.
std::size_t TrackFusion::correlate(const TrackReport* reports, std::size_t n_reports) noexcept {
    std::size_t clusters = 0;
    for (std::size_t k = 0; k < n_reports; ++k) {
        const TrackReport& report = reports[order_[k]];
        std::size_t best = clusters;
        double best_distance = kFusionGateM;
        for (std::size_t c = 0; c < clusters; ++c) {
            const Cluster& cluster = clusters_[c];
            const double members = static_cast<double>(cluster.members);
            const std::array<double, 3> centroid{cluster.sum_position[0] / members, cluster.sum_position[1] / members,
                                                 cluster.sum_position[2] / members};
            const double d = distance_m(report.position, centroid);
            if (d < best_distance) {
                best_distance = d;
                best = c;
            }
        }
        if (best == clusters) {
            clusters_[clusters] = Cluster{};
            clusters_[clusters].first_speed = magnitude(report.velocity);
            ++clusters;
        }
        Cluster& cluster = clusters_[best];
        ++cluster.members;
        for (std::size_t i = 0; i < 3; ++i) {
            cluster.sum_position[i] += report.position[i];
            cluster.weighted_position[i] += report.position[i] * report.weight;
            cluster.weighted_velocity[i] += report.velocity[i] * report.weight;
        }
        cluster.total_weight += report.weight;
        cluster.miss_product *= 1.0 - report.weight;
        cluster.sum_rcs += report.rcs;
        if (report.weight > cluster.anchor_weight) {  // max() keeps the first of equals
            cluster.anchor_weight = report.weight;
            cluster.anchor_class = report.classification;
            cluster.anchor_identity = report.identity;
        }
        cluster.sources = static_cast<std::uint8_t>(cluster.sources | report.sensor);
    }
    return clusters;
}

// _merge_cluster for every cluster, in cluster order; clusters beyond the block's capacity are not reported.
void TrackFusion::merge(std::size_t clusters) noexcept {
    count_ = std::min(clusters, kFusionMaxTracks);
    for (std::size_t c = 0; c < count_; ++c) {
        const Cluster& cluster = clusters_[c];
        FusedTrack& fused = tracks_[c];
        fused = FusedTrack{};
        for (std::size_t i = 0; i < 3; ++i) {
            fused.position[i] = cluster.weighted_position[i] / cluster.total_weight;
            fused.velocity[i] = cluster.weighted_velocity[i] / cluster.total_weight;
        }
        fused.confidence = std::fmin(1.0, 1.0 - cluster.miss_product);
        fused.classification = cluster.anchor_class;
        fused.identity = cluster.anchor_identity;
        const double average_rcs = cluster.sum_rcs / static_cast<double>(cluster.members);
        fused.threat = assess_threat(fused.classification, fused.identity, average_rcs, cluster.first_speed);
        fused.sources = cluster.sources;
    }
}

// _reassign_ids: each new track takes the id of the nearest unused previous
// track within the gate, else a new one.
void TrackFusion::carry_ids() noexcept {
    std::array<bool, kFusionMaxTracks> used{};
    for (std::size_t i = 0; i < count_; ++i) {
        std::size_t best = previous_count_;
        double best_distance = kFusionGateM;
        for (std::size_t j = 0; j < previous_count_; ++j) {
            if (used[j]) {
                continue;
            }
            const double d = distance_m(tracks_[i].position, previous_[j].position);
            if (d < best_distance) {
                best_distance = d;
                best = j;
            }
        }
        if (best < previous_count_) {
            tracks_[i].id = previous_[best].id;
            used[best] = true;
        } else {
            tracks_[i].id = next_id_;
            next_id_ = static_cast<std::uint16_t>(next_id_ == 0xFFFFU ? 1U : next_id_ + 1U);
        }
    }
}

void TrackFusion::epoch(const TrackReport* reports, std::size_t n_reports, bool targeting_present,
                        bool aewc_present) noexcept {
    const std::size_t n = reports == nullptr ? 0 : std::min(n_reports, kFusionMaxReports);
    sort_by_weight(reports, n);
    const std::size_t clusters = correlate(reports, n);
    merge(clusters);
    carry_ids();
    for (std::size_t i = 0; i < count_; ++i) {
        previous_[i] = tracks_[i];
    }
    previous_count_ = count_;
    epoch_ = static_cast<std::uint8_t>(epoch_ + 1U);
    fill_block(targeting_present, aewc_present);
    update_ready_ = true;
}

void TrackFusion::fill_block(bool targeting_present, bool aewc_present) noexcept {
    block_ = icd::RdrFusedTracks{};
    block_.epoch = epoch_;
    block_.hostile_count = hostile_count();
    block_.targeting_present = targeting_present;
    block_.aewc_present = aewc_present;
    std::array<std::size_t, kFusionMaxTracks> order{};
    for (std::size_t i = 0; i < count_; ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(count_),
              [this](std::size_t a, std::size_t b) { return tracks_[a].id < tracks_[b].id; });
    for (std::size_t i = 0; i < count_; ++i) {
        const FusedTrack& fused = tracks_[order[i]];
        icd::RdrFusedTracksTrack& rec = block_.track[i];
        rec.track_id = fused.id;
        rec.x = fused.position[0];
        rec.y = fused.position[1];
        rec.z = fused.position[2];
        rec.velocity_east = fused.velocity[0];
        rec.velocity_north = fused.velocity[1];
        rec.velocity_up = fused.velocity[2];
        rec.classification = fused.classification;
        rec.identity = fused.identity;
        rec.threat = fused.threat;
        rec.from_targeting = (fused.sources & kSensorTargeting) != 0U;
        rec.from_aewc = (fused.sources & kSensorAewc) != 0U;
        rec.confidence = fused.confidence;
    }
    block_.track_count = static_cast<std::uint16_t>(count_);
}

}  // namespace fmofp::radar
