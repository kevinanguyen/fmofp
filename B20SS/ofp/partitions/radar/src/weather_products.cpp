#include <fmofp/radar/weather_products.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::radar {

namespace {

constexpr double kVilCoefficient = 3.44 * 1e-3;  // the Python's 3.44 * 10**-3
constexpr double kVilExponent = 4.0 / 7.0;
constexpr double kMarshallPalmerA = 200.0;
constexpr double kMarshallPalmerB = 1.6;
constexpr double kMinLayerThicknessM = 100.0;
constexpr double kBeamHeightScale = 1000.0;  // sin(elevation) difference to metres, as the Python had it

}  // namespace

// Stable insertion sort: fine for the few hundred points and clusters it sees.
void stable_sort_descending(const double* keys, std::uint16_t* order, std::size_t n) noexcept {
    for (std::size_t i = 0; i < n; ++i) {
        order[i] = static_cast<std::uint16_t>(i);
    }
    for (std::size_t i = 1; i < n; ++i) {
        const std::uint16_t item = order[i];
        std::size_t j = i;
        while (j > 0 && keys[order[j - 1]] < keys[item]) {
            order[j] = order[j - 1];
            --j;
        }
        order[j] = item;
    }
}

namespace {

// Stable bottom-up merge sort of indices by key, largest first; `scratch` holds n entries.
void merge_sort_descending(const double* keys, std::uint16_t* order, std::uint16_t* scratch, std::size_t n) noexcept {
    for (std::size_t i = 0; i < n; ++i) {
        order[i] = static_cast<std::uint16_t>(i);
    }
    for (std::size_t width = 1; width < n; width *= 2) {
        for (std::size_t lo = 0; lo < n; lo += 2 * width) {
            const std::size_t mid = std::min(lo + width, n);
            const std::size_t hi = std::min(lo + 2 * width, n);
            std::size_t a = lo;
            std::size_t b = mid;
            std::size_t k = lo;
            while (a < mid && b < hi) {
                // Take from the right only when strictly larger, so equals keep their order.
                if (keys[order[b]] > keys[order[a]]) {
                    scratch[k++] = order[b++];
                } else {
                    scratch[k++] = order[a++];
                }
            }
            while (a < mid) {
                scratch[k++] = order[a++];
            }
            while (b < hi) {
                scratch[k++] = order[b++];
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            order[i] = scratch[i];
        }
    }
}

double distance(double x1, double y1, double x2, double y2) noexcept {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double region_intensity(double reflectivity, double min_dbz) noexcept {
    const double intensity = (reflectivity - min_dbz) / (65.0 - min_dbz);
    return std::fmin(1.0, std::fmax(0.0, intensity));
}

}  // namespace

// ---- rounding and the per-column products -----------------------------------

double round_to_sigfigs(double x, int sigfigs) noexcept {
    if (!(x != 0.0) || !std::isfinite(x)) {
        return x;
    }
    const double magnitude = std::floor(std::log10(std::fabs(x)));
    const double scale = std::pow(10.0, static_cast<double>(sigfigs) - magnitude - 1.0);
    return std::nearbyint(x * scale) / scale;
}

std::array<double, kMaxElevations> layer_thicknesses(const Vcp& vcp) noexcept {
    std::array<double, kMaxElevations> out{};
    const std::size_t n = std::min(vcp.count, kMaxElevations);
    if (n == 0) {
        return out;
    }
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const double thickness =
            kBeamHeightScale * (std::sin(vcp.elevations[i + 1] * kDegToRad) - std::sin(vcp.elevations[i] * kDegToRad));
        out[i] = std::fmax(kMinLayerThicknessM, thickness);
    }
    out[n - 1] = n > 1 ? out[n - 2] : 1000.0;
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = round_to_sigfigs(out[i]);
    }
    return out;
}

double vil_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r,
                  const std::array<double, kMaxElevations>& thickness) noexcept {
    double total = 0.0;
    for (std::size_t el = 0; el < nel && el < kMaxElevations; ++el) {
        const double dbz = v.dbz[az][el][r];
        if (dbz > 0.0) {
            const double z = round_to_sigfigs(std::pow(10.0, dbz / 10.0));
            const double contribution = round_to_sigfigs(kVilCoefficient * std::pow(z, kVilExponent) * thickness[el]);
            total += contribution;
        }
    }
    return round_to_sigfigs(total);
}

double composite_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r) noexcept {
    double max_dbz = -INFINITY;
    for (std::size_t el = 0; el < nel && el < kMaxElevations; ++el) {
        if (v.dbz[az][el][r] > max_dbz) {
            max_dbz = v.dbz[az][el][r];
        }
    }
    return max_dbz;
}

double precip_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r) noexcept {
    const double max_dbz = composite_column(v, nel, az, r);
    if (!(max_dbz > 0.0)) {
        return 0.0;
    }
    const double z = round_to_sigfigs(std::pow(10.0, max_dbz / 10.0));
    return round_to_sigfigs(round_to_sigfigs(std::pow(z / kMarshallPalmerA, 1.0 / kMarshallPalmerB)));
}

// ---- point products ----------------------------------------------------------

namespace {

icd::PrecipType precip_type_of(double rate, std::size_t az_index, std::size_t r_index, std::uint32_t epoch) noexcept {
    if (rate > 50.0) {
        return icd::PrecipType::HAIL;
    }
    if (rate > 25.0) {
        return icd::PrecipType::MIXED;
    }
    if (rate > 10.0) {
        return icd::PrecipType::RAIN;
    }
    const double u = texture_unit(epoch / kSnowEpochFrames, static_cast<std::uint32_t>(az_index),
                                  static_cast<std::uint32_t>(r_index), 7U);
    return u < kSnowFraction ? icd::PrecipType::SNOW : icd::PrecipType::RAIN;
}

// The coordinates the Python gave a point of the downsampled field, rounding
// at every step as it did (the VIL generator also rounded the azimuth in
// degrees; the precipitation generator did not), and whether the point
// survived its origin filter.
bool locate(PointKind kind, std::size_t az_index, std::size_t r_index, Point& p) noexcept {
    const double normalized_az = round_to_sigfigs(static_cast<double>(az_index) / static_cast<double>(kSubAzimuths));
    const double normalized_range = round_to_sigfigs(static_cast<double>(r_index) / static_cast<double>(kSubRanges));
    const double azimuth_degrees =
        kind == PointKind::kVil ? round_to_sigfigs(normalized_az * 360.0) : normalized_az * 360.0;
    const double azimuth_rad = azimuth_degrees * kDegToRad;
    const double range_nm = round_to_sigfigs(normalized_range * kSelectionRangeNm);
    p.az_index = az_index;
    p.r_index = r_index;
    p.x_nm = round_to_sigfigs(range_nm * std::sin(azimuth_rad));
    p.y_nm = round_to_sigfigs(range_nm * std::cos(azimuth_rad));
    return !(std::fabs(p.x_nm) < kOriginExclusionNm && std::fabs(p.y_nm) < kOriginExclusionNm);
}

void describe(PointKind kind, std::uint32_t epoch, Point& p) noexcept {
    if (kind == PointKind::kVil) {
        const double raw = std::fmin(1.0, p.value / kVilFullScale);
        p.layer_count = static_cast<std::uint8_t>(std::min(15.0, std::trunc(raw * 15.0) + 1.0));
        p.intensity = round_to_sigfigs(raw);
        p.type = icd::PrecipType::NONE;
    } else {
        p.layer_count = 0;
        p.intensity = round_to_sigfigs(std::fmin(1.0, p.value / kRateFullScale));
        p.type = precip_type_of(p.value, p.az_index, p.r_index, epoch);
    }
    p.show_values = p.intensity > 0.3;
}

// sample_points_with_trajectory with the continuing points matched by
// position alone, then clusters of the rest by intensity; returns how many
// were chosen into work.chosen, ascending.
std::size_t select(std::size_t n, std::size_t max_points, const PointTracker& tracker, PointWork& work) noexcept {
    std::size_t chosen = 0;
    std::array<bool, kMaxSelected> previous_used{};
    for (std::size_t i = 0; i < n; ++i) {
        work.matched[i] = false;
        double best_distance = INFINITY;
        std::size_t best = kMaxSelected;
        for (std::size_t k = 0; k < tracker.count && k < kMaxSelected; ++k) {
            if (previous_used[k]) {
                continue;
            }
            const double d = distance(work.candidates[i].x_nm, work.candidates[i].y_nm, tracker.positions[k][0],
                                      tracker.positions[k][1]);
            if (d < best_distance) {
                best_distance = d;
                best = k;
            }
        }
        if (best < kMaxSelected && best_distance < kMatchDistanceNm && chosen < kMaxSelected) {
            previous_used[best] = true;
            work.matched[i] = true;
            work.chosen[chosen++] = static_cast<std::uint16_t>(i);
        }
    }
    const std::size_t remaining = max_points > chosen ? max_points - chosen : 0;
    if (remaining > 0 && n > chosen) {
        std::size_t unmatched_count = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (!work.matched[i]) {
                work.unmatched[unmatched_count++] = static_cast<std::uint16_t>(i);
            }
        }
        // cluster_points: a breadth-first grouping within kClusterRadiusNm, in order.
        std::size_t clusters = 0;
        std::size_t members = 0;
        for (std::size_t i = 0; i < unmatched_count; ++i) {
            work.visited[i] = false;
        }
        for (std::size_t i = 0; i < unmatched_count; ++i) {
            if (work.visited[i]) {
                continue;
            }
            const std::size_t start = members;
            work.members[members++] = static_cast<std::uint16_t>(i);
            work.visited[i] = true;
            for (std::size_t j = start; j < members; ++j) {
                const Point& current = work.candidates[work.unmatched[work.members[j]]];
                for (std::size_t k = 0; k < unmatched_count; ++k) {
                    if (work.visited[k]) {
                        continue;
                    }
                    const Point& other = work.candidates[work.unmatched[k]];
                    if (distance(current.x_nm, current.y_nm, other.x_nm, other.y_nm) <= kClusterRadiusNm) {
                        work.members[members++] = static_cast<std::uint16_t>(k);
                        work.visited[k] = true;
                    }
                }
            }
            double sum = 0.0;
            for (std::size_t j = start; j < members; ++j) {
                sum += work.candidates[work.unmatched[work.members[j]]].intensity;
            }
            work.cluster_start[clusters] = static_cast<std::uint16_t>(start);
            work.cluster_size[clusters] = static_cast<std::uint16_t>(members - start);
            work.cluster_mean[clusters] = sum / static_cast<double>(members - start);
            ++clusters;
        }
        stable_sort_descending(work.cluster_mean.data(), work.cluster_order.data(), clusters);
        double total = 0.0;
        for (std::size_t c = 0; c < clusters; ++c) {
            total += work.cluster_mean[work.cluster_order[c]];
        }
        std::size_t added = 0;
        if (total > 0.0) {
            for (std::size_t c = 0; c < clusters && added < remaining; ++c) {
                const std::uint16_t id = work.cluster_order[c];
                const double share = std::nearbyint(work.cluster_mean[id] / total * static_cast<double>(remaining));
                const std::size_t want = static_cast<std::size_t>(std::fmax(1.0, share));
                const std::size_t take = std::min({want, static_cast<std::size_t>(work.cluster_size[id]), remaining - added});
                if (take == 0) {
                    continue;
                }
                // The most intense members of the cluster first, ties in cluster order.
                std::array<double, kMaxCandidates> keys{};
                std::array<std::uint16_t, kMaxCandidates> order{};
                const std::size_t start = work.cluster_start[id];
                const std::size_t count = work.cluster_size[id];
                for (std::size_t j = 0; j < count; ++j) {
                    keys[j] = work.candidates[work.unmatched[work.members[start + j]]].intensity;
                }
                stable_sort_descending(keys.data(), order.data(), count);
                for (std::size_t j = 0; j < take && chosen < kMaxSelected; ++j) {
                    work.chosen[chosen++] = work.unmatched[work.members[start + order[j]]];
                    ++added;
                }
            }
        } else {
            for (std::size_t i = 0; i < unmatched_count && added < remaining && chosen < kMaxSelected; ++i) {
                work.chosen[chosen++] = work.unmatched[i];
                ++added;
            }
        }
    }
    const std::size_t kept = std::min(chosen, max_points);
    std::sort(work.chosen.begin(), work.chosen.begin() + static_cast<std::ptrdiff_t>(kept));
    return kept;
}

}  // namespace

void generate_points(const Field& field, PointKind kind, std::size_t max_points, std::uint32_t epoch,
                     PointTracker& tracker, PointWork& work, PointProducts& out) noexcept {
    out.count = 0;
    out.significant = 0;
    out.max_value = 0.0;
    max_points = std::min(max_points, kMaxSelected);
    double sample_max = -INFINITY;
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            out.max_value = std::fmax(out.max_value, field[az][r]);
            if (az % kSampleStride == 0 && r % kSampleStride == 0 && field[az][r] > sample_max) {
                sample_max = field[az][r];
            }
        }
    }
    if (!(round_to_sigfigs(sample_max) > kPointThreshold)) {
        return;
    }
    // The 8x downsampled field, its significant bins in raster order.
    std::size_t found = 0;
    std::size_t candidates = 0;
    for (std::size_t i = 0; i < kSubAzimuths; ++i) {
        for (std::size_t j = 0; j < kSubRanges; ++j) {
            const double value = round_to_sigfigs(field[i * kDownsample][j * kDownsample]);
            if (!(value > kPointThreshold)) {
                continue;
            }
            ++found;
            if (candidates < kMaxCandidates) {
                work.candidates[candidates].az_index = i;
                work.candidates[candidates].r_index = j;
                work.candidates[candidates].value = value;
                ++candidates;
            }
        }
    }
    out.significant = static_cast<std::uint16_t>(std::min<std::size_t>(found, 65535));
    if (found > kMaxCandidates) {
        // The Python kept a random 500; the port keeps an even spread (the
        // same bins every time): work.candidates holds the first 500 found,
        // so re-walk the field taking every (found / 500)th significant bin.
        std::size_t kept = 0;
        std::size_t index = 0;
        std::size_t next_wanted = 0;
        for (std::size_t i = 0; i < kSubAzimuths && kept < kMaxCandidates; ++i) {
            for (std::size_t j = 0; j < kSubRanges && kept < kMaxCandidates; ++j) {
                const double value = round_to_sigfigs(field[i * kDownsample][j * kDownsample]);
                if (!(value > kPointThreshold)) {
                    continue;
                }
                if (index == next_wanted) {
                    work.candidates[kept].az_index = i;
                    work.candidates[kept].r_index = j;
                    work.candidates[kept].value = value;
                    ++kept;
                    next_wanted = kept * found / kMaxCandidates;
                }
                ++index;
            }
        }
        candidates = kept;
    }
    // Coordinates, the origin filter, and each point's description.
    std::size_t n = 0;
    for (std::size_t c = 0; c < candidates; ++c) {
        Point p = work.candidates[c];
        if (!locate(kind, p.az_index, p.r_index, p)) {
            continue;
        }
        describe(kind, epoch, p);
        work.candidates[n++] = p;
    }
    if (n <= max_points) {
        for (std::size_t i = 0; i < n; ++i) {
            out.points[i] = work.candidates[i];
        }
        out.count = n;
        return;
    }
    const std::size_t kept = select(n, max_points, tracker, work);
    for (std::size_t i = 0; i < kept; ++i) {
        out.points[i] = work.candidates[work.chosen[i]];
        tracker.positions[i] = {out.points[i].x_nm, out.points[i].y_nm};
    }
    out.count = kept;
    tracker.count = kept;
}

// ---- regions -----------------------------------------------------------------

std::size_t find_regions(const Field& field, double threshold, Labels& work, Region* out,
                         std::size_t capacity) noexcept {
    if (out == nullptr) {
        return 0;
    }
    for (auto& row : work.label) {
        row.fill(0);
    }
    std::size_t n = 0;
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            if (!(field[az][r] > threshold) || work.label[az][r] != 0) {
                continue;
            }
            if (n >= capacity || n >= 65535) {
                return n;
            }
            const auto id = static_cast<std::uint16_t>(n + 1);
            std::size_t head = 0;
            std::size_t tail = 0;
            work.queue[tail++] = static_cast<std::uint32_t>(az * kRanges + r);
            work.label[az][r] = id;
            double sum_az = 0.0;
            double sum_r = 0.0;
            double sum_dbz = 0.0;
            std::size_t pixels = 0;
            while (head < tail && tail <= work.queue.size()) {
                const std::uint32_t at = work.queue[head++];
                const std::size_t a = at / kRanges;
                const std::size_t b = at % kRanges;
                sum_az += static_cast<double>(a);
                sum_r += static_cast<double>(b);
                sum_dbz += field[a][b];
                ++pixels;
                const std::array<std::array<long, 2>, 4> neighbours{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
                for (const auto& d : neighbours) {
                    const long na = static_cast<long>(a) + d[0];
                    const long nb = static_cast<long>(b) + d[1];
                    if (na < 0 || nb < 0 || na >= static_cast<long>(kAzimuths) || nb >= static_cast<long>(kRanges)) {
                        continue;
                    }
                    const auto ua = static_cast<std::size_t>(na);
                    const auto ub = static_cast<std::size_t>(nb);
                    if (field[ua][ub] > threshold && work.label[ua][ub] == 0 && tail < work.queue.size()) {
                        work.label[ua][ub] = id;
                        work.queue[tail++] = static_cast<std::uint32_t>(ua * kRanges + ub);
                    }
                }
            }
            out[n].center_az = sum_az / static_cast<double>(pixels);
            out[n].center_r = sum_r / static_cast<double>(pixels);
            out[n].mean_dbz = sum_dbz / static_cast<double>(pixels);
            out[n].pixels = pixels;
            ++n;
        }
    }
    return n;
}

// ---- filters and gradients ---------------------------------------------------

namespace {

// scipy's NI_UniformFilter1D for size 3: a running sum over the line extended
// by one reflected sample at each end.
void uniform3_line(const double* in, std::size_t stride, std::size_t n, double* out, std::size_t out_stride) noexcept {
    if (n == 0) {
        return;
    }
    const auto padded = [in, stride, n](std::size_t k) -> double {
        const std::size_t i = k == 0 ? 0 : (k == n + 1 ? n - 1 : k - 1);
        return in[i * stride];
    };
    double tmp = padded(0) + padded(1) + padded(2);
    out[0] = tmp / 3.0;
    for (std::size_t i = 1; i < n; ++i) {
        tmp += padded(i + 2) - padded(i - 1);
        out[i * out_stride] = tmp / 3.0;
    }
}

}  // namespace

void uniform_filter3(const double* in, double* out, std::size_t rows, std::size_t cols, double* scratch) noexcept {
    if (in == nullptr || out == nullptr || scratch == nullptr || rows == 0 || cols == 0) {
        return;
    }
    for (std::size_t c = 0; c < cols; ++c) {  // axis 0: down each column
        uniform3_line(in + c, cols, rows, scratch + c, cols);
    }
    for (std::size_t r = 0; r < rows; ++r) {  // axis 1: along each row of the result
        uniform3_line(scratch + r * cols, 1, cols, out + r * cols, 1);
    }
}

double gradient_axis0(const Field& f, std::size_t az, std::size_t r) noexcept {
    if (az == 0) {
        return f[1][r] - f[0][r];
    }
    if (az + 1 >= kAzimuths) {
        return f[kAzimuths - 1][r] - f[kAzimuths - 2][r];
    }
    return (f[az + 1][r] - f[az - 1][r]) / 2.0;
}

double gradient_axis1(const Field& f, std::size_t az, std::size_t r) noexcept {
    if (r == 0) {
        return f[az][1] - f[az][0];
    }
    if (r + 1 >= kRanges) {
        return f[az][kRanges - 1] - f[az][kRanges - 2];
    }
    return (f[az][r + 1] - f[az][r - 1]) / 2.0;
}

// ---- StormCellTracker --------------------------------------------------------

void StormCellTracker::init() noexcept {
    cells_ = {};
    regions_ = {};
    matched_ = {};
    count_ = 0;
    next_id_ = 1;
}

void StormCellTracker::process(const Field& composite, double scan_elevation_deg, double now_s, double interval_s,
                               Labels& work) noexcept {
    // _remove_expired_cells
    std::size_t kept = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        if (!(now_s - cells_[i].last_update_s > kMaxTrackingAgeS)) {
            cells_[kept++] = cells_[i];
        }
    }
    count_ = kept;
    // _detect_cells: connected regions above 30 dBZ of at least half a nautical mile.
    const std::size_t found = find_regions(composite, kMinReflectivityDbz, work, regions_.data(), kMaxRegions);
    // _update_cell_tracking: each detection updates the first existing cell
    // within five bins, in the order the cells were created; the rest become
    // new cells; existing cells nothing matched are dropped.
    for (std::size_t i = 0; i < count_; ++i) {
        matched_[i] = false;
    }
    const std::size_t existing = count_;
    for (std::size_t d = 0; d < found; ++d) {
        const Region& region = regions_[d];
        const double size_nm = static_cast<double>(region.pixels) * kNmPerBin;
        if (size_nm < kMinCellSizeNm) {
            continue;
        }
        bool matched = false;
        for (std::size_t i = 0; i < existing; ++i) {
            TrackedCell& cell = cells_[i];
            if (!(distance(region.center_r, region.center_az, cell.center_r, cell.center_az) <= kSameCellDistanceBins)) {
                continue;
            }
            const double dt = now_s - cell.last_update_s;
            if (dt > 0.0) {
                cell.velocity_r = (region.center_r - cell.center_r) / dt;
                cell.velocity_az = (region.center_az - cell.center_az) / dt;
            }
            cell.center_az = region.center_az;
            cell.center_r = region.center_r;
            cell.reflectivity = region.mean_dbz;
            cell.size = size_nm;
            cell.intensity = region_intensity(region.mean_dbz, kMinReflectivityDbz);
            cell.last_update_s = now_s;
            matched_[i] = true;
            matched = true;
            break;
        }
        if (matched || count_ >= kMaxTrackedCells) {
            continue;
        }
        TrackedCell& cell = cells_[count_];
        cell = TrackedCell{};
        cell.id = next_id_;
        next_id_ = static_cast<std::uint16_t>(next_id_ == 65535U ? 1U : next_id_ + 1U);
        cell.center_az = region.center_az;
        cell.center_r = region.center_r;
        cell.reflectivity = region.mean_dbz;
        cell.size = size_nm;
        cell.intensity = region_intensity(region.mean_dbz, kMinReflectivityDbz);
        cell.last_update_s = now_s;
        matched_[count_] = true;
        ++count_;
    }
    kept = 0;
    for (std::size_t i = 0; i < count_; ++i) {
        if (matched_[i]) {
            cells_[kept++] = cells_[i];
        }
    }
    count_ = kept;
    // _analyze_vertical_development: the cell top from its size as a range
    // proxy, and the change of that estimate over the scan interval.
    if (scan_elevation_deg > 0.0 && interval_s > 0.0) {
        const double tan_elevation = std::tan(scan_elevation_deg * kDegToRad);
        for (std::size_t i = 0; i < count_; ++i) {
            TrackedCell& cell = cells_[i];
            if (cell.reflectivity >= kMinReflectivityDbz) {
                const double top = cell.size * kMetresPerNm * tan_elevation;
                cell.vertical_development = (top - cell.altitude) / interval_s;
                cell.altitude = top;
            }
        }
    }
}

// ---- PrecipitationAnalyzer ---------------------------------------------------

void PrecipitationAnalyzer::init() noexcept {
    areas_ = {};
    regions_ = {};
    count_ = 0;
}

void PrecipitationAnalyzer::process(const Field& composite, Labels& work) noexcept {
    const std::size_t found = find_regions(composite, kMinReflectivityDbz, work, regions_.data(), kMaxRegions);
    count_ = 0;
    for (std::size_t i = 0; i < found; ++i) {
        const Region& region = regions_[i];
        PrecipArea& area = areas_[count_++];
        area.center_az = region.center_az;
        area.center_r = region.center_r;
        area.reflectivity = region.mean_dbz;
        const double z = std::pow(10.0, region.mean_dbz / 10.0);
        area.rate = std::pow(z / kMarshallPalmerA, 1.0 / kMarshallPalmerB);
        area.type = region.mean_dbz > 55.0 ? icd::PrecipType::HAIL
                    : region.mean_dbz > 45.0 ? icd::PrecipType::HEAVY_RAIN
                                             : icd::PrecipType::RAIN;
        area.intensity = region_intensity(region.mean_dbz, kMinReflectivityDbz);
    }
}

// ---- WindShearProcessor ------------------------------------------------------

void WindShearProcessor::init() noexcept {
    events_ = {};
    order_ = {};
    reported_ = {};
    count_ = 0;
    microburst_ = false;
}

void WindShearProcessor::process(const Volume& v, double lowest_elevation_deg) noexcept {
    for (auto& row : reported_) {
        row.fill(false);
    }
    count_ = 0;
    microburst_ = false;
    // The lowest tilt as the Doppler proxy; np.gradient along azimuth is the
    // shear, along range the divergence.
    const double tan_elevation = std::tan(lowest_elevation_deg * kDegToRad);
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            const double dbz = v.dbz[az][0][r];
            if (dbz < kMinDbz) {
                continue;
            }
            const double above = az + 1 < kAzimuths ? v.dbz[az + 1][0][r] : 0.0;
            const double below = az > 0 ? v.dbz[az - 1][0][r] : 0.0;
            const double grad_az = az == 0 ? v.dbz[1][0][r] - v.dbz[0][0][r]
                                   : az + 1 >= kAzimuths ? v.dbz[kAzimuths - 1][0][r] - v.dbz[kAzimuths - 2][0][r]
                                                         : (above - below) / 2.0;
            const double shear = std::fabs(grad_az) * kShearScale;
            if (shear < kShearThresholdKts) {
                continue;
            }
            bool& reported = reported_[az / kShearCluster][r / kShearCluster];
            if (reported) {
                continue;
            }
            reported = true;
            const double grad_r = r == 0 ? v.dbz[az][0][1] - v.dbz[az][0][0]
                                  : r + 1 >= kRanges ? v.dbz[az][0][kRanges - 1] - v.dbz[az][0][kRanges - 2]
                                                     : (v.dbz[az][0][r + 1] - v.dbz[az][0][r - 1]) / 2.0;
            ShearEvent& e = events_[count_++];
            e.az = az;
            e.r = r;
            e.shear_kts = shear;
            e.divergence = grad_r / kDivergenceScale;
            e.microburst = e.divergence > kMicroburstThreshold;
            e.severity = shear > 30.0 ? icd::ShearSeverity::SEVERE
                         : shear > 20.0 ? icd::ShearSeverity::MODERATE
                                        : icd::ShearSeverity::LOW;
            const double range_m = static_cast<double>(r + 1) * kRangeBinMetres;
            e.altitude_ft = std::fmax(0.0, range_m * tan_elevation * kFeetPerMetre);
            microburst_ = microburst_ || e.microburst;
            if (count_ >= kMaxShearEvents) {
                break;
            }
        }
        if (count_ >= kMaxShearEvents) {
            break;
        }
    }
    // Severity order, highest first, detection order within a severity.
    std::size_t k = 0;
    for (const icd::ShearSeverity level :
         {icd::ShearSeverity::SEVERE, icd::ShearSeverity::MODERATE, icd::ShearSeverity::LOW}) {
        for (std::size_t i = 0; i < count_; ++i) {
            if (events_[i].severity == level) {
                order_[k++] = static_cast<std::uint16_t>(i);
            }
        }
    }
}

// ---- TurbulenceProcessor -----------------------------------------------------

void TurbulenceProcessor::init() noexcept {
    cells_ = {};
    order_ = {};
    scratch_ = {};
    reported_ = {};
    count_ = 0;
}

icd::TurbulenceCategory TurbulenceProcessor::max_category() const noexcept {
    return count_ == 0 ? icd::TurbulenceCategory::NONE : cells_[order_[0]].category;
}

void TurbulenceProcessor::process(const Volume& v, double lowest_elevation_deg, FilterWork& work) noexcept {
    for (auto& row : reported_) {
        row.fill(false);
    }
    count_ = 0;
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            work.lowest[az][r] = v.dbz[az][0][r];
        }
    }
    // Spectrum width proxy: the local 3x3 standard deviation of the lowest tilt.
    uniform_filter3(&work.lowest[0][0], &work.mean[0][0], kAzimuths, kRanges, &work.tmp[0][0]);
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            work.sq[az][r] = work.lowest[az][r] * work.lowest[az][r];
        }
    }
    uniform_filter3(&work.sq[0][0], &work.sq[0][0], kAzimuths, kRanges, &work.tmp[0][0]);
    const double tan_elevation = std::tan(lowest_elevation_deg * kDegToRad);
    for (std::size_t az = 0; az < kAzimuths; ++az) {
        for (std::size_t r = 0; r < kRanges; ++r) {
            const double variance = std::fmax(0.0, work.sq[az][r] - work.mean[az][r] * work.mean[az][r]);
            const double sw = std::sqrt(variance);
            const double dbz = work.lowest[az][r];
            if (dbz < kMinDbz || sw < kMinSpectrumWidth) {
                continue;
            }
            const double range_m = std::fmax(1.0, static_cast<double>(r + 1) * kRangeBinMetres);
            const double edr = std::pow(sw, 2.0 / 3.0) / std::pow(range_m, 1.0 / 3.0) * kEdrScale;
            if (edr < kLightEdr) {
                continue;
            }
            bool& reported = reported_[az / kTurbulenceCluster][r / kTurbulenceCluster];
            if (reported) {
                continue;
            }
            reported = true;
            TurbulenceCell& c = cells_[count_++];
            c.az = az;
            c.r = r;
            c.edr = edr;
            c.spectrum_width = sw;
            c.category = edr >= kExtremeEdr    ? icd::TurbulenceCategory::EXTREME
                         : edr >= kSevereEdr   ? icd::TurbulenceCategory::SEVERE
                         : edr >= kModerateEdr ? icd::TurbulenceCategory::MODERATE
                                               : icd::TurbulenceCategory::LIGHT;
            c.intensity = std::fmin(1.0, edr / kExtremeEdr);
            c.altitude_ft = std::fmax(0.0, range_m * tan_elevation * kFeetPerMetre);
            if (count_ >= kMaxTurbulenceCells) {
                break;
            }
        }
        if (count_ >= kMaxTurbulenceCells) {
            break;
        }
    }
    // Strongest first, detection order among equals.
    for (std::size_t i = 0; i < count_; ++i) {
        keys_[i] = cells_[i].intensity;
    }
    merge_sort_descending(keys_.data(), order_.data(), scratch_.data(), count_);
}

}  // namespace fmofp::radar
