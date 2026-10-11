#include <fmofp/radar/weather_model.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::radar {

namespace {

// Python's int(x) % n for x >= 0: truncate, then wrap.
std::size_t index_of(double value, std::size_t modulus) noexcept {
    const double truncated = std::trunc(std::fmax(0.0, value));
    const auto whole = static_cast<unsigned long long>(truncated);
    return static_cast<std::size_t>(whole % modulus);
}

// min(range_steps - 1, int(range)).
std::size_t range_index_of(double range_km, std::size_t ranges) noexcept {
    const double truncated = std::trunc(std::fmax(0.0, range_km));
    const double limit = static_cast<double>(ranges - 1);
    return static_cast<std::size_t>(std::fmin(limit, truncated));
}

// min(|a - b|, |a - b + n|, |a - b - n|): the shorter way round the circle.
double azimuth_distance(std::size_t a, std::size_t b, std::size_t n) noexcept {
    const double d = static_cast<double>(a) - static_cast<double>(b);
    const double steps = static_cast<double>(n);
    return std::fmin(std::fabs(d), std::fmin(std::fabs(d + steps), std::fabs(d - steps)));
}

void render_cells(const WeatherTruth& truth, std::size_t nel, const Geometry& g, std::size_t az,
                  std::array<std::array<double, kRanges>, kMaxElevations>& slice) noexcept {
    for (std::size_t i = 0; i < truth.cell_count && i < kMaxTruthCells; ++i) {
        const TruthCell& c = truth.cells[i];
        const std::size_t cell_az_idx = index_of(c.azimuth_deg, g.azimuths);
        const std::size_t cell_range_idx = range_index_of(c.range_km, g.ranges);
        const double az_diff = azimuth_distance(az, cell_az_idx, g.azimuths);
        const double az_factor = std::fmax(0.0, 1.0 - (az_diff / (c.size_km * 0.5)));
        for (std::size_t r = 0; r < g.ranges; ++r) {
            const double r_diff = std::fabs(static_cast<double>(r) - static_cast<double>(cell_range_idx));
            const double r_factor = std::fmax(0.0, 1.0 - (r_diff / c.size_km));
            const double combined = az_factor * r_factor;
            if (!(combined > 0.0)) {
                continue;
            }
            for (std::size_t el = 0; el < nel; ++el) {
                const double value = c.intensity_dbz * combined * vertical_development(c.entry_profile, el);
                slice[el][r] = std::fmax(slice[el][r], value);
            }
        }
    }
}

void render_area(const WeatherTruth& truth, std::size_t nel, const Geometry& g, const Texture& texture, std::size_t az,
                 std::array<std::array<double, kRanges>, kMaxElevations>& slice) noexcept {
    if (!truth.has_area) {
        return;
    }
    const TruthArea& a = truth.area;
    const std::size_t area_az_idx = index_of(a.azimuth_deg, g.azimuths);
    const std::size_t area_range_idx = range_index_of(a.range_km, g.ranges);
    const auto az_half_width = static_cast<long>(std::trunc(std::fmax(0.0, a.width_deg / 2.0)));
    const auto range_half_depth = static_cast<long>(std::trunc(std::fmax(0.0, a.depth_km / 2.0)));
    const auto steps = static_cast<long>(g.azimuths);
    const auto centre = static_cast<long>(area_az_idx);
    // (idx +- half) % steps, Python's modulo (never negative).
    const long az_start = ((centre - az_half_width) % steps + steps) % steps;
    const long az_end = ((centre + az_half_width) % steps + steps) % steps;
    const auto here = static_cast<long>(az);
    const bool in_azimuth = az_start > az_end ? (here >= az_start || here < az_end) : (here >= az_start && here < az_end);
    if (!in_azimuth) {
        return;
    }
    const auto range_centre = static_cast<long>(area_range_idx);
    const long range_start = std::max(0L, range_centre - range_half_depth);
    const long range_end = std::min(static_cast<long>(g.ranges) - 1L, range_centre + range_half_depth);
    const double az_dist = azimuth_distance(az, area_az_idx, g.azimuths) / static_cast<double>(std::max(1L, az_half_width));
    const std::size_t elevations = std::min(kStratiformElevations, nel);
    for (long r = range_start; r <= range_end; ++r) {
        const double r_dist = static_cast<double>(std::labs(r - range_centre)) / static_cast<double>(std::max(1L, range_half_depth));
        const double factor = std::fmax(0.0, 1.0 - std::sqrt(az_dist * az_dist + r_dist * r_dist));
        if (!(factor > 0.0)) {
            continue;
        }
        const auto rr = static_cast<std::size_t>(r);
        for (std::size_t el = 0; el < elevations; ++el) {
            const double u = texture_unit(texture.seed, static_cast<std::uint32_t>(az), static_cast<std::uint32_t>(el),
                                          static_cast<std::uint32_t>(rr));
            const double noise = texture.stratiform_dbz * (2.0 * u - 1.0);
            const double intensity = a.intensity_dbz * factor + noise;
            if (intensity > slice[el][rr]) {
                slice[el][rr] = intensity;
            }
        }
    }
}

void render_clutter(std::size_t nel, const Geometry& g, const Texture& texture, std::size_t az,
                    std::array<std::array<double, kRanges>, kMaxElevations>& slice) noexcept {
    const std::size_t bins = std::min(kClutterRangeBins, g.ranges);
    const std::size_t elevations = std::min(kClutterElevations, nel);
    for (std::size_t r = 0; r < bins; ++r) {
        for (std::size_t el = 0; el < elevations; ++el) {
            if (slice[el][r] < kClutterFloorDbz) {
                // A second hash stream (elevation offset) so the clutter does not
                // repeat the stratiform draw of the same bin.
                const double u = texture_unit(texture.seed, static_cast<std::uint32_t>(az),
                                              static_cast<std::uint32_t>(el + 16U), static_cast<std::uint32_t>(r));
                slice[el][r] = kClutterFloorDbz + texture.clutter_dbz * u;
            }
        }
    }
}

}  // namespace

WeatherTruth truth_from(const icd::PlantWeather& m) noexcept {
    WeatherTruth t;
    const std::array<TruthCell, kMaxTruthCells> all{{
        {m.cell_1_azimuth, m.cell_1_range, m.cell_1_size, m.cell_1_intensity, m.cell_1_entry_profile},
        {m.cell_2_azimuth, m.cell_2_range, m.cell_2_size, m.cell_2_intensity, m.cell_2_entry_profile},
        {m.cell_3_azimuth, m.cell_3_range, m.cell_3_size, m.cell_3_intensity, m.cell_3_entry_profile},
        {m.cell_4_azimuth, m.cell_4_range, m.cell_4_size, m.cell_4_intensity, m.cell_4_entry_profile},
        {m.cell_5_azimuth, m.cell_5_range, m.cell_5_size, m.cell_5_intensity, m.cell_5_entry_profile},
        {m.cell_6_azimuth, m.cell_6_range, m.cell_6_size, m.cell_6_intensity, m.cell_6_entry_profile},
        {m.cell_7_azimuth, m.cell_7_range, m.cell_7_size, m.cell_7_intensity, m.cell_7_entry_profile},
        {m.cell_8_azimuth, m.cell_8_range, m.cell_8_size, m.cell_8_intensity, m.cell_8_entry_profile},
    }};
    t.cell_count = std::min(static_cast<std::size_t>(m.cell_count), kMaxTruthCells);
    for (std::size_t i = 0; i < t.cell_count; ++i) {
        t.cells[i] = all[i];
    }
    t.has_area = m.area_count != 0U;
    t.area = TruthArea{m.area_azimuth, m.area_range, m.area_width, m.area_depth, m.area_intensity};
    return t;
}

Vcp vcp_for(icd::WeatherMode mode) noexcept {
    Vcp v;
    switch (mode) {
        case icd::WeatherMode::SURVEILLANCE:
            v.elevations = {0.5, 1.5, 2.4, 3.4};
            v.count = 4;
            break;
        case icd::WeatherMode::MAPPING:
        case icd::WeatherMode::PRECIPITATION:
            v.elevations = {0.5, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0};
            v.count = 7;
            break;
        case icd::WeatherMode::TURBULENCE:
        case icd::WeatherMode::WINDSHEAR:
            v.elevations = {0.5};
            v.count = 1;
            break;
        case icd::WeatherMode::NONE:
            v.count = 0;
            break;
    }
    return v;
}

double vertical_development(bool entry_profile, std::size_t elevation) noexcept {
    if (elevation >= 10) {
        return 0.1;
    }
    const auto i = static_cast<double>(elevation);
    if (entry_profile) {
        return std::fmax(0.2, 1.0 - (i * 0.15));
    }
    if (elevation < 4) {
        return 1.0 - (i * 0.1);
    }
    return std::fmax(0.2, 1.0 - (i * 0.2));
}

double texture_unit(std::uint32_t seed, std::uint32_t az, std::uint32_t el, std::uint32_t r) noexcept {
    std::uint32_t x = seed * 0x9E3779B9U;
    x ^= az * 0x85EBCA6BU;
    x = (x ^ (x >> 15U)) * 0x2C1B3C6DU;
    x ^= el * 0xC2B2AE35U;
    x = (x ^ (x >> 12U)) * 0x297A2D39U;
    x ^= r * 0x27D4EB2FU;
    x ^= x >> 15U;
    x *= 0x9E3779B9U;
    x ^= x >> 16U;
    return static_cast<double>(x >> 8U) / 16777216.0;
}

void render_azimuth(const WeatherTruth& truth, const Vcp& vcp, const Geometry& g, const Texture& texture,
                    std::size_t az, Volume& volume) noexcept {
    if (az >= g.azimuths || az >= kAzimuths || g.azimuths > kAzimuths || g.ranges == 0 || g.ranges > kRanges ||
        vcp.count > kMaxElevations) {
        return;
    }
    auto& slice = volume.dbz[az];
    for (auto& row : slice) {
        row.fill(0.0);
    }
    const std::size_t nel = vcp.count;
    render_cells(truth, nel, g, az, slice);
    render_area(truth, nel, g, texture, az, slice);
    render_clutter(nel, g, texture, az, slice);
    const std::size_t scanned = std::min(g.range_bins, g.ranges);
    for (std::size_t el = 0; el < nel; ++el) {
        for (std::size_t r = 0; r < g.ranges; ++r) {
            slice[el][r] = r < scanned ? std::fmin(kClipMaxDbz, std::fmax(kClipMinDbz, slice[el][r])) : 0.0;
        }
    }
}

}  // namespace fmofp::radar
