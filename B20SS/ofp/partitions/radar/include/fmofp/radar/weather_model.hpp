#ifndef FMOFP_RADAR_WEATHER_MODEL_HPP
#define FMOFP_RADAR_WEATHER_MODEL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/icd/enums.hpp>
#include <fmofp/icd/messages.hpp>

// The weather radar's sensor model, ported from
// Systems/radarManagement/weather/reflectivity_simulator.py: given the weather
// truth the plant publishes (PLANT_WEATHER), the reflectivity the antenna
// would see in each azimuth, elevation and range bin. The world itself (where
// the cells are, how they move and age) lives in the plant; this file only
// renders it, as _render_storm_cells, _render_stratiform_areas and
// _add_ground_clutter did, bin for bin.

namespace fmofp::radar {

constexpr std::size_t kAzimuths = 360;     // 1 degree bins (reflectivity_simulator azimuth_steps)
constexpr std::size_t kRanges = 200;       // 1 km bins (range_steps)
constexpr std::size_t kMaxElevations = 7;  // the longest volume coverage pattern (mapping)
constexpr std::size_t kMaxTruthCells = 8;  // PLANT_WEATHER carries eight
constexpr double kClipMinDbz = -20.0;
constexpr double kClipMaxDbz = 65.0;
constexpr std::size_t kClutterRangeBins = 10;     // ground clutter in the first 10 km ...
constexpr std::size_t kClutterElevations = 2;     // ... of the lowest two elevations
constexpr std::size_t kStratiformElevations = 3;  // a stratiform area fills the lowest three
constexpr double kClutterFloorDbz = 10.0;

struct TruthCell {
    double azimuth_deg{0.0};
    double range_km{0.0};
    double size_km{0.0};        // radius
    double intensity_dbz{0.0};  // peak
    bool entry_profile{false};  // which vertical development profile (PLANT_WEATHER)
};

struct TruthArea {
    double azimuth_deg{0.0};
    double range_km{0.0};
    double width_deg{0.0};
    double depth_km{0.0};
    double intensity_dbz{0.0};
};

struct WeatherTruth {
    std::array<TruthCell, kMaxTruthCells> cells{};
    std::size_t cell_count{0};
    TruthArea area{};
    bool has_area{false};
};

WeatherTruth truth_from(const icd::PlantWeather& m) noexcept;

// Volume coverage pattern: the elevation angles of a mode's scan
// (weather_radar.py config['vcp']; one 0.5 degree tilt for the modes the
// Python had no entry for, as its .get(mode, {}).get('elevs', (0.5,)) gave).
struct Vcp {
    std::array<double, kMaxElevations> elevations{};
    std::size_t count{0};
};
Vcp vcp_for(icd::WeatherMode mode) noexcept;

// StormCell.vertical_development: the multiplier of a cell's intensity at
// elevation index `elevation` (dict.get(el, 0.1) beyond the ten defined).
double vertical_development(bool entry_profile, std::size_t elevation) noexcept;

// The rendered volume, dbz[azimuth][elevation][range]. Four megabytes: it
// lives in a static object, never on the stack.
struct Volume {
    std::array<std::array<std::array<double, kRanges>, kMaxElevations>, kAzimuths> dbz{};
};

// What is rendered: the dimensions in use (the oracle tool renders smaller
// volumes) and the instrumented range (RDR_WX_MODE_CMD range_km).
struct Geometry {
    std::size_t azimuths{kAzimuths};
    std::size_t ranges{kRanges};
    std::size_t range_bins{kRanges};  // bins at or beyond this are not scanned
};

// The Python drew random.uniform(-2, 2) into every stratiform bin and
// 10 + 5 * random() into every clutter bin. The port draws the same
// amplitudes from a hash of the scan and the bin, so a scan is repeatable;
// amplitude 0 turns the texture off, which is how the oracle tests compare
// with the Python's draws patched to their midpoints.
struct Texture {
    std::uint32_t seed{0};
    double stratiform_dbz{2.0};  // noise = stratiform_dbz * (2 u - 1)
    double clutter_dbz{5.0};     // clutter = 10 + clutter_dbz * u
};
double texture_unit(std::uint32_t seed, std::uint32_t az, std::uint32_t el, std::uint32_t r) noexcept;  // [0, 1)

// Renders azimuth `az` of the volume: zero, then the cells, the area, the
// clutter, the clip to [-20, 65] dBZ, in the Python's order; bins at or beyond
// g.range_bins are left at zero. Elevations beyond the pattern are zero too.
void render_azimuth(const WeatherTruth& truth, const Vcp& vcp, const Geometry& g, const Texture& texture,
                    std::size_t az, Volume& volume) noexcept;

}  // namespace fmofp::radar

#endif
