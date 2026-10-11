#ifndef FMOFP_RADAR_WEATHER_PRODUCTS_HPP
#define FMOFP_RADAR_WEATHER_PRODUCTS_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/icd/enums.hpp>
#include <fmofp/radar/weather_model.hpp>

// The weather radar's signal processing, ported function for function from
// vil_data_generator_sync.py, precipitation_data_generator_sync.py,
// stormCellTracking.py, precipitation_analysis.py and weather_processor.py.
// Everything works on fixed-size fields handed in by the caller; nothing
// allocates, and nothing here knows about the bus.

namespace fmofp::radar {

using Field = std::array<std::array<double, kRanges>, kAzimuths>;  // [azimuth][range], 576 KB

constexpr double kPi = 3.141592653589793;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kFeetPerMetre = 1.0 / 0.3048;
constexpr double kMetresPerNm = 1852.0;

// Stable sort of the indices 0..n-1 by key, largest first; `order` holds n entries.
void stable_sort_descending(const double* keys, std::uint16_t* order, std::size_t n) noexcept;

// round_to_sigfigs(x, 3) of the Python generators: round half to even at the
// third significant figure. Zero, NaN and infinity pass through.
double round_to_sigfigs(double x, int sigfigs = 3) noexcept;

// VILDataGenerator.calculate_vil's layer thicknesses for a pattern: metres,
// 100 m at least, rounded to three figures; the last layer repeats the one
// before it.
std::array<double, kMaxElevations> layer_thicknesses(const Vcp& vcp) noexcept;

// One column (azimuth az, range r) of the volume with `nel` elevations:
// its VIL in kg/m2 as the Python computed it, the Marshall-Palmer rain rate
// of its strongest return in mm/h, and that strongest return (the composite).
double vil_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r,
                  const std::array<double, kMaxElevations>& thickness) noexcept;
double precip_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r) noexcept;
double composite_column(const Volume& v, std::size_t nel, std::size_t az, std::size_t r) noexcept;

// ---- point products (VIL and precipitation points) --------------------------

constexpr std::size_t kDownsample = 8;  // int(sqrt(72000 / 1000)) for a 360 x 200 field
constexpr std::size_t kSubAzimuths = kAzimuths / kDownsample;  // 45
constexpr std::size_t kSubRanges = kRanges / kDownsample;      // 25
constexpr std::size_t kSampleStride = 10;                      // the quick significance check samples [::10, ::10]
constexpr std::size_t kMaxCandidates = 500;                    // MAX_VIL_POINTS / MAX_PRECIPITATION_POINTS
constexpr double kPointThreshold = 0.1;                        // VIL_THRESHOLD / RATE_THRESHOLD
constexpr double kSelectionRangeNm = 40.0;  // the Python scaled the picture to 40 nm before clustering
constexpr double kMatchDistanceNm = 10.0;   // match_points_to_previous max_distance, in that scale
constexpr double kClusterRadiusNm = 5.0;    // cluster_points radius, in that scale
constexpr double kOriginExclusionNm = 0.5;  // points within this box of the aircraft were dropped
constexpr std::size_t kMaxVilPoints = 20;
constexpr std::size_t kMaxPrecipPoints = 15;
constexpr std::size_t kMaxSelected = kMaxVilPoints;
constexpr double kVilFullScale = 65.0;    // intensity = min(1, value / 65)
constexpr double kRateFullScale = 50.0;   // intensity = min(1, rate / 50)
constexpr double kSnowFraction = 0.3;     // light precipitation reads as snow three times in ten
constexpr std::uint32_t kSnowEpochFrames = 1500;  // the Python re-seeded that draw every 30 s

enum class PointKind : std::uint8_t { kVil, kPrecip };

struct Point {
    std::size_t az_index{0};  // in the downsampled field: bin azimuth = az_index * 8 degrees
    std::size_t r_index{0};   // bin range = r_index * 8 km
    double value{0.0};        // VIL kg/m2 or rate mm/h, rounded to three figures
    double intensity{0.0};
    std::uint8_t layer_count{0};
    bool show_values{false};
    icd::PrecipType type{icd::PrecipType::NONE};
    double x_nm{0.0};  // the Python's display coordinates, which its selection measured distances in
    double y_nm{0.0};
};

// The previous scan's selected positions, which the selection keeps when they
// persist (WeatherStateManager.tracked_*_points, without the wind drift).
struct PointTracker {
    std::array<std::array<double, 2>, kMaxSelected> positions{};
    std::size_t count{0};
};

struct PointProducts {
    std::array<Point, kMaxSelected> points{};
    std::size_t count{0};
    std::uint16_t significant{0};  // columns above the threshold before any limit
    double max_value{0.0};         // largest value anywhere in the field
};

// Scratch for generate_points: candidates and the clustering of the unmatched ones.
struct PointWork {
    std::array<Point, kMaxCandidates> candidates{};
    std::array<bool, kMaxCandidates> matched{};
    std::array<std::uint16_t, kMaxCandidates> unmatched{};
    std::array<std::uint16_t, kMaxCandidates> members{};  // cluster members, cluster after cluster
    std::array<std::uint16_t, kMaxCandidates> cluster_start{};
    std::array<std::uint16_t, kMaxCandidates> cluster_size{};
    std::array<double, kMaxCandidates> cluster_mean{};
    std::array<std::uint16_t, kMaxCandidates> cluster_order{};
    std::array<bool, kMaxCandidates> visited{};
    std::array<std::uint16_t, kMaxSelected> chosen{};
};

// generate_vil_data_objects / generate_precipitation_data_objects, from the
// field to the selected points. `epoch` is the partition frame (the rain-or-
// snow draw of light precipitation changes every kSnowEpochFrames).
void generate_points(const Field& field, PointKind kind, std::size_t max_points, std::uint32_t epoch,
                     PointTracker& tracker, PointWork& work, PointProducts& out) noexcept;

// ---- connected regions (scipy.ndimage.label, 4-connectivity) ----------------

constexpr std::size_t kMaxRegions = 512;

struct Region {
    double center_az{0.0};  // mean azimuth bin index (the Python's center_y)
    double center_r{0.0};   // mean range bin index (center_x)
    double mean_dbz{0.0};
    std::size_t pixels{0};
};

struct Labels {  // scratch for the region finder: a label per bin and a queue
    std::array<std::array<std::uint16_t, kRanges>, kAzimuths> label{};
    std::array<std::uint32_t, kAzimuths * kRanges> queue{};
};

// Regions of the field above `threshold`, in order of their first bin in
// raster order (azimuth, then range); at most `capacity`, the rest dropped.
std::size_t find_regions(const Field& field, double threshold, Labels& work, Region* out,
                         std::size_t capacity) noexcept;

// ---- scipy.ndimage.uniform_filter(size=3) and numpy.gradient -----------------

// A 3x3 mean with the 'reflect' boundary, along axis 0 then axis 1 as scipy
// does, with its running sums. `in`, `out` and `scratch` are rows x cols,
// row-major.
void uniform_filter3(const double* in, double* out, std::size_t rows, std::size_t cols, double* scratch) noexcept;
double gradient_axis0(const Field& f, std::size_t az, std::size_t r) noexcept;  // along azimuth
double gradient_axis1(const Field& f, std::size_t az, std::size_t r) noexcept;  // along range

// ---- stormCellTracking.StormCellTracker ------------------------------------

constexpr std::size_t kMaxTrackedCells = kMaxRegions;

struct TrackedCell {
    std::uint16_t id{0};
    double center_az{0.0};
    double center_r{0.0};
    double altitude{0.0};
    double reflectivity{0.0};
    double velocity_az{0.0};  // azimuth bins (degrees) per second
    double velocity_r{0.0};   // range bins (km) per second
    double size{0.0};         // nm: bins x 0.5
    double intensity{0.0};
    double vertical_development{0.0};
    double last_update_s{0.0};
};

class StormCellTracker {
public:
    void init() noexcept;
    // process_radar_data on the composite field at time `now_s`, with the
    // lowest elevation of the scan; `interval_s` is the scan interval the
    // vertical development rate is measured over (the Python divided by the
    // milliseconds between two of its own statements).
    void process(const Field& composite, double scan_elevation_deg, double now_s, double interval_s,
                 Labels& work) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] const TrackedCell& cell_at(std::size_t i) const noexcept { return cells_[i]; }

    static constexpr double kMinReflectivityDbz = 30.0;
    static constexpr double kMaxTrackingAgeS = 300.0;
    static constexpr double kMinCellSizeNm = 0.5;
    static constexpr double kNmPerBin = 0.5;
    static constexpr double kSameCellDistanceBins = 5.0;

private:
    std::array<TrackedCell, kMaxTrackedCells> cells_{};
    std::array<Region, kMaxRegions> regions_{};
    std::array<bool, kMaxTrackedCells> matched_{};
    std::size_t count_{0};
    std::uint16_t next_id_{1};
};

// ---- precipitation_analysis.PrecipitationAnalyzer ---------------------------

struct PrecipArea {
    double center_az{0.0};
    double center_r{0.0};
    double reflectivity{0.0};
    double rate{0.0};
    double intensity{0.0};
    icd::PrecipType type{icd::PrecipType::NONE};
};

class PrecipitationAnalyzer {
public:
    void init() noexcept;
    void process(const Field& composite, Labels& work) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] const PrecipArea& area_at(std::size_t i) const noexcept { return areas_[i]; }

    static constexpr double kMinReflectivityDbz = 30.0;

private:
    std::array<PrecipArea, kMaxRegions> areas_{};
    std::array<Region, kMaxRegions> regions_{};
    std::size_t count_{0};
};

// ---- weather_processor.WindShearProcessor ------------------------------------

constexpr std::size_t kShearCluster = 5;
constexpr std::size_t kMaxShearEvents = (kAzimuths / kShearCluster) * (kRanges / kShearCluster);  // one per cluster

struct ShearEvent {
    std::size_t az{0};
    std::size_t r{0};
    double shear_kts{0.0};
    double divergence{0.0};
    bool microburst{false};
    icd::ShearSeverity severity{icd::ShearSeverity::NONE};
    double altitude_ft{0.0};
};

class WindShearProcessor {
public:
    void init() noexcept;
    // On the lowest elevation of the volume.
    void process(const Volume& v, double lowest_elevation_deg) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    // Events in order of severity, highest first (ties in detection order).
    [[nodiscard]] const ShearEvent& event_at(std::size_t i) const noexcept { return events_[order_[i]]; }
    [[nodiscard]] bool has_microburst() const noexcept { return microburst_; }

    static constexpr double kShearThresholdKts = 15.0;
    static constexpr double kMicroburstThreshold = 0.08;
    static constexpr double kMinDbz = 20.0;
    static constexpr double kShearScale = 0.8;        // dBZ per bin to kt/nm
    static constexpr double kDivergenceScale = 20.0;  // dBZ per bin to normalised divergence
    static constexpr double kRangeBinMetres = 1000.0;

private:
    std::array<ShearEvent, kMaxShearEvents> events_{};
    std::array<std::uint16_t, kMaxShearEvents> order_{};
    std::array<std::array<bool, kRanges / kShearCluster>, kAzimuths / kShearCluster> reported_{};
    std::size_t count_{0};
    bool microburst_{false};
};

// ---- weather_processor.TurbulenceProcessor -----------------------------------

constexpr std::size_t kTurbulenceCluster = 4;
constexpr std::size_t kMaxTurbulenceCells = (kAzimuths / kTurbulenceCluster) * (kRanges / kTurbulenceCluster);

struct TurbulenceCell {
    std::size_t az{0};
    std::size_t r{0};
    double intensity{0.0};
    double spectrum_width{0.0};
    double edr{0.0};
    icd::TurbulenceCategory category{icd::TurbulenceCategory::NONE};
    double altitude_ft{0.0};
};

struct FilterWork {  // scratch for the spectrum width proxy
    Field lowest{};
    Field tmp{};
    Field mean{};
    Field sq{};
};

class TurbulenceProcessor {
public:
    void init() noexcept;
    void process(const Volume& v, double lowest_elevation_deg, FilterWork& work) noexcept;
    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    // Cells strongest first (ties in detection order).
    [[nodiscard]] const TurbulenceCell& cell_at(std::size_t i) const noexcept { return cells_[order_[i]]; }
    [[nodiscard]] icd::TurbulenceCategory max_category() const noexcept;

    static constexpr double kMinDbz = 15.0;
    static constexpr double kMinSpectrumWidth = 1.0;
    static constexpr double kLightEdr = 0.1;
    static constexpr double kModerateEdr = 0.2;
    static constexpr double kSevereEdr = 0.4;
    static constexpr double kExtremeEdr = 0.6;
    static constexpr double kEdrScale = 0.3;
    static constexpr double kRangeBinMetres = 1000.0;

private:
    std::array<TurbulenceCell, kMaxTurbulenceCells> cells_{};
    std::array<std::uint16_t, kMaxTurbulenceCells> order_{};
    std::array<std::uint16_t, kMaxTurbulenceCells> scratch_{};
    std::array<double, kMaxTurbulenceCells> keys_{};
    std::array<std::array<bool, kRanges / kTurbulenceCluster>, kAzimuths / kTurbulenceCluster> reported_{};
    std::size_t count_{0};
};

}  // namespace fmofp::radar

#endif
