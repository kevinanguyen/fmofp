#ifndef FMOFP_RADAR_WEATHER_RADAR_HPP
#define FMOFP_RADAR_WEATHER_RADAR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>
#include <fmofp/radar/weather_model.hpp>
#include <fmofp/radar/weather_products.hpp>

// The weather radar partition's logic: the mode and state of
// Systems/radarManagement/weather/weather_radar.py, a scan that sweeps the
// circle over sixty frames rendering the volume from the latched weather
// truth, and the products of each scan (ICD RDR_WX_STATUS, RDR_WX_SCAN,
// RDR_WX_PRODUCTS). Pure: inputs are ICD messages, outputs are ICD messages,
// nothing touches the bus or the clock. The object is eight megabytes of
// fixed arrays and lives in static storage.

namespace fmofp::radar {

constexpr std::uint32_t kFramesPerScan = 60;  // 1.2 s at 50 Hz
constexpr std::size_t kAzimuthsPerFrame = kAzimuths / kFramesPerScan;  // 6
constexpr double kFrameSeconds = 0.02;
constexpr double kScanIntervalS = kFramesPerScan * kFrameSeconds;
constexpr double kMinRangeKm = 10.0;  // RDR_WX_MODE_CMD range_km limits
constexpr double kMaxRangeKm = 200.0;
constexpr double kMinTiltDeg = 0.0;
constexpr double kMaxTiltDeg = 90.0;
constexpr std::size_t kMaxReportedCells = icd::RdrWxProducts::kMaxCells;
constexpr std::size_t kMaxReportedAreas = icd::RdrWxProducts::kMaxAreas;
constexpr std::size_t kMaxReportedShear = icd::RdrWxProducts::kMaxWindshear;
constexpr std::size_t kMaxReportedTurbulence = icd::RdrWxProducts::kMaxTurbulence;

class WeatherRadar {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `weather` is the RDR_WX_WEATHER received this
    // frame, or null; `frame` is the partition's own frame count.
    void step(const icd::RdrWxWeather* weather, std::uint32_t frame) noexcept;

    // Applied only when its cmd_seq is new (ICD README section 5). A new
    // state, mode or range restarts the scan in progress.
    void command_mode(const icd::RdrWxModeCmd& cmd) noexcept;

    // Mode code effects, relayed by the process from its RemoteTerminal.
    void reset() noexcept;
    void start_self_test() noexcept;

    // True once after each change of the products (a completed scan, or the
    // radar leaving its operating mode): the process publishes the block.
    [[nodiscard]] bool take_products_update() noexcept;
    void set_products_version(std::uint8_t version) noexcept { scan_.products_version = version; }

    [[nodiscard]] const icd::RdrWxStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::RdrWxScan& scan_summary() const noexcept { return scan_; }
    [[nodiscard]] const icd::RdrWxProducts& products() const noexcept { return products_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::WeatherMode mode() const noexcept { return mode_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] bool scanning() const noexcept { return scanning_; }
    [[nodiscard]] std::uint32_t scans_completed() const noexcept { return scans_completed_; }

    // ---- for the oracle tool and the tests -------------------------------
    // A whole scan at once from a truth, in a mode, as of `frame`.
    void scan_now(const WeatherTruth& truth, icd::WeatherMode scan_mode, std::uint32_t frame) noexcept;
    void set_texture(const Texture& texture) noexcept { texture_ = texture; }
    void set_range_km(double range_km) noexcept;
    [[nodiscard]] const Volume& volume() const noexcept { return volume_; }
    [[nodiscard]] const Field& vil_field() const noexcept { return vil_; }
    [[nodiscard]] const Field& precip_field() const noexcept { return precip_; }
    [[nodiscard]] const Field& composite_field() const noexcept { return composite_; }
    [[nodiscard]] const Vcp& scan_vcp() const noexcept { return vcp_; }
    [[nodiscard]] const WeatherTruth& scan_truth() const noexcept { return scan_truth_; }
    [[nodiscard]] const PointProducts& vil_points() const noexcept { return vil_points_; }
    [[nodiscard]] const PointProducts& precip_points() const noexcept { return precip_points_; }
    [[nodiscard]] const StormCellTracker& tracker() const noexcept { return tracker_; }
    [[nodiscard]] const PrecipitationAnalyzer& analyzer() const noexcept { return analyzer_; }
    [[nodiscard]] const WindShearProcessor& windshear() const noexcept { return windshear_; }
    [[nodiscard]] const TurbulenceProcessor& turbulence() const noexcept { return turbulence_; }
    [[nodiscard]] double max_reflectivity() const noexcept { return max_dbz_; }

private:
    struct Wanted {
        bool vil{false};
        bool precip{false};
        bool cells{false};
        bool areas{false};
        bool windshear{false};
        bool turbulence{false};
    };
    static Wanted wanted_for(icd::WeatherMode scan_mode) noexcept;
    [[nodiscard]] bool operating() const noexcept;
    void start_scan(std::uint32_t frame) noexcept;
    void render_slice() noexcept;
    void finish_scan(std::uint32_t frame) noexcept;
    void fill_products() noexcept;
    void clear_products() noexcept;
    void run_self_test(std::uint32_t frame) noexcept;

    // Outputs
    icd::RdrWxStatus status_{};
    icd::RdrWxScan scan_{};
    icd::RdrWxProducts products_{};
    bool products_dirty_{false};
    bool products_cleared_{false};
    // Settings and inputs
    icd::WeatherMode mode_{icd::WeatherMode::NONE};
    double range_km_{kMaxRangeKm};
    double tilt_deg_{0.0};
    WeatherTruth latest_truth_{};
    bool have_truth_{false};
    bus::CommandSequence mode_seq_{};
    // The scan in progress
    bool scanning_{false};
    icd::WeatherMode scan_mode_{icd::WeatherMode::NONE};
    Wanted wanted_{};
    WeatherTruth scan_truth_{};
    Vcp vcp_{};
    std::array<double, kMaxElevations> thickness_{};
    Geometry geometry_{};
    Texture texture_{};
    std::size_t next_az_{0};
    std::uint8_t scan_id_{0};
    std::uint32_t scans_completed_{0};
    double max_dbz_{kClipMinDbz};
    // Self test and health
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
    bus::BitWord bit_{};
    // The big arrays
    Volume volume_{};
    Field vil_{};
    Field precip_{};
    Field composite_{};
    FilterWork filter_{};
    Labels labels_{};
    PointWork point_work_{};
    PointTracker vil_tracker_{};
    PointTracker precip_tracker_{};
    PointProducts vil_points_{};
    PointProducts precip_points_{};
    StormCellTracker tracker_{};
    PrecipitationAnalyzer analyzer_{};
    WindShearProcessor windshear_{};
    TurbulenceProcessor turbulence_{};
    std::array<std::uint16_t, kMaxRegions> cell_order_{};
    std::array<std::uint16_t, kMaxRegions> area_order_{};
    std::array<double, kMaxRegions> order_keys_{};
};

}  // namespace fmofp::radar

#endif
