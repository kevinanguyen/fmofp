#include <fmofp/radar/weather_radar.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::radar {

namespace {

using icd::TerminalState;
using icd::WeatherMode;

// Self test takes a fixed number of frames so TERM_STATUS shows it in progress.
constexpr std::uint32_t kSelfTestFrames = 10;

// A state the radar can be commanded into from the bus (weather_radar.set_mode
// took any value; the ICD's terminal states split the lifecycle from the mode).
bool commandable(TerminalState state) noexcept {
    switch (state) {
        case TerminalState::STANDBY:
        case TerminalState::NORMAL:
        case TerminalState::TEST:
        case TerminalState::MAINTENANCE:
        case TerminalState::CALIBRATION:
            return true;
        case TerminalState::DEGRADED:
        case TerminalState::EMERGENCY:
        case TerminalState::FAILURE:
        case TerminalState::RECOVERY:
        case TerminalState::INITIALIZING:
            return false;
    }
    return false;
}

std::uint8_t saturate8(std::size_t n) noexcept { return static_cast<std::uint8_t>(std::min<std::size_t>(n, 255)); }

void zero(Field& f) noexcept {
    for (auto& row : f) {
        row.fill(0.0);
    }
}

}  // namespace

Status WeatherRadar::init() noexcept {
    status_ = icd::RdrWxStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    scan_ = icd::RdrWxScan{};
    scan_.range_km = kMaxRangeKm;
    scan_.max_reflectivity = kClipMinDbz;
    products_ = icd::RdrWxProducts{};
    products_dirty_ = true;  // the empty block is published first
    products_cleared_ = true;
    mode_ = WeatherMode::NONE;
    range_km_ = kMaxRangeKm;
    tilt_deg_ = 0.0;
    latest_truth_ = WeatherTruth{};
    have_truth_ = false;
    mode_seq_ = bus::CommandSequence{};
    scanning_ = false;
    scan_mode_ = WeatherMode::NONE;
    wanted_ = Wanted{};
    scan_truth_ = WeatherTruth{};
    vcp_ = Vcp{};
    thickness_ = {};
    geometry_ = Geometry{};
    texture_ = Texture{};
    next_az_ = 0;
    scan_id_ = 0;
    scans_completed_ = 0;
    max_dbz_ = kClipMinDbz;
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    // The big arrays are zeroed in place: a temporary of their size would not fit a stack.
    for (auto& az : volume_.dbz) {
        for (auto& el : az) {
            el.fill(0.0);
        }
    }
    zero(vil_);
    zero(precip_);
    zero(composite_);
    vil_tracker_ = PointTracker{};
    precip_tracker_ = PointTracker{};
    vil_points_ = PointProducts{};
    precip_points_ = PointProducts{};
    tracker_.init();
    analyzer_.init();
    windshear_.init();
    turbulence_.init();
    return Status::kOk;
}

void WeatherRadar::reset() noexcept { (void)init(); }

void WeatherRadar::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void WeatherRadar::run_self_test(std::uint32_t frame) noexcept {
    if (!self_test_running_) {
        return;
    }
    if (self_test_started_ == 0) {
        self_test_started_ = frame == 0 ? 1 : frame;
    } else if (frame - self_test_started_ >= kSelfTestFrames) {
        self_test_running_ = false;
        self_test_started_ = 0;
        bit_.in_progress = false;  // nothing in this partition can fail yet
    }
}

bool WeatherRadar::operating() const noexcept {
    return status_.state == TerminalState::NORMAL && mode_ != WeatherMode::NONE;
}

WeatherRadar::Wanted WeatherRadar::wanted_for(WeatherMode scan_mode) noexcept {
    Wanted w;
    switch (scan_mode) {
        case WeatherMode::SURVEILLANCE:
            w.vil = true;
            w.precip = true;
            w.cells = true;
            break;
        case WeatherMode::MAPPING:
        case WeatherMode::PRECIPITATION:
            w.precip = true;
            w.areas = true;
            break;
        case WeatherMode::WINDSHEAR:
            w.windshear = true;
            break;
        case WeatherMode::TURBULENCE:
            w.turbulence = true;
            break;
        case WeatherMode::NONE:
            break;
    }
    return w;
}

void WeatherRadar::set_range_km(double range_km) noexcept {
    if (std::isfinite(range_km)) {
        range_km_ = std::fmin(kMaxRangeKm, std::fmax(kMinRangeKm, range_km));
    }
}

void WeatherRadar::step(const icd::RdrWxWeather* weather, std::uint32_t frame) noexcept {
    if (weather != nullptr) {
        latest_truth_ = truth_from(weather->payload);
        have_truth_ = true;
    }
    run_self_test(frame);
    // Nothing has to arrive before the radar is ready: a clear sky scans like any other.
    if (status_.state == TerminalState::INITIALIZING) {
        status_.state = TerminalState::NORMAL;
    }
    status_.healthy = !bit_.failed;
    status_.mode = status_.state == TerminalState::NORMAL ? mode_ : WeatherMode::NONE;
    if (!operating()) {
        scanning_ = false;
        if (!products_cleared_) {
            clear_products();
        }
        status_.mode_transition = false;
        scan_.scanning = false;
        return;
    }
    if (!scanning_) {
        start_scan(frame);
    }
    render_slice();
    if (next_az_ >= geometry_.azimuths) {
        finish_scan(frame);
    }
    // A commanded mode is "in transition" until a scan made in it has completed.
    status_.mode_transition = scan_.mode != mode_;
    scan_.scanning = scanning_;
}

void WeatherRadar::command_mode(const icd::RdrWxModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state) || !icd::is_valid(cmd.requested_mode)) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    mode_ = cmd.requested_state == TerminalState::NORMAL ? cmd.requested_mode : WeatherMode::NONE;
    set_range_km(cmd.range_km);
    if (std::isfinite(cmd.tilt)) {
        tilt_deg_ = std::fmin(kMaxTiltDeg, std::fmax(kMinTiltDeg, cmd.tilt));
    }
    scan_.range_km = range_km_;
    scan_.tilt = tilt_deg_;
    scanning_ = false;  // the next frame starts a scan with the new settings
}

bool WeatherRadar::take_products_update() noexcept {
    const bool dirty = products_dirty_;
    products_dirty_ = false;
    return dirty;
}

void WeatherRadar::start_scan(std::uint32_t frame) noexcept {
    (void)frame;
    scan_truth_ = have_truth_ ? latest_truth_ : WeatherTruth{};
    scan_mode_ = mode_;
    wanted_ = wanted_for(mode_);
    vcp_ = vcp_for(mode_);
    thickness_ = layer_thicknesses(vcp_);
    geometry_ = Geometry{};
    geometry_.range_bins = static_cast<std::size_t>(std::lround(range_km_));
    texture_.seed = scans_completed_ + 1U;
    next_az_ = 0;
    max_dbz_ = kClipMinDbz;
    scanning_ = true;
}

void WeatherRadar::render_slice() noexcept {
    const std::size_t end = std::min(next_az_ + kAzimuthsPerFrame, geometry_.azimuths);
    for (std::size_t az = next_az_; az < end; ++az) {
        render_azimuth(scan_truth_, vcp_, geometry_, texture_, az, volume_);
        for (std::size_t r = 0; r < kRanges; ++r) {
            const double top = composite_column(volume_, vcp_.count, az, r);
            composite_[az][r] = top;
            max_dbz_ = std::fmax(max_dbz_, top);
            vil_[az][r] = wanted_.vil ? vil_column(volume_, vcp_.count, az, r, thickness_) : 0.0;
            precip_[az][r] = wanted_.precip ? precip_column(volume_, vcp_.count, az, r) : 0.0;
        }
    }
    next_az_ = end;
}

void WeatherRadar::finish_scan(std::uint32_t frame) noexcept {
    scanning_ = false;
    scan_id_ = static_cast<std::uint8_t>(scan_id_ + 1U);
    ++scans_completed_;
    const double now_s = static_cast<double>(frame) * kFrameSeconds;
    const double lowest = vcp_.count != 0 ? vcp_.elevations[0] : 0.0;
    if (wanted_.cells) {
        tracker_.process(composite_, lowest, now_s, kScanIntervalS, labels_);
    }
    if (wanted_.areas) {
        analyzer_.process(composite_, labels_);
    }
    if (wanted_.windshear) {
        windshear_.process(volume_, lowest);
    }
    if (wanted_.turbulence) {
        turbulence_.process(volume_, lowest, filter_);
    }
    if (wanted_.vil) {
        generate_points(vil_, PointKind::kVil, kMaxVilPoints, frame, vil_tracker_, point_work_, vil_points_);
    } else {
        vil_points_ = PointProducts{};
    }
    if (wanted_.precip) {
        generate_points(precip_, PointKind::kPrecip, kMaxPrecipPoints, frame, precip_tracker_, point_work_,
                        precip_points_);
    } else {
        precip_points_ = PointProducts{};
    }
    fill_products();

    scan_.scan_id = scan_id_;
    scan_.mode = scan_mode_;
    scan_.max_turbulence = wanted_.turbulence ? turbulence_.max_category() : icd::TurbulenceCategory::NONE;
    scan_.microburst = wanted_.windshear && windshear_.has_microburst();
    scan_.elevations = static_cast<std::uint8_t>(vcp_.count);
    scan_.range_km = range_km_;
    scan_.tilt = tilt_deg_;
    scan_.max_reflectivity = max_dbz_;
    scan_.storm_cells = wanted_.cells ? saturate8(tracker_.size()) : 0U;
    scan_.precip_areas = wanted_.areas ? saturate8(analyzer_.size()) : 0U;
    scan_.windshear_events = wanted_.windshear ? saturate8(windshear_.size()) : 0U;
    scan_.turbulence_cells = wanted_.turbulence ? saturate8(turbulence_.size()) : 0U;
    scan_.vil_points = saturate8(vil_points_.count);
    scan_.precip_points = saturate8(precip_points_.count);
    scan_.vil_max = vil_points_.max_value;
    scan_.scan_frame = static_cast<std::uint16_t>(frame & 0xFFFFU);
    products_dirty_ = true;
    products_cleared_ = false;
}

void WeatherRadar::fill_products() noexcept {
    products_ = icd::RdrWxProducts{};
    products_.scan_id = scan_id_;
    products_.mode = scan_mode_;
    products_.microburst = wanted_.windshear && windshear_.has_microburst();
    products_.max_turbulence = wanted_.turbulence ? turbulence_.max_category() : icd::TurbulenceCategory::NONE;
    products_.max_reflectivity = max_dbz_;

    const std::size_t vil_n = std::min(vil_points_.count, static_cast<std::size_t>(icd::RdrWxProducts::kMaxVil));
    for (std::size_t i = 0; i < vil_n; ++i) {
        const Point& p = vil_points_.points[i];
        icd::RdrWxProductsVil& rec = products_.vil[i];
        rec.azimuth = static_cast<double>(p.az_index * kDownsample);
        rec.range = static_cast<double>(p.r_index * kDownsample);
        rec.value = p.value;
        rec.intensity = p.intensity;
        rec.layer_count = p.layer_count;
        rec.show_values = p.show_values;
    }
    products_.vil_count = static_cast<std::uint16_t>(vil_n);

    const std::size_t precip_n =
        std::min(precip_points_.count, static_cast<std::size_t>(icd::RdrWxProducts::kMaxPrecip));
    for (std::size_t i = 0; i < precip_n; ++i) {
        const Point& p = precip_points_.points[i];
        icd::RdrWxProductsPrecip& rec = products_.precip[i];
        rec.azimuth = static_cast<double>(p.az_index * kDownsample);
        rec.range = static_cast<double>(p.r_index * kDownsample);
        rec.rate = p.value;
        rec.intensity = p.intensity;
        rec.type = p.type;
        rec.show_values = p.show_values;
    }
    products_.precip_count = static_cast<std::uint16_t>(precip_n);

    if (wanted_.cells) {
        // The largest cells first, then the strongest: sizes are multiples of
        // half a mile, so a thousandfold weight keeps size the first key.
        const std::size_t n = std::min(tracker_.size(), kMaxRegions);
        for (std::size_t i = 0; i < n; ++i) {
            order_keys_[i] = tracker_.cell_at(i).size * 1000.0 + tracker_.cell_at(i).reflectivity;
        }
        stable_sort_descending(order_keys_.data(), cell_order_.data(), n);
        const std::size_t reported = std::min(n, kMaxReportedCells);
        for (std::size_t i = 0; i < reported; ++i) {
            const TrackedCell& c = tracker_.cell_at(cell_order_[i]);
            icd::RdrWxProductsCells& rec = products_.cells[i];
            rec.cell_id = c.id;
            rec.azimuth = c.center_az;
            rec.range = c.center_r;
            rec.reflectivity = c.reflectivity;
            rec.size = c.size;
            rec.intensity = c.intensity;
            rec.azimuth_rate = c.velocity_az;
            rec.range_rate = c.velocity_r;
            rec.altitude = c.altitude;
            rec.vertical_development = c.vertical_development;
        }
        products_.cells_count = static_cast<std::uint16_t>(reported);
    }

    if (wanted_.areas) {
        const std::size_t n = std::min(analyzer_.size(), kMaxRegions);
        for (std::size_t i = 0; i < n; ++i) {
            order_keys_[i] = analyzer_.area_at(i).reflectivity;
        }
        stable_sort_descending(order_keys_.data(), area_order_.data(), n);
        const std::size_t reported = std::min(n, kMaxReportedAreas);
        for (std::size_t i = 0; i < reported; ++i) {
            const PrecipArea& a = analyzer_.area_at(area_order_[i]);
            icd::RdrWxProductsAreas& rec = products_.areas[i];
            rec.azimuth = a.center_az;
            rec.range = a.center_r;
            rec.reflectivity = a.reflectivity;
            rec.rate = a.rate;
            rec.intensity = a.intensity;
            rec.type = a.type;
        }
        products_.areas_count = static_cast<std::uint16_t>(reported);
    }

    if (wanted_.windshear) {
        const std::size_t reported = std::min(windshear_.size(), kMaxReportedShear);
        for (std::size_t i = 0; i < reported; ++i) {
            const ShearEvent& e = windshear_.event_at(i);
            icd::RdrWxProductsWindshear& rec = products_.windshear[i];
            rec.azimuth = static_cast<double>(e.az);
            rec.range = static_cast<double>(e.r);
            rec.shear = e.shear_kts;
            rec.severity = e.severity;
            rec.microburst = e.microburst;
            rec.divergence = e.divergence;
            rec.altitude = e.altitude_ft;
        }
        products_.windshear_count = static_cast<std::uint16_t>(reported);
    }

    if (wanted_.turbulence) {
        const std::size_t reported = std::min(turbulence_.size(), kMaxReportedTurbulence);
        for (std::size_t i = 0; i < reported; ++i) {
            const TurbulenceCell& c = turbulence_.cell_at(i);
            icd::RdrWxProductsTurbulence& rec = products_.turbulence[i];
            rec.azimuth = static_cast<double>(c.az);
            rec.range = static_cast<double>(c.r);
            rec.intensity = c.intensity;
            rec.spectrum_width = c.spectrum_width;
            rec.category = c.category;
            rec.altitude = c.altitude_ft;
        }
        products_.turbulence_count = static_cast<std::uint16_t>(reported);
    }
}

void WeatherRadar::clear_products() noexcept {
    products_ = icd::RdrWxProducts{};
    products_.scan_id = scan_id_;
    products_.mode = WeatherMode::NONE;
    products_.max_reflectivity = kClipMinDbz;
    const std::uint8_t version = scan_.products_version;
    scan_ = icd::RdrWxScan{};
    scan_.scan_id = scan_id_;
    scan_.products_version = version;
    scan_.range_km = range_km_;
    scan_.tilt = tilt_deg_;
    scan_.max_reflectivity = kClipMinDbz;
    products_dirty_ = true;
    products_cleared_ = true;
}

void WeatherRadar::scan_now(const WeatherTruth& truth, WeatherMode scan_mode, std::uint32_t frame) noexcept {
    latest_truth_ = truth;
    have_truth_ = true;
    mode_ = scan_mode;
    status_.state = TerminalState::NORMAL;
    status_.mode = scan_mode;
    scanning_ = false;
    if (!operating()) {
        clear_products();
        return;
    }
    start_scan(frame);
    for (std::uint32_t i = 0; i <= kFramesPerScan && next_az_ < geometry_.azimuths; ++i) {
        render_slice();
    }
    finish_scan(frame);
    status_.mode_transition = false;
    scan_.scanning = false;
}

}  // namespace fmofp::radar
