#include <fmofp/radar/sar_radar.hpp>

#include <algorithm>
#include <cmath>

#include <fmofp/radar/weather_model.hpp>  // texture_unit: the repeatable uniform draw

namespace fmofp::radar {

namespace {

using icd::ChangeType;
using icd::SarMode;
using icd::Ssm;
using icd::TerminalState;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kHalf = 0.5;
constexpr double kPictureCentre = static_cast<double>(kSarImageSize) / 2.0;  // pixels from the top-left corner
constexpr double kMinCosLatitude = 1e-6;
constexpr std::size_t kMaxPlaced = icd::PlantGround::kMaxArea + icd::PlantGround::kMaxLine + icd::PlantGround::kMaxPoint;
constexpr std::uint32_t kImageSeedStride = 0x9E3779B9U;  // a new speckle for every image
constexpr std::size_t kCfarHalfWindow = kCfarWindow / 2;
constexpr std::uint16_t kMaxArea = 4095;  // RDR_SAR_IMAGE change.area is 12 bits

bool commandable(TerminalState s) noexcept {
    return s == TerminalState::STANDBY || s == TerminalState::NORMAL || s == TerminalState::TEST ||
           s == TerminalState::MAINTENANCE || s == TerminalState::CALIBRATION;
}

// A feature of the scene in the picture's frame: metres along the heading and
// to the right of the picture's centre.
struct Placed {
    enum class Shape : std::uint8_t { kArea, kLine, kPoint };
    Shape shape{Shape::kArea};
    double level{0.0};
    double along{0.0};
    double cross{0.0};
    double along_2{0.0};  // a line's other end
    double cross_2{0.0};
    double extent{0.0};   // an area's radius, a line's width, a point's side
};

struct PlacedScene {
    std::array<Placed, kMaxPlaced> items{};
    std::size_t count{0};
};

// (along, cross) of a point on the ground from the picture's centre.
std::array<double, 2> place(const SarGeometry& g, double latitude_deg, double longitude_deg) noexcept {
    const std::array<double, 2> en = sar_offset_m(latitude_deg, longitude_deg, g.latitude_deg, g.longitude_deg);
    const double s = std::sin(g.heading_deg * kDegToRad);
    const double c = std::cos(g.heading_deg * kDegToRad);
    return {en[0] * s + en[1] * c, en[0] * c - en[1] * s};
}

void place_scene(const icd::PlantGround& scene, const SarGeometry& g, PlacedScene& out) noexcept {
    out.count = 0;
    const std::size_t areas = std::min<std::size_t>(scene.area_count, icd::PlantGround::kMaxArea);
    for (std::size_t i = 0; i < areas; ++i) {
        const icd::PlantGroundArea& a = scene.area[i];
        Placed& p = out.items[out.count++];
        p.shape = Placed::Shape::kArea;
        p.level = static_cast<double>(a.brightness);
        const std::array<double, 2> ac = place(g, a.latitude, a.longitude);
        p.along = ac[0];
        p.cross = ac[1];
        p.extent = a.radius;
    }
    const std::size_t lines = std::min<std::size_t>(scene.line_count, icd::PlantGround::kMaxLine);
    for (std::size_t i = 0; i < lines; ++i) {
        const icd::PlantGroundLine& l = scene.line[i];
        Placed& p = out.items[out.count++];
        p.shape = Placed::Shape::kLine;
        p.level = static_cast<double>(l.brightness);
        const std::array<double, 2> one = place(g, l.latitude_1, l.longitude_1);
        const std::array<double, 2> two = place(g, l.latitude_2, l.longitude_2);
        p.along = one[0];
        p.cross = one[1];
        p.along_2 = two[0];
        p.cross_2 = two[1];
        p.extent = l.width;
    }
    const std::size_t points = std::min<std::size_t>(scene.point_count, icd::PlantGround::kMaxPoint);
    for (std::size_t i = 0; i < points; ++i) {
        const icd::PlantGroundPoint& pt = scene.point[i];
        Placed& p = out.items[out.count++];
        p.shape = Placed::Shape::kPoint;
        p.level = static_cast<double>(pt.brightness);
        const std::array<double, 2> ac = place(g, pt.latitude, pt.longitude);
        p.along = ac[0];
        p.cross = ac[1];
        p.extent = pt.size;
    }
}

double overlap(double a0, double a1, double b0, double b1) noexcept {
    return std::fmax(0.0, std::fmin(a1, b1) - std::fmax(a0, b0));
}

double segment_distance(double px, double py, double ax, double ay, double bx, double by) noexcept {
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 0.0) {
        t = std::fmax(0.0, std::fmin(1.0, ((px - ax) * dx + (py - ay) * dy) / len2));
    }
    const double cx = ax + t * dx;
    const double cy = ay + t * dy;
    return std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

// The pixel's mean backscatter before speckle. Areas paint the ground, lines
// cover the fraction of the pixel their width crosses, points the fraction of
// the pixel their square overlaps: the Python's 1024-pixel image, block-averaged.
double level_at(const PlacedScene& scene, double pixel_m, std::size_t row, std::size_t col) noexcept {
    const double cross_0 = (static_cast<double>(col) - kPictureCentre) * pixel_m;  // the pixel's left edge
    const double cross_1 = cross_0 + pixel_m;
    const double along_1 = (kPictureCentre - static_cast<double>(row)) * pixel_m;  // its top edge
    const double along_0 = along_1 - pixel_m;
    const double cx = cross_0 + kHalf * pixel_m;
    const double ay = along_0 + kHalf * pixel_m;
    double level = kSarOpenGround;
    for (std::size_t i = 0; i < scene.count; ++i) {
        const Placed& p = scene.items[i];
        double cover = 0.0;
        if (p.shape == Placed::Shape::kArea) {
            const double d = std::sqrt((cx - p.cross) * (cx - p.cross) + (ay - p.along) * (ay - p.along));
            cover = d <= p.extent ? 1.0 : 0.0;
        } else if (p.shape == Placed::Shape::kLine) {
            const double d = segment_distance(cx, ay, p.cross, p.along, p.cross_2, p.along_2);
            cover = overlap(-kHalf * p.extent, kHalf * p.extent, d - kHalf * pixel_m, d + kHalf * pixel_m) / pixel_m;
        } else {
            const double half = kHalf * p.extent;
            cover = overlap(cross_0, cross_1, p.cross - half, p.cross + half) *
                    overlap(along_0, along_1, p.along - half, p.along + half) / (pixel_m * pixel_m);
        }
        level += (p.level - level) * cover;
    }
    return level;
}

}  // namespace

double sar_pixel_m(SarMode mode, double resolution_m) noexcept {
    const double factor = mode == SarMode::SCANSAR ? kSarThumbnailFactor * kSarScanSarFactor : kSarThumbnailFactor;
    return factor * resolution_m;
}

std::array<double, 2> sar_offset_m(double latitude_deg, double longitude_deg, double from_latitude_deg,
                                   double from_longitude_deg) noexcept {
    const double east = (longitude_deg - from_longitude_deg) * kDegToRad * std::cos(from_latitude_deg * kDegToRad) *
                        kSarEarthRadiusM;
    const double north = (latitude_deg - from_latitude_deg) * kDegToRad * kSarEarthRadiusM;
    return {east, north};
}

std::array<double, 2> sar_position_deg(double from_latitude_deg, double from_longitude_deg, double east_m,
                                       double north_m) noexcept {
    const double cos_lat = std::fmax(kMinCosLatitude, std::cos(from_latitude_deg * kDegToRad));
    return {from_latitude_deg + north_m / kSarEarthRadiusM * kRadToDeg,
            from_longitude_deg + east_m / (kSarEarthRadiusM * cos_lat) * kRadToDeg};
}

SarGeometry sar_strip_geometry(SarMode mode, double resolution_m, double latitude_deg, double longitude_deg,
                               double heading_deg) noexcept {
    SarGeometry g{};
    g.pixel_m = sar_pixel_m(mode, resolution_m);
    g.centre_along_m = 0.0;
    g.centre_cross_m = kSarStandoffM + kPictureCentre * g.pixel_m;
    g.heading_deg = heading_deg;
    const double s = std::sin(heading_deg * kDegToRad);
    const double c = std::cos(heading_deg * kDegToRad);
    const double east = g.centre_along_m * s + g.centre_cross_m * c;
    const double north = g.centre_along_m * c - g.centre_cross_m * s;
    const std::array<double, 2> centre = sar_position_deg(latitude_deg, longitude_deg, east, north);
    g.latitude_deg = centre[0];
    g.longitude_deg = centre[1];
    return g;
}

double sar_feature_level(const icd::PlantGround& scene, const SarGeometry& g, std::size_t row, std::size_t col) noexcept {
    PlacedScene placed{};
    place_scene(scene, g, placed);
    return level_at(placed, g.pixel_m, row, col);
}

double sar_speckle(std::uint32_t seed, std::uint32_t row, std::uint32_t col) noexcept {
    double sum = 0.0;
    for (std::uint32_t look = 0; look < static_cast<std::uint32_t>(kSarLooks); ++look) {
        const double u = 1.0 - texture_unit(seed, row, col, look);  // (0, 1]
        sum -= std::log(u);
    }
    return sum / static_cast<double>(kSarLooks);
}

void sar_render(const icd::PlantGround& scene, const SarGeometry& g, std::uint32_t seed, SarImage& out) noexcept {
    PlacedScene placed{};
    place_scene(scene, g, placed);
    for (std::size_t row = 0; row < kSarImageSize; ++row) {
        for (std::size_t col = 0; col < kSarImageSize; ++col) {
            const double level = level_at(placed, g.pixel_m, row, col) *
                                 sar_speckle(seed, static_cast<std::uint32_t>(row), static_cast<std::uint32_t>(col));
            const double clipped = std::fmin(kSarMaxLevel, std::fmax(0.0, level));
            out[row * kSarImageSize + col] = static_cast<std::uint8_t>(std::lround(clipped));
        }
    }
}

// ---- ChangeDetector ----------------------------------------------------------------

void ChangeDetector::init() noexcept {
    have_reference_ = false;
    next_id_ = 1;
    regions_ = 0;
}

// scipy's NI_UniformFilter1D: a running sum over the line extended by five
// reflected samples at each end (mode 'reflect': d c b a | a b c d | d c b a).
void ChangeDetector::uniform_line(const double* in, std::size_t stride, std::size_t n, double* out,
                                  std::size_t out_stride) noexcept {
    if (n == 0) {
        return;
    }
    const auto extended = [in, stride, n](std::size_t k) -> double {
        long i = static_cast<long>(k) - static_cast<long>(kCfarHalfWindow);
        if (i < 0) {
            i = -i - 1;
        }
        if (i >= static_cast<long>(n)) {
            i = 2 * static_cast<long>(n) - i - 1;
        }
        return in[static_cast<std::size_t>(i) * stride];
    };
    double tmp = 0.0;
    for (std::size_t k = 0; k < kCfarWindow; ++k) {
        tmp += extended(k);
    }
    out[0] = tmp / static_cast<double>(kCfarWindow);
    for (std::size_t i = 1; i < n; ++i) {
        tmp += extended(i + kCfarWindow - 1) - extended(i - 1);
        out[i * out_stride] = tmp / static_cast<double>(kCfarWindow);
    }
}

void ChangeDetector::uniform_filter(const Plane& in, Plane& out, Plane& scratch) noexcept {
    for (std::size_t c = 0; c < kSarImageSize; ++c) {  // axis 0: down each column
        uniform_line(&in[c], kSarImageSize, kSarImageSize, &scratch[c], kSarImageSize);
    }
    for (std::size_t r = 0; r < kSarImageSize; ++r) {  // axis 1: along each row of the result
        uniform_line(&scratch[r * kSarImageSize], 1, kSarImageSize, &out[r * kSarImageSize], 1);
    }
}

// binary_erosion with the default cross structure and border_value 0: a pixel
// stays only when its four neighbours are set and inside the image.
void ChangeDetector::erode(const Mask& in, Mask& out) noexcept {
    for (std::size_t r = 0; r < kSarImageSize; ++r) {
        for (std::size_t c = 0; c < kSarImageSize; ++c) {
            const std::size_t i = r * kSarImageSize + c;
            const bool kept = in[i] != 0 && r > 0 && in[i - kSarImageSize] != 0 && r + 1 < kSarImageSize &&
                              in[i + kSarImageSize] != 0 && c > 0 && in[i - 1] != 0 && c + 1 < kSarImageSize &&
                              in[i + 1] != 0;
            out[i] = kept ? 1U : 0U;
        }
    }
}

// binary_dilation with the cross: a pixel is set when it or a neighbour inside the image is.
void ChangeDetector::dilate(const Mask& in, Mask& out) noexcept {
    for (std::size_t r = 0; r < kSarImageSize; ++r) {
        for (std::size_t c = 0; c < kSarImageSize; ++c) {
            const std::size_t i = r * kSarImageSize + c;
            const bool set = in[i] != 0 || (r > 0 && in[i - kSarImageSize] != 0) ||
                             (r + 1 < kSarImageSize && in[i + kSarImageSize] != 0) || (c > 0 && in[i - 1] != 0) ||
                             (c + 1 < kSarImageSize && in[i + 1] != 0);
            out[i] = set ? 1U : 0U;
        }
    }
}

// ndimage.label with the cross: 4-connected components numbered in the order
// of their first pixel in raster order.
std::size_t ChangeDetector::label(const Mask& mask) noexcept {
    labels_.fill(0);
    std::uint16_t n = 0;
    for (std::size_t seed = 0; seed < kSarPixels; ++seed) {
        if (mask[seed] == 0 || labels_[seed] != 0) {
            continue;
        }
        ++n;
        std::size_t head = 0;
        std::size_t tail = 0;
        queue_[tail++] = static_cast<std::uint16_t>(seed);
        labels_[seed] = n;
        while (head < tail) {
            const std::size_t at = queue_[head++];
            const std::size_t r = at / kSarImageSize;
            const std::size_t c = at % kSarImageSize;
            const std::array<std::size_t, 4> neighbours{r > 0 ? at - kSarImageSize : at,
                                                        r + 1 < kSarImageSize ? at + kSarImageSize : at,
                                                        c > 0 ? at - 1 : at, c + 1 < kSarImageSize ? at + 1 : at};
            for (const std::size_t nb : neighbours) {
                if (nb != at && mask[nb] != 0 && labels_[nb] == 0 && tail < kSarPixels) {
                    labels_[nb] = n;
                    queue_[tail++] = static_cast<std::uint16_t>(nb);
                }
            }
        }
    }
    return n;
}

std::size_t ChangeDetector::process(const SarImage& image, ChangeEvent* out, std::size_t capacity) noexcept {
    if (!have_reference_) {
        reference_ = image;
        have_reference_ = true;
        regions_ = 0;
        return 0;
    }
    for (std::size_t i = 0; i < kSarPixels; ++i) {
        const double cur = std::fmax(static_cast<double>(image[i]), kChangeLogFloor);
        const double ref = std::fmax(static_cast<double>(reference_[i]), kChangeLogFloor);
        log_ratio_[i] = std::log(cur) - std::log(ref);
        log_abs_[i] = std::fabs(log_ratio_[i]);
        square_[i] = log_abs_[i] * log_abs_[i];
    }
    uniform_filter(log_abs_, mean_, scratch_);
    uniform_filter(square_, variance_, scratch_);
    for (std::size_t i = 0; i < kSarPixels; ++i) {
        const double sd = std::sqrt(std::fmax(0.0, variance_[i] - mean_[i] * mean_[i]));
        mask_[i] = log_abs_[i] > mean_[i] + kCfarK * sd ? 1U : 0U;
    }
    erode(mask_, tmp_);   // binary_opening
    dilate(tmp_, mask_);
    dilate(mask_, tmp_);  // binary_closing
    erode(tmp_, mask_);
    regions_ = label(mask_);
    std::size_t written = 0;
    const std::size_t considered = std::min(regions_, kChangeMaxEvents);
    for (std::uint16_t id = 1; id <= considered; ++id) {
        std::size_t area = 0;
        std::size_t sum_rows = 0;
        std::size_t sum_cols = 0;
        double sum_lr = 0.0;
        for (std::size_t i = 0; i < kSarPixels; ++i) {
            if (labels_[i] == id) {
                ++area;
                sum_rows += i / kSarImageSize;
                sum_cols += i % kSarImageSize;
                sum_lr += log_ratio_[i];
            }
        }
        if (area < kChangeMinAreaPx) {
            continue;
        }
        ChangeEvent e{};
        e.id = next_id_;
        next_id_ = static_cast<std::uint16_t>(next_id_ == 0xFFFFU ? 1U : next_id_ + 1U);
        e.row = static_cast<std::uint8_t>(sum_rows / area);  // int() of the mean: the integer part
        e.col = static_cast<std::uint8_t>(sum_cols / area);
        e.area = static_cast<std::uint16_t>(area);
        e.log_ratio_mean = sum_lr / static_cast<double>(area);
        if (e.log_ratio_mean > kChangeTypeThreshold) {
            e.type = ChangeType::ADDITION;
        } else if (e.log_ratio_mean < -kChangeTypeThreshold) {
            e.type = ChangeType::REMOVAL;
        } else {
            e.type = ChangeType::CHANGE;
        }
        e.confidence = std::fmin(kChangeMaxConfidence, kHalf + std::fabs(e.log_ratio_mean) * 0.3 +
                                                           std::log10(static_cast<double>(area)) * 0.05);
        e.significant = area > kChangeSignificantAreaPx && e.confidence > kChangeSignificantConfidence;
        if (written < capacity && out != nullptr) {
            out[written++] = e;
        }
    }
    reference_ = image;
    return written;
}

// ---- SarRadar ----------------------------------------------------------------------

Status SarRadar::init() noexcept {
    status_ = icd::RdrSarStatus{};
    status_.state = TerminalState::INITIALIZING;
    status_.healthy = true;
    block_ = icd::RdrSarImage{};
    block_ready_ = true;  // the empty block is published first
    mode_ = SarMode::NONE;
    mode_seq_ = bus::CommandSequence{};
    mode_transition_ = false;
    resolution_m_ = kSarDefaultResolutionM;
    scene_ = icd::PlantGround{};
    have_scene_ = false;
    nav_ = SarNav{};
    have_nav_ = false;
    nav_valid_ = false;
    nav_frame_ = 0;
    imaging_ = false;
    image_held_ = false;
    last_image_frame_ = 0;
    spot_ = SarGeometry{};
    spot_valid_ = false;
    geometry_ = SarGeometry{};
    image_.fill(0);
    images_ = 0;
    image_seq_ = 0;
    events_.fill(ChangeEvent{});
    event_count_ = 0;
    significant_ = 0;
    detector_.init();
    self_test_running_ = false;
    self_test_started_ = 0;
    bit_ = bus::BitWord{};
    return Status::kOk;
}

void SarRadar::reset() noexcept { (void)init(); }

void SarRadar::start_self_test() noexcept {
    self_test_running_ = true;
    bit_.in_progress = true;
}

void SarRadar::run_self_test(std::uint32_t frame) noexcept {
    if (!self_test_running_) {
        return;
    }
    if (self_test_started_ == 0) {
        self_test_started_ = frame == 0 ? 1 : frame;
    } else if (frame - self_test_started_ >= kSarSelfTestFrames) {
        self_test_running_ = false;
        self_test_started_ = 0;
        bit_.in_progress = false;
    }
}

bool SarRadar::operating() const noexcept { return status_.state == TerminalState::NORMAL && mode_ != SarMode::NONE; }

bool SarRadar::take_image_update() noexcept {
    const bool ready = block_ready_;
    block_ready_ = false;
    return ready;
}

// set_mode: a new mode or resolution is a new geometry, so the change detector
// starts over (reset_reference, which the Python offered for this and never
// called) and a spotlight is designated afresh.
void SarRadar::command_mode(const icd::RdrSarModeCmd& cmd) noexcept {
    if (!mode_seq_.accept(cmd.cmd_seq)) {
        return;
    }
    status_.last_cmd_seq = cmd.cmd_seq;
    if (!commandable(cmd.requested_state) || !icd::is_valid(cmd.requested_mode) || !std::isfinite(cmd.resolution) ||
        cmd.resolution < kSarMinResolutionM || cmd.resolution > kSarMaxResolutionM) {
        status_.cmd_rejected = true;
        return;
    }
    status_.cmd_rejected = false;
    status_.state = cmd.requested_state;
    const SarMode requested = cmd.requested_state == TerminalState::NORMAL ? cmd.requested_mode : SarMode::NONE;
    const bool changed = requested != mode_ || std::fabs(cmd.resolution - resolution_m_) > 1e-9;
    mode_transition_ = requested != mode_ && requested != SarMode::NONE;
    mode_ = requested;
    resolution_m_ = cmd.resolution;
    if (changed) {
        detector_.reset_reference();
        spot_valid_ = false;
        imaging_ = false;
    }
    if (!operating()) {
        clear_image();
    }
}

void SarRadar::step(const icd::PlantGround* scene, const icd::FmsFlightData* flight, std::uint32_t frame) noexcept {
    run_self_test(frame);
    if (status_.state == TerminalState::INITIALIZING) {
        status_.state = TerminalState::NORMAL;
    }
    if (scene != nullptr) {
        scene_ = *scene;
        have_scene_ = true;
    }
    if (flight != nullptr && flight->position_ssm == Ssm::NORMAL && flight->heading_ssm == Ssm::NORMAL &&
        std::isfinite(flight->latitude) && std::isfinite(flight->longitude) && std::isfinite(flight->heading)) {
        nav_.latitude_deg = flight->latitude;
        nav_.longitude_deg = flight->longitude;
        nav_.heading_deg = flight->heading;
        have_nav_ = true;
        nav_frame_ = frame;
    }
    nav_valid_ = have_nav_ && frame - nav_frame_ <= kSarNavStaleFrames;
    if (operating()) {
        if (nav_valid_ && have_scene_) {
            if (!imaging_) {
                imaging_ = true;  // the integration starts; the image comes kSarImageFrames later
                last_image_frame_ = frame;
                if (mode_ == SarMode::SPOTLIGHT && !spot_valid_) {
                    spot_ = sar_strip_geometry(mode_, resolution_m_, nav_.latitude_deg, nav_.longitude_deg,
                                               nav_.heading_deg);
                    spot_valid_ = true;
                }
            } else if (frame - last_image_frame_ >= kSarImageFrames) {
                form_image();
                last_image_frame_ = frame;
            }
        } else {
            imaging_ = false;
        }
    } else if (image_held_ || imaging_) {
        clear_image();
    }
    status_.healthy = !bit_.failed;
    fill_status();
}

void SarRadar::form_image() noexcept {
    geometry_ = (mode_ == SarMode::SPOTLIGHT && spot_valid_)
                    ? spot_
                    : sar_strip_geometry(mode_, resolution_m_, nav_.latitude_deg, nav_.longitude_deg, nav_.heading_deg);
    sar_render(scene_, geometry_, seed_ + images_ * kImageSeedStride, image_);
    event_count_ = detector_.process(image_, events_.data(), events_.size());
    significant_ = static_cast<std::size_t>(std::count_if(events_.begin(), events_.begin() + static_cast<std::ptrdiff_t>(event_count_),
                                                          [](const ChangeEvent& e) { return e.significant; }));
    ++images_;
    image_seq_ = static_cast<std::uint8_t>(image_seq_ == 255U ? 1U : image_seq_ + 1U);
    fill_block();
    image_held_ = true;
    block_ready_ = true;
    mode_transition_ = false;
}

void SarRadar::clear_image() noexcept {
    block_ = icd::RdrSarImage{};
    block_ready_ = true;
    image_held_ = false;
    imaging_ = false;
    spot_valid_ = false;
    event_count_ = 0;
    significant_ = 0;
    detector_.reset_reference();
}

void SarRadar::fill_status() noexcept {
    status_.mode = status_.state == TerminalState::NORMAL ? mode_ : SarMode::NONE;
    status_.mode_transition = mode_transition_;
    status_.scene_valid = have_scene_;
    status_.nav_valid = nav_valid_;
    status_.image_seq = image_seq_;
}

void SarRadar::fill_block() noexcept {
    block_ = icd::RdrSarImage{};
    block_.image_seq = image_seq_;
    block_.mode = mode_;
    block_.changes = static_cast<std::uint8_t>(std::min<std::size_t>(significant_, kSarMaxReportedChanges));
    block_.resolution = resolution_m_;
    block_.pixel_size = geometry_.pixel_m;
    block_.centre_latitude = geometry_.latitude_deg;
    block_.centre_longitude = geometry_.longitude_deg;
    block_.heading = geometry_.heading_deg;
    block_.centre_along = geometry_.centre_along_m;
    block_.centre_cross = geometry_.centre_cross_m;
    for (std::size_t i = 0; i < icd::RdrSarImage::kMaxPixel; ++i) {
        block_.pixel[i].even = image_[2 * i];
        block_.pixel[i].odd = image_[2 * i + 1];
    }
    block_.pixel_count = icd::RdrSarImage::kMaxPixel;
    const std::size_t n = std::min(event_count_, kSarMaxBlockChanges);
    for (std::size_t i = 0; i < n; ++i) {
        const ChangeEvent& e = events_[i];
        icd::RdrSarImageChange& rec = block_.change[i];
        rec.row = e.row;
        rec.col = e.col;
        rec.area = std::min(e.area, kMaxArea);
        rec.change = e.type;
        rec.significant = e.significant;
        rec.confidence = e.confidence;
        rec.log_ratio = e.log_ratio_mean;
    }
    block_.change_count = static_cast<std::uint16_t>(n);
}

}  // namespace fmofp::radar
