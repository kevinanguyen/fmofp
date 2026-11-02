#ifndef FMOFP_RADAR_SAR_RADAR_HPP
#define FMOFP_RADAR_SAR_RADAR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/remote_terminal.hpp>
#include <fmofp/bus/status.hpp>
#include <fmofp/icd/blocks.hpp>
#include <fmofp/icd/messages.hpp>

// The synthetic aperture radar, ported from
// Systems/radarManagement/syntheticAperture/sar_radar.py and sar_processor.py.
// The Python radar drew its own 1024 x 1024 image on request (random bars for
// STRIPMAP, a disc for SPOTLIGHT, swaths for SCANSAR, noise on everything) and
// ran a change detector over successive images. Here the ground is the plant's
// (PLANT_GROUND, relayed as RDR_SAR_SCENE: point scatterers, lines and areas
// with their backscatter), the radar knows where it is from the FMS flight data
// broadcast, and it images the strip beside its track, or the patch it stared
// at when SPOTLIGHT was commanded, every 150 frames: a 64 x 64 picture, the
// Python's image reduced sixteen-fold on each side (RDR_SAR_IMAGE), speckled as
// a four-look SAR image is, with sar_processor.ChangeDetector's regions of
// change against the previous image. Pure: inputs are ICD messages and blocks,
// outputs are ICD messages and a block.

namespace fmofp::radar {

constexpr std::size_t kSarImageSize = 64;                         // the picture on the bus, rows and columns
constexpr std::size_t kSarPixels = kSarImageSize * kSarImageSize;
constexpr double kSarNativeSize = 1024.0;                         // sar_radar.image_width / image_height
constexpr double kSarThumbnailFactor = kSarNativeSize / static_cast<double>(kSarImageSize);  // 16 native pixels per pixel
constexpr double kSarScanSarFactor = 4.0;                         // ScanSAR: four times the swath at a quarter of the resolution
constexpr double kSarDefaultResolutionM = 1.0;                    // sar_radar.resolution
constexpr double kSarMinResolutionM = 0.1;                        // rmConfig.xml sar_radar resolution
constexpr double kSarMaxResolutionM = 10.0;
constexpr double kSarStandoffM = 2000.0;                          // the strip's near edge, to the right of the track
constexpr std::uint32_t kSarImageFrames = 150;                    // an image every 3 s: 67 segments take 134 frames at 25 Hz
constexpr std::uint32_t kSarNavStaleFrames = 15;                  // the flight data comes every 5 frames
constexpr std::uint32_t kSarSelfTestFrames = 50;
constexpr double kSarFrameSeconds = 0.02;
constexpr double kSarOpenGround = 60.0;                           // the mean backscatter of ground that is nothing in particular
constexpr std::size_t kSarLooks = 4;                              // speckle: the mean of four exponential draws
constexpr double kSarMaxLevel = 255.0;
constexpr double kSarEarthRadiusM = 6371000.0;                    // the plant's flat frame (fmofp_plant.traffic.ownship_enu)
constexpr std::size_t kSarMaxBlockChanges = icd::RdrSarImage::kMaxChange;
constexpr std::uint8_t kSarMaxReportedChanges = 15;               // the header's `changes` saturates
// sar_processor.ChangeDetector
constexpr double kCfarK = 3.0;                                    // CFAR_K
constexpr std::size_t kCfarWindow = 11;                           // CFAR_WINDOW
constexpr std::size_t kChangeMinAreaPx = 10;                      // MIN_AREA_PX
constexpr double kChangeLogFloor = 1.0;                           // LOG_FLOOR
constexpr std::size_t kChangeMaxEvents = 32;                      // MAX_EVENTS
constexpr std::size_t kChangeSignificantAreaPx = 50;              // is_significant: area > 50
constexpr double kChangeSignificantConfidence = 0.6;              // and confidence > 0.6
constexpr double kChangeTypeThreshold = 0.3;                      // ADDITION above, REMOVAL below minus it
constexpr double kChangeMaxConfidence = 0.95;

using SarImage = std::array<std::uint8_t, kSarPixels>;  // row-major, row 0 at the top (farthest along the heading)

// Where a picture lies: its pixel, its centre from the aircraft (along the
// heading, to the right) and on the ground, and its up direction.
struct SarGeometry {
    double pixel_m{0.0};
    double centre_along_m{0.0};
    double centre_cross_m{0.0};
    double heading_deg{0.0};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
};

// Metres of ground per picture pixel: 16 native pixels, 64 in SCANSAR.
[[nodiscard]] double sar_pixel_m(icd::SarMode mode, double resolution_m) noexcept;

// The strip abeam: its near edge kSarStandoffM to the right of an aircraft at
// (latitude, longitude) on `heading`, its centre level with the aircraft.
[[nodiscard]] SarGeometry sar_strip_geometry(icd::SarMode mode, double resolution_m, double latitude_deg,
                                             double longitude_deg, double heading_deg) noexcept;

// Metres east and north of a point from a reference, and back (the plant's flat frame).
[[nodiscard]] std::array<double, 2> sar_offset_m(double latitude_deg, double longitude_deg, double from_latitude_deg,
                                                 double from_longitude_deg) noexcept;
[[nodiscard]] std::array<double, 2> sar_position_deg(double from_latitude_deg, double from_longitude_deg, double east_m,
                                                     double north_m) noexcept;

// The mean backscatter of one pixel before speckle: open ground, or what the
// scene's areas, lines and points put there (areas under lines under points).
[[nodiscard]] double sar_feature_level(const icd::PlantGround& scene, const SarGeometry& g, std::size_t row,
                                       std::size_t col) noexcept;

// The speckle factor of one pixel: the mean of kSarLooks exponential draws, repeatable by seed.
[[nodiscard]] double sar_speckle(std::uint32_t seed, std::uint32_t row, std::uint32_t col) noexcept;

// The whole picture: every pixel's level times its speckle, rounded into 0..255.
void sar_render(const icd::PlantGround& scene, const SarGeometry& g, std::uint32_t seed, SarImage& out) noexcept;

// sar_processor.ChangeEvent.
struct ChangeEvent {
    std::uint16_t id{0};
    std::uint8_t row{0};               // centroid, the integer part of the mean
    std::uint8_t col{0};
    std::uint16_t area{0};
    double log_ratio_mean{0.0};
    icd::ChangeType type{icd::ChangeType::NONE};
    double confidence{0.0};
    bool significant{false};
};

// sar_processor.ChangeDetector: the log-ratio of two images, a CFAR threshold
// over an 11 x 11 window, a morphological opening and closing, 4-connected
// regions of at least ten pixels reported with their centroid, area, mean
// log-ratio, type and confidence. Big (its working planes): a member of a
// static object, never on the stack.
class ChangeDetector {
public:
    void init() noexcept;

    // process(): the first image is stored as the reference and 0 returned; each
    // later one is compared with the reference and becomes it. Returns the
    // number of events written to `out` (at most `capacity`, at most 32 regions
    // considered, as the Python's MAX_EVENTS).
    [[nodiscard]] std::size_t process(const SarImage& image, ChangeEvent* out, std::size_t capacity) noexcept;

    void reset_reference() noexcept { have_reference_ = false; }
    [[nodiscard]] bool has_reference() const noexcept { return have_reference_; }
    [[nodiscard]] std::size_t regions() const noexcept { return regions_; }  // connected components in the last mask

private:
    using Plane = std::array<double, kSarPixels>;
    using Mask = std::array<std::uint8_t, kSarPixels>;

    static void uniform_line(const double* in, std::size_t stride, std::size_t n, double* out,
                             std::size_t out_stride) noexcept;
    static void uniform_filter(const Plane& in, Plane& out, Plane& scratch) noexcept;
    static void erode(const Mask& in, Mask& out) noexcept;
    static void dilate(const Mask& in, Mask& out) noexcept;
    [[nodiscard]] std::size_t label(const Mask& mask) noexcept;

    SarImage reference_{};
    bool have_reference_{false};
    std::uint16_t next_id_{1};
    std::size_t regions_{0};
    Plane log_ratio_{};
    Plane log_abs_{};
    Plane mean_{};
    Plane square_{};
    Plane variance_{};
    Plane scratch_{};
    Mask mask_{};
    Mask tmp_{};
    std::array<std::uint16_t, kSarPixels> labels_{};
    std::array<std::uint16_t, kSarPixels> queue_{};
};

// The aircraft's whereabouts the radar images by, from the flight data.
struct SarNav {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double heading_deg{0.0};
};

class SarRadar {
public:
    [[nodiscard]] Status init() noexcept;

    // The partition's frame. `scene` is an RDR_SAR_SCENE block that completed
    // this frame, `flight` a flight data broadcast that arrived this frame; null
    // otherwise (the radar keeps the last of each).
    void step(const icd::PlantGround* scene, const icd::FmsFlightData* flight, std::uint32_t frame) noexcept;

    // Applied only when its cmd_seq is new (ICD README section 5).
    void command_mode(const icd::RdrSarModeCmd& cmd) noexcept;

    void reset() noexcept;
    void start_self_test() noexcept;

    // True once after each refresh of the image block: the process publishes it.
    [[nodiscard]] bool take_image_update() noexcept;

    [[nodiscard]] const icd::RdrSarStatus& status() const noexcept { return status_; }
    [[nodiscard]] const icd::RdrSarImage& image_block() const noexcept { return block_; }
    [[nodiscard]] icd::TerminalState state() const noexcept { return status_.state; }
    [[nodiscard]] icd::SarMode mode() const noexcept { return mode_; }
    [[nodiscard]] bus::BitWord bit() const noexcept { return bit_; }
    [[nodiscard]] bool operating() const noexcept;
    [[nodiscard]] double resolution_m() const noexcept { return resolution_m_; }
    [[nodiscard]] std::uint32_t images() const noexcept { return images_; }
    [[nodiscard]] bool scene_valid() const noexcept { return have_scene_; }
    [[nodiscard]] bool nav_valid() const noexcept { return nav_valid_; }
    [[nodiscard]] const SarGeometry& geometry() const noexcept { return geometry_; }  // of the last image
    [[nodiscard]] const SarImage& image() const noexcept { return image_; }
    [[nodiscard]] std::size_t events() const noexcept { return event_count_; }
    [[nodiscard]] const ChangeEvent& event(std::size_t index) const noexcept { return events_[index]; }
    [[nodiscard]] std::size_t significant_changes() const noexcept { return significant_; }

    // The seed of the speckle; the tests set it.
    void set_seed(std::uint32_t seed) noexcept { seed_ = seed; }

private:
    void run_self_test(std::uint32_t frame) noexcept;
    void clear_image() noexcept;
    void form_image() noexcept;
    void fill_status() noexcept;
    void fill_block() noexcept;

    // Outputs
    icd::RdrSarStatus status_{};
    icd::RdrSarImage block_{};
    bool block_ready_{false};
    // Settings
    icd::SarMode mode_{icd::SarMode::NONE};
    bus::CommandSequence mode_seq_{};
    bool mode_transition_{false};
    double resolution_m_{kSarDefaultResolutionM};
    // Inputs kept
    icd::PlantGround scene_{};
    bool have_scene_{false};
    SarNav nav_{};
    bool have_nav_{false};
    bool nav_valid_{false};
    std::uint32_t nav_frame_{0};
    // Imaging
    bool imaging_{false};               // an integration is under way (operating, with a scene and a fix)
    bool image_held_{false};            // the block carries an image
    std::uint32_t last_image_frame_{0};
    SarGeometry spot_{};
    bool spot_valid_{false};
    SarGeometry geometry_{};
    SarImage image_{};
    std::uint32_t images_{0};
    std::uint8_t image_seq_{0};
    std::array<ChangeEvent, kChangeMaxEvents> events_{};
    std::size_t event_count_{0};
    std::size_t significant_{0};
    ChangeDetector detector_{};
    std::uint32_t seed_{0x53415220U};
    // Built-in test
    bool self_test_running_{false};
    std::uint32_t self_test_started_{0};
    bus::BitWord bit_{};
};

}  // namespace fmofp::radar

#endif
