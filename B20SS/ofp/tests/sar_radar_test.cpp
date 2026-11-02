// The synthetic aperture radar: its geometry, its picture of the plant's ground, the
// change detector ported from sar_processor.py, and the radar's imaging rhythm.

#include <fmofp/radar/sar_radar.hpp>

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

namespace radar = fmofp::radar;
namespace icd = fmofp::icd;
using fmofp::Status;
using icd::SarMode;
using icd::TerminalState;

namespace {

icd::FmsFlightData fix(double latitude, double longitude, double heading) {
    icd::FmsFlightData f{};
    f.position_ssm = icd::Ssm::NORMAL;
    f.heading_ssm = icd::Ssm::NORMAL;
    f.latitude = latitude;
    f.longitude = longitude;
    f.heading = heading;
    return f;
}

// Features at metres east and north of an aircraft at 0 N 0 E.
icd::PlantGroundPoint point_at(double east, double north, double size, std::uint8_t brightness) {
    icd::PlantGroundPoint p{};
    p.kind = icd::GroundFeature::BUILDING;
    p.brightness = brightness;
    p.size = size;
    const std::array<double, 2> pos = radar::sar_position_deg(0.0, 0.0, east, north);
    p.latitude = pos[0];
    p.longitude = pos[1];
    return p;
}

icd::PlantGroundLine line_between(double east_1, double north_1, double east_2, double north_2, double width,
                                  std::uint8_t brightness) {
    icd::PlantGroundLine l{};
    l.kind = icd::GroundFeature::ROAD;
    l.brightness = brightness;
    l.width = width;
    const std::array<double, 2> one = radar::sar_position_deg(0.0, 0.0, east_1, north_1);
    const std::array<double, 2> two = radar::sar_position_deg(0.0, 0.0, east_2, north_2);
    l.latitude_1 = one[0];
    l.longitude_1 = one[1];
    l.latitude_2 = two[0];
    l.longitude_2 = two[1];
    return l;
}

icd::PlantGroundArea area_at(double east, double north, double radius, std::uint8_t brightness) {
    icd::PlantGroundArea a{};
    a.kind = icd::GroundFeature::LAKE;
    a.brightness = brightness;
    a.radius = radius;
    const std::array<double, 2> pos = radar::sar_position_deg(0.0, 0.0, east, north);
    a.latitude = pos[0];
    a.longitude = pos[1];
    return a;
}

icd::RdrSarModeCmd command(std::uint8_t seq, TerminalState state, SarMode mode, double resolution = 1.0) {
    icd::RdrSarModeCmd c{};
    c.cmd_seq = seq;
    c.requested_state = state;
    c.requested_mode = mode;
    c.resolution = resolution;
    return c;
}

radar::SarGeometry strip(SarMode mode, double resolution) {
    return radar::sar_strip_geometry(mode, resolution, 0.0, 0.0, 0.0);
}

// The road beside a northbound track, 2.5 km east: in column 31 of a 1 m strip.
icd::PlantGround road_scene() {
    icd::PlantGround scene{};
    scene.line[0] = line_between(2500.0, -10000.0, 2500.0, 10000.0, 12.0, 20);
    scene.line_count = 1;
    return scene;
}

double column_mean(const radar::SarImage& image, std::size_t col) {
    double sum = 0.0;
    for (std::size_t row = 0; row < radar::kSarImageSize; ++row) {
        sum += image[row * radar::kSarImageSize + col];
    }
    return sum / static_cast<double>(radar::kSarImageSize);
}

// Runs the radar to `frame`, feeding the fix every fifth frame as the broadcast does.
void run_to(radar::SarRadar& r, std::uint32_t& frame, std::uint32_t until, const icd::FmsFlightData* flight) {
    while (frame < until) {
        ++frame;
        r.step(nullptr, frame % 5 == 0 ? flight : nullptr, frame);
    }
}

}  // namespace

TEST(SarRadar, APixelIsSixteenNativePixelsAndSixtyFourInScanSar) {
    EXPECT_DOUBLE_EQ(16.0, radar::sar_pixel_m(SarMode::STRIPMAP, 1.0));
    EXPECT_DOUBLE_EQ(4.8, radar::sar_pixel_m(SarMode::SPOTLIGHT, 0.3));
    EXPECT_DOUBLE_EQ(640.0, radar::sar_pixel_m(SarMode::SCANSAR, 10.0));
}

TEST(SarRadar, TheStripLiesTwoKilometresToTheRightOfTheTrack) {
    const radar::SarGeometry north = strip(SarMode::STRIPMAP, 1.0);
    EXPECT_DOUBLE_EQ(16.0, north.pixel_m);
    EXPECT_DOUBLE_EQ(0.0, north.centre_along_m);
    EXPECT_DOUBLE_EQ(2000.0 + 32.0 * 16.0, north.centre_cross_m);
    const std::array<double, 2> en = radar::sar_offset_m(north.latitude_deg, north.longitude_deg, 0.0, 0.0);
    EXPECT_NEAR(2512.0, en[0], 1e-6);  // east of a northbound aircraft
    EXPECT_NEAR(0.0, en[1], 1e-6);
    const radar::SarGeometry east = radar::sar_strip_geometry(SarMode::STRIPMAP, 1.0, 0.0, 0.0, 90.0);
    const std::array<double, 2> en_east = radar::sar_offset_m(east.latitude_deg, east.longitude_deg, 0.0, 0.0);
    EXPECT_NEAR(0.0, en_east[0], 1e-6);
    EXPECT_NEAR(-2512.0, en_east[1], 1e-6);  // south of an eastbound one
    const radar::SarGeometry wide = strip(SarMode::SCANSAR, 10.0);
    EXPECT_DOUBLE_EQ(2000.0 + 32.0 * 640.0, wide.centre_cross_m);
}

TEST(SarRadar, OpenGroundIsSixtyAndSpeckled) {
    const icd::PlantGround empty{};
    const radar::SarGeometry g = strip(SarMode::STRIPMAP, 1.0);
    EXPECT_DOUBLE_EQ(60.0, radar::sar_feature_level(empty, g, 10, 10));
    auto image = std::make_unique<radar::SarImage>();
    radar::sar_render(empty, g, 7, *image);
    double sum = 0.0;
    int lo = 255;
    int hi = 0;
    for (const std::uint8_t v : *image) {
        sum += v;
        lo = std::min(lo, static_cast<int>(v));
        hi = std::max(hi, static_cast<int>(v));
    }
    EXPECT_NEAR(60.0, sum / static_cast<double>(radar::kSarPixels), 3.0) << "four-look speckle averages to one";
    EXPECT_LT(lo, 40);
    EXPECT_GT(hi, 90);
    auto again = std::make_unique<radar::SarImage>();
    radar::sar_render(empty, g, 7, *again);
    EXPECT_EQ(*image, *again) << "the same seed draws the same speckle";
    radar::sar_render(empty, g, 8, *again);
    EXPECT_NE(*image, *again);
}

TEST(SarRadar, ARoadDarkensTheColumnItCrosses) {
    const icd::PlantGround scene = road_scene();
    const radar::SarGeometry g = strip(SarMode::STRIPMAP, 1.0);
    // Column 31's centre is 2504 m east: 4 m from the road's axis, 10 of its 16 m under the 12 m road.
    EXPECT_NEAR(35.0, radar::sar_feature_level(scene, g, 20, 31), 1e-6);
    EXPECT_NEAR(55.0, radar::sar_feature_level(scene, g, 20, 30), 1e-6);
    EXPECT_NEAR(60.0, radar::sar_feature_level(scene, g, 20, 32), 1e-6);
    // In a 64 m ScanSAR pixel the same road covers less of column 7.
    const radar::SarGeometry wide = strip(SarMode::SCANSAR, 1.0);
    EXPECT_NEAR(52.5, radar::sar_feature_level(scene, wide, 20, 7), 1e-6);
    EXPECT_NEAR(60.0, radar::sar_feature_level(scene, wide, 20, 8), 1e-6);
    auto image = std::make_unique<radar::SarImage>();
    radar::sar_render(scene, g, 1, *image);
    EXPECT_LT(column_mean(*image, 31) + 10.0, column_mean(*image, 10)) << "the road shows through the speckle";
}

TEST(SarRadar, ABuildingBrightensThePixelItCovers) {
    icd::PlantGround scene{};
    scene.point[0] = point_at(2648.0, 184.0, 20.0, 230);  // the centre of pixel (20, 40) of the 1 m strip
    scene.point_count = 1;
    const radar::SarGeometry g = strip(SarMode::STRIPMAP, 1.0);
    EXPECT_NEAR(230.0, radar::sar_feature_level(scene, g, 20, 40), 1e-6);
    EXPECT_NEAR(81.25, radar::sar_feature_level(scene, g, 20, 41), 1e-6) << "two of sixteen metres";
    EXPECT_NEAR(81.25, radar::sar_feature_level(scene, g, 21, 40), 1e-6);
    EXPECT_NEAR(60.0, radar::sar_feature_level(scene, g, 22, 40), 1e-6);
    // A 20 m building in a 64 m pixel is a tenth of it.
    scene.point[0] = point_at(4592.0, 736.0, 20.0, 230);  // the centre of pixel (20, 40) of the 1 m ScanSAR picture
    const radar::SarGeometry wide = strip(SarMode::SCANSAR, 1.0);
    EXPECT_NEAR(60.0 + 170.0 * 400.0 / 4096.0, radar::sar_feature_level(scene, wide, 20, 40), 1e-6);
}

TEST(SarRadar, ALakeIsDarkAndTheRoadAcrossItShows) {
    icd::PlantGround scene{};
    scene.area[0] = area_at(2512.0, 0.0, 500.0, 6);  // the picture's centre
    scene.area_count = 1;
    const radar::SarGeometry g = strip(SarMode::STRIPMAP, 1.0);
    EXPECT_NEAR(6.0, radar::sar_feature_level(scene, g, 32, 32), 1e-6);
    EXPECT_NEAR(60.0, radar::sar_feature_level(scene, g, 0, 32), 1e-6) << "504 m from the centre";
    scene.line[0] = line_between(2512.0, -10000.0, 2512.0, 10000.0, 12.0, 20);
    scene.line_count = 1;
    EXPECT_NEAR(6.0 + 14.0 * 6.0 / 16.0, radar::sar_feature_level(scene, g, 32, 32), 1e-6) << "lines paint over areas";
}

TEST(SarRadar, TheChangeDetectorKeepsTheFirstImageAsItsReference) {
    auto d = std::make_unique<radar::ChangeDetector>();
    d->init();
    radar::SarImage a{};
    a.fill(60);
    std::array<radar::ChangeEvent, radar::kChangeMaxEvents> events{};
    EXPECT_FALSE(d->has_reference());
    EXPECT_EQ(0U, d->process(a, events.data(), events.size()));
    EXPECT_TRUE(d->has_reference());
    EXPECT_EQ(0U, d->process(a, events.data(), events.size())) << "identical images: nothing exceeds a zero threshold";
    EXPECT_EQ(0U, d->regions());
}

TEST(SarRadar, TheChangeDetectorReportsACompactChangeOfTenPixels) {
    auto d = std::make_unique<radar::ChangeDetector>();
    d->init();
    radar::SarImage a{};
    a.fill(60);
    radar::SarImage b = a;
    // Two crosses sharing a row: the smallest shape that survives the opening and
    // the closing with ten pixels, and few enough for the CFAR window not to mask it.
    const std::array<std::array<std::size_t, 2>, 10> pixels{
        {{30, 30}, {29, 30}, {31, 30}, {30, 29}, {30, 31}, {30, 33}, {29, 33}, {31, 33}, {30, 32}, {30, 34}}};
    for (const auto& p : pixels) {
        b[p[0] * radar::kSarImageSize + p[1]] = 255;
    }
    std::array<radar::ChangeEvent, radar::kChangeMaxEvents> events{};
    EXPECT_EQ(0U, d->process(a, events.data(), events.size()));
    ASSERT_EQ(1U, d->process(b, events.data(), events.size()));
    const radar::ChangeEvent& e = events[0];
    EXPECT_EQ(1U, e.id);
    EXPECT_EQ(10U, e.area);
    EXPECT_EQ(30U, e.row);
    EXPECT_EQ(31U, e.col) << "the integer part of 31.5";
    EXPECT_NEAR(std::log(255.0 / 60.0), e.log_ratio_mean, 1e-12);
    EXPECT_EQ(icd::ChangeType::ADDITION, e.type);
    EXPECT_DOUBLE_EQ(0.95, e.confidence);
    EXPECT_FALSE(e.significant) << "ten pixels, not fifty";
    // The reverse change is a removal, with the next id.
    ASSERT_EQ(1U, d->process(a, events.data(), events.size()));
    EXPECT_EQ(2U, events[0].id);
    EXPECT_EQ(icd::ChangeType::REMOVAL, events[0].type);
}

TEST(SarRadar, AUniformBlockOfChangeMasksItselfInTheCfarWindow) {
    // sar_processor's CFAR takes its statistics from the window around each pixel,
    // the pixel's own block included: a 10 x 10 change raises the threshold above
    // itself and is never reported. What is reported is unchanged ground: the
    // running-sum box filter leaves a rounding residue of a few ulps where its
    // window has passed the block, so a threshold a hair below zero lets a
    // log-ratio of exactly zero through. scipy's filter does the same and the
    // Python reports the same region (test_sar_oracle.py). Kept as written
    // (ICD README section 2).
    auto d = std::make_unique<radar::ChangeDetector>();
    d->init();
    radar::SarImage a{};
    a.fill(60);
    radar::SarImage b = a;
    for (std::size_t r = 20; r < 30; ++r) {
        for (std::size_t c = 20; c < 30; ++c) {
            b[r * radar::kSarImageSize + c] = 255;
        }
    }
    std::array<radar::ChangeEvent, radar::kChangeMaxEvents> events{};
    EXPECT_EQ(0U, d->process(a, events.data(), events.size()));
    const std::size_t n = d->process(b, events.data(), events.size());
    for (std::size_t i = 0; i < n; ++i) {
        EXPECT_NEAR(0.0, events[i].log_ratio_mean, 1e-9) << "not the block: unchanged ground beside it";
        EXPECT_EQ(icd::ChangeType::CHANGE, events[i].type);
    }
}

TEST(SarRadar, ImagesEveryHundredAndFiftyFramesOnceItHasASceneAndAFix) {
    auto r = std::make_unique<radar::SarRadar>();
    ASSERT_EQ(Status::kOk, r->init());
    r->set_seed(3);
    const icd::PlantGround scene = road_scene();
    const icd::FmsFlightData flight = fix(0.0, 0.0, 0.0);
    std::uint32_t frame = 1;
    r->step(nullptr, nullptr, frame);
    EXPECT_EQ(TerminalState::NORMAL, r->state());
    EXPECT_TRUE(r->take_image_update()) << "the empty block is published first";
    EXPECT_EQ(SarMode::NONE, r->image_block().mode);
    EXPECT_EQ(0U, r->image_block().pixel_count);

    ++frame;
    r->command_mode(command(1, TerminalState::NORMAL, SarMode::STRIPMAP, 1.0));
    r->step(&scene, &flight, frame);
    EXPECT_TRUE(r->operating());
    EXPECT_FALSE(r->take_image_update());
    const icd::RdrSarStatus& s = r->status();
    EXPECT_EQ(SarMode::STRIPMAP, s.mode);
    EXPECT_TRUE(s.mode_transition);
    EXPECT_TRUE(s.scene_valid);
    EXPECT_TRUE(s.nav_valid);
    EXPECT_EQ(0U, s.image_seq);

    run_to(*r, frame, 2 + radar::kSarImageFrames - 1, &flight);
    EXPECT_FALSE(r->take_image_update()) << "the integration is not over";
    run_to(*r, frame, 2 + radar::kSarImageFrames, &flight);
    ASSERT_TRUE(r->take_image_update());
    const icd::RdrSarImage& block = r->image_block();
    EXPECT_EQ(SarMode::STRIPMAP, block.mode);
    EXPECT_EQ(1U, block.image_seq);
    EXPECT_EQ(2048U, block.pixel_count);
    EXPECT_DOUBLE_EQ(1.0, block.resolution);
    EXPECT_DOUBLE_EQ(16.0, block.pixel_size);
    EXPECT_DOUBLE_EQ(0.0, block.heading);
    EXPECT_DOUBLE_EQ(0.0, block.centre_along);
    EXPECT_DOUBLE_EQ(2512.0, block.centre_cross);
    EXPECT_EQ(0U, block.changes);
    EXPECT_EQ(0U, block.change_count);
    EXPECT_EQ(1U, r->status().image_seq);
    EXPECT_FALSE(r->status().mode_transition);
    EXPECT_EQ(1U, r->images());
    EXPECT_LT(column_mean(r->image(), 31) + 10.0, column_mean(r->image(), 10)) << "the road is in the picture";
    EXPECT_EQ(r->image()[0], block.pixel[0].even);
    EXPECT_EQ(r->image()[1], block.pixel[0].odd);

    std::array<fmofp::bus::Word, icd::RdrSarImage::kMaxPayloadWords> words{};
    std::uint16_t length = 0;
    EXPECT_EQ(Status::kOk, block.encode(words.data(), static_cast<std::uint16_t>(words.size()), length));
    EXPECT_EQ(2060U + 3U * block.change_count, length) << "ten header words, a count, 2048 pixel words, a count, the changes";

    run_to(*r, frame, 2 + 2 * radar::kSarImageFrames, &flight);
    ASSERT_TRUE(r->take_image_update());
    EXPECT_EQ(2U, r->image_block().image_seq);
    EXPECT_EQ(r->events(), r->image_block().change_count);
}

TEST(SarRadar, ASpotlightStaysOnThePatchWhereItWasCommanded) {
    auto r = std::make_unique<radar::SarRadar>();
    ASSERT_EQ(Status::kOk, r->init());
    const icd::PlantGround scene = road_scene();
    std::uint32_t frame = 0;
    r->command_mode(command(1, TerminalState::NORMAL, SarMode::SPOTLIGHT, 0.3));
    // The aircraft flies north at 231.5 m/s: a degree of latitude every 111 km.
    const auto flight_at = [](std::uint32_t f) { return fix(231.5 * 0.02 * f / 111195.0, 0.0, 0.0); };
    icd::FmsFlightData flight = flight_at(0);
    r->step(&scene, &flight, ++frame);
    while (frame < 1 + radar::kSarImageFrames) {
        flight = flight_at(frame);
        r->step(nullptr, &flight, ++frame);
    }
    ASSERT_TRUE(r->take_image_update());
    const radar::SarGeometry first = r->geometry();
    EXPECT_DOUBLE_EQ(4.8, first.pixel_m);
    EXPECT_NEAR(0.0, first.latitude_deg, 1e-9) << "the patch abeam when the mode was commanded";
    while (frame < 1 + 2 * radar::kSarImageFrames) {
        flight = flight_at(frame);
        r->step(nullptr, &flight, ++frame);
    }
    ASSERT_TRUE(r->take_image_update());
    EXPECT_DOUBLE_EQ(first.latitude_deg, r->geometry().latitude_deg);
    EXPECT_DOUBLE_EQ(first.longitude_deg, r->geometry().longitude_deg);
    EXPECT_DOUBLE_EQ(first.latitude_deg, r->image_block().centre_latitude);
    // A strip would have moved with the aircraft.
    r->command_mode(command(2, TerminalState::NORMAL, SarMode::STRIPMAP, 0.3));
    while (frame < 1 + 3 * radar::kSarImageFrames + 1) {
        flight = flight_at(frame);
        r->step(nullptr, &flight, ++frame);
    }
    ASSERT_TRUE(r->take_image_update());
    EXPECT_GT(r->geometry().latitude_deg, first.latitude_deg + 0.005);
}

TEST(SarRadar, StandbyEmptiesTheBlockAndABadResolutionIsRejected) {
    auto r = std::make_unique<radar::SarRadar>();
    ASSERT_EQ(Status::kOk, r->init());
    const icd::PlantGround scene = road_scene();
    const icd::FmsFlightData flight = fix(0.0, 0.0, 0.0);
    std::uint32_t frame = 0;
    r->command_mode(command(1, TerminalState::NORMAL, SarMode::SCANSAR, 2.0));
    r->step(&scene, &flight, ++frame);
    (void)r->take_image_update();
    run_to(*r, frame, 1 + radar::kSarImageFrames, &flight);
    ASSERT_TRUE(r->take_image_update());
    EXPECT_DOUBLE_EQ(128.0, r->image_block().pixel_size);

    r->command_mode(command(2, TerminalState::NORMAL, SarMode::SCANSAR, 50.0));
    EXPECT_TRUE(r->status().cmd_rejected);
    EXPECT_DOUBLE_EQ(2.0, r->resolution_m());
    r->command_mode(command(2, TerminalState::NORMAL, SarMode::SCANSAR, 0.5)) /* the same seq: ignored */;
    EXPECT_DOUBLE_EQ(2.0, r->resolution_m());

    r->command_mode(command(3, TerminalState::STANDBY, SarMode::NONE));
    r->step(nullptr, &flight, ++frame);
    EXPECT_FALSE(r->operating());
    ASSERT_TRUE(r->take_image_update());
    EXPECT_EQ(SarMode::NONE, r->image_block().mode);
    EXPECT_EQ(0U, r->image_block().pixel_count);
    EXPECT_EQ(0U, r->image_block().image_seq);
    EXPECT_EQ(SarMode::NONE, r->status().mode);
    EXPECT_EQ(1U, r->status().image_seq) << "the last image's number stays in the status";
    EXPECT_EQ(3U, r->status().last_cmd_seq);
    EXPECT_FALSE(r->status().cmd_rejected);
}

TEST(SarRadar, NoImageWithoutASceneOrAFix) {
    auto r = std::make_unique<radar::SarRadar>();
    ASSERT_EQ(Status::kOk, r->init());
    const icd::FmsFlightData flight = fix(0.0, 0.0, 0.0);
    std::uint32_t frame = 0;
    r->command_mode(command(1, TerminalState::NORMAL, SarMode::STRIPMAP));
    r->step(nullptr, &flight, ++frame);
    (void)r->take_image_update();
    run_to(*r, frame, 2 * radar::kSarImageFrames, &flight);
    EXPECT_FALSE(r->take_image_update());
    EXPECT_EQ(0U, r->images());
    EXPECT_FALSE(r->status().scene_valid);
    EXPECT_TRUE(r->status().nav_valid);
    EXPECT_TRUE(r->status().mode_transition) << "no image yet in the commanded mode";
    // The scene arrives but the fix goes stale: the integration restarts when it returns.
    const icd::PlantGround scene = road_scene();
    r->step(&scene, nullptr, ++frame);
    run_to(*r, frame, 4 * radar::kSarImageFrames, nullptr);
    EXPECT_FALSE(r->take_image_update());
    EXPECT_FALSE(r->status().nav_valid);
    EXPECT_TRUE(r->status().scene_valid);
    r->step(nullptr, &flight, ++frame);
    run_to(*r, frame, frame + radar::kSarImageFrames - 1, &flight);
    EXPECT_FALSE(r->take_image_update());
    run_to(*r, frame, frame + 1, &flight);
    EXPECT_TRUE(r->take_image_update());
    EXPECT_EQ(1U, r->images());
}
