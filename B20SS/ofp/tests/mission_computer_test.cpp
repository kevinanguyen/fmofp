// The mission computer against the Python it replaces: handleRoute.py's route
// (progress, ETEs, target insertion, threat deviation), targeting.py's status
// machine and decay, orderOfBattle.py's units, and the partition's state,
// commands and plan block the ICD defines.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <fmofp/mission/mission_computer.hpp>

namespace {

using fmofp::Status;
using fmofp::mission::MissionComputer;
using fmofp::mission::Name;
using fmofp::mission::NameWords;
using fmofp::mission::OrderOfBattle;
using fmofp::mission::RouteManager;
using fmofp::mission::Targeting;
using fmofp::mission::haversine_nm;
using fmofp::mission::kMaxTargets;
using fmofp::mission::kMaxWaypoints;
using fmofp::mission::kPlanMinFrames;
using fmofp::mission::kTickFrames;
using fmofp::mission::make_name;
using fmofp::mission::pack_name;
using fmofp::mission::unpack_name;
using namespace fmofp::icd;  // NOLINT: test

std::string text_of(const Name& name) {
    std::string s;
    for (const char c : name) {
        if (c == '\0') {
            break;
        }
        s += c;
    }
    return s;
}

BcastFmsFlightData at(double lat, double lon, double alt_ft = 10000.0, double gs = 450.0) {
    BcastFmsFlightData f{};
    f.payload.position_ssm = Ssm::NORMAL;
    f.payload.latitude = lat;
    f.payload.longitude = lon;
    f.payload.altitude_ssm = Ssm::NORMAL;
    f.payload.altitude = alt_ft;
    f.payload.ground_speed_ssm = Ssm::NORMAL;
    f.payload.ground_speed = gs;
    return f;
}

MsnCmd waypoint_cmd(std::uint8_t seq, const char* name, double lat, double lon, double alt,
                    WaypointType type = WaypointType::NAV) {
    MsnCmd c{};
    c.command = MissionCommand::ADD_WAYPOINT;
    c.cmd_seq = seq;
    c.waypoint_type = type;
    c.fly_over = true;
    c.latitude = lat;
    c.longitude = lon;
    c.altitude = alt;
    const NameWords words = pack_name(make_name(name));
    c.name_0 = words[0];
    c.name_1 = words[1];
    c.name_2 = words[2];
    c.name_3 = words[3];
    return c;
}

TEST(MissionNames, PackAndUnpackEightCharacters) {
    const Name name = make_name("TGT-0001");
    const NameWords words = pack_name(name);
    EXPECT_EQ(0x5447U, words[0]);  // 'T' 'G'
    EXPECT_EQ(0x542DU, words[1]);  // 'T' '-'
    EXPECT_EQ(name, unpack_name(words));
    EXPECT_EQ("IP", text_of(unpack_name(pack_name(make_name("IP")))));
    EXPECT_EQ("LONGNAME", text_of(make_name("LONGNAME_TRUNCATED")));
}

TEST(MissionRoute, HaversineAndBearingAsThePython) {
    // One degree of latitude is 60.04 nm on the Python's 3440.065 nm sphere.
    EXPECT_NEAR(60.04, haversine_nm(0.0, 0.0, 1.0, 0.0), 0.01);
    EXPECT_NEAR(0.0, fmofp::mission::bearing_deg(0.0, 0.0, 1.0, 0.0), 1e-9);
    EXPECT_NEAR(90.0, fmofp::mission::bearing_deg(0.0, 0.0, 0.0, 1.0), 1e-9);
    EXPECT_NEAR(270.0, fmofp::mission::bearing_deg(0.0, 0.0, 0.0, -1.0), 1e-9);
}

TEST(MissionRoute, ReachedWithinHalfAMileAndEtesAtTheGroundSpeed) {
    RouteManager route;
    route.init();
    EXPECT_EQ(1U, route.add(make_name("WP1"), 1.0, 0.0, 10000.0, WaypointType::NAV, true));
    EXPECT_EQ(2U, route.add(make_name("WP2"), 2.0, 0.0, 10000.0, WaypointType::NAV, true));
    route.update_position(0.0, 0.0, 10000.0, 450.0);
    EXPECT_EQ(0U, route.active_index());
    EXPECT_NEAR(60.04, route.distance_to_next_nm(), 0.01);
    EXPECT_NEAR(0.0, route.bearing_to_next_deg(), 1e-9);
    // ete_min = round(cumulative / gs * 60, 1): 60.04 nm at 450 kt is 8.0 min, 120.08 nm 16.0 min.
    EXPECT_TRUE(route.waypoint(0).has_ete);
    EXPECT_NEAR(8.0, route.waypoint(0).ete_min, 1e-9);
    EXPECT_NEAR(16.0, route.waypoint(1).ete_min, 1e-9);
    // Within 0.5 nm of WP1: reached, and WP1's ETE is None.
    route.update_position(1.0 - 0.4 / 60.04, 0.0, 10000.0, 450.0);
    EXPECT_EQ(1U, route.active_index());
    EXPECT_FALSE(route.waypoint(0).has_ete);
    EXPECT_TRUE(route.waypoint(1).has_ete);
    EXPECT_EQ(1U, route.remaining());
    // One waypoint per call: landing on WP2 as well advances once more only on the next call.
    route.update_position(2.0, 0.0, 10000.0, 450.0);
    EXPECT_EQ(2U, route.active_index());
    EXPECT_FALSE(route.has_next());
    EXPECT_EQ(0.0, route.distance_to_next_nm());
    // The ground speed floor of 1 kt keeps the ETE finite.
    route.update_position(0.0, 0.0, 10000.0, 0.0);
    EXPECT_EQ(2U, route.active_index());
}

TEST(MissionRoute, InsertTargetBeforeTheNearestWaypointAheadAndAppendToAnEmptyRoute) {
    RouteManager route;
    route.init();
    // Empty route: the Python deadlocked here (insert_target held the lock add_waypoint takes); we append.
    EXPECT_EQ(1U, route.insert_target(make_name("TGT-0001"), 0.5, 0.0, 0.0));
    EXPECT_EQ(WaypointType::TGT, route.waypoint(0).type);
    route.clear();
    route.add(make_name("A"), 1.0, 0.0, 10000.0, WaypointType::NAV, true);
    route.add(make_name("B"), 2.0, 0.0, 10000.0, WaypointType::NAV, true);
    route.add(make_name("C"), 3.0, 0.0, 10000.0, WaypointType::NAV, true);
    route.update_position(0.0, 0.0, 10000.0, 450.0);
    // Nearest to (2.1, 0) is B at index 1: the target goes before it. Ids keep counting across a clear_route,
    // as the Python's _next_id did: A, B, C took 2-4.
    EXPECT_EQ(5U, route.insert_target(make_name("TGT-0002"), 2.1, 0.0, 0.0));
    EXPECT_EQ(4U, route.count());
    EXPECT_EQ("TGT-0002", text_of(route.waypoint(1).name));
    EXPECT_EQ("B", text_of(route.waypoint(2).name));
    // Only the waypoints ahead count: once A and the target are behind, a target near A goes before C.
    route.update_position(1.0, 0.0, 10000.0, 450.0);  // reaches A
    route.update_position(2.1, 0.0, 10000.0, 450.0);  // reaches TGT-0002
    EXPECT_EQ(2U, route.active_index());
    EXPECT_EQ(6U, route.insert_target(make_name("TGT-0003"), 1.1, 0.0, 0.0));
    EXPECT_EQ("TGT-0003", text_of(route.waypoint(2).name));
    EXPECT_EQ("B", text_of(route.waypoint(3).name));
}

TEST(MissionRoute, DeviateFromThreatPushesInsideWaypointsOutward) {
    RouteManager route;
    route.init();
    route.add(make_name("NEAR"), 1.0, 0.1, 10000.0, WaypointType::NAV, true);  // 6 nm east of the threat
    route.add(make_name("FAR"), 1.0, 1.0, 10000.0, WaypointType::NAV, true);   // 60 nm east: outside 20 nm
    const double d = haversine_nm(1.0, 0.1, 1.0, 0.0);
    EXPECT_EQ(1U, route.deviate_from_threat(1.0, 0.0, 20.0));
    const double factor = (20.0 - d) / d * 1.1;
    EXPECT_NEAR(0.1 + 0.1 * factor, route.waypoint(0).lon, 1e-9);
    EXPECT_NEAR(1.0, route.waypoint(0).lat, 1e-9);
    EXPECT_NEAR(1.0, route.waypoint(1).lon, 1e-9);
    // A waypoint on the threat itself (distance 0) is left where it is, as the Python's 0 < dist test did.
    route.clear();
    route.add(make_name("ON"), 1.0, 0.0, 10000.0, WaypointType::NAV, true);
    EXPECT_EQ(0U, route.deviate_from_threat(1.0, 0.0, 20.0));
}

TEST(MissionRoute, RemoveKeepsTheActiveIndexInRange) {
    RouteManager route;
    route.init();
    route.add(make_name("A"), 1.0, 0.0, 0.0, WaypointType::NAV, true);
    route.add(make_name("B"), 2.0, 0.0, 0.0, WaypointType::NAV, true);
    route.update_position(1.0, 0.0, 0.0, 450.0);  // A reached, active 1
    EXPECT_TRUE(route.remove(2));
    EXPECT_EQ(0U, route.active_index());  // max(0, min(1, len - 1 = 0))
    EXPECT_FALSE(route.remove(9));
    EXPECT_TRUE(route.remove(1));
    EXPECT_EQ(0U, route.count());
    EXPECT_EQ(0U, route.active_index());
    for (std::size_t i = 0; i < kMaxWaypoints; ++i) {
        EXPECT_NE(0U, route.add(make_name("X"), 0.0, 0.0, 0.0, WaypointType::NAV, true));
    }
    EXPECT_EQ(0U, route.add(make_name("FULL"), 0.0, 0.0, 0.0, WaypointType::NAV, true));
}

TEST(MissionTargeting, StatusMachineAsTargetingPy) {
    Targeting t;
    t.init();
    const std::uint8_t id = t.add(TargetKind::SAM, 1.0, 1.0, 0.0, 2, 100);
    EXPECT_EQ(1U, id);
    EXPECT_EQ(TargetStatus::DESIGNATED, t.target(0).status);
    EXPECT_FALSE(t.engage(id, Weapon::JDAM)) << "only an acquired or tracked target is engaged";
    EXPECT_TRUE(t.acquire(id, 150));
    EXPECT_EQ(TargetStatus::ACQUIRED, t.target(0).status);
    EXPECT_DOUBLE_EQ(0.5, t.target(0).track_quality);
    EXPECT_FALSE(t.acquire(id, 150)) << "acquire only from DESIGNATED";
    EXPECT_TRUE(t.engage(id, Weapon::JDAM));
    EXPECT_EQ(Weapon::JDAM, t.target(0).weapon);
    EXPECT_TRUE(t.disengage(id));
    EXPECT_EQ(TargetStatus::TRACKING, t.target(0).status);
    EXPECT_EQ(Weapon::NONE, t.target(0).weapon);
    EXPECT_TRUE(t.engage(id, Weapon::MK_82));
    EXPECT_FALSE(t.assess_damage(id, true)) << "BDA needs the pending mark first";
    EXPECT_TRUE(t.mark_bda_pending(id));
    EXPECT_TRUE(t.assess_damage(id, false));
    EXPECT_EQ(TargetStatus::ACQUIRED, t.target(0).status) << "survived: back to ACQUIRED";
    // Radar updates step DESIGNATED -> ACQUIRED -> TRACKING and cap the quality at 1.
    const std::uint8_t other = t.add(TargetKind::VEHICLE, 1.0, 1.0, 0.0, 1, 200);
    EXPECT_TRUE(t.update_from_radar(other, 1.1, 1.1, 100.0, 30.0, 90.0, 1.7, 210));
    EXPECT_EQ(TargetStatus::ACQUIRED, t.target(1).status);
    EXPECT_DOUBLE_EQ(1.0, t.target(1).track_quality);
    EXPECT_TRUE(t.update_from_radar(other, 1.1, 1.1, 100.0, 30.0, 90.0, 0.8, 220));
    EXPECT_EQ(TargetStatus::TRACKING, t.target(1).status);
    EXPECT_FALSE(t.update_from_radar(9, 0, 0, 0, 0, 0, 0.5, 220));
    // Prioritised: priority 1 before priority 2.
    std::array<std::size_t, kMaxTargets> order{};
    EXPECT_EQ(2U, t.prioritised(order));
    EXPECT_EQ(other, t.target(order[0]).id);
    EXPECT_EQ(id, t.target(order[1]).id);
    EXPECT_TRUE(t.remove(id));
    EXPECT_EQ(1U, t.prioritised(order));
    EXPECT_FALSE(t.remove(id));
}

TEST(MissionTargeting, DecayIsTheQuadraticOfSimulateTick) {
    Targeting t;
    t.init();
    const std::uint8_t id = t.add(TargetKind::POINT, 0.0, 0.0, 0.0, 3, 0);
    EXPECT_TRUE(t.update_from_radar(id, 0, 0, 0, 0, 0, 0.8, 0));
    EXPECT_TRUE(t.update_from_radar(id, 0, 0, 0, 0, 0, 0.8, 0));  // TRACKING
    // Each tick n seconds after the update takes n / 30 * 0.05 (the staleness keeps growing: the Python's
    // last_updated is not touched by the decay), so after n ticks the quality is 0.8 - 0.05 n(n+1) / 60.
    double expected = 0.8;
    for (std::uint32_t n = 1; n <= 10; ++n) {
        EXPECT_TRUE(t.tick(n * kTickFrames));
        expected -= (static_cast<double>(n) / 30.0) * 0.05;
        EXPECT_NEAR(expected, t.target(0).track_quality, 1e-12);
    }
    EXPECT_EQ(TargetStatus::TRACKING, t.target(0).status);
    // 0.8 - 0.05 n(n+1)/60 < 0.1 when n(n+1) > 840: at the 29th tick the track is lost.
    for (std::uint32_t n = 11; n <= 28; ++n) {
        (void)t.tick(n * kTickFrames);
    }
    EXPECT_EQ(TargetStatus::TRACKING, t.target(0).status);
    EXPECT_TRUE(t.tick(29 * kTickFrames));
    EXPECT_EQ(TargetStatus::LOST, t.target(0).status);
    std::array<std::size_t, kMaxTargets> order{};
    EXPECT_EQ(0U, t.prioritised(order)) << "a lost target is not listed";
    // A lost target keeps decaying towards 0 (the Python never stopped); a designated target at quality 0
    // stays at 0 and is never lost (the rule needs TRACKING).
    EXPECT_TRUE(t.tick(30 * kTickFrames));
    EXPECT_TRUE(t.remove(id));
    const std::uint8_t d = t.add(TargetKind::POINT, 0.0, 0.0, 0.0, 3, 30 * kTickFrames);
    EXPECT_FALSE(t.tick(40 * kTickFrames));
    EXPECT_EQ(TargetStatus::DESIGNATED, t.target(t.find(d)).status);
}

TEST(MissionOrderOfBattle, UnitsByAffiliationAndThreats) {
    OrderOfBattle oob;
    oob.init();
    EXPECT_EQ(1U, oob.add(UnitKind::FIGHTER, Affiliation::FRIENDLY, UnitStatus::UNKNOWN, 0, 1.0, 1.0, 20000.0));
    EXPECT_EQ(2U, oob.add(UnitKind::SAM_SITE, Affiliation::ENEMY, UnitStatus::OPERATIONAL, 4, 1.0, 1.1, 0.0));
    EXPECT_EQ(3U, oob.add(UnitKind::ARMOR, Affiliation::ENEMY, UnitStatus::OPERATIONAL, 1, 1.0, 1.2, 0.0));
    EXPECT_EQ(4U, oob.add(UnitKind::SHIP, Affiliation::UNKNOWN, UnitStatus::OPERATIONAL, 0, 1.0, 1.3, 0.0));
    EXPECT_EQ(UnitStatus::OPERATIONAL, oob.unit(0).status) << "the Python's default for a new unit";
    EXPECT_EQ(1U, oob.friendly());
    EXPECT_EQ(2U, oob.enemy());
    EXPECT_EQ(1U, oob.neutral()) << "an unknown affiliation is filed with the neutral forces";
    std::array<std::size_t, fmofp::mission::kMaxUnits> idx{};
    EXPECT_EQ(1U, oob.threats(2, idx));
    EXPECT_EQ(2U, oob.unit(idx[0]).id);
    EXPECT_EQ(2U, oob.threats(1, idx));
    EXPECT_TRUE(oob.update_status(3, UnitStatus::DESTROYED));
    EXPECT_TRUE(oob.update_position(3, 2.0, 2.0, 0.0));
    EXPECT_EQ(2.0, oob.unit(2).lat);
    EXPECT_TRUE(oob.remove(2));
    EXPECT_EQ(3U, oob.count());
    EXPECT_EQ(3U, oob.unit(1).id);
    EXPECT_FALSE(oob.remove(2));
}

TEST(MissionComputerTest, StateFollowsTheFlightDataAndThePlanGoesOutOnce) {
    MissionComputer mc;
    ASSERT_EQ(Status::kOk, mc.init());
    EXPECT_EQ(TerminalState::INITIALIZING, mc.state());
    EXPECT_TRUE(mc.take_plan_update()) << "the first plan, an empty route, goes out at init";
    EXPECT_FALSE(mc.take_plan_update());
    EXPECT_EQ(0U, mc.plan().waypoint_count);
    EXPECT_EQ(1U, mc.plan().plan_seq);
    std::uint32_t frame = 1;
    const BcastFmsFlightData f = at(0.0, 0.0);
    mc.step(&f, frame);
    EXPECT_EQ(TerminalState::NORMAL, mc.state());
    EXPECT_TRUE(mc.status().position_valid);
    EXPECT_FALSE(mc.status().route_active);
    for (; frame < 40; ++frame) {
        mc.step(nullptr, frame);
    }
    EXPECT_EQ(TerminalState::DEGRADED, mc.state());
    EXPECT_FALSE(mc.status().position_valid);
    EXPECT_FALSE(mc.take_plan_update()) << "nothing changed";
}

TEST(MissionComputerTest, CommandsBuildTheRouteTheTargetsAndTheUnits) {
    MissionComputer mc;
    ASSERT_EQ(Status::kOk, mc.init());
    (void)mc.take_plan_update();
    std::uint32_t frame = 1;
    BcastFmsFlightData f = at(0.0, 0.0, 10000.0, 450.0);
    mc.step(&f, frame);

    mc.command(waypoint_cmd(1, "INGRESS", 1.0, 0.0, 15000.0, WaypointType::IP));
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    mc.command(waypoint_cmd(2, "EGRESS", 2.0, 0.0, 20000.0, WaypointType::EGR));
    mc.command(waypoint_cmd(2, "DUPLICATE", 3.0, 0.0, 0.0));  // same cmd_seq: ignored
    EXPECT_EQ(2U, mc.route().count());
    MsnCmd phase{};
    phase.command = MissionCommand::SET_PHASE;
    phase.cmd_seq = 3;
    phase.phase = MissionPhase::INGRESS;
    mc.command(phase);
    EXPECT_EQ(MissionPhase::INGRESS, mc.phase());
    MsnCmd designate{};
    designate.command = MissionCommand::DESIGNATE_TARGET;
    designate.cmd_seq = 4;
    designate.target_kind = TargetKind::BRIDGE;
    designate.priority = 1;
    designate.latitude = 1.9;
    designate.longitude = 0.0;
    designate.altitude = 100.0;
    mc.command(designate);
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    EXPECT_EQ(3U, mc.route().count());
    EXPECT_EQ("TGT-0001", text_of(mc.route().waypoint(1).name)) << "before EGRESS, the nearest waypoint ahead";
    EXPECT_EQ(WaypointType::TGT, mc.route().waypoint(1).type);
    MsnCmd unit{};
    unit.command = MissionCommand::ADD_UNIT;
    unit.cmd_seq = 5;
    unit.unit_kind = UnitKind::SAM_SITE;
    unit.affiliation = Affiliation::ENEMY;
    unit.threat_level = 4;
    unit.latitude = 1.0;
    unit.longitude = 0.1;
    mc.command(unit);
    EXPECT_EQ(1U, mc.oob().enemy());
    // Step and read the status: counts, the active waypoint, the distance to it.
    frame = 2;
    mc.step(&f, frame);
    const MsnStatus& st = mc.status();
    EXPECT_EQ(MissionPhase::INGRESS, st.phase);
    EXPECT_EQ(3U, st.waypoint_count);
    EXPECT_EQ(0U, st.active_index);
    EXPECT_TRUE(st.route_active);
    EXPECT_EQ(1U, st.target_count);
    EXPECT_EQ(1U, st.enemy_units);
    EXPECT_NEAR(60.04, st.distance_to_next, 0.01);
    // The plan: dirty, first publication already made at init -> waits kPlanMinFrames, then carries it all.
    EXPECT_FALSE(mc.take_plan_update());
    for (; frame <= kPlanMinFrames + 1; ++frame) {
        mc.step(&f, frame);
    }
    ASSERT_TRUE(mc.take_plan_update());
    const MsnPlan& plan = mc.plan();
    EXPECT_EQ(2U, plan.plan_seq);
    EXPECT_EQ(3U, plan.waypoint_count);
    EXPECT_EQ(1U, plan.target_count);
    EXPECT_EQ(1U, plan.unit_count);
    EXPECT_EQ(TargetStatus::DESIGNATED, plan.target[0].status);
    EXPECT_EQ(UnitKind::SAM_SITE, plan.unit[0].kind);
    const NameWords words = {plan.waypoint[0].name_0, plan.waypoint[0].name_1, plan.waypoint[0].name_2,
                             plan.waypoint[0].name_3};
    EXPECT_EQ("INGRESS", text_of(unpack_name(words)));
    EXPECT_NEAR(8.0, plan.waypoint[0].ete, 1e-9);
    std::array<fmofp::bus::Word, MsnPlan::kMaxPayloadWords> payload{};
    std::uint16_t length = 0;
    EXPECT_EQ(Status::kOk, plan.encode(payload.data(), static_cast<std::uint16_t>(payload.size()), length));
    EXPECT_EQ(3U + 3U + 3U * 11U + 1U * 9U + 1U * 8U, length);

    // Optimise: the threat is 6 nm east of INGRESS, which is pushed west by deviate_from_threat's factor; the
    // target and EGRESS, 54 nm off, do not move.
    MsnCmd optimise{};
    optimise.command = MissionCommand::OPTIMISE_ROUTE;
    optimise.cmd_seq = 6;
    mc.command(optimise);
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    const double d = haversine_nm(1.0, 0.0, 1.0, 0.1);
    EXPECT_NEAR(-0.1 * (20.0 - d) / d * 1.1, mc.route().waypoint(0).lon, 1e-9);
    EXPECT_NEAR(0.0, mc.route().waypoint(2).lon, 1e-9);

    // Acquire, engage, assess: the target leaves the plan when destroyed.
    MsnCmd acquire{};
    acquire.command = MissionCommand::ACQUIRE_TARGET;
    acquire.cmd_seq = 7;
    acquire.id = 1;
    mc.command(acquire);
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    MsnCmd engage{};
    engage.command = MissionCommand::ENGAGE_TARGET;
    engage.cmd_seq = 8;
    engage.id = 1;
    engage.weapon = Weapon::JDAM;
    mc.command(engage);
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    MsnCmd assess{};
    assess.command = MissionCommand::ASSESS_DAMAGE;
    assess.cmd_seq = 9;
    assess.id = 1;
    assess.destroyed = true;
    mc.command(assess);
    EXPECT_EQ(CmdStatus::ACCEPTED, mc.status().mission_cmd_status);
    EXPECT_EQ(0U, mc.status().target_count);
    // Unknown ids fail; an unknown command is rejected.
    MsnCmd missing{};
    missing.command = MissionCommand::REMOVE_UNIT;
    missing.cmd_seq = 10;
    missing.id = 9;
    mc.command(missing);
    EXPECT_EQ(CmdStatus::FAILED, mc.status().mission_cmd_status);
    MsnCmd none{};
    none.command = MissionCommand::NONE;
    none.cmd_seq = 11;
    mc.command(none);
    EXPECT_EQ(CmdStatus::REJECTED_INVALID, mc.status().mission_cmd_status);
    // Advance the phase to the end and no further.
    MsnCmd advance{};
    advance.command = MissionCommand::ADVANCE_PHASE;
    for (std::uint8_t i = 0; i < 5; ++i) {
        advance.cmd_seq = static_cast<std::uint8_t>(20 + i);
        mc.command(advance);
    }
    EXPECT_EQ(MissionPhase::COMPLETE, mc.phase());
}

TEST(MissionComputerTest, TheRouteProgressesWithThePositionAndRepublishesThePlan) {
    MissionComputer mc;
    ASSERT_EQ(Status::kOk, mc.init());
    (void)mc.take_plan_update();
    std::uint32_t frame = 1;
    BcastFmsFlightData f = at(0.0, 0.0);
    mc.step(&f, frame);
    mc.command(waypoint_cmd(1, "A", 0.01, 0.0, 1000.0));
    mc.command(waypoint_cmd(2, "B", 0.02, 0.0, 1000.0));
    for (frame = 2; frame <= kPlanMinFrames + 2; ++frame) {
        mc.step(&f, frame);
    }
    ASSERT_TRUE(mc.take_plan_update());
    EXPECT_EQ(0U, mc.plan().active_index);
    // Fly onto A: the active index moves and a new plan is due after kPlanMinFrames.
    f = at(0.01, 0.0);
    mc.step(&f, frame);
    EXPECT_EQ(1U, mc.status().active_index);
    EXPECT_FALSE(mc.take_plan_update());
    for (++frame; frame <= 2 * kPlanMinFrames + 10; ++frame) {
        mc.step(&f, frame);
    }
    ASSERT_TRUE(mc.take_plan_update());
    EXPECT_EQ(1U, mc.plan().active_index);
    EXPECT_TRUE(mc.plan().waypoint[0].passed);
    EXPECT_FALSE(mc.plan().waypoint[1].passed);
    EXPECT_EQ(0.0, mc.plan().waypoint[0].ete);
    // A position without a valid SSM leaves the route where it was.
    BcastFmsFlightData bad = at(0.02, 0.0);
    bad.payload.position_ssm = Ssm::NO_COMPUTED_DATA;
    mc.step(&bad, frame);
    EXPECT_EQ(1U, mc.status().active_index);
}

}  // namespace
