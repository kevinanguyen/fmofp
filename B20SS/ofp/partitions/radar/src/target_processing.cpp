#include <fmofp/radar/target_processing.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace fmofp::radar {

namespace {

struct RcsReference {
    icd::TargetClass type;
    double rcs_m2;
};

// target_processor._RCS_REFERENCE in the Python dictionary's order; the
// nearest-neighbour search keeps the earlier entry on a tie (strict less-than).
constexpr std::array<RcsReference, 7> kRcsReference{{
    {icd::TargetClass::FIGHTER, 2.0},
    {icd::TargetClass::BOMBER, 40.0},
    {icd::TargetClass::HIGH_ALT, 10.0},
    {icd::TargetClass::MISSILE, 0.05},
    {icd::TargetClass::HELICOPTER, 3.0},
    {icd::TargetClass::DRONE, 0.01},
    {icd::TargetClass::UNKNOWN, 5.0},
}};

constexpr double kRcsFloorM2 = 0.001;

}  // namespace

SignatureResult analyse_signature(double rcs_m2, double scintillation, double speed_mps, double altitude_m) noexcept {
    SignatureResult out{};
    out.rcs_corrected = std::fmax(kRcsFloorM2, rcs_m2 * scintillation);
    const double log_rcs = std::log10(std::fmax(out.rcs_corrected, kRcsFloorM2));
    double best_delta = std::numeric_limits<double>::infinity();
    icd::TargetClass best = icd::TargetClass::UNKNOWN;
    for (const RcsReference& reference : kRcsReference) {
        const double delta = std::fabs(log_rcs - std::log10(std::fmax(reference.rcs_m2, kRcsFloorM2)));
        if (delta < best_delta) {
            best_delta = delta;
            best = reference.type;
        }
    }
    out.confidence = std::fmax(0.2, std::fmin(0.95, 1.0 - best_delta / 3.0));
    if (speed_mps > kMissileSpeedMps) {
        best = icd::TargetClass::MISSILE;
    } else if (speed_mps > kFighterSpeedMps && altitude_m > kFighterAltitudeM) {
        best = icd::TargetClass::FIGHTER;
    } else if (altitude_m > kHighAltitudeM) {
        best = icd::TargetClass::HIGH_ALT;
    }
    out.type = best;
    return out;
}

StealthResult assess_stealth(StealthState& state, double rcs_m2, double snr_db) noexcept {
    StealthResult out{};
    const std::uint32_t pulses = std::max(kStealthMinPulses, state.pulses + 1U);
    out.snr_integrated_db = snr_db + 10.0 * std::log10(std::sqrt(static_cast<double>(pulses)));
    double likelihood_ratio = 0.2;
    if (rcs_m2 < kStealthRcsThresholdM2) {
        likelihood_ratio = 8.0;
    } else if (rcs_m2 < kStealthLowRcsM2) {
        likelihood_ratio = 2.0;
    }
    const double prior = state.seen ? state.probability : kStealthPrior;
    double posterior = prior * likelihood_ratio / (prior * likelihood_ratio + (1.0 - prior));
    posterior = std::fmax(0.01, std::fmin(0.99, posterior));
    state.seen = true;
    state.probability = posterior;
    state.pulses = pulses;
    out.posterior = posterior;
    out.pulses = pulses;
    out.is_stealth = posterior > kStealthDecision && rcs_m2 < kStealthRcsThresholdM2;
    return out;
}

icd::Threat assess_threat(icd::TargetClass type, icd::IffIdentity identity, double average_rcs_m2,
                          double speed_mps) noexcept {
    (void)type;  // FIGHTER, BOMBER, HIGH_ALT, MISSILE, HELICOPTER, DRONE, STEALTH, UNKNOWN: no keyword among them
    if (identity == icd::IffIdentity::HOSTILE) {
        return icd::Threat::HOSTILE;
    }
    if (identity == icd::IffIdentity::FRIENDLY) {
        return icd::Threat::FRIENDLY;
    }
    if (average_rcs_m2 < kHostileRcsM2 && speed_mps > kHostileSpeedMps) {
        return icd::Threat::HOSTILE;
    }
    return icd::Threat::UNKNOWN;
}

}  // namespace fmofp::radar
