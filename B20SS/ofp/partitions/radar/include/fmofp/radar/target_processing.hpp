#ifndef FMOFP_RADAR_TARGET_PROCESSING_HPP
#define FMOFP_RADAR_TARGET_PROCESSING_HPP

#include <cstdint>

#include <fmofp/icd/enums.hpp>

// Systems/radarManagement/targeting/target_processor.py (the signature
// analyser and the stealth detector) and radar_data_fusion's threat rule, as
// pure functions over one track's numbers. The Python drew the Swerling
// scintillation from the process-wide random generator; here it is an
// argument, so a caller can draw it from a repeatable source and the oracle
// test can feed the Python the same value.

namespace fmofp::radar {

constexpr double kStealthRcsThresholdM2 = 0.1;  // target_processor._STEALTH_RCS_THRESHOLD_M2
constexpr double kStealthLowRcsM2 = 0.5;        // the middle likelihood band
constexpr double kStealthPrior = 0.05;          // StealthDetector._PRIOR_STEALTH
constexpr std::uint32_t kStealthMinPulses = 8;  // StealthDetector._MIN_PULSES
constexpr double kStealthDecision = 0.6;        // is_stealth above this posterior
constexpr double kMissileSpeedMps = 500.0;      // SignatureAnalyzer's speed and altitude overrides
constexpr double kFighterSpeedMps = 250.0;
constexpr double kFighterAltitudeM = 5000.0;
constexpr double kHighAltitudeM = 15000.0;
constexpr double kHostileRcsM2 = 3.0;    // RadarDataFusion._assess_threat's heuristic
constexpr double kHostileSpeedMps = 200.0;

struct SignatureResult {
    icd::TargetClass type{icd::TargetClass::UNKNOWN};
    double confidence{0.0};     // the analyser's classification confidence, 0.2..0.95
    double rcs_corrected{0.0};  // the measured RCS after the scintillation draw, at least 0.001 m2
};

// SignatureAnalyzer.analyse: `scintillation` is random.expovariate(1.0)'s
// value, which multiplies the RCS; the nearest reference RCS (in log space,
// ties to the earlier entry) names the type, then the speed and altitude
// overrides apply. `altitude_m` is the track's height above the aircraft, as
// the Python's relative z was.
[[nodiscard]] SignatureResult analyse_signature(double rcs_m2, double scintillation, double speed_mps,
                                                double altitude_m) noexcept;

// StealthDetector's state for one track.
struct StealthState {
    bool seen{false};
    double probability{kStealthPrior};
    std::uint32_t pulses{0};
};

struct StealthResult {
    double posterior{0.0};
    double snr_integrated_db{0.0};
    std::uint32_t pulses{0};
    bool is_stealth{false};
};

// StealthDetector.assess for one detection of a track: the non-coherent
// integration gain over max(8, hits + 1) pulses, the likelihood ratio by RCS
// band (8 below 0.1 m2, 2 below 0.5, else 0.2), the Bayesian update of the
// state's probability limited to 0.01..0.99.
[[nodiscard]] StealthResult assess_stealth(StealthState& state, double rcs_m2, double snr_db) noexcept;

// RadarDataFusion._assess_threat: a hostile or friendly identity decides (no
// classification name carries a keyword); otherwise HOSTILE when the average
// RCS is under 3 m2 and the speed over 200 m/s, else UNKNOWN.
[[nodiscard]] icd::Threat assess_threat(icd::TargetClass type, icd::IffIdentity identity, double average_rcs_m2,
                                        double speed_mps) noexcept;

}  // namespace fmofp::radar

#endif
