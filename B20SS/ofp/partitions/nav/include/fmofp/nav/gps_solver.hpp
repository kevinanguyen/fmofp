#ifndef FMOFP_NAV_GPS_SOLVER_HPP
#define FMOFP_NAV_GPS_SOLVER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include <fmofp/bus/status.hpp>

// The GPS position solver, ported from Systems/nav/gps/globalPositionSystem.py:
// the 12-satellite constellation (_build_constellation), the iterative
// weighted-least-squares fix (GPSReceiver._wls_solve with its 4x4 Gaussian
// elimination, _solve_4x4) and the ECEF-to-WGS-84 conversion
// (GPSReceiver._ecef_to_wgs84, Bowring's method). Operation for operation,
// so the Python can be run beside it as the oracle (test_nav_oracle.py).
//
// The pseudoranges come from the bus (PLANT_GPS). Nothing here draws noise
// or knows the true position; the Python receiver did both, being the
// simulation as well as the solver.

namespace fmofp::nav {

constexpr double kPi = 3.141592653589793;
constexpr double kDegToRad = kPi / 180.0;  // math.radians multiplies by this constant
constexpr double kRadToDeg = 180.0 / kPi;  // math.degrees likewise
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84B = 6356752.314245;
constexpr double kWgs84E2 = 1.0 - (kWgs84B / kWgs84A) * (kWgs84B / kWgs84A);
constexpr double kSpeedOfLight = 299792458.0;
constexpr double kMetresPerFoot = 0.3048;
constexpr std::size_t kSatellites = 12;
constexpr std::size_t kMinSatellites = 4;
constexpr int kDefaultMaxIterations = 8;
constexpr double kDefaultToleranceM = 0.01;
constexpr double kSingularPivot = 1e-14;

struct Ecef {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Geodetic {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_ft{0.0};
};

struct Satellite {
    std::uint8_t id{0};  // 1..12
    Ecef position{};     // metres
    double clock_bias_s{0.0};
};

using Constellation = std::array<Satellite, kSatellites>;

// Three orbital planes 120 degrees apart, inclined 55 degrees, four slots 90
// degrees apart, radius 26 560 km; clock bias (id x 1.3e-9) mod 1e-6 s.
Constellation build_constellation() noexcept;

struct Observation {
    std::uint8_t satellite{0};  // 1..12
    double pseudorange_m{0.0};
};

struct Fix {
    Ecef position{};
    double clock_bias_s{0.0};
    std::uint8_t iterations{0};
    std::uint8_t satellites{0};
    bool converged{false};  // the last step was shorter than the tolerance
};

// GPSReceiver._wls_solve. `guess` and `clock_bias_s` seed the iteration: the
// last fix, or an acquisition from bancroft() before there is one.
// kInvalidArgument with fewer than four usable observations or an unknown
// satellite, kMalformed when the normal equations are singular; `fix` is then
// untouched. As in the Python, the result after max_iterations is returned
// whether or not it converged; `fix.converged` says which.
[[nodiscard]] Status wls_solve(const Constellation& sats, const Observation* obs, std::size_t count,
                               const Ecef& guess, double clock_bias_s, Fix& fix,
                               int max_iterations = kDefaultMaxIterations,
                               double tolerance_m = kDefaultToleranceM) noexcept;

// Bancroft's closed-form solution of the same equations: the acquisition the
// Python never needed, because it started its first iteration from the
// simulation's true position. From anywhere else, the four satellites the
// home position sees (which are coplanar) make the iteration's normal
// equations singular. Of the two roots, the one nearer the Earth's surface is
// returned. Status as wls_solve; a non-converging quadratic is kMalformed.
[[nodiscard]] Status bancroft(const Constellation& sats, const Observation* obs, std::size_t count,
                              Fix& fix) noexcept;

using Matrix4 = std::array<std::array<double, 4>, 4>;
using Vector4 = std::array<double, 4>;

// _solve_4x4: Gaussian elimination with partial pivoting. kMalformed when a
// pivot is smaller than kSingularPivot.
[[nodiscard]] Status solve_4x4(const Matrix4& a, const Vector4& b, Vector4& x) noexcept;

// GPSReceiver._ecef_to_wgs84 (Bowring, 10 iterations) and the inverse the
// receiver used to place the simulated truth (update_true_position).
Geodetic ecef_to_wgs84(const Ecef& p) noexcept;
Ecef wgs84_to_ecef(const Geodetic& g) noexcept;

// GPSReceiver._is_visible's elevation angle, degrees. 90 for a receiver at
// the Earth's centre and -90 for a satellite at the receiver, as that code behaves.
double elevation_deg(const Ecef& receiver, const Ecef& satellite) noexcept;

}  // namespace fmofp::nav

#endif
