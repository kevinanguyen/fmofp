#include <fmofp/nav/gps_solver.hpp>

#include <algorithm>
#include <cmath>

namespace fmofp::nav {

namespace {

constexpr double kOrbitRadiusM = 26560000.0;
constexpr double kInclinationDeg = 55.0;

double clamp_unit(double v) noexcept { return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v); }

double distance(const Ecef& a, const Ecef& b) noexcept {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

const Satellite* find_satellite(const Constellation& sats, std::uint8_t id) noexcept {
    const auto it = std::find_if(sats.begin(), sats.end(), [id](const Satellite& s) { return s.id == id; });
    return it == sats.end() ? nullptr : &*it;
}

}  // namespace

Constellation build_constellation() noexcept {
    Constellation sats{};
    const double inclination = kInclinationDeg * kDegToRad;
    std::size_t index = 0;
    for (int plane = 0; plane < 3; ++plane) {
        const double raan = static_cast<double>(plane * 120) * kDegToRad;
        for (int slot = 0; slot < 4; ++slot) {
            const double u = static_cast<double>(slot * 90) * kDegToRad;
            // Position in the orbital plane, inclined about x, then rotated by RAAN about z.
            const double x_p = kOrbitRadiusM * std::cos(u);
            const double y_p = kOrbitRadiusM * std::sin(u);
            const double x_i = x_p;
            const double y_i = y_p * std::cos(inclination);
            const double z_i = y_p * std::sin(inclination);
            Satellite& s = sats[index];
            s.id = static_cast<std::uint8_t>(index + 1U);
            s.position.x = x_i * std::cos(raan) - y_i * std::sin(raan);
            s.position.y = x_i * std::sin(raan) + y_i * std::cos(raan);
            s.position.z = z_i;
            s.clock_bias_s = std::fmod(static_cast<double>(s.id) * 1.3e-9, 1e-6);
            ++index;
        }
    }
    return sats;
}

Status solve_4x4(const Matrix4& a, const Vector4& b, Vector4& x) noexcept {
    // The augmented matrix [A | b].
    std::array<std::array<double, 5>, 4> m{};
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            m[r][c] = a[r][c];
        }
        m[r][4] = b[r];
    }
    for (std::size_t col = 0; col < 4; ++col) {
        // Partial pivot: the first row with the largest magnitude in this column.
        std::size_t max_row = col;
        for (std::size_t r = col + 1; r < 4; ++r) {
            if (std::fabs(m[r][col]) > std::fabs(m[max_row][col])) {
                max_row = r;
            }
        }
        if (max_row != col) {
            const std::array<double, 5> tmp = m[col];
            m[col] = m[max_row];
            m[max_row] = tmp;
        }
        const double pivot = m[col][col];
        if (std::fabs(pivot) < kSingularPivot) {
            return Status::kMalformed;
        }
        for (std::size_t row = col + 1; row < 4; ++row) {
            const double factor = m[row][col] / pivot;
            for (std::size_t c = col; c < 5; ++c) {
                m[row][c] -= factor * m[col][c];
            }
        }
    }
    // Back substitution.
    for (std::size_t i = 4; i-- > 0;) {
        double v = m[i][4];
        for (std::size_t c = i + 1; c < 4; ++c) {
            v -= m[i][c] * x[c];
        }
        x[i] = v / m[i][i];
    }
    return Status::kOk;
}

Status wls_solve(const Constellation& sats, const Observation* obs, std::size_t count, const Ecef& guess,
                 double clock_bias_s, Fix& fix, int max_iterations, double tolerance_m) noexcept {
    if (obs == nullptr || count < kMinSatellites || count > kSatellites || max_iterations < 1) {
        return Status::kInvalidArgument;
    }
    std::array<const Satellite*, kSatellites> used{};
    for (std::size_t i = 0; i < count; ++i) {
        used[i] = find_satellite(sats, obs[i].satellite);
        if (used[i] == nullptr) {
            return Status::kInvalidArgument;
        }
    }

    double x = guess.x;
    double y = guess.y;
    double z = guess.z;
    double cb = clock_bias_s * kSpeedOfLight;  // receiver clock bias in metres
    std::uint8_t iterations = 0;
    bool converged = false;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        ++iterations;
        Matrix4 htwh{};
        Vector4 htwr{};
        std::size_t n = 0;
        for (std::size_t i = 0; i < count; ++i) {
            const Ecef& sp = used[i]->position;
            const double sx = sp.x;
            const double sy = sp.y;
            const double sz = sp.z;
            const double rng_est = std::sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y) + (sz - z) * (sz - z));
            if (rng_est == 0.0) {
                continue;
            }
            const double residual = obs[i].pseudorange_m - rng_est - cb + kSpeedOfLight * used[i]->clock_bias_s;
            // Partial derivatives: the unit vector and the clock column.
            const Vector4 h{-(sx - x) / rng_est, -(sy - y) / rng_est, -(sz - z) / rng_est, 1.0};
            // Elevation-dependent weight, sin^2(el) with a floor of 0.01.
            double r_mag = std::sqrt(x * x + y * y + z * z);
            r_mag = r_mag == 0.0 ? 1.0 : r_mag;
            const double nx = x / r_mag;
            const double ny = y / r_mag;
            const double nz = z / r_mag;
            double d_mag = std::sqrt((sx - x) * (sx - x) + (sy - y) * (sy - y) + (sz - z) * (sz - z));
            d_mag = d_mag == 0.0 ? 1.0 : d_mag;
            const double cos_el = ((sx - x) * nx + (sy - y) * ny + (sz - z) * nz) / d_mag;
            const double el = std::asin(clamp_unit(cos_el));
            const double sin_el = std::sin(el);
            const double w = sin_el * sin_el + 0.01;
            // Normal equations (H^T W H) dx = H^T W r, accumulated as the Python did.
            for (std::size_t row = 0; row < 4; ++row) {
                htwr[row] += w * h[row] * residual;
                for (std::size_t col = 0; col < 4; ++col) {
                    htwh[row][col] += w * h[row] * h[col];
                }
            }
            ++n;
        }
        if (n < kMinSatellites) {
            return Status::kInvalidArgument;
        }
        Vector4 dx{};
        const Status s = solve_4x4(htwh, htwr, dx);
        if (s != Status::kOk) {
            return s;
        }
        x += dx[0];
        y += dx[1];
        z += dx[2];
        cb += dx[3];
        if (std::sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]) < tolerance_m) {
            converged = true;
            break;
        }
    }

    fix.position = Ecef{x, y, z};
    fix.clock_bias_s = cb / kSpeedOfLight;
    fix.iterations = iterations;
    fix.satellites = static_cast<std::uint8_t>(count);
    fix.converged = converged;
    return Status::kOk;
}

Status bancroft(const Constellation& sats, const Observation* obs, std::size_t count, Fix& fix) noexcept {
    if (obs == nullptr || count < kMinSatellites || count > kSatellites) {
        return Status::kInvalidArgument;
    }
    // Rows a_i = (satellite position, pseudorange corrected for the satellite
    // clock), in megametres so the 4x4 pivots stay of order one, and
    // alpha_i = <a_i, a_i> / 2 under the Lorentz product (x, y, z, +; t, -).
    constexpr double kScale = 1.0e6;
    std::array<Vector4, kSatellites> a{};
    std::array<double, kSatellites> alpha{};
    for (std::size_t i = 0; i < count; ++i) {
        const Satellite* s = find_satellite(sats, obs[i].satellite);
        if (s == nullptr) {
            return Status::kInvalidArgument;
        }
        a[i] = {s->position.x / kScale, s->position.y / kScale, s->position.z / kScale,
                (obs[i].pseudorange_m + kSpeedOfLight * s->clock_bias_s) / kScale};
        alpha[i] = 0.5 * (a[i][0] * a[i][0] + a[i][1] * a[i][1] + a[i][2] * a[i][2] - a[i][3] * a[i][3]);
    }
    // u = B+ 1 and v = B+ alpha: B itself when it is square (far better
    // conditioned), the normal equations B^T B otherwise.
    Matrix4 m{};
    Vector4 rhs_ones{};
    Vector4 rhs_alpha{};
    if (count == 4) {
        for (std::size_t r = 0; r < 4; ++r) {
            m[r] = a[r];
            rhs_ones[r] = 1.0;
            rhs_alpha[r] = alpha[r];
        }
    } else {
        for (std::size_t i = 0; i < count; ++i) {
            for (std::size_t r = 0; r < 4; ++r) {
                rhs_ones[r] += a[i][r];
                rhs_alpha[r] += a[i][r] * alpha[i];
                for (std::size_t c = 0; c < 4; ++c) {
                    m[r][c] += a[i][r] * a[i][c];
                }
            }
        }
    }
    Vector4 u{};
    Vector4 v{};
    Status s = solve_4x4(m, rhs_ones, u);
    if (s != Status::kOk) {
        return s;
    }
    s = solve_4x4(m, rhs_alpha, v);
    if (s != Status::kOk) {
        return s;
    }
    const auto lorentz = [](const Vector4& p, const Vector4& q) noexcept {
        return p[0] * q[0] + p[1] * q[1] + p[2] * q[2] - p[3] * q[3];
    };
    // <u,u> L^2 + 2 (<u,v> - 1) L + <v,v> = 0, where L = <y,y> / 2 and y = M (v + L u).
    const double e = lorentz(u, u);
    const double f = lorentz(u, v) - 1.0;
    const double g = lorentz(v, v);
    std::array<double, 2> roots{};
    std::size_t root_count = 0;
    if (std::fabs(e) < 1e-12) {
        if (std::fabs(f) < 1e-12) {
            return Status::kMalformed;
        }
        roots[root_count++] = -g / (2.0 * f);
    } else {
        double disc = f * f - e * g;
        disc = disc < 0.0 ? 0.0 : disc;  // measurement noise can push an exact double root slightly negative
        const double sq = std::sqrt(disc);
        roots[root_count++] = (-f + sq) / e;
        roots[root_count++] = (-f - sq) / e;
    }
    // Two roots: the receiver, and a mirror image that may also lie near the
    // Earth. Prefer a root near the surface; among those, the one that fits
    // the pseudoranges better (with four satellites both fit exactly, and
    // only one is near the Earth).
    bool chosen = false;
    bool best_near = false;
    double best_residual = 0.0;
    for (std::size_t i = 0; i < root_count; ++i) {
        const Ecef r{(v[0] + roots[i] * u[0]) * kScale, (v[1] + roots[i] * u[1]) * kScale,
                     (v[2] + roots[i] * u[2]) * kScale};
        const double cb = -(v[3] + roots[i] * u[3]) * kScale;
        const double radius = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
        const bool near = radius > kWgs84A - 100.0e3 && radius < kWgs84A + 100.0e3;
        double residual = 0.0;
        for (std::size_t k = 0; k < count; ++k) {
            const double predicted = std::sqrt((a[k][0] * kScale - r.x) * (a[k][0] * kScale - r.x) +
                                               (a[k][1] * kScale - r.y) * (a[k][1] * kScale - r.y) +
                                               (a[k][2] * kScale - r.z) * (a[k][2] * kScale - r.z)) +
                                     cb;
            const double d = predicted - a[k][3] * kScale;
            residual += d * d;
        }
        const bool better = !chosen || (near && !best_near) || (near == best_near && residual < best_residual);
        if (better) {
            chosen = true;
            best_near = near;
            best_residual = residual;
            fix.position = r;
            fix.clock_bias_s = cb / kSpeedOfLight;
        }
    }
    fix.iterations = 0;
    fix.satellites = static_cast<std::uint8_t>(count);
    fix.converged = true;
    return Status::kOk;
}

Geodetic ecef_to_wgs84(const Ecef& p) noexcept {
    const double lon = std::atan2(p.y, p.x);
    const double rho = std::sqrt(p.x * p.x + p.y * p.y);
    double lat = std::atan2(p.z, rho * (1.0 - kWgs84E2));  // initial estimate
    for (int i = 0; i < 10; ++i) {
        const double sin_lat = std::sin(lat);
        const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
        lat = std::atan2(p.z + kWgs84E2 * n * sin_lat, rho);
    }
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double n_final = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    double alt_m = 0.0;
    if (std::fabs(cos_lat) > 1e-10) {
        alt_m = rho / cos_lat - n_final;
    } else {
        alt_m = std::fabs(p.z) / std::fabs(sin_lat) - n_final * (1.0 - kWgs84E2);
    }
    Geodetic g;
    g.latitude_deg = lat * kRadToDeg;
    g.longitude_deg = lon * kRadToDeg;
    g.altitude_ft = alt_m / kMetresPerFoot;
    return g;
}

Ecef wgs84_to_ecef(const Geodetic& g) noexcept {
    const double alt_m = g.altitude_ft * kMetresPerFoot;
    const double lat = g.latitude_deg * kDegToRad;
    const double lon = g.longitude_deg * kDegToRad;
    const double sin_lat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    Ecef p;
    p.x = (n + alt_m) * std::cos(lat) * std::cos(lon);
    p.y = (n + alt_m) * std::cos(lat) * std::sin(lon);
    p.z = (n * (1.0 - kWgs84E2) + alt_m) * std::sin(lat);
    return p;
}

double elevation_deg(const Ecef& receiver, const Ecef& satellite) noexcept {
    const double dx = satellite.x - receiver.x;
    const double dy = satellite.y - receiver.y;
    const double dz = satellite.z - receiver.z;
    const double r_mag = std::sqrt(receiver.x * receiver.x + receiver.y * receiver.y + receiver.z * receiver.z);
    if (r_mag == 0.0) {
        return 90.0;
    }
    const double nx = receiver.x / r_mag;
    const double ny = receiver.y / r_mag;
    const double nz = receiver.z / r_mag;
    const double d_mag = distance(receiver, satellite);
    if (d_mag == 0.0) {
        return -90.0;
    }
    const double cos_el = (dx * nx + dy * ny + dz * nz) / d_mag;
    return std::asin(clamp_unit(cos_el)) * kRadToDeg;
}

}  // namespace fmofp::nav
