#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace mu {
constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }
template <class T> inline T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double wrapDeg(double a) {           // -> (-180, 180]
    while (a <= -180.0) a += 360.0;
    while (a > 180.0) a -= 360.0;
    return a;
}
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
inline int roundInt(double v) { return (int)std::lround(v); }
inline double sign(double v) { return v < 0 ? -1.0 : 1.0; }

// First-order low-pass / exponential moving average
struct LowPass {
    double value = 0; bool init = false; double alpha = 0.5;
    double push(double v) { if (!init) { value = v; init = true; } else value += alpha * (v - value); return value; }
    void reset() { init = false; }
};

// Rate limiter for command channels (units per second)
struct SlewLimiter {
    double value = 0; double rate = 1e9;
    double push(double target, double dt) {
        double maxStep = rate * std::max(dt, 1e-3);
        value += clamp(target - value, -maxStep, maxStep);
        return value;
    }
    void reset(double v = 0) { value = v; }
};
}
