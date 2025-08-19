#pragma once

#include <cmath>
#include <raylib-cpp.hpp>

struct DVec2 {
    double x{0.0};
    double y{0.0};
};

inline DVec2 dvec2(double x, double y) { return DVec2{x, y}; }
inline DVec2 dvec2(const raylib::Vector2& v) { return DVec2{static_cast<double>(v.x), static_cast<double>(v.y)}; }
inline raylib::Vector2 fvec2(const DVec2& v) { return raylib::Vector2{static_cast<float>(v.x), static_cast<float>(v.y)}; }

inline DVec2 operator+(const DVec2& a, const DVec2& b) { return {a.x + b.x, a.y + b.y}; }
inline DVec2 operator-(const DVec2& a, const DVec2& b) { return {a.x - b.x, a.y - b.y}; }
inline DVec2 operator*(const DVec2& a, double s) { return {a.x * s, a.y * s}; }
inline DVec2 operator*(double s, const DVec2& a) { return {a.x * s, a.y * s}; }
inline DVec2& operator+=(DVec2& a, const DVec2& b) {
    a.x += b.x;
    a.y += b.y;
    return a;
}
inline DVec2& operator-=(DVec2& a, const DVec2& b) {
    a.x -= b.x;
    a.y -= b.y;
    return a;
}
inline DVec2& operator*=(DVec2& a, double s) {
    a.x *= s;
    a.y *= s;
    return a;
}

inline double dot(const DVec2& a, const DVec2& b) { return a.x * b.x + a.y * b.y; }
inline double length2(const DVec2& a) { return dot(a, a); }
inline double length(const DVec2& a) { return std::sqrt(length2(a)); }
inline DVec2 clamp_length(const DVec2& v, double maxLen) {
    const double l = length(v);
    if (maxLen > 0.0 && l > maxLen && l > 0.0) {
        const double s = maxLen / l;
        return v * s;
    }
    return v;
}

