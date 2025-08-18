#pragma once

#include <cmath>
#include <raylib-cpp.hpp>

struct DVec2 {
    double x{};
    double y{};
    DVec2 operator+(const DVec2& o) const { return {x + o.x, y + o.y}; }
    DVec2 operator-(const DVec2& o) const { return {x - o.x, y - o.y}; }
    DVec2 operator*(double s) const { return {x * s, y * s}; }
    DVec2& operator+=(const DVec2& o) {
        x += o.x;
        y += o.y;
        return *this;
    }
    DVec2& operator-=(const DVec2& o) {
        x -= o.x;
        y -= o.y;
        return *this;
    }
    DVec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }
};

inline DVec2 operator*(double s, const DVec2& v) { return v * s; }
inline double Length(const DVec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }
inline raylib::Vector2 ToVector2(const DVec2& v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }
inline DVec2 FromVector2(const raylib::Vector2& v) { return {static_cast<double>(v.x), static_cast<double>(v.y)}; }
