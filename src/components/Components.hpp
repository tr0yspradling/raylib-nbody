#pragma once

#include <algorithm>
#include <cmath>
#include <raylib-cpp.hpp>
#include <vector>

#include "../core/Constants.hpp"

// Basic physics/render components
struct Position {
    raylib::Vector2 value;
};
struct Velocity {
    raylib::Vector2 value;
};
struct Acceleration {
    raylib::Vector2 value;
};
struct PrevAcceleration {
    raylib::Vector2 value;
};
struct Mass {
    float value;
};
struct Radius {
    float value;
};
struct Pinned {
    bool value;
};
struct Tint {
    raylib::Color value;
};

// Trail history per entity
struct Trail {
    std::vector<raylib::Vector2> points;
};

// Selection and interaction components
struct Selectable {
    bool canSelect = true;
};

struct Selected {
    // Tag component to mark currently selected entity
};

struct Draggable {
    bool canDragVelocity = true;
    float dragScale = nbody::constants::dragVelScale;
};

inline float MassToRadius(float mass) {
    const double safeMass = std::max(1.0, static_cast<double>(mass));
    return std::max(nbody::constants::minBodyRadius, static_cast<float>(std::cbrt(safeMass)));
}

// (removed) MouseInteraction: legacy input state no longer used
