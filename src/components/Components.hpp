#pragma once

#include <raylib-cpp.hpp>
#include <vector>

#include "../core/Constants.hpp"
#include "../core/Math.hpp"

// Basic physics/render components
struct Position {
    DVec2 value;
};
struct Velocity {
    DVec2 value;
};
struct Acceleration {
    DVec2 value;
};
struct PrevAcceleration {
    DVec2 value;
};
struct Mass {
    float value;
};
// Optional per-body material density (kg/m^3). Used to derive radius when
// an explicit Radius component is absent.
struct Density {
    double value = nbody::constants::body_density;
};

// Optional physical radius (meters). If absent, systems may derive radius
// from mass and density.
struct Radius {
    double value;
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
    bool can_drag_velocity = true;
    float drag_scale = nbody::constants::drag_vel_scale;
};

// (removed) MouseInteraction: legacy input state no longer used
