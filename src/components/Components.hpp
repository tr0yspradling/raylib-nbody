#pragma once

#include <raylib-cpp.hpp>
#include <vector>

#include "../core/Constants.hpp"
#include "../core/Vec2.hpp"

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
struct Pinned {
    bool value;
};
struct Tint {
    raylib::Color value;
};

// Trail history per entity
struct Trail {
    std::vector<DVec2> points;
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

// (removed) MouseInteraction: legacy input state no longer used
