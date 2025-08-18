#pragma once

#include <raylib-cpp.hpp>
#include <utility>
#include <vector>

#include "../core/Constants.hpp"
#include "../core/TrailPool.hpp"

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
struct Pinned {
    bool value;
};
struct Tint {
    raylib::Color value;
};

// Trail history per entity
struct Trail {
    std::vector<raylib::Vector2> points;
    ~Trail() {
        if (points.capacity() > 0) nbody::TrailPool::Release(std::move(points));
    }
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
