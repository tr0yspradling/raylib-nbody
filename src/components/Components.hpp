#pragma once

#include <raylib-cpp.hpp>
#include <vector>

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
};
