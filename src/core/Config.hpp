// src/core/Config.hpp
#pragma once

#include <cstdint>
#include <raylib-cpp.hpp>

// Global/singleton simulation configuration stored in flecs as a singleton component.
struct Config {
    // Physics
    double G = 6.67430e-3;
    float softening = 4.0f;  // epsilon
    float maxSpeed = 0.0f;  // 0 = uncapped

    // Time & integrator
    bool paused = false;
    bool useFixedDt = false;
    float fixedDt = 1.0f / 120.0f;
    float timeScale = 1.0f;
    int integrator = 1;  // 0 = Semi-Implicit Euler, 1 = Velocity Verlet

    // Visuals
    bool drawTrails = true;
    bool drawVelocity = true;
    bool drawAcceleration = false;
    int trailMax = 200;

    // UI/runtime
    double lastStepMs = 0.0;
};

// (removed) Legacy Selection/CameraState: superseded by Interaction/Camera systems
