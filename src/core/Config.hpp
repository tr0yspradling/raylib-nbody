// src/core/Config.hpp
#pragma once

#include <cstdint>
#include <raylib-cpp.hpp>

// Global/singleton simulation configuration stored in flecs as a singleton component.
// All physics values use SI units (meters, kilograms, seconds).
struct Config {
    // Physics
    double G = 6.67430e-11;  // gravitational constant (m^3 kg^-1 s^-2)
    double meterToPixel = 1e-6;  // display scale: meters -> pixels
    float softening = 4.0f;  // epsilon (m)
    float maxSpeed = 0.0f;  // 0 = uncapped
    int bhThreshold = 100;  // use Barnes-Hut when entity count exceeds this
    float bhTheta = 0.5f;  // opening angle criterion

    // Time & integrator
    bool paused = false;
    bool useFixedDt = false;
    float fixedDt = 1.0f / 120.0f;
    float timeScale = 1e6f;
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
