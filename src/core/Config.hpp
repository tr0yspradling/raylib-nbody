// src/core/Config.hpp
#pragma once

#include <cstdint>
#include <raylib-cpp.hpp>

#include "Constants.hpp"

// Global/singleton simulation configuration stored in flecs as a singleton component.
// All physics values use SI units (meters, kilograms, seconds).
struct Config {
    // Physics
    double g = nbody::constants::default_g;  // gravitational constant (m^3 kg^-1 s^-2)
    double meter_to_pixel = nbody::constants::default_meter_to_pixel;  // display scale: meters -> pixels
    float softening = nbody::constants::default_softening;  // epsilon (m)
    float max_speed = nbody::constants::default_max_speed;  // 0 = uncapped
    int bh_threshold = nbody::constants::default_bh_threshold;  // use Barnes-Hut above this entity count
    float bh_theta = nbody::constants::default_bh_theta;  // opening angle criterion

    // Time & integrator
    bool paused = false;
    bool use_fixed_dt = false;
    float fixed_dt = nbody::constants::default_fixed_dt;
    float time_scale = nbody::constants::default_time_scale;
    int integrator = 1;  // 0 = Semi-Implicit Euler, 1 = Velocity Verlet

    // Stability controls
    float max_substep = nbody::constants::default_max_substep;          // seconds (cap per physics substep)
    int max_substeps_per_frame = nbody::constants::default_max_substeps;  // safety cap on CPU work

    // Visuals
    bool draw_trails = true;
    bool draw_velocity = true;
    bool draw_acceleration = false;
    int trail_max = nbody::constants::default_trail_max;
    float radius_scale = nbody::constants::default_radius_scale;  // visual size multiplier

    // UI/runtime
    double last_step_ms = 0.0;

    // Add/Edit defaults and shortcuts
    // Defaults for adding bodies from UI or shortcut
    float add_spawn_mass = static_cast<float>(nbody::constants::seed_small_mass);
    raylib::Vector2 add_spawn_velocity{0.0f, 0.0f};
    bool add_spawn_pinned = false;
    // Default drag sensitivity applied to newly added/duplicated bodies
    float add_drag_vel_scale = nbody::constants::drag_vel_scale;
    // Enable Shift+Click to add a body at mouse
    bool enable_shift_click_add = false;
};

// (removed) Legacy Selection/CameraState: superseded by Interaction/Camera systems
