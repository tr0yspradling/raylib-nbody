#pragma once

#include <raylib.h>

namespace nbody::constants {
inline constexpr int window_width = 1280;
inline constexpr int window_height = 720;
inline constexpr int target_fps = 120;

inline constexpr ::Color background{10, 10, 14, 255};

inline constexpr float pick_radius_px = 24.0F;
inline constexpr float select_threshold_sq = 9.0F;
// Right-drag sensitivity: fraction of the drag line applied as movement per physics step.
// For example, 0.05 means a 100 px drag results in ~5 px of motion per step (at current time_scale).
inline constexpr float drag_vel_scale = 0.05F;

inline constexpr float drag_line_width = 2.0F;
inline constexpr float drag_circle_radius = 3.0F;
inline constexpr float ring_extra_radius = 4.0F;
inline constexpr float ring_thickness = 3.5F;
inline constexpr float ring_inner_offset = 2.0F;
inline constexpr float ring_start_angle = 0.0F;
inline constexpr float ring_end_angle = 360.0F;
inline constexpr int ring_segments = 32;
inline constexpr float vel_line_width = 1.5F;
inline constexpr float acc_line_width = 1.0F;
inline constexpr float vel_vector_scale = 10.0F;
inline constexpr float acc_vector_scale = 500.0F;
inline constexpr float min_body_radius = 6.0F;
inline constexpr float default_radius_scale = 1.0F;  // visual size multiplier for body radii
inline constexpr float selected_circle_alpha = 0.5F;

inline constexpr double body_density = 5510.0;  // kg/m^3, approx. Earth average
inline constexpr float radius_scale_min = 0.1F;
inline constexpr float radius_scale_max = 100.0F;
inline constexpr float grid_spacing = 1.0e7F;
inline constexpr float grid_axis_epsilon = 1e-4F;
inline constexpr float grid_steps_epsilon = 1e-6F;
inline constexpr ::Color grid_color{40, 40, 40, 255};
inline constexpr ::Color axis_color{80, 80, 80, 255};

inline constexpr int trail_alpha_min = 20;
inline constexpr int trail_alpha_max = 250;
inline constexpr float trail_alpha_range = 230.0F;

inline constexpr double seed_small_mass = 7.342e22;  // kg (Moon mass)
inline constexpr double seed_central_mass = 5.972e24;  // kg (Earth mass)
inline constexpr double seed_center_x = 0.0;
inline constexpr double seed_center_y = 0.0;
inline constexpr double seed_offset_x = 3.844e8;  // m (Earth-Moon distance)

inline constexpr float zoom_wheel_scale = 0.1F;
// Camera zoom bounds aligned with meter-to-pixel display scale (~1e-6)
inline constexpr float min_zoom = 1e-9F;
inline constexpr float max_zoom = 1e-3F;

inline constexpr float fixed_dt_min = 1e-4F;
inline constexpr float fixed_dt_max = 0.05F;
inline constexpr float time_scale_min = 1e-6F;
inline constexpr float time_scale_max = 1e8F;
inline constexpr float g_min = 0.0F;
inline constexpr float g_max = 1e-9F;
inline constexpr float softening_min = 0.0F;
inline constexpr float softening_max = 1e9F;
inline constexpr float velocity_cap_min = 0.0F;
inline constexpr float velocity_cap_max = 1e6F;
inline constexpr int trail_length_max = 2000;

inline constexpr float spawn_mass_min = 1e20F;
inline constexpr float spawn_mass_max = 1e28F;
inline constexpr float spawn_vel_min = -1e4F;
inline constexpr float spawn_vel_max = 1e4F;
inline constexpr float drag_vel_scale_min = 0.001F;
inline constexpr float drag_vel_scale_max = 1.0F;
inline constexpr float selected_mass_min = 1e20F;
inline constexpr float selected_mass_max = 1e30F;
inline constexpr float selected_vel_min = -1e5F;
inline constexpr float selected_vel_max = 1e5F;
inline constexpr float duplicate_offset_x = 20.0F;

inline constexpr int random_color_min = 64;
inline constexpr int random_color_max = 255;

// Default configuration values (used to initialize Config)
inline constexpr double default_g = 6.67430e-11;  // m^3 kg^-1 s^-2
inline constexpr double default_meter_to_pixel = 1e-6;  // meters -> pixels
inline constexpr float default_softening = 4.0F;  // meters
inline constexpr float default_max_speed = 0.0F;  // 0 = uncapped
inline constexpr int default_bh_threshold = 100;  // entities
inline constexpr float default_bh_theta = 0.5F;  // opening angle
inline constexpr float default_fixed_dt = 1.0F / target_fps;  // seconds
inline constexpr float default_time_scale = 1e6F;  // simulation speed
inline constexpr int default_trail_max = 200;  // points
inline constexpr unsigned char alpha_opaque = 255;  // fully opaque

// Substepping defaults: split large effective dt into smaller steps for stability
// Make substepping opt-in by default: a huge cap yields 1 substep for typical dt
inline constexpr float default_max_substep = 1.0e9F;         // seconds
inline constexpr int default_max_substeps = 200;             // at most 200 substeps per frame
}  // namespace nbody::constants
