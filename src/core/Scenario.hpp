#pragma once

#include <string>
#include <vector>
#include <flecs.h>
#include <raylib-cpp.hpp>

#include "Config.hpp"
#include "Constants.hpp"
#include "../components/Components.hpp"

namespace nbody {

struct BodySnapshot {
    DVec2 pos{0.0, 0.0};
    DVec2 vel{0.0, 0.0};
    float mass = 0.0f;
    bool pinned = false;
    raylib::Color tint{WHITE};
};

struct Scenario {
    std::string name;
    std::string description;
    std::vector<std::string> tags;  // simple labels; UI uses comma-separated input
    std::vector<BodySnapshot> bodies;
    // Minimal config subset to replay scenario faithfully
    double g = nbody::constants::default_g;
    double meter_to_pixel = nbody::constants::default_meter_to_pixel;
    float softening = nbody::constants::default_softening;
    float max_speed = nbody::constants::default_max_speed;
    int bh_threshold = nbody::constants::default_bh_threshold;
    float bh_theta = nbody::constants::default_bh_theta;
    bool use_fixed_dt = false;
    float fixed_dt = nbody::constants::default_fixed_dt;
    float time_scale = nbody::constants::default_time_scale;
    int integrator = 1;
    float max_substep = nbody::constants::default_max_substep;
    int max_substeps_per_frame = nbody::constants::default_max_substeps;
    bool draw_trails = true;
    bool draw_velocity = true;
    bool draw_acceleration = false;
    int trail_max = nbody::constants::default_trail_max;
    float radius_scale = nbody::constants::default_radius_scale;
};

struct ScenarioStore {
    std::vector<Scenario> items;
    int selected = -1;
};

inline Scenario snapshot_from_world(const flecs::world& w, const std::string& name, const std::string& desc) {
    Scenario s{};
    s.name = name;
    s.description = desc;
    if (const auto* cfg = w.get<Config>()) {
        s.g = cfg->g;
        s.meter_to_pixel = cfg->meter_to_pixel;
        s.softening = cfg->softening;
        s.max_speed = cfg->max_speed;
        s.bh_threshold = cfg->bh_threshold;
        s.bh_theta = cfg->bh_theta;
        s.use_fixed_dt = cfg->use_fixed_dt;
        s.fixed_dt = cfg->fixed_dt;
        s.time_scale = cfg->time_scale;
        s.integrator = cfg->integrator;
        s.max_substep = cfg->max_substep;
        s.max_substeps_per_frame = cfg->max_substeps_per_frame;
        s.draw_trails = cfg->draw_trails;
        s.draw_velocity = cfg->draw_velocity;
        s.draw_acceleration = cfg->draw_acceleration;
        s.trail_max = cfg->trail_max;
        s.radius_scale = cfg->radius_scale;
    }
    w.each([&](const Position& p, const Velocity& v, const Mass& m, const Pinned& pin, const Tint& tint) {
        s.bodies.push_back(BodySnapshot{p.value, v.value, m.value, pin.value, tint.value});
    });
    return s;
}

inline void apply_scenario_bodies_only(const flecs::world& w, const Scenario& s) {
    // Clear all current bodies
    std::vector<flecs::entity> toDel;
    w.each([&](const flecs::entity e, const Position&) { toDel.push_back(e); });
    for (auto& e : toDel) e.destruct();

    // Rebuild bodies
    for (const auto& b : s.bodies) {
        w.entity()
            .set<Position>({b.pos})
            .set<Velocity>({b.vel})
            .set<Acceleration>({DVec2{0.0, 0.0}})
            .set<PrevAcceleration>({DVec2{0.0, 0.0}})
            .set<Mass>({std::max(0.0f, b.mass)})
            .set<Pinned>({b.pinned})
            .set<Tint>({b.tint})
            .set<Trail>({{}})
            .add<Selectable>()
            .set<Draggable>({true, nbody::constants::drag_vel_scale});
    }
}

inline void apply_scenario_to_world(const flecs::world& w, const Scenario& s) {
    // Apply bodies only first
    apply_scenario_bodies_only(w, s);

    // Apply config subset
    if (auto* cfg = w.get_mut<Config>()) {
        cfg->g = s.g;
        cfg->meter_to_pixel = s.meter_to_pixel;
        cfg->softening = s.softening;
        cfg->max_speed = s.max_speed;
        cfg->bh_threshold = s.bh_threshold;
        cfg->bh_theta = s.bh_theta;
        cfg->use_fixed_dt = s.use_fixed_dt;
        cfg->fixed_dt = s.fixed_dt;
        cfg->time_scale = s.time_scale;
        cfg->integrator = s.integrator;
        cfg->max_substep = s.max_substep;
        cfg->max_substeps_per_frame = s.max_substeps_per_frame;
        cfg->draw_trails = s.draw_trails;
        cfg->draw_velocity = s.draw_velocity;
        cfg->draw_acceleration = s.draw_acceleration;
        cfg->trail_max = s.trail_max;
        cfg->radius_scale = s.radius_scale;
        cfg->paused = false;
    }
}

}  // namespace nbody
