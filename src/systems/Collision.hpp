#pragma once

#include <cmath>
#include <flecs.h>
#include <raymath.h>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Config.hpp"

namespace nbody {

class Collision {
public:
    static void Process(const flecs::world& world) {
        const Config& cfg = *world.get<Config>();
        std::vector<flecs::entity> entities;
        std::vector<Position*> positions;
        std::vector<Velocity*> velocities;
        std::vector<Mass*> masses;
        std::vector<Radius*> radii;
        std::vector<Pinned*> pins;

        world.each([&](flecs::entity e, Position& p, Velocity& v, Mass& m, Radius& r, Pinned& pin) {
            entities.push_back(e);
            positions.push_back(&p);
            velocities.push_back(&v);
            masses.push_back(&m);
            radii.push_back(&r);
            pins.push_back(&pin);
        });

        const size_t n = entities.size();
        if (n < 2) return;

        std::vector<bool> removed(n, false);

        for (size_t i = 0; i < n; ++i) {
            if (removed[i]) continue;
            for (size_t j = i + 1; j < n; ++j) {
                if (removed[j]) continue;
                const raylib::Vector2 delta = positions[j]->value - positions[i]->value;
                const float dist2 = delta.x * delta.x + delta.y * delta.y;
                const float rsum = radii[i]->value + radii[j]->value;
                if (dist2 > rsum * rsum) continue;

                const float dist = std::sqrt(dist2);
                raylib::Vector2 nvec = (dist > 0.0f) ? delta * (1.0f / dist) : raylib::Vector2{1.0f, 0.0f};

                if (cfg.elasticCollisions) {
                    const raylib::Vector2 relVel = velocities[i]->value - velocities[j]->value;
                    const float rel = Vector2DotProduct(relVel, nvec);
                    if (rel > 0.0f) continue;
                    const float p = (2.0f * rel) / (masses[i]->value + masses[j]->value);
                    if (!pins[i]->value) velocities[i]->value -= nvec * (p * masses[j]->value);
                    if (!pins[j]->value) velocities[j]->value += nvec * (p * masses[i]->value);
                } else {
                    const float totalMass = masses[i]->value + masses[j]->value;
                    if (totalMass <= 0.0f) continue;
                    const raylib::Vector2 newVel =
                        (velocities[i]->value * masses[i]->value + velocities[j]->value * masses[j]->value) / totalMass;
                    const raylib::Vector2 newPos =
                        (positions[i]->value * masses[i]->value + positions[j]->value * masses[j]->value) / totalMass;
                    positions[i]->value = newPos;
                    velocities[i]->value = newVel;
                    masses[i]->value = totalMass;
                    radii[i]->value = MassToRadius(totalMass);
                    pins[i]->value = pins[i]->value || pins[j]->value;
                    removed[j] = true;
                }
            }
        }

        for (size_t i = 0; i < n; ++i) {
            if (removed[i]) entities[i].destruct();
        }
    }
};

}  // namespace nbody
