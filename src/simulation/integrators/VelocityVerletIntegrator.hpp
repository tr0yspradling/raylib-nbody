#pragma once

#include "../IIntegrator.hpp"
#include "../../components/Components.hpp"
#include "../../core/Config.hpp"
#include <algorithm>
#include <cmath>

namespace nbody::simulation::integrators {

class VelocityVerletIntegrator : public IIntegrator {
public:
    void integrate(const flecs::world& world, float deltaTime) override {
        const Config& cfg = *world.get<Config>();
        const float maxSpeed = cfg.max_speed;
        
        // Substep splitting for stability
        const float cap = std::max(1e-6f, cfg.max_substep);
        int nSteps = static_cast<int>(std::ceil(deltaTime / cap));
        nSteps = std::max(1, std::min(nSteps, std::max(1, cfg.max_substeps_per_frame)));
        const float dtSub = deltaTime / static_cast<float>(nSteps);

        for (int step = 0; step < nSteps; ++step) {
            // Update positions using current velocity and acceleration
            world.each([&](Position& p, const Velocity& v, const Acceleration& a, PrevAcceleration& a0, const Pinned& pin) {
                if (pin.value) return;
                
                const double half_dt2 = 0.5 * static_cast<double>(dtSub) * static_cast<double>(dtSub);
                p.value.x += v.value.x * dtSub + a.value.x * half_dt2;
                p.value.y += v.value.y * dtSub + a.value.y * half_dt2;
                a0.value = a.value;  // Store a_t for velocity update
            });

            // Compute new acceleration at t+dt
            // Note: This requires access to gravity computation
            // We'll need to refactor this when we extract gravity calculation

            // Update velocities using average of old and new accelerations
            world.each([&](Velocity& v, const Acceleration& a, const PrevAcceleration& a0, const Pinned& pin) {
                if (pin.value) return;
                
                const double ax = (a0.value.x + a.value.x) * 0.5;
                const double ay = (a0.value.y + a.value.y) * 0.5;
                v.value.x += ax * dtSub;
                v.value.y += ay * dtSub;
                
                // Apply velocity cap if configured
                if (maxSpeed > 0.0f) {
                    const double vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                    if (vlen > static_cast<double>(maxSpeed)) {
                        const double s = static_cast<double>(maxSpeed) / vlen;
                        v.value.x *= s;
                        v.value.y *= s;
                    }
                }
            });
        }
    }
    
    const char* getName() const override {
        return "Velocity Verlet";
    }
    
    int getId() const override {
        return 1;
    }
};

}  // namespace nbody::simulation::integrators