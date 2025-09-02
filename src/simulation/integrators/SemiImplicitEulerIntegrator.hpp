#pragma once

#include "../IIntegrator.hpp"
#include "../../components/Components.hpp"
#include "../../core/Config.hpp"
#include <algorithm>
#include <cmath>

namespace nbody::simulation::integrators {

class SemiImplicitEulerIntegrator : public IIntegrator {
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
            world.each([&](Position& p, Velocity& v, const Acceleration& a, const Pinned& pin) {
                if (pin.value) return;
                
                // Update velocity first
                v.value.x += a.value.x * dtSub;
                v.value.y += a.value.y * dtSub;
                
                // Apply velocity cap if configured
                if (maxSpeed > 0.0f) {
                    const double vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                    if (vlen > static_cast<double>(maxSpeed)) {
                        const double s = static_cast<double>(maxSpeed) / vlen;
                        v.value.x *= s;
                        v.value.y *= s;
                    }
                }
                
                // Update position with new velocity
                p.value.x += v.value.x * dtSub;
                p.value.y += v.value.y * dtSub;
            });
            
            // Recompute acceleration for next substep if needed
            if (step + 1 < nSteps) {
                // Note: This requires access to gravity computation
                // We'll need to refactor this when we extract gravity calculation
            }
        }
    }
    
    const char* getName() const override {
        return "Semi-Implicit Euler";
    }
    
    int getId() const override {
        return 0;
    }
};

}  // namespace nbody::simulation::integrators