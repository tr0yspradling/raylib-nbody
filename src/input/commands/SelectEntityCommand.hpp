#pragma once

#include "../ICommand.hpp"
#include "../../core/Math.hpp"
#include "../../components/Components.hpp"
#include "../../core/Constants.hpp"
#include "../../core/Config.hpp"
#include "../../systems/Camera.hpp"
#include <cmath>
#include <numbers>
#include <limits>

namespace nbody::input::commands {

class SelectEntityCommand : public ICommand {
public:
    explicit SelectEntityCommand(DVec2 worldPosition) : worldPosition_(worldPosition) {}

    void execute(const flecs::world& world) override {
        flecs::entity entityToSelect = findEntityAtPosition(world, worldPosition_);
        
        // Clear previous selection
        world.each([&world](flecs::entity e, const Selected&) {
            // Get mutable entity to remove component
            flecs::entity mutableEntity = world.entity(e.id());
            mutableEntity.remove<Selected>();
        });
        
        // Set new selection
        if (entityToSelect.is_alive()) {
            entityToSelect.add<Selected>();
        }
    }

    const char* getName() const override {
        return "SelectEntity";
    }

private:
    flecs::entity findEntityAtPosition(const flecs::world& world, const DVec2& worldPos) {
        flecs::entity best = flecs::entity::null();
        float bestDist2 = std::numeric_limits<float>::max();
        
        const auto* cam = nbody::Camera::get(world);
        const float zoom = cam ? cam->zoom : 1.0f;
        const float pickRadius = nbody::constants::pick_radius_px / zoom;
        
        world.each([&](const flecs::entity ent, const Position& pos, const Mass& mass, const Selectable& selectable) {
            if (!selectable.canSelect) return;
            
            const DVec2 delta = worldPos - pos.value;
            const double dist2d = delta.x * delta.x + delta.y * delta.y;
            const float dist2 = static_cast<float>(dist2d);
            
            // Calculate entity radius for picking
            double rMeters = 0.0;
            if (const auto* rad = ent.get<Radius>()) {
                rMeters = rad->value;
            } else {
                const double safeMass = std::max(1.0, static_cast<double>(mass.value));
                rMeters = std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::body_density));
            }
            
            const Config* cfg = world.get<Config>();
            const float minRadiusWorld = nbody::constants::min_body_radius / zoom;
            const float bodyRadius = std::max(minRadiusWorld,
                                            (cfg ? cfg->radius_scale : nbody::constants::default_radius_scale) *
                                                static_cast<float>(rMeters));
            
            const float totalPickRadius = pickRadius + bodyRadius;
            const float totalPickRadius2 = totalPickRadius * totalPickRadius;
            
            if (dist2 <= totalPickRadius2 && dist2 < bestDist2) {
                best = ent;
                bestDist2 = dist2;
            }
        });
        
        return best;
    }

    DVec2 worldPosition_;
};

}  // namespace nbody::input::commands