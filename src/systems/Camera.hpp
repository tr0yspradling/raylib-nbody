#pragma once

#include <flecs.h>
#include <raylib-cpp.hpp>
#include "../components/Components.hpp"
#include "../core/Camera.hpp"

namespace nbody {

// Wraps the simulation camera as a singleton component and provides helpers.
class Camera {
public:
    struct CameraComponent {
        raylib::Camera2D camera;
        CameraComponent() { nbody::CameraUtils::Init(camera); }
    };

    static void Register(const flecs::world& world) {
        world.set<CameraComponent>({});
        CenterOnCenterOfMass(world);
    }

    static raylib::Camera2D* Get(const flecs::world& world) {
        if (auto* cam = world.get_mut<CameraComponent>()) return &cam->camera;
        return nullptr;
    }

    static void CenterOnCenterOfMass(const flecs::world& world) {
        auto* camComp = world.get_mut<CameraComponent>();
        if (!camComp) return;

        double Cx = 0, Cy = 0, M = 0;
        world.each([&](const Position& p, const Mass& m) {
            Cx += static_cast<double>(m.value) * static_cast<double>(p.value.x);
            Cy += static_cast<double>(m.value) * static_cast<double>(p.value.y);
            M += static_cast<double>(m.value);
        });
        if (M > 0.0) camComp->camera.target = {static_cast<float>(Cx / M), static_cast<float>(Cy / M)};
    }

    static void FocusOnEntity(const flecs::world& world, flecs::entity entity) {
        auto* camComp = world.get_mut<CameraComponent>();
        if (!camComp) return;
        if (const auto* pos = entity.get<Position>()) camComp->camera.target = pos->value;
    }
};

}  // namespace nbody
