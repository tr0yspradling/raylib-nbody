#pragma once

#include <algorithm>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"

namespace nbody {

    // Wraps the simulation camera as a singleton component and provides helpers.
    class Camera {
    public:
        static inline void Init(raylib::Camera2D& cam) {
            constexpr float kHalf = 0.5F;
            cam.zoom = 1.0F;
            cam.offset = {static_cast<float>(GetScreenWidth()) * kHalf, static_cast<float>(GetScreenHeight()) * kHalf};
            cam.target = {640.0F, 360.0F};
        }

        static inline void ZoomAtMouse(raylib::Camera2D& cam, const float wheel) {
            if (wheel == 0.0F) return;
            const raylib::Vector2 mouse = GetMousePosition();
            const raylib::Vector2 worldBefore = GetScreenToWorld2D(mouse, cam);
            constexpr float kWheelScale = 0.1F;
            constexpr float kMinZoom = 0.05F;
            constexpr float kMaxZoom = 10.0F;
            const float newZoom = std::clamp(cam.zoom * (1.0F + wheel * kWheelScale), kMinZoom, kMaxZoom);
            cam.zoom = newZoom;
            const raylib::Vector2 worldAfter = GetScreenToWorld2D(mouse, cam);
            cam.target = Vector2Add(cam.target, Vector2Subtract(worldBefore, worldAfter));
        }

        struct CameraComponent {
            raylib::Camera2D camera;
            CameraComponent() { Init(camera); }
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
