#pragma once

#include <algorithm>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"

namespace nbody {

// Wraps the simulation camera as a singleton component and provides helpers.
class Camera {
public:
    static void init(raylib::Camera2D& cam) {
        constexpr float kHalf = 0.5F;
        cam.zoom = 1.0F;
        cam.offset = {static_cast<float>(GetScreenWidth()) * kHalf, static_cast<float>(GetScreenHeight()) * kHalf};
        cam.target = {static_cast<float>(nbody::constants::seed_center_x),
                      static_cast<float>(nbody::constants::seed_center_y)};
    }

    // Reset the active camera to defaults and re-center on current COM
    static void reset_view(const flecs::world& world) {
        if (auto* cam = get(world)) {
            init(*cam);
            if (const auto* cfg = world.get<Config>())
                cam->zoom = std::clamp(static_cast<float>(cfg->meter_to_pixel), nbody::constants::min_zoom,
                                       nbody::constants::max_zoom);
            center_on_center_of_mass(world);
        }
    }

    static void zoom_at_mouse(raylib::Camera2D& cam, const float wheel) {
        if (wheel == 0.0F) return;
        const raylib::Vector2 mouse = GetMousePosition();
        const raylib::Vector2 worldBefore = GetScreenToWorld2D(mouse, cam);
        const float newZoom = std::clamp(cam.zoom * (1.0F + wheel * nbody::constants::zoom_wheel_scale),
                                         nbody::constants::min_zoom, nbody::constants::max_zoom);
        cam.zoom = newZoom;
        const raylib::Vector2 worldAfter = GetScreenToWorld2D(mouse, cam);
        cam.target = Vector2Add(cam.target, Vector2Subtract(worldBefore, worldAfter));
    }

    struct CameraComponent {
        raylib::Camera2D camera;
        CameraComponent() { init(camera); }
    };

    static void register_systems(const flecs::world& world) {
        world.set<CameraComponent>({});
        if (auto* cam = world.get_mut<CameraComponent>()) {
            if (const Config* cfg = world.get<Config>()) {
                cam->camera.zoom = std::clamp(static_cast<float>(cfg->meter_to_pixel), nbody::constants::min_zoom,
                                              nbody::constants::max_zoom);
            }
        }
        center_on_center_of_mass(world);
    }

    static raylib::Camera2D* get(const flecs::world& world) {
        if (auto* cam = world.get_mut<CameraComponent>()) return &cam->camera;
        return nullptr;
    }

    static void center_on_center_of_mass(const flecs::world& world) {
        auto* camComp = world.get_mut<CameraComponent>();
        if (!camComp) return;

        double Cx = 0, Cy = 0, M = 0;
        world.each([&](const Position& p, const Mass& m) {
            Cx += static_cast<double>(m.value) * p.value.x;
            Cy += static_cast<double>(m.value) * p.value.y;
            M += static_cast<double>(m.value);
        });
        if (M > 0.0) camComp->camera.target = {static_cast<float>(Cx / M), static_cast<float>(Cy / M)};
    }

    static void focus_on_entity(const flecs::world& world, flecs::entity entity) {
        auto* camComp = world.get_mut<CameraComponent>();
        if (!camComp) return;
        if (const auto* pos = entity.get<Position>()) camComp->camera.target = raylib::Vector2{static_cast<float>(pos->value.x), static_cast<float>(pos->value.y)};
    }
};

}  // namespace nbody
