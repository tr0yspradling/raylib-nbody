#pragma once

#include <algorithm>
#include <raylib-cpp.hpp>
#include <raymath.h>

namespace nbody {

class CameraUtils {
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
};

}  // namespace nbody
