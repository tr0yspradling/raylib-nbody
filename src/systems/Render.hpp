#pragma once

#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"

namespace nbody {

    class Renderer {
    public:
        static void RenderScene(const flecs::world& w, const Config& cfg, raylib::Camera2D& cam) {
            cam.BeginMode();
            DrawWorldGrid(cam, nbody::constants::gridSpacing);

            if (cfg.drawTrails) {
                w.each([&](const Trail& t, const Tint& tint) {
                    for (size_t k = 1; k < t.points.size(); ++k) {
                        Color c = tint.value;
                        const double denom = std::max(1.0, static_cast<double>(t.points.size()));
                        c.a = static_cast<unsigned char>(std::clamp(
                            nbody::constants::trailAlphaMin +
                                static_cast<int>(nbody::constants::trailAlphaRange * static_cast<double>(k) / denom),
                            nbody::constants::trailAlphaMin, nbody::constants::trailAlphaMax));
                        DrawLineV(ToVector2(t.points[k - 1]), ToVector2(t.points[k]), c);
                    }
                });
            }

            w.each([&](const Position& p, const Velocity& v, const Acceleration& a, const Mass& m, const Tint& tint) {
                const float r = static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(m.value))));
                const raylib::Vector2 pos = ToVector2(p.value);
                DrawCircleV(pos, r, tint.value);
                if (cfg.drawVelocity) {
                    const raylib::Vector2 tip =
                        ToVector2(p.value + v.value * static_cast<double>(nbody::constants::velVectorScale));
                    DrawLineEx(pos, tip, nbody::constants::velLineWidth, WHITE);
                }
                if (cfg.drawAcceleration) {
                    const raylib::Vector2 tip =
                        ToVector2(p.value + a.value * static_cast<double>(nbody::constants::accVectorScale));
                    DrawLineEx(pos, tip, nbody::constants::accLineWidth, ORANGE);
                }
            });

            EndMode2D();
        }

    private:
        static void DrawWorldGrid(const raylib::Camera2D& cam, const float spacing) {
            const raylib::Vector2 tl = GetScreenToWorld2D(::Vector2{0, 0}, cam);
            const raylib::Vector2 br = GetScreenToWorld2D(
                ::Vector2{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())}, cam);

            const float startX = std::floor(tl.x / spacing) * spacing;
            const float endX = std::ceil(br.x / spacing) * spacing;
            const float startY = std::floor(tl.y / spacing) * spacing;
            const float endY = std::ceil(br.y / spacing) * spacing;

            const int stepsX = static_cast<int>(
                std::max(0.0f, std::floor((endX - startX) / spacing + nbody::constants::gridStepsEpsilon)));
            for (int i = 0; i <= stepsX; ++i) {
                const float x = startX + static_cast<float>(i) * spacing;
                DrawLineV(::Vector2{x, startY}, ::Vector2{x, endY},
                          (std::abs(x) < nbody::constants::gridAxisEpsilon) ? nbody::constants::axisColor
                                                                            : nbody::constants::gridColor);
            }
            const int stepsY = static_cast<int>(
                std::max(0.0f, std::floor((endY - startY) / spacing + nbody::constants::gridStepsEpsilon)));
            for (int j = 0; j <= stepsY; ++j) {
                const float y = startY + static_cast<float>(j) * spacing;
                DrawLineV(::Vector2{startX, y}, ::Vector2{endX, y},
                          (std::abs(y) < nbody::constants::gridAxisEpsilon) ? nbody::constants::axisColor
                                                                            : nbody::constants::gridColor);
            }
        }
    };

}  // namespace nbody
