#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"

static void DrawWorldGrid(const raylib::Camera2D& cam, const float spacing) {
    const raylib::Vector2 tl = GetScreenToWorld2D(::Vector2{0, 0}, cam);
    const raylib::Vector2 br =
        GetScreenToWorld2D(::Vector2{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())}, cam);

    const float startX = std::floor(tl.x / spacing) * spacing;
    const float endX = std::ceil(br.x / spacing) * spacing;
    const float startY = std::floor(tl.y / spacing) * spacing;
    const float endY = std::ceil(br.y / spacing) * spacing;

    constexpr Color gridC = {40, 40, 40, 255};
    constexpr Color axisC = {80, 80, 80, 255};

    // Use integer loop counters to avoid floating-point loop counter warnings
    const int stepsX = static_cast<int>(std::max(0.0f, std::floor((endX - startX) / spacing + 1e-6f)));
    for (int i = 0; i <= stepsX; ++i) {
        const float x = startX + static_cast<float>(i) * spacing;
        DrawLineV(::Vector2{x, startY}, ::Vector2{x, endY}, (std::abs(x) < 1e-4f) ? axisC : gridC);
    }
    const int stepsY = static_cast<int>(std::max(0.0f, std::floor((endY - startY) / spacing + 1e-6f)));
    for (int j = 0; j <= stepsY; ++j) {
        const float y = startY + static_cast<float>(j) * spacing;
        DrawLineV(::Vector2{startX, y}, ::Vector2{endX, y}, (std::abs(y) < 1e-4f) ? axisC : gridC);
    }
}

namespace ecs {

    void render_scene(const flecs::world& w, const Config& cfg, raylib::Camera2D& cam) {
        cam.BeginMode();

        DrawWorldGrid(cam, 50.0f);

        // Trails
        if (cfg.drawTrails) {
            w.each([&](const Trail& t, const Tint& tint) {
                for (size_t k = 1; k < t.points.size(); ++k) {
                    Color c = tint.value;
                    const double denom = std::max(1.0, static_cast<double>(t.points.size()));
                    c.a = static_cast<unsigned char>(
                        std::clamp(20 + static_cast<int>(230.0 * static_cast<double>(k) / denom), 20, 250));
                    DrawLineV(t.points[k - 1], t.points[k], c);
                }
            });
        }

        // Bodies
        w.each([&](const Position& p, const Velocity& v, const Acceleration& a, const Mass& m, const Tint& tint) {
            const float r = static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(m.value))));
            DrawCircleV(p.value, r, tint.value);
            if (cfg.drawVelocity) {
                const raylib::Vector2 tip = p.value + v.value * 10.0f;
                DrawLineEx(p.value, tip, 1.5f, WHITE);
            }
            if (cfg.drawAcceleration) {
                const raylib::Vector2 tip = p.value + a.value * 500.0f;
                DrawLineEx(p.value, tip, 1.0f, ORANGE);
            }
        });

        EndMode2D();
    }

}  // namespace ecs
