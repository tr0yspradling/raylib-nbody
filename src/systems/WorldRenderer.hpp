#pragma once

#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <numbers>
#include <raylib-cpp.hpp>
#include <raymath.h>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"

namespace nbody::systems {

class WorldRenderer {
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
                    DrawLineV(t.points[k - 1], t.points[k], c);
                }
            });
        }

        std::vector<flecs::entity> entities;
        w.each([&](const flecs::entity e, const Position&, const Velocity&, const Acceleration&, const Mass&,
                   const Tint&) { entities.push_back(e); });
        std::sort(entities.begin(), entities.end(), [](flecs::entity a, flecs::entity b) {
            const auto* ma = a.get<Mass>();
            const auto* mb = b.get<Mass>();
            const float av = ma ? ma->value : 0.0f;
            const float bv = mb ? mb->value : 0.0f;
            return av > bv;
        });
        for (auto e : entities) {
            const auto* p = e.get<Position>();
            const auto* v = e.get<Velocity>();
            const auto* a = e.get<Acceleration>();
            const auto* m = e.get<Mass>();
            const auto* tint = e.get<Tint>();
            if (!p || !v || !a || !m || !tint) continue;
            const double safeMass = std::max(1.0, static_cast<double>(m->value));
            const double rMeters =
                std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::bodyDensity));
            const float minRadiusWorld = nbody::constants::minBodyRadius / cam.zoom;
            const float r = std::max(minRadiusWorld, static_cast<float>(rMeters));
            DrawCircleV(p->value, r, tint->value);
            if (cfg.drawVelocity) {
                float velScale = nbody::constants::velVectorScale / cam.zoom;
                if (const auto* drag = e.get<Draggable>()) velScale = 1.0f / drag->dragScale;
                const raylib::Vector2 tip = p->value + v->value * velScale;
                DrawLineEx(p->value, tip, nbody::constants::velLineWidth / cam.zoom, WHITE);
            }
            if (cfg.drawAcceleration) {
                const float accScale = nbody::constants::accVectorScale / cam.zoom;
                const raylib::Vector2 tip = p->value + a->value * accScale;
                DrawLineEx(p->value, tip, nbody::constants::accLineWidth / cam.zoom, ORANGE);
            }
        }

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

}  // namespace nbody::systems
