#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../physics/SpatialPartition.hpp"

namespace nbody {

    class Physics {
    public:
        static void Register(const flecs::world& w) {
            // Gravity: once per frame before integration.
            w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
                if (const Config& cfg = *w.get<Config>(); cfg.paused) return;
                compute_gravity(w);
            });

            // Integration: once per frame using it.delta_time
            w.system<>().kind(flecs::OnUpdate).iter([&](const flecs::iter& it) {
                const Config& cfg = *w.get<Config>();
                if (cfg.paused) return;
                const float dt =
                    (cfg.useFixedDt ? cfg.fixedDt : (float)it.delta_time()) * std::max(0.0f, cfg.timeScale);
                integrate(w, dt);
            });

            // Trails update after integration
            w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
                if (const Config& cfg = *w.get<Config>(); cfg.paused) return;
                update_trails(w);
            });
        }

    private:
        static inline bool is_finite(const float v) { return std::isfinite(static_cast<double>(v)); }

        static void compute_gravity(const flecs::world& w) {
            const Config& cfg = *w.get<Config>();
            const double G = cfg.G;
            const double eps2 = static_cast<double>(cfg.softening) * static_cast<double>(cfg.softening);

            std::vector<raylib::Vector2> positions;
            std::vector<float> masses;
            std::vector<uint8_t> pins;
            std::vector<Acceleration*> accPtrs;
            positions.reserve(1000);
            masses.reserve(1000);
            pins.reserve(1000);
            accPtrs.reserve(1000);

            w.each([&](Position& p, Velocity& v, Mass& m, Pinned& pin, Acceleration& a) {
                if (is_finite(p.value.x) && is_finite(p.value.y) && is_finite(v.value.x) && is_finite(v.value.y) &&
                    m.value > 0.0f && is_finite(m.value)) {
                    positions.push_back(p.value);
                    masses.push_back(m.value);
                    pins.push_back(pin.value ? 1 : 0);
                    accPtrs.push_back(&a);
                }
            });

            const size_t n = positions.size();
            if (n == 0) return;

            std::vector acc(n, raylib::Vector2{0, 0});

            if (n > static_cast<size_t>(cfg.bhThreshold)) {
                std::vector<SpatialPartition::Body> bodies;
                bodies.reserve(n);
                for (size_t i = 0; i < n; ++i) bodies.push_back({positions[i], masses[i], static_cast<int>(i)});

                SpatialPartition tree;
                tree.Build(bodies);
                const double theta = static_cast<double>(cfg.bhTheta);

                for (size_t i = 0; i < n; ++i) {
                    if (pins[i]) continue;
                    tree.ComputeForce(bodies[i], theta, G, eps2, acc[i]);
                }
            } else {
                const raylib::Vector2* pos = positions.data();
                const float* mass = masses.data();
                const uint8_t* pin = pins.data();
                for (size_t i = 0; i < n; ++i) {
                    for (size_t j = i + 1; j < n; ++j) {
                        const double dx = static_cast<double>(pos[j].x) - static_cast<double>(pos[i].x);
                        const double dy = static_cast<double>(pos[j].y) - static_cast<double>(pos[i].y);
                        const double r2 = dx * dx + dy * dy + eps2;
                        const double invR = 1.0 / std::sqrt(r2);
                        const double invR3 = invR * invR * invR;

                        const double ax_i = G * static_cast<double>(mass[j]) * dx * invR3;
                        const double ay_i = G * static_cast<double>(mass[j]) * dy * invR3;
                        const double ax_j = -G * static_cast<double>(mass[i]) * dx * invR3;
                        const double ay_j = -G * static_cast<double>(mass[i]) * dy * invR3;

                        if (!pin[i]) {
                            acc[i].x += static_cast<float>(ax_i);
                            acc[i].y += static_cast<float>(ay_i);
                        }
                        if (!pin[j]) {
                            acc[j].x += static_cast<float>(ax_j);
                            acc[j].y += static_cast<float>(ay_j);
                        }
                    }
                }
            }

            for (size_t i = 0; i < n; ++i) accPtrs[i]->value = acc[i];
        }

        static void integrate(const flecs::world& w, const float dt) {
            const Config& cfg = *w.get<Config>();
            const float maxSpeed = cfg.maxSpeed;

            if (const int integrator = cfg.integrator; integrator == 0) {
                w.each([&](Position& p, Velocity& v, const Acceleration& a, const Pinned& pin) {
                    if (pin.value) return;
                    v.value += a.value * dt;
                    if (maxSpeed > 0.0f) {
                        if (const float vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                            vlen > maxSpeed)
                            v.value = v.value * (maxSpeed / vlen);
                    }
                    p.value += v.value * dt;
                });
            } else {
                w.each([&](Position& p, const Velocity& v, const Acceleration& a, PrevAcceleration& a0,
                           const Pinned& pin) {
                    if (pin.value) return;
                    const raylib::Vector2 dx = v.value * dt + a.value * (0.5f * dt * dt);
                    p.value += dx;
                    a0.value = a.value;
                });

                compute_gravity(w);

                w.each([&](Velocity& v, const Acceleration& a, const PrevAcceleration& a0, const Pinned& pin) {
                    if (pin.value) return;
                    const raylib::Vector2 avg = (a0.value + a.value) * 0.5f;
                    v.value += avg * dt;
                    if (maxSpeed > 0.0f) {
                        if (const float vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                            vlen > maxSpeed)
                            v.value = v.value * (maxSpeed / vlen);
                    }
                });
            }
        }

        static void update_trails(const flecs::world& w) {
            const Config& cfg = *w.get<Config>();
            if (!cfg.drawTrails) return;
            const int maxLen = std::max(0, cfg.trailMax);
            w.each([&](Trail& t, const Position& p) {
                t.points.push_back(p.value);
                if (static_cast<int>(t.points.size()) > maxLen) t.points.erase(t.points.begin());
            });
        }
    };

}  // namespace nbody
