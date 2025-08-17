#pragma once

#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"

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

            struct Row {
                flecs::entity e;
                Position* p;
                Velocity* v;
                Mass* m;
                Pinned* pin;
                Acceleration* a;
            };
            std::vector<Row> rows;
            rows.reserve(static_cast<size_t>(w.count(flecs::type<Position, Velocity, Mass, Pinned, Acceleration>())));
            w.each([&](const flecs::entity e, Position& p, Velocity& v, Mass& m, Pinned& pin, Acceleration& a) {
                rows.push_back(Row{e, &p, &v, &m, &pin, &a});
            });

            std::erase_if(rows, [](const Row& r) {
                return !(is_finite(r.p->value.x) && is_finite(r.p->value.y) && is_finite(r.v->value.x) &&
                         is_finite(r.v->value.y) && r.m->value > 0.0f && is_finite(r.m->value));
            });

            const size_t n = rows.size();
            if (n == 0) return;
            std::vector acc(n, raylib::Vector2{0, 0});

            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    const double dx = static_cast<double>(rows[j].p->value.x) - static_cast<double>(rows[i].p->value.x);
                    const double dy = static_cast<double>(rows[j].p->value.y) - static_cast<double>(rows[i].p->value.y);
                    const double r2 = dx * dx + dy * dy + eps2;
                    const double invR = 1.0 / std::sqrt(r2);
                    const double invR3 = invR * invR * invR;

                    const double ax_i = G * static_cast<double>(rows[j].m->value) * dx * invR3;
                    const double ay_i = G * static_cast<double>(rows[j].m->value) * dy * invR3;
                    const double ax_j = -G * static_cast<double>(rows[i].m->value) * dx * invR3;
                    const double ay_j = -G * static_cast<double>(rows[i].m->value) * dy * invR3;

                    if (!rows[i].pin->value) {
                        acc[i].x += static_cast<float>(ax_i);
                        acc[i].y += static_cast<float>(ay_i);
                    }
                    if (!rows[j].pin->value) {
                        acc[j].x += static_cast<float>(ax_j);
                        acc[j].y += static_cast<float>(ay_j);
                    }
                }
            }

            for (size_t i = 0; i < n; ++i) rows[i].e.get_mut<Acceleration>()->value = acc[i];
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
