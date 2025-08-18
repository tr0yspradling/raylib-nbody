#pragma once

#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>
#include <tuple>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"

namespace nbody {

    class Physics {
    public:
        struct Diagnostics {
            double kinetic = 0.0;
            double potential = 0.0;
            double energy = 0.0;
            raylib::Vector2 momentum{0, 0};
            raylib::Vector2 com{0, 0};
            double totalMass = 0.0;
        };

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

        static void zero_net_momentum(const flecs::world& w) {
            double Px = 0.0, Py = 0.0, M = 0.0;
            w.each([&](const Mass& m, const Velocity& v) {
                Px += static_cast<double>(m.value) * static_cast<double>(v.value.x);
                Py += static_cast<double>(m.value) * static_cast<double>(v.value.y);
                M += static_cast<double>(m.value);
            });
            if (M <= 0.0) return;
            const raylib::Vector2 v0 = {static_cast<float>(Px / M), static_cast<float>(Py / M)};
            w.each([&](const Pinned& pin, Velocity& v) {
                if (!pin.value) v.value -= v0;
            });
        }

        static void reset_scenario(const flecs::world& w) {
            std::vector<flecs::entity> to_del;
            w.each([&](const flecs::entity e, Position&) { to_del.push_back(e); });
            for (auto& e : to_del) e.destruct();
            auto mk = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, const float mass,
                          const raylib::Color col, const bool pinned) {
                w.entity()
                    .set<Position>({pos})
                    .set<Velocity>({vel})
                    .set<Acceleration>({raylib::Vector2{0, 0}})
                    .set<PrevAcceleration>({raylib::Vector2{0, 0}})
                    .set<Mass>({mass})
                    .set<Pinned>({pinned})
                    .set<Tint>({col})
                    .set<Trail>({{}})
                    .add<Selectable>()
                    .set<Draggable>({true, nbody::constants::dragVelScale});
            };
            mk({nbody::constants::seedCenterX, nbody::constants::seedCenterY}, {0.0f, 0.0f},
               nbody::constants::seedCentralMass, RED, false);
            mk({nbody::constants::seedCenterX + nbody::constants::seedOffsetX, nbody::constants::seedCenterY},
               {0.0f, nbody::constants::seedSpeed}, nbody::constants::seedSmallMass, BLUE, false);
            mk({nbody::constants::seedCenterX - nbody::constants::seedOffsetX, nbody::constants::seedCenterY},
               {0.0f, -nbody::constants::seedSpeed}, nbody::constants::seedSmallMass, GREEN, false);
        }

        static bool compute_diagnostics(const flecs::world& w, const double G, const double eps2, Diagnostics& out) {
            std::vector<std::tuple<raylib::Vector2, raylib::Vector2, float>> data;
            data.reserve(1024);
            w.each([&](const Position& p, const Velocity& v, const Mass& m) {
                data.emplace_back(p.value, v.value, m.value);
            });
            const size_t n = data.size();
            out = Diagnostics{};
            if (n == 0) return true;

            double KE = 0.0, M = 0.0, Px = 0.0, Py = 0.0, Cx = 0.0, Cy = 0.0;
            for (size_t i = 0; i < n; ++i) {
                auto [p, v, m] = data[i];
                KE += 0.5 * static_cast<double>(m) *
                    (static_cast<double>(v.x) * static_cast<double>(v.x) +
                     static_cast<double>(v.y) * static_cast<double>(v.y));
                Px += static_cast<double>(m) * static_cast<double>(v.x);
                Py += static_cast<double>(m) * static_cast<double>(v.y);
                Cx += static_cast<double>(m) * static_cast<double>(p.x);
                Cy += static_cast<double>(m) * static_cast<double>(p.y);
                M += static_cast<double>(m);
            }
            double PE = 0.0;
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    const double dx =
                        static_cast<double>(std::get<0>(data[j]).x) - static_cast<double>(std::get<0>(data[i]).x);
                    const double dy =
                        static_cast<double>(std::get<0>(data[j]).y) - static_cast<double>(std::get<0>(data[i]).y);
                    const double r2 = dx * dx + dy * dy + eps2;
                    const double r = std::sqrt(r2);
                    PE +=
                        -G * static_cast<double>(std::get<2>(data[i])) * static_cast<double>(std::get<2>(data[j])) / r;
                }
            }

            out.kinetic = KE;
            out.potential = PE;
            out.energy = KE + PE;
            out.momentum = raylib::Vector2{static_cast<float>(Px), static_cast<float>(Py)};
            out.totalMass = M;
            out.com = (M > 0.0) ? raylib::Vector2{static_cast<float>(Cx / M), static_cast<float>(Cy / M)}
                                : raylib::Vector2{0, 0};
            return true;
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
            rows.reserve(1000);
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
