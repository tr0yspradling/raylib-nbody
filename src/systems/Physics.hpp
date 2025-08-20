#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>
#include <tuple>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "../physics/SpatialPartition.hpp"
#include "Collision.hpp"

namespace nbody {

class Physics {
public:
    struct Diagnostics {
        double kinetic = 0.0;
        double potential = 0.0;
        double energy = 0.0;
        DVec2 momentum{0.0, 0.0};
        DVec2 com{0.0, 0.0};
        double totalMass = 0.0;
        bool ok = true;
    };

    static void register_systems(const flecs::world& w) {
        // Collisions: resolve overlaps before computing forces.
        w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            auto* cfg = w.get<Config>();
            if (!cfg || cfg->paused) return;
            nbody::systems::Collision::resolve(w);
            Diagnostics d{};
            d.ok = compute_diagnostics(w, cfg->g,
                                       static_cast<double>(cfg->softening) * static_cast<double>(cfg->softening), d);
            w.set<Diagnostics>(d);
        });

        // Gravity: once per frame before integration.
        w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            if (const Config& cfg = *w.get<Config>(); cfg.paused) return;
            compute_gravity(w);
        });

        // Integration: once per frame using it.delta_time
        w.system<>().kind(flecs::OnUpdate).iter([&](const flecs::iter& it) {
            const Config& cfg = *w.get<Config>();
            if (cfg.paused) return;
            const float baseDt = cfg.use_fixed_dt ? cfg.fixed_dt : (float)it.delta_time();
            const float dtEff = baseDt * std::max(0.0f, cfg.time_scale);
            integrate(w, dtEff);
        });

        // Trails update after integration
        w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            if (const Config& cfg = *w.get<Config>(); cfg.paused) return;
            update_trails(w);
        });
    }

    static void zero_net_momentum(const flecs::world& w) {
        double Px = 0.0, Py = 0.0, M = 0.0;
        w.each([&](const Mass& m, Velocity& v, const Pinned& pin) {
            if (pin.value) {
                v.value = DVec2{0.0, 0.0};
                return;
            }
            Px += static_cast<double>(m.value) * v.value.x;
            Py += static_cast<double>(m.value) * v.value.y;
            M += static_cast<double>(m.value);
        });
        if (M <= 0.0) return;
        const DVec2 v0 = {Px / M, Py / M};
        w.each([&](const Pinned& pin, Velocity& v) {
            if (!pin.value) {
                v.value.x -= v0.x;
                v.value.y -= v0.y;
            }
        });
    }

    static void reset_scenario(const flecs::world& w) {
        const Config& cfg = *w.get<Config>();
        std::vector<flecs::entity> toDel;
        w.each([&](const flecs::entity e, Position&) { toDel.push_back(e); });
        for (auto& e : toDel) e.destruct();
        auto mk = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, const float mass, const raylib::Color col,
                      const bool pinned) {
            w.entity()
                .set<Position>({dvec2(pos)})
                .set<Velocity>({dvec2(vel)})
                .set<Acceleration>({DVec2{0.0, 0.0}})
                .set<PrevAcceleration>({DVec2{0.0, 0.0}})
                .set<Mass>({mass})
                .set<Pinned>({pinned})
                .set<Tint>({col})
                .set<Trail>({{}})
                .add<Selectable>()
                .set<Draggable>({true, constants::drag_vel_scale});
        };
        mk({static_cast<float>(constants::seed_center_x), static_cast<float>(constants::seed_center_y)}, {0.0f, 0.0f},
           static_cast<float>(constants::seed_central_mass), RED, false);
        const double radius = constants::seed_offset_x;
        const float v = static_cast<float>(std::sqrt(cfg.g * constants::seed_central_mass / radius));
        mk({static_cast<float>(constants::seed_center_x + radius), static_cast<float>(constants::seed_center_y)},
           {0.0f, v}, static_cast<float>(constants::seed_small_mass), BLUE, false);
        mk({static_cast<float>(constants::seed_center_x - radius), static_cast<float>(constants::seed_center_y)},
           {0.0f, -v}, static_cast<float>(constants::seed_small_mass), GREEN, false);
        zero_net_momentum(w);
    }

    static bool compute_diagnostics(const flecs::world& w, const double G, const double eps2, Diagnostics& out) {
        std::vector<std::tuple<DVec2, DVec2, float>> data;
        data.reserve(1024);
        w.each(
            [&](const Position& p, const Velocity& v, const Mass& m) { data.emplace_back(p.value, v.value, m.value); });
        const size_t n = data.size();
        out = Diagnostics{};
        if (n == 0) {
            out.ok = true;
            return true;
        }

        auto* cfg = w.get_mut<Config>();

        double KE = 0.0, M = 0.0, Px = 0.0, Py = 0.0, Cx = 0.0, Cy = 0.0;
        for (size_t i = 0; i < n; ++i) {
            auto [p, v, m] = data[i];
            KE += 0.5 * static_cast<double>(m) * (v.x * v.x + v.y * v.y);
            Px += static_cast<double>(m) * v.x;
            Py += static_cast<double>(m) * v.y;
            Cx += static_cast<double>(m) * p.x;
            Cy += static_cast<double>(m) * p.y;
            M += static_cast<double>(m);
            if (!(std::isfinite(KE) && std::isfinite(Px) && std::isfinite(Py) && std::isfinite(Cx) &&
                  std::isfinite(Cy) && std::isfinite(M))) {
                if (cfg) cfg->paused = true;
                out.ok = false;
                return false;
            }
        }
        double PE = 0.0;
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const double dx = std::get<0>(data[j]).x - std::get<0>(data[i]).x;
                const double dy = std::get<0>(data[j]).y - std::get<0>(data[i]).y;
                const double r2 = dx * dx + dy * dy + eps2;
                const double r = std::sqrt(r2);
                PE += -G * static_cast<double>(std::get<2>(data[i])) * static_cast<double>(std::get<2>(data[j])) / r;
                if (!std::isfinite(PE)) {
                    if (cfg) cfg->paused = true;
                    out.ok = false;
                    return false;
                }
            }
        }

        out.kinetic = KE;
        out.potential = PE;
        out.energy = KE + PE;
        out.momentum = DVec2{Px, Py};
        out.totalMass = M;
        out.com = (M > 0.0) ? DVec2{Cx / M, Cy / M} : DVec2{0.0, 0.0};

        out.ok = std::isfinite(out.kinetic) && std::isfinite(out.potential) && std::isfinite(out.energy) &&
            std::isfinite(out.momentum.x) && std::isfinite(out.momentum.y) && std::isfinite(out.totalMass) &&
            std::isfinite(out.com.x) && std::isfinite(out.com.y);
        if (!out.ok) {
            if (cfg) cfg->paused = true;
        }
        return out.ok;
    }
    // No backward-compatible aliases: use snake_case API

private:
    static inline bool is_finite(const float v) { return std::isfinite(static_cast<double>(v)); }

    static void compute_gravity(const flecs::world& w) {
        const Config& cfg = *w.get<Config>();
        const double G = cfg.g;
        const double eps2 = static_cast<double>(cfg.softening) * static_cast<double>(cfg.softening);

        std::vector<DVec2> positions;
        std::vector<float> masses;
        std::vector<uint8_t> pins;
        std::vector<Acceleration*> accPtrs;
        positions.reserve(1000);
        masses.reserve(1000);
        pins.reserve(1000);
        accPtrs.reserve(1000);

        w.each([&](Position& p, Velocity& v, Mass& m, Pinned& pin, Acceleration& a) {
            if (std::isfinite(p.value.x) && std::isfinite(p.value.y) && std::isfinite(v.value.x) &&
                std::isfinite(v.value.y) && m.value > 0.0f && std::isfinite(static_cast<double>(m.value))) {
                positions.push_back(p.value);
                masses.push_back(m.value);
                pins.push_back(pin.value ? 1 : 0);
                accPtrs.push_back(&a);
            }
        });

        const size_t n = positions.size();
        if (n == 0) return;

        std::vector acc(n, DVec2{0.0, 0.0});

        if (n > static_cast<size_t>(cfg.bh_threshold)) {
            std::vector<SpatialPartition::Body> bodies;
            bodies.reserve(n);
            for (size_t i = 0; i < n; ++i)
                bodies.push_back(
                    {raylib::Vector2{static_cast<float>(positions[i].x), static_cast<float>(positions[i].y)}, masses[i],
                     static_cast<int>(i)});

            SpatialPartition tree;
            tree.build(bodies);
            const double theta = static_cast<double>(cfg.bh_theta);

            for (size_t i = 0; i < n; ++i) {
                if (pins[i]) continue;
                raylib::Vector2 af{0.0f, 0.0f};
                tree.compute_force(bodies[i], theta, G, eps2, af);
                acc[i].x += static_cast<double>(af.x);
                acc[i].y += static_cast<double>(af.y);
            }
        } else {
            const DVec2* pos = positions.data();
            const float* mass = masses.data();
            const uint8_t* pin = pins.data();
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    const double dx = pos[j].x - pos[i].x;
                    const double dy = pos[j].y - pos[i].y;
                    const double r2 = dx * dx + dy * dy + eps2;
                    const double invR = 1.0 / std::sqrt(r2);
                    const double invR3 = invR * invR * invR;

                    const double ax_i = G * static_cast<double>(mass[j]) * dx * invR3;
                    const double ay_i = G * static_cast<double>(mass[j]) * dy * invR3;
                    const double ax_j = -G * static_cast<double>(mass[i]) * dx * invR3;
                    const double ay_j = -G * static_cast<double>(mass[i]) * dy * invR3;

                    if (!pin[i]) {
                        acc[i].x += ax_i;
                        acc[i].y += ay_i;
                    }
                    if (!pin[j]) {
                        acc[j].x += ax_j;
                        acc[j].y += ay_j;
                    }
                }
            }
        }

        for (size_t i = 0; i < n; ++i) accPtrs[i]->value = acc[i];
    }

    static void integrate(const flecs::world& w, const float dt) {
        const Config& cfg = *w.get<Config>();
        const float maxSpeed = cfg.max_speed;

        // Substep splitting for stability at large dt
        const float cap = std::max(1e-6f, cfg.max_substep);
        int nSteps = static_cast<int>(std::ceil(dt / cap));
        nSteps = std::max(1, std::min(nSteps, std::max(1, cfg.max_substeps_per_frame)));
        const float dtSub = dt / static_cast<float>(nSteps);

        if (const int integrator = cfg.integrator; integrator == 0) {
            // Semi-Implicit Euler with substeps
            for (int step = 0; step < nSteps; ++step) {
                w.each([&](Position& p, Velocity& v, const Acceleration& a, const Pinned& pin) {
                    if (pin.value) return;
                    v.value.x += a.value.x * dtSub;
                    v.value.y += a.value.y * dtSub;
                    if (maxSpeed > 0.0f) {
                        const double vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                        if (vlen > static_cast<double>(maxSpeed)) {
                            const double s = static_cast<double>(maxSpeed) / vlen;
                            v.value.x *= s;
                            v.value.y *= s;
                        }
                    }
                    p.value.x += v.value.x * dtSub;
                    p.value.y += v.value.y * dtSub;
                });
                // Refresh acceleration for next substep
                if (step + 1 < nSteps) compute_gravity(w);
            }
        } else {
            // Velocity Verlet with substeps
            for (int step = 0; step < nSteps; ++step) {
                w.each([&](Position& p, const Velocity& v, const Acceleration& a, PrevAcceleration& a0,
                           const Pinned& pin) {
                    if (pin.value) return;
                    const double half_dt2 = 0.5 * static_cast<double>(dtSub) * static_cast<double>(dtSub);
                    p.value.x += v.value.x * dtSub + a.value.x * half_dt2;
                    p.value.y += v.value.y * dtSub + a.value.y * half_dt2;
                    a0.value = a.value;  // store a_t
                });

                // Compute a_{t+dtSub}
                compute_gravity(w);

                w.each([&](Velocity& v, const Acceleration& a, const PrevAcceleration& a0, const Pinned& pin) {
                    if (pin.value) return;
                    const double ax = (a0.value.x + a.value.x) * 0.5;
                    const double ay = (a0.value.y + a.value.y) * 0.5;
                    v.value.x += ax * dtSub;
                    v.value.y += ay * dtSub;
                    if (maxSpeed > 0.0f) {
                        const double vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                        if (vlen > static_cast<double>(maxSpeed)) {
                            const double s = static_cast<double>(maxSpeed) / vlen;
                            v.value.x *= s;
                            v.value.y *= s;
                        }
                    }
                });
                // After velocity update, 'a' already equals a_{t+dtSub} for the next substep
            }
        }
    }

    static void update_trails(const flecs::world& w) {
        const Config& cfg = *w.get<Config>();
        if (!cfg.draw_trails) return;
        const int maxLen = std::max(0, cfg.trail_max);
        w.each([&](Trail& t, const Position& p) {
            t.points.push_back(raylib::Vector2{static_cast<float>(p.value.x), static_cast<float>(p.value.y)});
            if (static_cast<int>(t.points.size()) > maxLen) t.points.erase(t.points.begin());
        });
    }
};

}  // namespace nbody
