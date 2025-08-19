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
            const float dt = (cfg.useFixedDt ? cfg.fixedDt : (float)it.delta_time()) * std::max(0.0f, cfg.timeScale);
            integrate(w, dt);
        });

        // Trails update after integration
        w.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            if (const Config& cfg = *w.get<Config>(); cfg.paused) return;
            update_trails(w);
        });
    }

    static void ZeroNetMomentum(const flecs::world& w) {
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

    static void ResetScenario(const flecs::world& w) {
        std::vector<flecs::entity> toDel;
        w.each([&](const flecs::entity e, Position&) { toDel.push_back(e); });
        for (auto& e : toDel) e.destruct();
        auto mk = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, const float mass, const raylib::Color col,
                      const bool pinned) {
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
                .set<Draggable>({true, constants::dragVelScale});
        };
        mk({constants::seedCenterX, constants::seedCenterY}, {0.0f, 0.0f}, constants::seedCentralMass, RED, false);
        mk({constants::seedCenterX + constants::seedOffsetX, constants::seedCenterY}, {0.0f, constants::seedSpeed},
           constants::seedSmallMass, BLUE, false);
        mk({constants::seedCenterX - constants::seedOffsetX, constants::seedCenterY}, {0.0f, -constants::seedSpeed},
           constants::seedSmallMass, GREEN, false);
    }

    static bool ComputeDiagnostics(const flecs::world& w, const double G, const double eps2, Diagnostics& out) {
        std::vector<std::tuple<raylib::Vector2, raylib::Vector2, float>> data;
        data.reserve(1024);
        w.each(
            [&](const Position& p, const Velocity& v, const Mass& m) { data.emplace_back(p.value, v.value, m.value); });
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
                PE += -G * static_cast<double>(std::get<2>(data[i])) * static_cast<double>(std::get<2>(data[j])) / r;
            }
        }

        out.kinetic = KE;
        out.potential = PE;
        out.energy = KE + PE;
        out.momentum = raylib::Vector2{static_cast<float>(Px), static_cast<float>(Py)};
        out.totalMass = M;
        out.com =
            (M > 0.0) ? raylib::Vector2{static_cast<float>(Cx / M), static_cast<float>(Cy / M)} : raylib::Vector2{0, 0};
        return true;
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
                    if (const float vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y); vlen > maxSpeed)
                        v.value = v.value * (maxSpeed / vlen);
                }
                p.value += v.value * dt;
            });
        } else {
            w.each([&](Position& p, const Velocity& v, const Acceleration& a, PrevAcceleration& a0, const Pinned& pin) {
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
                    if (const float vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y); vlen > maxSpeed)
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
