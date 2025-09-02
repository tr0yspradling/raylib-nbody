#pragma once

#include <memory>
#include <flecs.h>
#include "IIntegrator.hpp"
#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "../physics/SpatialPartition.hpp"
#include "../systems/Collision.hpp"

namespace nbody::simulation {

class PhysicsEngine {
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

    explicit PhysicsEngine(std::shared_ptr<IIntegrator> integrator)
        : integrator_(std::move(integrator)) {}

    void registerSystems(const flecs::world& world) {
        // Note: We could store the physics engine instance in the world if needed,
        // but for now we'll use a simpler approach where systems are self-contained

        // Collision system
        world.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            auto* cfg = world.get<Config>();
            if (!cfg || cfg->paused) return;
            
            nbody::systems::Collision::resolve(world);
            updateDiagnostics(world);
        });

        // Gravity computation system
        world.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            const Config& cfg = *world.get<Config>();
            if (cfg.paused) return;
            computeGravity(world);
        });

        // Integration system - for now, we'll use the original physics integration
        // until we fully extract the integrator dependency
        world.system<>().kind(flecs::OnUpdate).iter([this](const flecs::iter& it) {
            const Config& cfg = *it.world().get<Config>();
            if (cfg.paused) return;
            
            const float baseDt = cfg.use_fixed_dt ? cfg.fixed_dt : static_cast<float>(it.delta_time());
            const float dtEff = baseDt * std::max(0.0f, cfg.time_scale);
            
            // Use original integration method for now
            integrate(it.world(), dtEff);
        });

        // Trail update system
        world.system<>().kind(flecs::OnUpdate).iter([&](flecs::iter&) {
            const Config& cfg = *world.get<Config>();
            if (cfg.paused) return;
            updateTrails(world);
        });
    }

    void setIntegrator(std::shared_ptr<IIntegrator> integrator) {
        integrator_ = std::move(integrator);
    }

    std::shared_ptr<IIntegrator> getIntegrator() const {
        return integrator_;
    }

    static void zeroNetMomentum(const flecs::world& world) {
        double Px = 0.0, Py = 0.0, M = 0.0;
        
        world.each([&](const Mass& m, Velocity& v, const Pinned& pin) {
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
        world.each([&](const Pinned& pin, Velocity& v) {
            if (!pin.value) {
                v.value.x -= v0.x;
                v.value.y -= v0.y;
            }
        });
    }

    static bool computeDiagnostics(const flecs::world& world, const double G, 
                                  const double eps2, Diagnostics& out) {
        std::vector<std::tuple<DVec2, DVec2, float>> data;
        data.reserve(1024);
        
        world.each([&](const Position& p, const Velocity& v, const Mass& m) {
            data.emplace_back(p.value, v.value, m.value);
        });
        
        const size_t n = data.size();
        out = Diagnostics{};
        
        if (n == 0) {
            out.ok = true;
            return true;
        }

        auto* cfg = world.get_mut<Config>();
        
        // Calculate kinetic energy and momentum
        double KE = 0.0, M = 0.0, Px = 0.0, Py = 0.0, Cx = 0.0, Cy = 0.0;
        for (size_t i = 0; i < n; ++i) {
            auto [p, v, m] = data[i];
            KE += 0.5 * static_cast<double>(m) * (v.x * v.x + v.y * v.y);
            Px += static_cast<double>(m) * v.x;
            Py += static_cast<double>(m) * v.y;
            Cx += static_cast<double>(m) * p.x;
            Cy += static_cast<double>(m) * p.y;
            M += static_cast<double>(m);
            
            if (!(std::isfinite(KE) && std::isfinite(Px) && std::isfinite(Py) && 
                  std::isfinite(Cx) && std::isfinite(Cy) && std::isfinite(M))) {
                if (cfg) cfg->paused = true;
                out.ok = false;
                return false;
            }
        }
        
        // Calculate potential energy
        double PE = 0.0;
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const double dx = std::get<0>(data[j]).x - std::get<0>(data[i]).x;
                const double dy = std::get<0>(data[j]).y - std::get<0>(data[i]).y;
                const double r2 = dx * dx + dy * dy + eps2;
                const double r = std::sqrt(r2);
                PE += -G * static_cast<double>(std::get<2>(data[i])) * 
                      static_cast<double>(std::get<2>(data[j])) / r;
                
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

        out.ok = std::isfinite(out.kinetic) && std::isfinite(out.potential) && 
                 std::isfinite(out.energy) && std::isfinite(out.momentum.x) && 
                 std::isfinite(out.momentum.y) && std::isfinite(out.totalMass) && 
                 std::isfinite(out.com.x) && std::isfinite(out.com.y);
        
        if (!out.ok && cfg) {
            cfg->paused = true;
        }
        
        return out.ok;
    }

    void integrate(const flecs::world& world, float deltaTime) {
        // For now, use a simple integration approach similar to the original
        // In the future, this could delegate to the strategy pattern integrators
        const Config& cfg = *world.get<Config>();
        const float maxSpeed = cfg.max_speed;
        
        // Simple Euler integration for now
        world.each([&](Position& p, Velocity& v, const Acceleration& a, const Pinned& pin) {
            if (pin.value) return;
            
            // Update velocity
            v.value.x += a.value.x * deltaTime;
            v.value.y += a.value.y * deltaTime;
            
            // Apply velocity cap if configured
            if (maxSpeed > 0.0f) {
                const double vlen = std::sqrt(v.value.x * v.value.x + v.value.y * v.value.y);
                if (vlen > static_cast<double>(maxSpeed)) {
                    const double s = static_cast<double>(maxSpeed) / vlen;
                    v.value.x *= s;
                    v.value.y *= s;
                }
            }
            
            // Update position
            p.value.x += v.value.x * deltaTime;
            p.value.y += v.value.y * deltaTime;
        });
    }

private:
    void updateDiagnostics(const flecs::world& world) {
        const Config& cfg = *world.get<Config>();
        Diagnostics d{};
        d.ok = computeDiagnostics(world, cfg.g, 
                                 static_cast<double>(cfg.softening) * static_cast<double>(cfg.softening), d);
        world.set<Diagnostics>(d);
    }

    void computeGravity(const flecs::world& world) {
        const Config& cfg = *world.get<Config>();
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

        world.each([&](Position& p, Velocity& v, Mass& m, Pinned& pin, Acceleration& a) {
            if (std::isfinite(p.value.x) && std::isfinite(p.value.y) && 
                std::isfinite(v.value.x) && std::isfinite(v.value.y) && 
                m.value > 0.0f && std::isfinite(static_cast<double>(m.value))) {
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
            computeGravityBarnesHut(positions, masses, pins, acc, cfg, G, eps2);
        } else {
            computeGravityDirect(positions, masses, pins, acc, G, eps2);
        }

        for (size_t i = 0; i < n; ++i) {
            accPtrs[i]->value = acc[i];
        }
    }

    void computeGravityDirect(const std::vector<DVec2>& positions, 
                             const std::vector<float>& masses,
                             const std::vector<uint8_t>& pins,
                             std::vector<DVec2>& acc,
                             double G, double eps2) {
        const size_t n = positions.size();
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

    void computeGravityBarnesHut(const std::vector<DVec2>& positions,
                                const std::vector<float>& masses,
                                const std::vector<uint8_t>& pins,
                                std::vector<DVec2>& acc,
                                const Config& cfg, double G, double eps2) {
        const size_t n = positions.size();
        
        std::vector<SpatialPartition::Body> bodies;
        bodies.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            bodies.push_back({
                raylib::Vector2{static_cast<float>(positions[i].x), static_cast<float>(positions[i].y)},
                masses[i],
                static_cast<int>(i)
            });
        }

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
    }

    void updateTrails(const flecs::world& world) {
        const Config& cfg = *world.get<Config>();
        if (!cfg.draw_trails) return;
        
        const int maxLen = std::max(0, cfg.trail_max);
        world.each([&](Trail& t, const Position& p) {
            t.points.push_back(raylib::Vector2{static_cast<float>(p.value.x), static_cast<float>(p.value.y)});
            if (static_cast<int>(t.points.size()) > maxLen) {
                t.points.erase(t.points.begin());
            }
        });
    }

private:
    std::shared_ptr<IIntegrator> integrator_;
};

}  // namespace nbody::simulation