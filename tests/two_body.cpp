#include <cassert>
#include <cmath>
#include <flecs.h>

#include "../src/components/Components.hpp"
#include "../src/core/Config.hpp"
#include "../src/systems/Diagnostics.hpp"
#include "../src/systems/Physics.hpp"

int main() {
    flecs::world w;
    Config cfg{};
    cfg.G = 1.0;
    cfg.softening = 0.0f;
    cfg.maxSpeed = 0.0f;
    cfg.paused = false;
    cfg.useFixedDt = true;
    cfg.fixedDt = 1e-3f;
    cfg.timeScale = 1.0f;
    cfg.integrator = 1;
    w.set<Config>(cfg);
    nbody::Physics::Register(w);

    const double m = 1.0;
    const double v = std::sqrt(0.5 * cfg.G * m);
    w.entity()
        .set<Position>({DVec2{-0.5, 0.0}})
        .set<Velocity>({DVec2{0.0, v}})
        .set<Acceleration>({DVec2{0.0, 0.0}})
        .set<PrevAcceleration>({DVec2{0.0, 0.0}})
        .set<Mass>({static_cast<float>(m)})
        .set<Pinned>({false});
    w.entity()
        .set<Position>({DVec2{0.5, 0.0}})
        .set<Velocity>({DVec2{0.0, -v}})
        .set<Acceleration>({DVec2{0.0, 0.0}})
        .set<PrevAcceleration>({DVec2{0.0, 0.0}})
        .set<Mass>({static_cast<float>(m)})
        .set<Pinned>({false});

    nbody::Diagnostics d0{};
    nbody::ComputeDiagnostics(w, cfg.G, 0.0, d0);

    for (int i = 0; i < 10000; ++i) {
        w.progress(cfg.fixedDt);
    }

    nbody::Diagnostics d1{};
    nbody::ComputeDiagnostics(w, cfg.G, 0.0, d1);

    const double energyDiff = std::abs(d1.energy - d0.energy) / std::abs(d0.energy);
    assert(energyDiff < 1e-3);
    const double pxDiff = std::abs(d1.momentum.x - d0.momentum.x);
    const double pyDiff = std::abs(d1.momentum.y - d0.momentum.y);
    assert(pxDiff < 1e-6);
    assert(pyDiff < 1e-6);
    return 0;
}
