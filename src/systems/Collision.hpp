#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <flecs.h>

#include "../components/Components.hpp"
#include "../core/Constants.hpp"

namespace nbody::systems {

// Basic collision detection & resolution for spherical bodies.
// - Detects overlaps using radii (component if present, otherwise derived from mass & density).
// - Resolution mode:
//   - Inelastic merge (default): combine masses, conserve momentum; delete one body.
//   - Elastic impulse: conserve momentum and kinetic energy; positional separation to resolve penetration.
//
// Notes:
// - All math is in SI units (meters, kilograms, seconds) using double precision for positions/velocities.
// - Pinned bodies are treated as immovable (infinite mass) in elastic mode; in inelastic mode, merge into the pinned.
struct Collision {
    // Toggle between merge and elastic response. Default: false (merge).
    // Could be elevated to Config if runtime toggling is desired.
    static constexpr bool kElastic = false;

    static inline double radius_of(const flecs::entity& e, const Mass& m) {
        if (const auto* r = e.get<Radius>()) return r->value;
        const double safeMass = std::max(1.0, static_cast<double>(m.value));
        // Sphere radius from mass and density: r = cbrt(3M / (4πρ))
        return std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::body_density));
    }

    static inline void update_radius_from_mass(flecs::entity& e, const Mass& m) {
        const double r = std::cbrt((3.0 * std::max(1.0, static_cast<double>(m.value))) /
                                   (4.0 * std::numbers::pi * nbody::constants::body_density));
        if (auto* rp = e.get_mut<Radius>()) {
            rp->value = r;
        } else {
            e.set<Radius>({r});
        }
    }

    static void resolve(const flecs::world& w) {
        // Snapshot dynamic bodies
        struct BodyRef {
            flecs::entity e;
            DVec2 p;
            DVec2 v;
            float m;
            double r;
            bool pinned;
        };

        std::vector<BodyRef> bodies;
        bodies.reserve(512);
        w.each([&](const flecs::entity e, const Position& p, const Velocity& v, const Mass& m, const Pinned& pin) {
            if (!(std::isfinite(p.value.x) && std::isfinite(p.value.y) && std::isfinite(static_cast<double>(m.value))))
                return;
            if (m.value <= 0.0f) return;
            bodies.push_back(BodyRef{e, p.value, v.value, m.value, radius_of(e, m), pin.value});
        });

        const size_t n = bodies.size();
        if (n < 2) return;

        std::vector<uint8_t> alive(n, 1);

        // Pairwise detection and resolution
        for (size_t i = 0; i < n; ++i) {
            if (!alive[i]) continue;
            for (size_t j = i + 1; j < n; ++j) {
                if (!alive[j]) continue;

                BodyRef &A = bodies[i], &B = bodies[j];
                const double dx = B.p.x - A.p.x;
                const double dy = B.p.y - A.p.y;
                const double rsum = A.r + B.r;
                const double dist2 = dx * dx + dy * dy;
                if (dist2 > rsum * rsum) continue;  // no overlap

                // Handle degenerate distance
                const double dist = std::sqrt(std::max(dist2, 1e-20));
                DVec2 nrm{(dist > 0.0) ? dx / dist : 1.0, (dist > 0.0) ? dy / dist : 0.0};

                // Inelastic merge (default)
                if (!kElastic) {
                    // Choose survivor: heavier mass wins to reduce jitter
                    const bool a_survives = (A.m >= B.m) || B.pinned;
                    BodyRef &S = a_survives ? A : B;
                    BodyRef &D = a_survives ? B : A;
                    if (D.pinned && S.pinned) {
                        // Both pinned: just separate a bit to avoid sticking
                        const double penetration = rsum - dist;
                        if (penetration > 0.0) {
                            // Move neither (pinned), nothing to do
                        }
                        continue;
                    }

                    // Combine mass and momentum
                    const double M = static_cast<double>(S.m) + static_cast<double>(D.m);
                    const DVec2 P{static_cast<double>(S.m) * S.v.x + static_cast<double>(D.m) * D.v.x,
                                  static_cast<double>(S.m) * S.v.y + static_cast<double>(D.m) * D.v.y};
                    const DVec2 V{P.x / M, P.y / M};
                    const DVec2 X{(static_cast<double>(S.m) * S.p.x + static_cast<double>(D.m) * D.p.x) / M,
                                  (static_cast<double>(S.m) * S.p.y + static_cast<double>(D.m) * D.p.y) / M};

                    // Apply to survivor entity
                    if (auto* mp = S.e.get_mut<Mass>()) mp->value = static_cast<float>(M);
                    if (auto* vp = S.e.get_mut<Velocity>()) vp->value = V;
                    if (auto* pp = S.e.get_mut<Position>()) pp->value = X;
                    if (auto* ap = S.e.get_mut<Acceleration>()) ap->value = DVec2{0.0, 0.0};
                    if (auto* a0 = S.e.get_mut<PrevAcceleration>()) a0->value = DVec2{0.0, 0.0};
                    if (auto* pinp = S.e.get_mut<Pinned>()) pinp->value = S.pinned || D.pinned;
                    update_radius_from_mass(S.e, *S.e.get<Mass>());

                    // Delete the other
                    D.e.destruct();
                    alive[a_survives ? j : i] = 0;

                    // Update snapshot for survivor
                    S.m = static_cast<float>(M);
                    S.v = V;
                    S.p = X;
                    S.pinned = (S.pinned || D.pinned);
                    S.r = radius_of(S.e, *S.e.get<Mass>());
                } else {
                    // Elastic impulse (e = 1)
                    if (A.pinned && B.pinned) {
                        // Separate positions only if possible (both pinned: leave as-is)
                        continue;
                    }

                    const double m1 = static_cast<double>(A.m);
                    const double m2 = static_cast<double>(B.m);

                    const DVec2 v1 = A.v;
                    const DVec2 v2 = B.v;
                    const DVec2 x1 = A.p;
                    const DVec2 x2 = B.p;
                    const DVec2 x1mx2{x1.x - x2.x, x1.y - x2.y};
                    const DVec2 x2mx1{x2.x - x1.x, x2.y - x1.y};
                    const double l2 = std::max(1e-20, x1mx2.x * x1mx2.x + x1mx2.y * x1mx2.y);

                    DVec2 nv1 = v1;
                    DVec2 nv2 = v2;

                    if (!A.pinned && !B.pinned) {
                        const double f1 = (2.0 * m2 / (m1 + m2)) * ((v1.x - v2.x) * x1mx2.x + (v1.y - v2.y) * x1mx2.y) / l2;
                        const double f2 = (2.0 * m1 / (m1 + m2)) * ((v2.x - v1.x) * x2mx1.x + (v2.y - v1.y) * x2mx1.y) / l2;
                        nv1.x = v1.x - f1 * x1mx2.x;
                        nv1.y = v1.y - f1 * x1mx2.y;
                        nv2.x = v2.x - f2 * x2mx1.x;
                        nv2.y = v2.y - f2 * x2mx1.y;
                    } else if (A.pinned && !B.pinned) {
                        // Reflect B about normal
                        const double vn = v2.x * nrm.x + v2.y * nrm.y;
                        nv2.x = v2.x - 2.0 * vn * nrm.x;
                        nv2.y = v2.y - 2.0 * vn * nrm.y;
                    } else if (!A.pinned && B.pinned) {
                        const double vn = v1.x * nrm.x + v1.y * nrm.y;
                        nv1.x = v1.x - 2.0 * vn * nrm.x;
                        nv1.y = v1.y - 2.0 * vn * nrm.y;
                    }

                    // Positional correction to resolve penetration
                    const double penetration = rsum - dist;
                    if (penetration > 0.0) {
                        const double total = (A.pinned ? 0.0 : m1) + (B.pinned ? 0.0 : m2);
                        const double w1 = (A.pinned || total == 0.0) ? 0.0 : (m2 / total);
                        const double w2 = (B.pinned || total == 0.0) ? 0.0 : (m1 / total);
                        if (!A.pinned) {
                            A.p.x -= nrm.x * penetration * w1;
                            A.p.y -= nrm.y * penetration * w1;
                            if (auto* pp = A.e.get_mut<Position>()) pp->value = A.p;
                        }
                        if (!B.pinned) {
                            B.p.x += nrm.x * penetration * w2;
                            B.p.y += nrm.y * penetration * w2;
                            if (auto* pp = B.e.get_mut<Position>()) pp->value = B.p;
                        }
                    }

                    if (!A.pinned) {
                        if (auto* vp = A.e.get_mut<Velocity>()) vp->value = nv1;
                        A.v = nv1;
                    }
                    if (!B.pinned) {
                        if (auto* vp = B.e.get_mut<Velocity>()) vp->value = nv2;
                        B.v = nv2;
                    }
                }
            }
        }
    }
};

}  // namespace nbody::systems
