#pragma once

#include <flecs.h>
#include <tuple>
#include <vector>

#include "../components/Components.hpp"

namespace nbody {

    struct Diagnostics {
        double kinetic = 0.0;
        double potential = 0.0;
        double energy = 0.0;
        DVec2 momentum{0.0, 0.0};
        DVec2 com{0.0, 0.0};
        double totalMass = 0.0;
    };

    inline bool ComputeDiagnostics(const flecs::world& w, double G, double eps2, Diagnostics& out) {
        std::vector<std::tuple<DVec2, DVec2, float>> data;
        data.reserve(1024);
        w.each(
            [&](const Position& p, const Velocity& v, const Mass& m) { data.emplace_back(p.value, v.value, m.value); });
        const size_t n = data.size();
        out = Diagnostics{};
        if (n == 0) return true;

        double KE = 0.0, M = 0.0, Px = 0.0, Py = 0.0, Cx = 0.0, Cy = 0.0;
        for (size_t i = 0; i < n; ++i) {
            auto [p, v, m] = data[i];
            KE += 0.5 * static_cast<double>(m) * (v.x * v.x + v.y * v.y);
            Px += static_cast<double>(m) * v.x;
            Py += static_cast<double>(m) * v.y;
            Cx += static_cast<double>(m) * p.x;
            Cy += static_cast<double>(m) * p.y;
            M += static_cast<double>(m);
        }
        double PE = 0.0;
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const double dx = std::get<0>(data[j]).x - std::get<0>(data[i]).x;
                const double dy = std::get<0>(data[j]).y - std::get<0>(data[i]).y;
                const double r2 = dx * dx + dy * dy + eps2;
                const double r = std::sqrt(r2);
                PE += -G * static_cast<double>(std::get<2>(data[i])) * static_cast<double>(std::get<2>(data[j])) / r;
            }
        }

        out.kinetic = KE;
        out.potential = PE;
        out.energy = KE + PE;
        out.momentum = {Px, Py};
        out.totalMass = M;
        out.com = (M > 0.0) ? DVec2{Cx / M, Cy / M} : DVec2{0.0, 0.0};
        return true;
    }

}  // namespace nbody
