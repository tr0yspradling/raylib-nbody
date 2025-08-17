#include <cfloat>
#include <cmath>
#include <algorithm>
#include <flecs.h>
#include <imgui.h>
#include <rlImGui.h>
#include <string>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "raylib-cpp.hpp"
#include "raymath.h"

// Forward declarations for interaction system
namespace ecs {
    flecs::entity get_selected_entity(const flecs::world&);
    void select_entity(const flecs::world&, flecs::entity);
}

namespace ecs {

    void ui_begin() { rlImGuiBegin(); }

    void ui_end() { rlImGuiEnd(); }

    static raylib::Color RandomNiceColor() {
        return {static_cast<unsigned char>(GetRandomValue(64, 255)),
                static_cast<unsigned char>(GetRandomValue(64, 255)),
                static_cast<unsigned char>(GetRandomValue(64, 255)), 255};
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
        // Clear existing bodies
        std::vector<flecs::entity> to_del;
        w.each([&](const flecs::entity e, Position&) { to_del.push_back(e); });
        for (auto& e : to_del) e.destruct();

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
                .add<Selectable>()        // Make reset bodies selectable
                .set<Draggable>({true, 0.01f}); // Make reset bodies draggable
        };

        mk({640, 360}, {0.0f, 0.0f}, 4000.0f, RED, false);
        mk({840, 360}, {0.0f, 1.20f}, 12.0f, BLUE, false);
        mk({440, 360}, {0.0f, -1.20f}, 12.0f, GREEN, false);
    }

    struct Diagnostics {
        double kinetic = 0.0;
        double potential = 0.0;
        double energy = 0.0;
        raylib::Vector2 momentum{0, 0};
        raylib::Vector2 com{0, 0};
        double totalMass = 0.0;
    };

    static bool compute_diagnostics(const flecs::world& w, const double G, const double eps2, Diagnostics& out) {
        std::vector<std::tuple<raylib::Vector2, raylib::Vector2, float>> data;  // pos, vel, mass
        data.reserve(1024);
        w.each([&](const Position& p, const Velocity& v, const Mass& m) {
            data.emplace_back(p.value, v.value, m.value);
        });
        const size_t n = data.size();
        out = Diagnostics{};
        if (n == 0) return true;

        double KE = 0.0;
        double M = 0.0;
        double Px = 0.0, Py = 0.0;
        double Cx = 0.0, Cy = 0.0;
        for (size_t i = 0; i < n; ++i) {
            auto [p, v, m] = data[i];
            KE += 0.5 * static_cast<double>(m) * (static_cast<double>(v.x) * static_cast<double>(v.x) + static_cast<double>(v.y) * static_cast<double>(v.y));
            Px += static_cast<double>(m) * static_cast<double>(v.x);
            Py += static_cast<double>(m) * static_cast<double>(v.y);
            Cx += static_cast<double>(m) * static_cast<double>(p.x);
            Cy += static_cast<double>(m) * static_cast<double>(p.y);
            M += static_cast<double>(m);
        }
        double PE = 0.0;
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 1; j < n; ++j) {
                const double dx = static_cast<double>(std::get<0>(data[j]).x) - static_cast<double>(std::get<0>(data[i]).x);
                const double dy = static_cast<double>(std::get<0>(data[j]).y) - static_cast<double>(std::get<0>(data[i]).y);
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
        out.com = (M > 0.0) ? raylib::Vector2{static_cast<float>(Cx / M), static_cast<float>(Cy / M)} : raylib::Vector2{0, 0};
        return true;
    }

    void draw_ui(const flecs::world& w, raylib::Camera2D& cam) {
        auto* cfg = w.get_mut<Config>();
        bool requestStep = false;
        ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);

        ImGui::Begin("Time & Integrator");
        ImGui::Checkbox("Paused", &cfg->paused);
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            requestStep = true;
        }
        ImGui::Checkbox("Use Fixed dt", &cfg->useFixedDt);
        ImGui::SliderFloat("Fixed dt", &cfg->fixedDt, 1e-4f, 0.05f, "%.6f");
        ImGui::SliderFloat("Time Scale", &cfg->timeScale, 0.0f, 10.0f, "%.3f");
        ImGui::RadioButton("Semi-Implicit Euler", &cfg->integrator, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Velocity Verlet", &cfg->integrator, 1);
        ImGui::Text("Last step: %.3f ms", cfg->lastStepMs);
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(12, 140), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Physics");
        auto Gf = static_cast<float>(cfg->G);
        ImGui::SliderFloat("G", &Gf, 0.0f, 0.02f, "%.6f");
        cfg->G = Gf;
        ImGui::SliderFloat("Softening (epsilon)", &cfg->softening, 0.0f, 20.0f, "%.3f");
        ImGui::SliderFloat("Velocity Cap", &cfg->maxSpeed, 0.0f, 200.0f, "%.1f");
        if (ImGui::Button("Zero Net Momentum")) {
            zero_net_momentum(w);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Scenario")) {
            reset_scenario(w);
            zero_net_momentum(w);
            select_entity(w, flecs::entity::null()); // Clear selection in new system
            cfg->paused = false;
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(12, 280), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Visuals");
        ImGui::Checkbox("Trails", &cfg->drawTrails);
        ImGui::SameLine();
        ImGui::Checkbox("Velocity Vectors", &cfg->drawVelocity);
        ImGui::SameLine();
        ImGui::Checkbox("Acceleration Vectors", &cfg->drawAcceleration);
        ImGui::SliderInt("Trail Length", &cfg->trailMax, 0, 2000);
        ImGui::End();

        static float spawnMass = 12.0f;
        static raylib::Vector2 spawnVel{0, 0};
        static bool spawnPinned = false;
        static float dragVelScale = 0.01f;
        ImGui::SetNextWindowPos(ImVec2(12, 420), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Add / Edit");
        ImGui::SliderFloat("Spawn Mass", &spawnMass, 1.0f, 5000.0f, "%.1f");
        ImGui::SliderFloat2("Spawn Velocity", &spawnVel.x, -5.0f, 5.0f, "%.3f");
        ImGui::Checkbox("Spawn Pinned", &spawnPinned);
        if (ImGui::Button("Add Body At Mouse")) {
            const raylib::Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), cam);
            w.entity()
                .set<Position>({mouseWorld})
                .set<Velocity>({spawnVel})
                .set<Acceleration>({raylib::Vector2{0, 0}})
                .set<PrevAcceleration>({raylib::Vector2{0, 0}})
                .set<Mass>({std::max(1.0f, spawnMass)})
                .set<Pinned>({spawnPinned})
                .set<Tint>({RandomNiceColor()})
                .set<Trail>({{}})
                .add<Selectable>()        // Make new body selectable
                .set<Draggable>({true, dragVelScale}); // Make new body draggable
        }
        ImGui::SliderFloat("Right-Drag Vel Scale", &dragVelScale, 0.001f, 0.2f, "%.3f");

        if (flecs::entity selected = get_selected_entity(w); selected.is_alive()) {
            const auto mass = selected.get_mut<Mass>();
            const auto vel = selected.get_mut<Velocity>();
            const auto pin = selected.get_mut<Pinned>();
            if (ImGui::CollapsingHeader("Selected Body", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("Entity: %lld", static_cast<long long>(selected.id()));
                ImGui::Checkbox("Pinned", &pin->value);
                ImGui::SliderFloat("Mass", &mass->value, 1.0f, 10000.0f, "%.1f");
                ImGui::SliderFloat2("Velocity", &vel->value.x, -200.0f, 200.0f, "%.3f");
                if (ImGui::Button("Zero Velocity")) vel->value = raylib::Vector2{0.0f, 0.0f};
                ImGui::SameLine();
                if (ImGui::Button("Remove Body")) {
                    selected.destruct();
                    select_entity(w, flecs::entity::null()); // Clear selection in new system
                }
                ImGui::SameLine();
                if (ImGui::Button("Focus Camera")) {
                    if (const auto p = selected.get<Position>()) cam.target = p->value;
                }
            }
        }
        ImGui::End();

        // Bodies list
        ImGui::SetNextWindowPos(ImVec2(400, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 360), ImGuiCond_FirstUseEver);
        ImGui::Begin("Bodies");
        // Defer selection changes until after iterating to avoid mutating during iteration
        flecs::entity pendingSelection = flecs::entity::null();
        if (ImGui::BeginListBox("##BodyList", ImVec2(-FLT_MIN, 300))) {
            struct Row { flecs::entity e; raylib::Vector2 pos; float mass; raylib::Color tint; };
            std::vector<Row> rows;
            rows.reserve(128);
            w.each([&](const flecs::entity e, const Position& p, const Mass& m, const Tint& tint) {
                rows.push_back(Row{e, p.value, m.value, tint.value});
            });
            std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b){ return a.e.id() < b.e.id(); });
            
            flecs::entity currentSelected = get_selected_entity(w);
            for (const Row& r : rows) {
                const std::string label = std::to_string(static_cast<int>(r.e.id())) +
                    "  m=" + std::to_string(static_cast<int>(r.mass)) + "  pos=(" +
                    std::to_string(static_cast<int>(r.pos.x)) + "," + std::to_string(static_cast<int>(r.pos.y)) +
                    ")";
                const auto col = ImVec4(r.tint.r / 255.0f, r.tint.g / 255.0f, r.tint.b / 255.0f, 1.0f);
                ImGui::PushID(static_cast<int>(r.e.id()));
                ImGui::ColorButton("##c", col, ImGuiColorEditFlags_NoTooltip, ImVec2(16, 16));
                ImGui::SameLine();
                const bool isSel = currentSelected.is_alive() && currentSelected.id() == r.e.id();
                if (ImGui::Selectable(label.c_str(), isSel)) {
                    pendingSelection = r.e;
                }
                ImGui::PopID();
            }
            ImGui::EndListBox();
        }
        if (pendingSelection.is_alive()) {
            select_entity(w, pendingSelection);
        }
        if (ImGui::Button("Duplicate Selected")) {
            if (flecs::entity selected = get_selected_entity(w); selected.is_alive()) {
                const auto e = selected;
                auto p = *e.get<Position>();
                auto v = *e.get<Velocity>();
                auto m = *e.get<Mass>();
                auto t = *e.get<Tint>();
                auto pin = *e.get<Pinned>();
                p.value += raylib::Vector2{20.0f, 0.0f};
                w.entity()
                    .set(p)
                    .set(v)
                    .set(Acceleration{raylib::Vector2{0, 0}})
                    .set(PrevAcceleration{raylib::Vector2{0, 0}})
                    .set(m)
                    .set(pin)
                    .set(t)
                    .set(Trail{{}})
                    .add<Selectable>()        // Make duplicated body selectable
                    .set<Draggable>({true, dragVelScale}); // Make duplicated body draggable
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Recenter to COM")) {
            if (Diagnostics d{}; compute_diagnostics(w, cfg->G, static_cast<double>(cfg->softening) * static_cast<double>(cfg->softening), d)) {
                cam.target = d.com;
            }
        }
        ImGui::End();

        // Optional: show ImGui demo to verify backend rendering when diagnosing UI issues
        // static bool show_demo = true; if (show_demo) ImGui::ShowDemoWindow(&show_demo);

        Diagnostics d{};
        const bool okDiag = compute_diagnostics(w, cfg->G, static_cast<double>(cfg->softening) * static_cast<double>(cfg->softening), d);
        ImGui::SetNextWindowPos(ImVec2(400, 390), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Diagnostics");
        ImGui::Text("Kinetic: %.6g", d.kinetic);
        ImGui::Text("Potential: %.6g", d.potential);
        ImGui::Text("Total: %.6g", d.energy);
        ImGui::Text("Momentum: (%.6g, %.6g)", d.momentum.x, d.momentum.y);
        ImGui::Text("COM: (%.3f, %.3f)  Mass: %.3f", d.com.x, (d.totalMass > 0.0) ? d.com.y : 0.0f, d.totalMass);
        if (!okDiag) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Non-finite detected; auto-paused.");
        ImGui::End();

        // If user pressed Step, do one fixed-step tick by toggling pause and relying on systems
        if (requestStep) {
            const bool old = cfg->paused;
            cfg->paused = false;
            w.progress(cfg->fixedDt);
            cfg->paused = old;
        }
    }

}  // namespace ecs
