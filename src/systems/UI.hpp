#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raymath.h>
#include <rlImGui.h>
#include <string>
#include <vector>

#include "../components/Components.hpp"
#include "../core/Colors.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "Camera.hpp"
#include "Interaction.hpp"
#include "Physics.hpp"
#include "../core/Scenario.hpp"

namespace nbody {

class UI {
public:
    static void begin() { rlImGuiBegin(); }
    static void end() { rlImGuiEnd(); }
    // Internal: allow a one-shot reset of Add/Edit inputs next frame
    static inline bool s_pending_reset_inputs = false;

    static void draw(const flecs::world& w, raylib::Camera2D& cam) {
        auto* cfg = w.get_mut<Config>();
        if (!cfg) return;
        bool requestStep = false;
        static float drag_vel_scale = nbody::constants::drag_vel_scale;
        flecs::entity pendingSelection = flecs::entity::null();

        // Keyboard shortcuts (when UI isn't typing into a widget)
        const ImGuiIO& io = ImGui::GetIO();
        const bool kb_free = !(io.WantCaptureKeyboard);
        if (kb_free) {
            if (IsKeyPressed(KEY_R)) s_open_confirm_reset_all = true;              // Reset All (with confirm)
            if (IsKeyPressed(KEY_S)) perform_reset_scenario(w, *cfg);             // Reset Scenario
            if (IsKeyPressed(KEY_V)) Camera::reset_view(w);                      // Reset View
            if (IsKeyPressed(KEY_C)) Camera::center_on_center_of_mass(w);        // Center View to COM
            if (IsKeyPressed(KEY_Z)) Physics::zero_net_momentum(w);              // Zero Momentum
        }

        draw_time_integrator_panel(w, *cfg, requestStep);
        draw_physics_panel(w, *cfg);
        draw_visuals_panel(*cfg);
        draw_add_edit_panel(w, cam, drag_vel_scale);
        draw_bodies_panel(w, drag_vel_scale, pendingSelection);
        draw_diagnostics_panel(w, *cfg);
        draw_scenarios_panel(w);

        if (pendingSelection.is_alive() ||
            (!Interaction::get_selected(w).is_alive() && pendingSelection == flecs::entity::null())) {
            Interaction::select(w, pendingSelection);
        }

        if (requestStep) {
            const bool old = cfg->paused;
            cfg->paused = false;
            w.progress(cfg->fixed_dt);
            cfg->paused = old;
        }
    }

private:
    // Modal state for confirmation
    static inline bool s_open_confirm_reset_all = false;

    static void draw_time_integrator_panel(const flecs::world& w, Config& cfg, bool& requestStep) {
        ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Time & Integrator");
        ImGui::Checkbox("Paused", &cfg.paused);
        ImGui::SameLine();
        if (ImGui::Button("Step")) requestStep = true;
        ImGui::SameLine();
        if (ImGui::Button("Reset View (V)")) Camera::reset_view(w);
        ImGui::SameLine();
        if (ImGui::Button("Center View (C)")) Camera::center_on_center_of_mass(w);
        if (ImGui::Button("Reset Scenario (S)")) perform_reset_scenario(w, cfg);
        ImGui::SameLine();
        if (ImGui::Button("Reset ALL (R)")) s_open_confirm_reset_all = true;
        if (s_open_confirm_reset_all) ImGui::OpenPopup("Confirm Reset All");
        if (ImGui::BeginPopupModal("Confirm Reset All", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextWrapped("This will reset: bodies, configuration (time scale, integrator, visuals), camera view, and UI inputs. Are you sure?");
            if (ImGui::Button("Reset", ImVec2(120, 0))) {
                perform_reset_all(w);
                ImGui::CloseCurrentPopup();
                s_open_confirm_reset_all = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                ImGui::CloseCurrentPopup();
                s_open_confirm_reset_all = false;
            }
            ImGui::EndPopup();
        }
        ImGui::Checkbox("Use Fixed dt", &cfg.use_fixed_dt);
        ImGui::SliderFloat("Fixed dt", &cfg.fixed_dt, nbody::constants::fixed_dt_min, nbody::constants::fixed_dt_max,
                           "%.6f");
        ImGui::SliderFloat("Time Scale", &cfg.time_scale, nbody::constants::time_scale_min, nbody::constants::time_scale_max,
                           "%.2e", ImGuiSliderFlags_Logarithmic);
        ImGui::RadioButton("Semi-Implicit Euler", &cfg.integrator, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Velocity Verlet", &cfg.integrator, 1);
        if (ImGui::CollapsingHeader("Advanced Stability", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Max Substep (s)", &cfg.max_substep, 0.01f, 3600.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderInt("Max Substeps / Frame", &cfg.max_substeps_per_frame, 1, 2000);
        }
        ImGui::Text("Last step: %.3f ms", cfg.last_step_ms);
        ImGui::End();
    }

    static void draw_physics_panel(const flecs::world& w, Config& cfg) {
        ImGui::SetNextWindowPos(ImVec2(12, 140), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Physics");
        auto Gf = static_cast<float>(cfg.g);
        ImGui::SliderFloat("G", &Gf, nbody::constants::g_min, nbody::constants::g_max, "%.2e");
        cfg.g = Gf;
        ImGui::SliderFloat("Softening (epsilon)", &cfg.softening, nbody::constants::softening_min,
                           nbody::constants::softening_max, "%.2e");
        ImGui::SliderFloat("Velocity Cap", &cfg.max_speed, nbody::constants::velocity_cap_min,
                           nbody::constants::velocity_cap_max, "%.0f");
        if (ImGui::Button("Zero Net Momentum (Z)")) Physics::zero_net_momentum(w);
        ImGui::End();
    }

    static void draw_visuals_panel(Config& cfg) {
        ImGui::SetNextWindowPos(ImVec2(12, 280), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Visuals");
        ImGui::Checkbox("Trails", &cfg.draw_trails);
        ImGui::SameLine();
        ImGui::Checkbox("Velocity Vectors", &cfg.draw_velocity);
        ImGui::SameLine();
        ImGui::Checkbox("Acceleration Vectors", &cfg.draw_acceleration);
        ImGui::SliderInt("Trail Length", &cfg.trail_max, 0, nbody::constants::trail_length_max);
        ImGui::SliderFloat("Radius Scale", &cfg.radius_scale, nbody::constants::radius_scale_min,
                           nbody::constants::radius_scale_max, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::End();
    }

    static void draw_add_edit_panel(const flecs::world& w, raylib::Camera2D& cam, float& drag_vel_scale) {
        static float spawn_mass = static_cast<float>(nbody::constants::seed_small_mass);
        static raylib::Vector2 spawn_vel{0, 0};
        static bool spawn_pinned = false;
        if (s_pending_reset_inputs) {
            spawn_mass = static_cast<float>(nbody::constants::seed_small_mass);
            spawn_vel = raylib::Vector2{0.0f, 0.0f};
            spawn_pinned = false;
            drag_vel_scale = nbody::constants::drag_vel_scale;
            s_pending_reset_inputs = false;
        }
        ImGui::SetNextWindowPos(ImVec2(12, 420), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Add / Edit");
        ImGui::SliderFloat("Spawn Mass", &spawn_mass, nbody::constants::spawn_mass_min, nbody::constants::spawn_mass_max,
                            "%.2e", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat2("Spawn Velocity", &spawn_vel.x, nbody::constants::spawn_vel_min, nbody::constants::spawn_vel_max,
                            "%.1f");
        ImGui::Checkbox("Spawn Pinned", &spawn_pinned);
        if (ImGui::Button("Add Body At Mouse")) {
            const raylib::Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), cam);
            w.entity()
                .set<Position>({dvec2(mouseWorld)})
                .set<Velocity>({dvec2(spawn_vel)})
                .set<Acceleration>({DVec2{0.0, 0.0}})
                .set<PrevAcceleration>({DVec2{0.0, 0.0}})
                .set<Mass>({std::max(nbody::constants::spawn_mass_min, spawn_mass)})
                .set<Pinned>({spawn_pinned})
                .set<Tint>({random_nice_color()})
                .set<Trail>({{}})
                .add<Selectable>()
                .set<Draggable>({true, drag_vel_scale});
        }
        ImGui::SliderFloat("Right-Drag Sensitivity", &drag_vel_scale, nbody::constants::drag_vel_scale_min,
                           nbody::constants::drag_vel_scale_max, "%.3f", ImGuiSliderFlags_Logarithmic);

        if (flecs::entity selected = Interaction::get_selected(w); selected.is_alive()) {
            const auto mass = selected.get_mut<Mass>();
            const auto vel = selected.get_mut<Velocity>();
            const auto pin = selected.get_mut<Pinned>();
            if (ImGui::CollapsingHeader("Selected Body", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("Entity: %lld", static_cast<long long>(selected.id()));
                ImGui::Checkbox("Pinned", &pin->value);
                if (ImGui::SliderFloat("Mass", &mass->value, nbody::constants::selected_mass_min,
                                       nbody::constants::selected_mass_max, "%.2e", ImGuiSliderFlags_Logarithmic)) {
                    if (auto* r = selected.get_mut<Radius>()) {
                        const double safeMass = std::max(1.0, static_cast<double>(mass->value));
                        r->value = std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::body_density));
                    }
                }
                float velTmp[2] = {static_cast<float>(vel->value.x), static_cast<float>(vel->value.y)};
                if (ImGui::SliderFloat2("Velocity", velTmp, nbody::constants::selected_vel_min,
                                        nbody::constants::selected_vel_max, "%.1f")) {
                    vel->value.x = static_cast<double>(velTmp[0]);
                    vel->value.y = static_cast<double>(velTmp[1]);
                }
                if (ImGui::Button("Zero Velocity")) vel->value = DVec2{0.0, 0.0};
                ImGui::SameLine();
                if (ImGui::Button("Remove Body")) {
                    selected.destruct();
                    Interaction::select(w, flecs::entity::null());
                }
                ImGui::SameLine();
                if (ImGui::Button("Focus Camera")) {
                    if (const auto p = selected.get<Position>()) cam.target = raylib::Vector2{static_cast<float>(p->value.x), static_cast<float>(p->value.y)};
                }
            }
        }
        ImGui::End();
    }

    static void draw_bodies_panel(const flecs::world& w, float drag_vel_scale, flecs::entity& pendingSelection) {
        ImGui::SetNextWindowPos(ImVec2(400, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 360), ImGuiCond_FirstUseEver);
        ImGui::Begin("Bodies");
        pendingSelection = flecs::entity::null();

        const float footer_h = ImGui::GetFrameHeightWithSpacing();
        if (ImGui::BeginChild("##BodyList", ImVec2(0, -footer_h), true)) {
            std::vector<flecs::entity> entities;
            w.each([&](const flecs::entity e, const Position&, const Mass&, const Tint&, const Selectable&) {
                entities.push_back(e);
            });
            std::sort(entities.begin(), entities.end(),
                      [](const flecs::entity& a, const flecs::entity& b) { return a.id() < b.id(); });

            for (auto e : entities) {
                const auto* p = e.get<Position>();
                const auto* m = e.get<Mass>();
                const auto* t = e.get<Tint>();
                if (!p || !m || !t) continue;
                const bool isSel =
                    (Interaction::get_selected(w).is_alive() && Interaction::get_selected(w).id() == e.id());
                ImGui::PushID(static_cast<int>(e.id()));
                ImGui::ColorButton("##c",
                                   ImVec4(t->value.r / static_cast<float>(nbody::constants::random_color_max),
                                          t->value.g / static_cast<float>(nbody::constants::random_color_max),
                                          t->value.b / static_cast<float>(nbody::constants::random_color_max), 1.0f),
                                   0, ImVec2(16, 16));
                ImGui::SameLine();
                if (ImGui::Selectable(("Entity " + std::to_string(e.id())).c_str(), isSel)) pendingSelection = e;
                ImGui::SameLine();
                ImGui::Text("pos(%.2e, %.2e) m=%.2e", p->value.x, p->value.y, m->value);
                ImGui::PopID();
            }
        }
        ImGui::EndChild();

        if (ImGui::Button("Duplicate Selected")) {
            if (flecs::entity selected = Interaction::get_selected(w); selected.is_alive()) {
                const auto e = selected;
                if (const auto p0 = e.get<Position>();
                    p0 && e.get<Velocity>() && e.get<Mass>() && e.get<Tint>() && e.get<Pinned>()) {
                    auto p = *p0;
                    auto v = *e.get<Velocity>();
                    auto m = *e.get<Mass>();
                    auto t = *e.get<Tint>();
                    auto pin = *e.get<Pinned>();
                    p.value.x += static_cast<double>(nbody::constants::duplicate_offset_x);
                    w.entity()
                        .set(p)
                        .set(v)
                        .set(Acceleration{DVec2{0.0, 0.0}})
                        .set(PrevAcceleration{DVec2{0.0, 0.0}})
                        .set(m)
                        .set(pin)
                        .set(t)
                        .set(Trail{{}})
                        .add<Selectable>()
                        .set<Draggable>({true, drag_vel_scale});
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Recenter to COM")) {
            nbody::Camera::center_on_center_of_mass(w);
        }
        ImGui::End();
    }

    static void draw_diagnostics_panel(const flecs::world& w, const Config& cfg) {
        Physics::Diagnostics d{};
        const bool okDiag = Physics::compute_diagnostics(
            w, cfg.g, static_cast<double>(cfg.softening) * static_cast<double>(cfg.softening), d);
        ImGui::SetNextWindowPos(ImVec2(400, 390), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Diagnostics");
        ImGui::Text("Kinetic: %.6g", d.kinetic);
        ImGui::Text("Potential: %.6g", d.potential);
        ImGui::Text("Total: %.6g", d.energy);
        ImGui::Text("Momentum: (%.6g, %.6g)", d.momentum.x, d.momentum.y);
        ImGui::Text("COM: (%.3f, %.3f)  Mass: %.3f", d.com.x, (d.totalMass > 0.0) ? d.com.y : 0.0, d.totalMass);
        if (!okDiag) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Non-finite diagnostics detected; auto-paused.");
        ImGui::End();
    }

    static void draw_scenarios_panel(const flecs::world& w) {
        ImGui::SetNextWindowPos(ImVec2(800, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(460, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Scenarios");

        // Ensure store exists
        auto* store = w.get_mut<ScenarioStore>();
        if (!store) {
            w.set<ScenarioStore>({});
            store = w.get_mut<ScenarioStore>();
        }

        static char nameBuf[96] = {0};
        static char descBuf[512] = {0};
        static char tagsBuf[256] = {0};
        static bool applyConfigOnLoad = true;

        ImGui::InputText("Name", nameBuf, sizeof(nameBuf));
        ImGui::InputTextMultiline("Description", descBuf, sizeof(descBuf), ImVec2(-1, 60));
        ImGui::InputText("Tags (comma-separated)", tagsBuf, sizeof(tagsBuf));
        if (ImGui::Button("Save New")) {
            const std::string name = (nameBuf[0] != '\0') ? std::string(nameBuf) : std::string("Untitled");
            const std::string desc = std::string(descBuf);
            Scenario s = snapshot_from_world(w, name, desc);
            // Parse tags
            std::string tagsStr(tagsBuf);
            s.tags.clear();
            size_t start = 0;
            while (start < tagsStr.size()) {
                size_t comma = tagsStr.find(',', start);
                std::string t = tagsStr.substr(start, comma == std::string::npos ? std::string::npos : (comma - start));
                // trim spaces
                size_t a = t.find_first_not_of(" \t\n\r");
                size_t b = t.find_last_not_of(" \t\n\r");
                if (a != std::string::npos && b != std::string::npos) s.tags.push_back(t.substr(a, b - a + 1));
                if (comma == std::string::npos) break;
                start = comma + 1;
            }
            store->items.push_back(std::move(s));
            store->selected = static_cast<int>(store->items.size()) - 1;
        }
        ImGui::SameLine();
        const bool canSel = (store->selected >= 0 && store->selected < (int)store->items.size());
        if (ImGui::Button("Overwrite Selected") && canSel) {
            const std::string name = (nameBuf[0] != '\0') ? std::string(nameBuf) : store->items[store->selected].name;
            const std::string desc = std::string(descBuf);
            Scenario s = snapshot_from_world(w, name, desc);
            // use current tagsBuf
            s.tags.clear();
            std::string tagsStr(tagsBuf);
            size_t start = 0;
            while (start < tagsStr.size()) {
                size_t comma = tagsStr.find(',', start);
                std::string t = tagsStr.substr(start, comma == std::string::npos ? std::string::npos : (comma - start));
                size_t a = t.find_first_not_of(" \t\n\r");
                size_t b = t.find_last_not_of(" \t\n\r");
                if (a != std::string::npos && b != std::string::npos) s.tags.push_back(t.substr(a, b - a + 1));
                if (comma == std::string::npos) break;
                start = comma + 1;
            }
            store->items[store->selected] = std::move(s);
        }

        ImGui::Separator();
        static char filterBuf[96] = {0};
        ImGui::Text("Saved Scenarios (%zu)", store->items.size());
        ImGui::InputTextWithHint("##filter", "Filter by name or tag", filterBuf, sizeof(filterBuf));
        std::string filterStr(filterBuf);
        ImGui::BeginChild("##ScenarioList", ImVec2(0, 140), true);
        for (int i = 0; i < static_cast<int>(store->items.size()); ++i) {
            const bool selected = (store->selected == i);
            const auto& s = store->items[i];
            // If filter present, skip items that do not match name or any tag
            if (!filterStr.empty()) {
                bool match = s.name.find(filterStr) != std::string::npos;
                if (!match) {
                    for (const auto& t : s.tags) {
                        if (t.find(filterStr) != std::string::npos) {
                            match = true;
                            break;
                        }
                    }
                }
                if (!match) continue;
            }
            if (ImGui::Selectable((s.name + "##" + std::to_string(i)).c_str(), selected)) {
                store->selected = i;
                // Prime inputs with selected scenario's metadata
                strncpy(nameBuf, s.name.c_str(), sizeof(nameBuf));
                nameBuf[sizeof(nameBuf) - 1] = '\0';
                strncpy(descBuf, s.description.c_str(), sizeof(descBuf));
                descBuf[sizeof(descBuf) - 1] = '\0';
                // Fill tags buffer
                std::string tagLine;
                for (size_t k = 0; k < s.tags.size(); ++k) {
                    if (k) tagLine += ", ";
                    tagLine += s.tags[k];
                }
                strncpy(tagsBuf, tagLine.c_str(), sizeof(tagsBuf));
                tagsBuf[sizeof(tagsBuf) - 1] = '\0';
            }
            if (selected) {
                ImGui::TextWrapped("%s", s.description.c_str());
                // Show tags and key config summary
                if (!s.tags.empty()) {
                    ImGui::Text("Tags:");
                    ImGui::SameLine();
                    for (size_t k = 0; k < s.tags.size(); ++k) {
                        ImGui::TextDisabled("%s%s", s.tags[k].c_str(), (k + 1 < s.tags.size()) ? "," : "");
                        ImGui::SameLine();
                    }
                    ImGui::NewLine();
                }
                ImGui::Text("Bodies: %zu  G: %.2e  Soft: %.2e  dtScale: %.2e  Integrator: %d  RadScale: %.2f  Trails:%s",
                            s.bodies.size(), s.g, s.softening, s.time_scale, s.integrator, s.radius_scale,
                            s.draw_trails ? "on" : "off");
            }
        }
        ImGui::EndChild();

        const bool canAct = canSel;
        ImGui::Checkbox("Apply Config on Load", &applyConfigOnLoad);
        if (ImGui::Button("Load Selected") && canAct) {
            if (applyConfigOnLoad) {
                apply_scenario_to_world(w, store->items[store->selected]);
            } else {
                apply_scenario_bodies_only(w, store->items[store->selected]);
            }
            // Reset camera for a clean view
            Camera::reset_view(w);
        }
        ImGui::SameLine();
        if (ImGui::Button("Delete Selected") && canAct) {
            store->items.erase(store->items.begin() + store->selected);
            store->selected = -1;
        }

        ImGui::End();
    }

    // No extra bridge helpers needed when including Interaction.hpp
    static void perform_reset_scenario(const flecs::world& w, Config& cfg) {
        Physics::reset_scenario(w);
        Physics::zero_net_momentum(w);
        Interaction::select(w, flecs::entity::null());
        cfg.paused = false;
    }

    static void perform_reset_all(const flecs::world& w) {
        // Clear selection and interaction state
        Interaction::select(w, flecs::entity::null());
        w.set<Interaction::State>({});

        // Reset configuration to defaults
        w.set<Config>({});
        auto* cfg = w.get_mut<Config>();
        if (cfg) cfg->paused = false;

        // Rebuild bodies
        Physics::reset_scenario(w);
        Physics::zero_net_momentum(w);

        // Reset camera view
        if (auto* cam = Camera::get(w)) {
            Camera::init(*cam);
            if (const auto* c = w.get<Config>())
                cam->zoom = std::clamp(static_cast<float>(c->meter_to_pixel), nbody::constants::min_zoom,
                                        nbody::constants::max_zoom);
        }
        Camera::center_on_center_of_mass(w);

        // Reset UI inputs next frame
        s_pending_reset_inputs = true;
    }
};

}  // namespace nbody
