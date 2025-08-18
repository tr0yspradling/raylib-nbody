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
#include "../core/Color.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "Camera.hpp"
#include "Interaction.hpp"
#include "Physics.hpp"

namespace nbody {

    class UI {
    public:
        static void Begin() { rlImGuiBegin(); }
        static void End() { rlImGuiEnd(); }

        static void Draw(const flecs::world& w, raylib::Camera2D& cam) {
            auto* cfg = w.get_mut<Config>();
            if (!cfg) return;
            bool requestStep = false;

            ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
            ImGui::Begin("Time & Integrator");
            ImGui::Checkbox("Paused", &cfg->paused);
            ImGui::SameLine();
            if (ImGui::Button("Step")) requestStep = true;
            ImGui::Checkbox("Use Fixed dt", &cfg->useFixedDt);
            ImGui::SliderFloat("Fixed dt", &cfg->fixedDt, nbody::constants::fixedDtMin, nbody::constants::fixedDtMax,
                               "%.6f");
            ImGui::SliderFloat("Time Scale", &cfg->timeScale, nbody::constants::timeScaleMin,
                               nbody::constants::timeScaleMax, "%.3f");
            ImGui::RadioButton("Semi-Implicit Euler", &cfg->integrator, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Velocity Verlet", &cfg->integrator, 1);
            ImGui::Text("Last step: %.3f ms", cfg->lastStepMs);
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(12, 140), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
            ImGui::Begin("Physics");
            auto Gf = static_cast<float>(cfg->G);
            ImGui::SliderFloat("G", &Gf, nbody::constants::gMin, nbody::constants::gMax, "%.6f");
            cfg->G = Gf;
            ImGui::SliderFloat("Softening (epsilon)", &cfg->softening, nbody::constants::softeningMin,
                               nbody::constants::softeningMax, "%.3f");
            ImGui::SliderFloat("Velocity Cap", &cfg->maxSpeed, nbody::constants::velocityCapMin,
                               nbody::constants::velocityCapMax, "%.1f");
            if (ImGui::Button("Zero Net Momentum")) Physics::zero_net_momentum(w);
            ImGui::SameLine();
            if (ImGui::Button("Reset Scenario")) {
                Physics::reset_scenario(w);
                Physics::zero_net_momentum(w);
                Interaction::Select(w, flecs::entity::null());
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
            ImGui::SliderInt("Trail Length", &cfg->trailMax, 0, nbody::constants::trailLengthMax);
            ImGui::End();

            static float spawnMass = nbody::constants::seedSmallMass;
            static raylib::Vector2 spawnVel{0, 0};
            static bool spawnPinned = false;
            static float dragVelScale = nbody::constants::dragVelScale;
            ImGui::SetNextWindowPos(ImVec2(12, 420), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_FirstUseEver);
            ImGui::Begin("Add / Edit");
            ImGui::SliderFloat("Spawn Mass", &spawnMass, nbody::constants::spawnMassMin, nbody::constants::spawnMassMax,
                               "%.1f");
            ImGui::SliderFloat2("Spawn Velocity", &spawnVel.x, nbody::constants::spawnVelMin,
                                nbody::constants::spawnVelMax, "%.3f");
            ImGui::Checkbox("Spawn Pinned", &spawnPinned);
            if (ImGui::Button("Add Body At Mouse")) {
                const raylib::Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), cam);
                w.entity()
                    .set<Position>({mouseWorld})
                    .set<Velocity>({spawnVel})
                    .set<Acceleration>({raylib::Vector2{0, 0}})
                    .set<PrevAcceleration>({raylib::Vector2{0, 0}})
                    .set<Mass>({std::max(nbody::constants::spawnMassMin, spawnMass)})
                    .set<Pinned>({spawnPinned})
                    .set<Tint>({RandomNiceColor()})
                    .set<Trail>({{}})
                    .add<Selectable>()
                    .set<Draggable>({true, dragVelScale});
            }
            ImGui::SliderFloat("Right-Drag Vel Scale", &dragVelScale, nbody::constants::dragVelScaleMin,
                               nbody::constants::dragVelScaleMax, "%.3f");

            if (flecs::entity selected = Interaction::GetSelected(w); selected.is_alive()) {
                const auto mass = selected.get_mut<Mass>();
                const auto vel = selected.get_mut<Velocity>();
                const auto pin = selected.get_mut<Pinned>();
                if (ImGui::CollapsingHeader("Selected Body", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Text("Entity: %lld", static_cast<long long>(selected.id()));
                    ImGui::Checkbox("Pinned", &pin->value);
                    ImGui::SliderFloat("Mass", &mass->value, nbody::constants::selectedMassMin,
                                       nbody::constants::selectedMassMax, "%.1f");
                    ImGui::SliderFloat2("Velocity", &vel->value.x, nbody::constants::selectedVelMin,
                                        nbody::constants::selectedVelMax, "%.3f");
                    if (ImGui::Button("Zero Velocity")) vel->value = raylib::Vector2{0.0f, 0.0f};
                    ImGui::SameLine();
                    if (ImGui::Button("Remove Body")) {
                        selected.destruct();
                        Interaction::Select(w, flecs::entity::null());
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Focus Camera")) {
                        if (const auto p = selected.get<Position>()) cam.target = p->value;
                    }
                }
            }
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(400, 12), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(380, 360), ImGuiCond_FirstUseEver);
            ImGui::Begin("Bodies");
            flecs::entity pendingSelection = flecs::entity::null();

            // Create a scrolling list with a fixed-height footer for action buttons.
            const float footer_h = ImGui::GetFrameHeightWithSpacing();
            if (ImGui::BeginChild("##BodyList", ImVec2(0, -footer_h), true)) {
                // Build stable-ordered list to avoid reordering issues
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
                        (Interaction::GetSelected(w).is_alive() && Interaction::GetSelected(w).id() == e.id());
                    ImGui::PushID(static_cast<int>(e.id()));
                    ImGui::ColorButton("##c",
                                       ImVec4(t->value.r / static_cast<float>(nbody::constants::randomColorMax),
                                              t->value.g / static_cast<float>(nbody::constants::randomColorMax),
                                              t->value.b / static_cast<float>(nbody::constants::randomColorMax), 1.0f),
                                       0, ImVec2(16, 16));
                    ImGui::SameLine();
                    if (ImGui::Selectable(("Entity " + std::to_string(e.id())).c_str(), isSel)) pendingSelection = e;
                    ImGui::SameLine();
                    ImGui::Text("pos(%.1f, %.1f) m=%.1f", p->value.x, p->value.y, m->value);
                    ImGui::PopID();
                }
            }
            ImGui::EndChild();

            // Footer actions (always visible at bottom)
            if (ImGui::Button("Duplicate Selected")) {
                if (flecs::entity selected = Interaction::GetSelected(w); selected.is_alive()) {
                    const auto e = selected;
                    if (const auto p0 = e.get<Position>();
                        p0 && e.get<Velocity>() && e.get<Mass>() && e.get<Tint>() && e.get<Pinned>()) {
                        auto p = *p0;
                        auto v = *e.get<Velocity>();
                        auto m = *e.get<Mass>();
                        auto t = *e.get<Tint>();
                        auto pin = *e.get<Pinned>();
                        p.value += raylib::Vector2{nbody::constants::duplicateOffsetX, 0.0f};
                        w.entity()
                            .set(p)
                            .set(v)
                            .set(Acceleration{raylib::Vector2{0, 0}})
                            .set(PrevAcceleration{raylib::Vector2{0, 0}})
                            .set(m)
                            .set(pin)
                            .set(t)
                            .set(Trail{{}})
                            .add<Selectable>()
                            .set<Draggable>({true, dragVelScale});
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Recenter to COM")) {
                nbody::Camera::CenterOnCenterOfMass(w);
            }
            ImGui::End();

            if (pendingSelection.is_alive() ||
                (!Interaction::GetSelected(w).is_alive() && pendingSelection == flecs::entity::null())) {
                Interaction::Select(w, pendingSelection);
            }

            Physics::Diagnostics d{};
            const bool okDiag = Physics::compute_diagnostics(
                w, cfg->G, static_cast<double>(cfg->softening) * static_cast<double>(cfg->softening), d);
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

            if (requestStep) {
                const bool old = cfg->paused;
                cfg->paused = false;
                w.progress(cfg->fixedDt);
                cfg->paused = old;
            }
        }
    };

}  // namespace nbody
