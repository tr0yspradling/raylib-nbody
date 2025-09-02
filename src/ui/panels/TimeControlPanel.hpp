#pragma once

#include "../IUIPanel.hpp"
#include "../../core/Config.hpp"
#include "../../core/Constants.hpp"
#include "../../systems/Camera.hpp"
#include <imgui.h>

namespace nbody::ui::panels {

class TimeControlPanel : public IUIPanel {
public:
    TimeControlPanel() = default;

    void update(const flecs::world& world, const raylib::Camera2D& camera) override {
        if (!isVisible()) return;

        auto* cfg = world.get_mut<Config>();
        if (!cfg) return;

        ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
        
        if (ImGui::Begin("Time & Integrator", &visible_)) {
            drawTimeControls(*cfg);
            drawViewControls(world);
            drawResetControls(world, *cfg);
            drawIntegratorSettings(*cfg);
            drawAdvancedSettings(*cfg);
            drawPerformanceInfo(*cfg);
        }
        ImGui::End();
    }

    bool isVisible() const override {
        return visible_;
    }

    void setVisible(bool visible) override {
        visible_ = visible;
    }

    const char* getName() const override {
        return "Time & Integrator";
    }

private:
    void drawTimeControls(Config& cfg) {
        ImGui::Checkbox("Paused", &cfg.paused);
        ImGui::SameLine();
        
        if (ImGui::Button("Step")) {
            requestSingleStep_ = true;
        }
        
        ImGui::Checkbox("Use Fixed dt", &cfg.use_fixed_dt);
        ImGui::SliderFloat("Fixed dt", &cfg.fixed_dt, 
                          nbody::constants::fixed_dt_min, nbody::constants::fixed_dt_max, "%.6f");
        ImGui::SliderFloat("Time Scale", &cfg.time_scale, 
                          nbody::constants::time_scale_min, nbody::constants::time_scale_max, 
                          "%.2e", ImGuiSliderFlags_Logarithmic);
    }

    void drawViewControls(const flecs::world& world) {
        if (ImGui::Button("Reset View (V)")) {
            nbody::Camera::reset_view(world);
        }
        ImGui::SameLine();
        if (ImGui::Button("Center View (C)")) {
            nbody::Camera::center_on_center_of_mass(world);
        }
    }

    void drawResetControls([[maybe_unused]] const flecs::world& world, [[maybe_unused]] Config& cfg) {
        if (ImGui::Button("Reset Scenario (S)")) {
            // This would need to be handled by a command or event
            requestScenarioReset_ = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset ALL (R)")) {
            ImGui::OpenPopup("Confirm Reset All");
        }
        
        if (ImGui::BeginPopupModal("Confirm Reset All", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextWrapped(
                "This will reset: bodies, configuration (time scale, integrator, visuals), "
                "camera view, and UI inputs. Are you sure?");
            
            if (ImGui::Button("Reset", ImVec2(120, 0))) {
                requestFullReset_ = true;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    void drawIntegratorSettings(Config& cfg) {
        ImGui::RadioButton("Semi-Implicit Euler", &cfg.integrator, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Velocity Verlet", &cfg.integrator, 1);
    }

    void drawAdvancedSettings(Config& cfg) {
        if (ImGui::CollapsingHeader("Advanced Stability", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Max Substep (s)", &cfg.max_substep, 0.01f, 3600.0f, 
                              "%.2f", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderInt("Max Substeps / Frame", &cfg.max_substeps_per_frame, 1, 2000);
        }
    }

    void drawPerformanceInfo(const Config& cfg) {
        ImGui::Text("Last step: %.3f ms", cfg.last_step_ms);
    }

private:
    bool visible_ = true;
    [[maybe_unused]] bool requestSingleStep_ = false;
    [[maybe_unused]] bool requestScenarioReset_ = false;
    [[maybe_unused]] bool requestFullReset_ = false;
};

}  // namespace nbody::ui::panels