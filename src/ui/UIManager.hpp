#pragma once

#include <memory>
#include <vector>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <rlImGui.h>
#include "IUIPanel.hpp"
#include "panels/TimeControlPanel.hpp"

namespace nbody::ui {

class UIManager {
public:
    UIManager() {
        initializePanels();
    }

    void begin() {
        rlImGuiBegin();
    }

    void update(const flecs::world& world, const raylib::Camera2D& camera) {
        for (auto& panel : panels_) {
            if (panel) {
                panel->update(world, camera);
            }
        }
    }

    void end() {
        rlImGuiEnd();
    }

    void addPanel(std::unique_ptr<IUIPanel> panel) {
        if (panel) {
            panels_.push_back(std::move(panel));
        }
    }

    void removePanel(const char* name) {
        panels_.erase(
            std::remove_if(panels_.begin(), panels_.end(),
                [name](const std::unique_ptr<IUIPanel>& panel) {
                    return panel && std::string(panel->getName()) == name;
                }),
            panels_.end());
    }

    IUIPanel* getPanel(const char* name) {
        for (auto& panel : panels_) {
            if (panel && std::string(panel->getName()) == name) {
                return panel.get();
            }
        }
        return nullptr;
    }

    void showPanel(const char* name, bool show = true) {
        if (auto* panel = getPanel(name)) {
            panel->setVisible(show);
        }
    }

    void hidePanel(const char* name) {
        showPanel(name, false);
    }

    void togglePanel(const char* name) {
        if (auto* panel = getPanel(name)) {
            panel->setVisible(!panel->isVisible());
        }
    }

    size_t getPanelCount() const {
        return panels_.size();
    }

    bool isPanelVisible(const char* name) const {
        for (const auto& panel : panels_) {
            if (panel && std::string(panel->getName()) == name) {
                return panel->isVisible();
            }
        }
        return false;
    }

private:
    void initializePanels() {
        // Add default panels
        panels_.push_back(std::make_unique<panels::TimeControlPanel>());
        
        // Additional panels would be added here:
        // panels_.push_back(std::make_unique<panels::PhysicsPanel>());
        // panels_.push_back(std::make_unique<panels::VisualsPanel>());
        // panels_.push_back(std::make_unique<panels::AddEditPanel>());
        // panels_.push_back(std::make_unique<panels::BodiesPanel>());
        // panels_.push_back(std::make_unique<panels::DiagnosticsPanel>());
        // panels_.push_back(std::make_unique<panels::ScenariosPanel>());
    }

private:
    std::vector<std::unique_ptr<IUIPanel>> panels_;
};

}  // namespace nbody::ui