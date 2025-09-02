#pragma once

#include <memory>
#include <queue>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <imgui.h>
#include "ICommand.hpp"
#include "commands/AddBodyCommand.hpp"
#include "commands/SelectEntityCommand.hpp"
#include "../core/Config.hpp"
#include "../core/Math.hpp"
#include "../systems/Camera.hpp"

namespace nbody::input {

struct InputContext {
    DVec2 mouseWorldPos;
    DVec2 mouseDelta;
    bool isShiftHeld;
    bool isCtrlHeld;
    bool isAltHeld;
    bool uiBlocksMouse;
    bool uiBlocksKeyboard;
    const Config* config;
    const raylib::Camera2D* camera;
};

class InputManager {
public:
    InputManager() = default;

    void update(const flecs::world& world, const raylib::Camera2D& camera) {
        processCommands(world);
        
        InputContext context = createInputContext(world, camera);
        
        // Process keyboard input
        if (!context.uiBlocksKeyboard) {
            processKeyboardInput(world, context);
        }
        
        // Process mouse input
        if (!context.uiBlocksMouse) {
            processMouseInput(world, context);
        }
    }

private:
    InputContext createInputContext(const flecs::world& world, const raylib::Camera2D& camera) {
        InputContext context{};
        
        const raylib::Vector2 mouseScreen = GetMousePosition();
        context.mouseWorldPos = dvec2(GetScreenToWorld2D(mouseScreen, camera));
        context.mouseDelta = dvec2(GetMouseDelta());
        context.isShiftHeld = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        context.isCtrlHeld = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
        context.isAltHeld = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
        
        const ImGuiIO& io = ImGui::GetIO();
        context.uiBlocksMouse = io.WantCaptureMouse && 
            (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());
        context.uiBlocksKeyboard = io.WantCaptureKeyboard;
        
        context.config = world.get<Config>();
        context.camera = &camera;
        
        return context;
    }

    void processKeyboardInput(const flecs::world& world, const InputContext& context) {
        // Scene reset shortcuts
        if (IsKeyPressed(KEY_S)) {
            auto cmd = std::make_unique<ResetScenarioCommand>();
            commandQueue_.push(std::move(cmd));
        }
        
        if (IsKeyPressed(KEY_R)) {
            auto cmd = std::make_unique<ResetAllCommand>();
            commandQueue_.push(std::move(cmd));
        }
        
        // View shortcuts
        if (IsKeyPressed(KEY_V)) {
            auto cmd = std::make_unique<ResetViewCommand>();
            commandQueue_.push(std::move(cmd));
        }
        
        if (IsKeyPressed(KEY_C)) {
            auto cmd = std::make_unique<CenterViewCommand>();
            commandQueue_.push(std::move(cmd));
        }
        
        // Physics shortcuts
        if (IsKeyPressed(KEY_Z)) {
            auto cmd = std::make_unique<ZeroMomentumCommand>();
            commandQueue_.push(std::move(cmd));
        }
    }

    void processMouseInput(const flecs::world& world, const InputContext& context) {
        // Left mouse button - selection and adding bodies
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            handleLeftMousePress(world, context);
        }
        
        // Right mouse button - velocity drag (handled by existing Interaction system for now)
        // This could be refactored to commands as well in the future
        
        // Mouse wheel - zoom
        if (const float wheel = GetMouseWheelMove(); wheel != 0.0f) {
            auto cmd = std::make_unique<ZoomAtMouseCommand>(context.mouseWorldPos, wheel);
            commandQueue_.push(std::move(cmd));
        }
    }

    void handleLeftMousePress(const flecs::world& world, const InputContext& context) {
        // Shift+Click to add body (if enabled)
        if (context.isShiftHeld && context.config && context.config->enable_shift_click_add) {
            auto cmd = std::make_unique<commands::AddBodyCommand>(context.mouseWorldPos);
            commandQueue_.push(std::move(cmd));
        } else {
            // Regular click for selection
            auto cmd = std::make_unique<commands::SelectEntityCommand>(context.mouseWorldPos);
            commandQueue_.push(std::move(cmd));
        }
    }

    void processCommands(const flecs::world& world) {
        while (!commandQueue_.empty()) {
            auto command = std::move(commandQueue_.front());
            commandQueue_.pop();
            command->execute(world);
        }
    }

    // Forward declarations for commands that would be implemented
    class ResetScenarioCommand : public ICommand {
    public:
        void execute(const flecs::world& world) override {
            // Would call Physics::reset_scenario equivalent
        }
        const char* getName() const override { return "ResetScenario"; }
    };

    class ResetAllCommand : public ICommand {
    public:
        void execute(const flecs::world& world) override {
            // Would reset everything to defaults
        }
        const char* getName() const override { return "ResetAll"; }
    };

    class ResetViewCommand : public ICommand {
    public:
        void execute(const flecs::world& world) override {
            nbody::Camera::reset_view(world);
        }
        const char* getName() const override { return "ResetView"; }
    };

    class CenterViewCommand : public ICommand {
    public:
        void execute(const flecs::world& world) override {
            nbody::Camera::center_on_center_of_mass(world);
        }
        const char* getName() const override { return "CenterView"; }
    };

    class ZeroMomentumCommand : public ICommand {
    public:
        void execute(const flecs::world& world) override {
            // Would call Physics::zero_net_momentum equivalent
        }
        const char* getName() const override { return "ZeroMomentum"; }
    };

    class ZoomAtMouseCommand : public ICommand {
    public:
        ZoomAtMouseCommand(DVec2 mousePos, float wheelDelta) 
            : mousePos_(mousePos), wheelDelta_(wheelDelta) {}
            
        void execute(const flecs::world& world) override {
            if (auto* cam = nbody::Camera::get(world)) {
                nbody::Camera::zoom_at_mouse(*cam, wheelDelta_);
            }
        }
        const char* getName() const override { return "ZoomAtMouse"; }
        
    private:
        DVec2 mousePos_;
        float wheelDelta_;
    };

private:
    std::queue<std::unique_ptr<ICommand>> commandQueue_;
};

}  // namespace nbody::input