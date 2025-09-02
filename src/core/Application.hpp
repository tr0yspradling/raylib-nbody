#pragma once

#include <memory>
#include <flecs.h>
#include <raylib-cpp.hpp>
#include <rlImGui.h>
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "../simulation/PhysicsEngine.hpp"
#include "../simulation/IntegratorFactory.hpp"
#include "../input/InputManager.hpp"
#include "../ui/UIManager.hpp"
#include "../systems/Camera.hpp"
#include "../systems/Interaction.hpp"
#include "../systems/WorldRenderer.hpp"

namespace nbody::core {

class Application {
public:
    Application() {
        initializeWindow();
        initializeWorld();
        initializeSubsystems();
    }

    ~Application() {
        cleanup();
    }

    void run() {
        while (!WindowShouldClose()) {
            update();
            render();
        }
    }

private:
    void initializeWindow() {
        SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
        InitWindow(nbody::constants::window_width, nbody::constants::window_height, 
                  "N-Body Gravity Simulation • ECS • Refactored");
        SetTargetFPS(nbody::constants::target_fps);
        rlImGuiSetup(true);
    }

    void initializeWorld() {
        // Initialize singleton components
        world_.set<Config>({});
        
        // Create physics engine with default integrator
        auto integrator = simulation::IntegratorFactory::createDefault();
        physicsEngine_ = std::make_unique<simulation::PhysicsEngine>(std::move(integrator));
        
        // Register all systems
        physicsEngine_->registerSystems(world_);
        nbody::Camera::register_systems(world_);
        nbody::Interaction::register_systems(world_);
        
        // Create initial scenario
        createInitialBodies();
        
        // Center camera
        nbody::Camera::center_on_center_of_mass(world_);
    }

    void initializeSubsystems() {
        inputManager_ = std::make_unique<input::InputManager>();
        uiManager_ = std::make_unique<ui::UIManager>();
    }

    void createInitialBodies() {
        auto makeBody = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, 
                           const float mass, const raylib::Color col, const bool pinned) {
            world_.entity()
                .set<Position>({dvec2(pos)})
                .set<Velocity>({dvec2(vel)})
                .set<Acceleration>({DVec2{0.0, 0.0}})
                .set<PrevAcceleration>({DVec2{0.0, 0.0}})
                .set<Mass>({mass})
                .set<Pinned>({pinned})
                .set<Tint>({col})
                .set<Trail>({{}})
                .add<Selectable>()
                .set<Draggable>({.can_drag_velocity = true, 
                               .drag_scale = nbody::constants::drag_vel_scale});
        };

        // Central massive body
        makeBody({static_cast<float>(nbody::constants::seed_center_x), 
                 static_cast<float>(nbody::constants::seed_center_y)},
                {0.0F, 0.0F}, 
                static_cast<float>(nbody::constants::seed_central_mass), 
                RED, false);

        // Calculate orbital velocity
        const auto* cfg = world_.get<Config>();
        const double radius = nbody::constants::seed_offset_x;
        const float orbitalSpeed = cfg ? 
            static_cast<float>(std::sqrt(cfg->g * nbody::constants::seed_central_mass / radius)) : 0.0F;

        // Orbiting bodies
        makeBody({static_cast<float>(nbody::constants::seed_center_x + nbody::constants::seed_offset_x),
                 static_cast<float>(nbody::constants::seed_center_y)},
                {0.0F, orbitalSpeed}, 
                static_cast<float>(nbody::constants::seed_small_mass), 
                BLUE, false);
        
        makeBody({static_cast<float>(nbody::constants::seed_center_x - nbody::constants::seed_offset_x),
                 static_cast<float>(nbody::constants::seed_center_y)},
                {0.0F, -orbitalSpeed}, 
                static_cast<float>(nbody::constants::seed_small_mass), 
                GREEN, false);
    }

    void update() {
        const double frameStart = GetTime();
        
        // Get camera and configuration
        raylib::Camera2D* camera = nbody::Camera::get(world_);
        auto* cfg = world_.get_mut<Config>();
        
        if (!cfg || !camera) return;

        // Update UI first (sets up ImGui state)
        uiManager_->begin();
        uiManager_->update(world_, *camera);
        
        // Handle integrator changes
        handleIntegratorChange(*cfg);
        
        // Process input
        inputManager_->update(world_, *camera);
        
        // Process interaction input (for now, still using original system)
        nbody::Interaction::process_input(world_, *camera);
        
        // Calculate physics timestep
        const float deltaTime = cfg->use_fixed_dt ? cfg->fixed_dt : GetFrameTime();
        
        // Progress simulation
        if (!cfg->paused) {
            world_.progress(deltaTime);
        }
        
        // Track frame timing
        constexpr double kMsPerSec = 1000.0;
        cfg->last_step_ms = (GetTime() - frameStart) * kMsPerSec;
    }

    void render() {
        BeginDrawing();
        ClearBackground(nbody::constants::background);
        
        raylib::Camera2D* camera = nbody::Camera::get(world_);
        if (!camera) {
            uiManager_->end();
            EndDrawing();
            return;
        }
        
        // Render physics scene
        if (const auto* cfg = world_.get<Config>()) {
            nbody::systems::WorldRenderer::render_scene(world_, *cfg, *camera);
        }
        
        // Render interaction overlays
        nbody::Interaction::render_overlay(world_, *camera);
        
        // Debug HUD
        renderDebugHud(*camera);
        
        // End UI and drawing
        uiManager_->end();
        EndDrawing();
    }

    void handleIntegratorChange(const Config& cfg) {
        if (physicsEngine_) {
            auto currentIntegrator = physicsEngine_->getIntegrator();
            if (!currentIntegrator || currentIntegrator->getId() != cfg.integrator) {
                auto newIntegrator = simulation::IntegratorFactory::create(cfg.integrator);
                physicsEngine_->setIntegrator(std::move(newIntegrator));
            }
        }
    }

    void renderDebugHud(const raylib::Camera2D& cam) {
        auto [x, y] = GetWindowScaleDPI();
        const int screenW = GetScreenWidth();
        const int screenH = GetScreenHeight();
        const int renderW = GetRenderWidth();
        const int renderH = GetRenderHeight();
        const ImGuiIO& imguiIO = ImGui::GetIO();

        std::array<char, 256> buf{};
        snprintf(buf.data(), buf.size(),
                 "SWxSH=%dx%d RWxRH=%dx%d DPI=(%.2f,%.2f) cam.zoom=%.3f off=(%.1f,%.1f) tgt=(%.1f,%.1f) "
                 "io.Display=(%.0f,%.0f) FBScale=(%.2f,%.2f)",
                 screenW, screenH, renderW, renderH, x, y, cam.zoom, cam.offset.x, cam.offset.y, 
                 cam.target.x, cam.target.y, imguiIO.DisplaySize.x, imguiIO.DisplaySize.y, 
                 imguiIO.DisplayFramebufferScale.x, imguiIO.DisplayFramebufferScale.y);
        
        constexpr int kHudX = 10;
        constexpr int kHudY = 10;
        constexpr int kHudFont = 12;
        DrawText(buf.data(), kHudX, kHudY, kHudFont, RAYWHITE);
    }

    void cleanup() {
        rlImGuiShutdown();
        CloseWindow();
    }

private:
    flecs::world world_;
    std::unique_ptr<simulation::PhysicsEngine> physicsEngine_;
    std::unique_ptr<input::InputManager> inputManager_;
    std::unique_ptr<ui::UIManager> uiManager_;
};

}  // namespace nbody::core