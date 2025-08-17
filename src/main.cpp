#include <algorithm>
#include <cmath>
#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>

#include "components/Components.hpp"
#include "core/Camera.hpp"
#include "core/Config.hpp"

// Forward declarations for system registration
namespace ecs {
    void register_physics_systems(const flecs::world&);
    void register_camera_system(const flecs::world&);
    void register_interaction_system(const flecs::world&);
    
    // System functions
    void render_scene(const flecs::world&, const Config&, raylib::Camera2D&);
    void draw_ui(const flecs::world&, raylib::Camera2D&);
    void ui_begin();
    void ui_end();
    
    // Camera system functions
    raylib::Camera2D* get_camera(const flecs::world&);
    void center_camera_on_com(const flecs::world&);
    
    // Interaction system functions  
    void process_interaction_input(const flecs::world&, const raylib::Camera2D&);
    void render_interaction_overlay(const flecs::world&, const raylib::Camera2D&);
}

namespace constants {
    // Window and timing
    constexpr int window_width = 1280;
    constexpr int window_height = 720;
    constexpr int target_fps = 120;

    // Background color
    constexpr ::Color background{10, 10, 14, 255};

    // Picking and dragging
    constexpr float pick_radius_px = 24.0F;  // screen-space pick radius before zoom scaling
    constexpr float select_threshold_sq = 9.0F;  // squared pixel drag threshold to treat as click
    constexpr float drag_vel_scale = 0.01F;  // scale from drag vector to velocity

    // Visual thickness/radii
    constexpr float drag_line_width = 2.0F;
    constexpr float drag_circle_radius = 3.0F;
    constexpr float ring_extra_radius = 4.0F;
    constexpr float ring_thickness = 3.5F;
    [[maybe_unused]] constexpr float vel_line_width = 1.5F;
    [[maybe_unused]] constexpr float acc_line_width = 1.0F;

    // Seed scenario
    constexpr float seed_small_mass = 12.0F;
    constexpr float seed_central_mass = 4000.0F;
    constexpr float seed_speed = 1.20F;
    constexpr float seed_center_x = 640.0F;
    constexpr float seed_center_y = 360.0F;
    constexpr float seed_offset_x = 200.0F;  // +/- from center for initial bodies
}

namespace scenario {
    void CreateInitialBodies(const flecs::world& world) {
        // Create entities with both original and new interaction components
        auto mk = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, const float mass, const raylib::Color col,
                      const bool pinned) {
            world.entity()
                .set<Position>({pos})
                .set<Velocity>({vel})
                .set<Acceleration>({raylib::Vector2{0, 0}})
                .set<PrevAcceleration>({raylib::Vector2{0, 0}})
                .set<Mass>({mass})
                .set<Pinned>({pinned})
                .set<Tint>({col})
                .set<Trail>({{}})
                .add<Selectable>()        // Make all bodies selectable
                .set<Draggable>({true, constants::drag_vel_scale}); // Make all bodies draggable
        };
        
        mk({constants::seed_center_x, constants::seed_center_y}, {0.0f, 0.0f}, constants::seed_central_mass, RED, false);
        mk({constants::seed_center_x + constants::seed_offset_x, constants::seed_center_y}, {0.0f, constants::seed_speed},
           constants::seed_small_mass, BLUE, false);
        mk({constants::seed_center_x - constants::seed_offset_x, constants::seed_center_y}, {0.0f, -constants::seed_speed},
           constants::seed_small_mass, GREEN, false);
    }
}

class Application {
public:
    Application() {
        SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
        // Initialize window after setting flags
        InitWindow(constants::window_width, constants::window_height, "N-Body Gravity Simulation • ECS");
        SetTargetFPS(constants::target_fps);
        rlImGuiSetup(true);
        
        InitializeWorld();
    }

    ~Application() {
        rlImGuiShutdown();
        CloseWindow();
    }

    void Run() {
        while (!WindowShouldClose()) {
            Update();
            Render();
        }
    }

private:
    flecs::world world_;

    void InitializeWorld() const {
        // Initialize singleton components
        world_.set<Config>({});

        // Register all systems
        ecs::register_physics_systems(world_);
        ecs::register_camera_system(world_);
        ecs::register_interaction_system(world_);

        // Create initial scenario
        scenario::CreateInitialBodies(world_);

        // Center camera to initial COM
        ecs::center_camera_on_com(world_);
    }

    void Update() const {
        const double frameStart = GetTime();

        // Get camera and configuration
        raylib::Camera2D* camera = ecs::get_camera(world_);
        auto* cfg = world_.get_mut<Config>();
        
        if (!cfg || !camera) return;

        // UI first (this sets up ImGui state)
        ecs::ui_begin();
        ecs::draw_ui(world_, *camera);

        // Check if UI wants to capture mouse
        const ImGuiIO& io = ImGui::GetIO();
        const bool ui_blocks_mouse = 
            io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

        // Zoom at mouse when UI not captured
        if (!ui_blocks_mouse) {
            if (const float wheel = GetMouseWheelMove(); wheel != 0.0F) {
                ZoomCameraAtMouse(*camera, wheel);
            }
        }

        // Calculate delta time for physics
        const float dt = (cfg->useFixedDt ? cfg->fixedDt : GetFrameTime()) * std::max(0.0f, cfg->timeScale);
        
        // Process interaction input (mouse handling, selection, etc.)
        if (!ui_blocks_mouse) {
            ecs::process_interaction_input(world_, *camera);
        }

        // Progress ECS world (runs physics and other systems)
        if (!cfg->paused) {
            [[maybe_unused]] auto progress = world_.progress(dt);
        }

        // Track frame timing
        cfg->lastStepMs = (GetTime() - frameStart) * 1000.0;
    }

    void Render() {
        BeginDrawing();
        ClearBackground(constants::background);

        // Get camera for rendering
        raylib::Camera2D* camera = ecs::get_camera(world_);
        if (!camera) {
            ecs::ui_end();
            EndDrawing();
            return;
        }

        // Get configuration for rendering
        const auto* cfg = world_.get<Config>();
        if (cfg) {
            // Render the physics scene
            ecs::render_scene(world_, *cfg, *camera);
        }

        // Render interaction overlays (selection rings, drag visuals)
        ecs::render_interaction_overlay(world_, *camera);

        // Debug HUD for camera/DPI diagnostics
        RenderDebugHUD(*camera);

        // End UI frame and drawing (UI was started in Update)
        ecs::ui_end();
        EndDrawing();
    }

    void RenderDebugHUD(const raylib::Camera2D& cam) {
        auto [x, y] = GetWindowScaleDPI();
        const int sw = GetScreenWidth();
        const int sh = GetScreenHeight();
        const int rw = GetRenderWidth();
        const int rh = GetRenderHeight();
        const ImGuiIO& io = ImGui::GetIO();
        
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "SWxSH=%dx%d RWxRH=%dx%d DPI=(%.2f,%.2f) cam.zoom=%.3f off=(%.1f,%.1f) tgt=(%.1f,%.1f) "
                 "io.Display=(%.0f,%.0f) FBScale=(%.2f,%.2f)",
                 sw, sh, rw, rh, x, y, cam.zoom, cam.offset.x, cam.offset.y, cam.target.x, cam.target.y,
                 io.DisplaySize.x, io.DisplaySize.y, io.DisplayFramebufferScale.x,
                 io.DisplayFramebufferScale.y);
        DrawText(buf, 10, 10, 12, RAYWHITE);
    }
};

auto main() -> int {
    try {
        Application app;
        app.Run();
        return 0;
    } catch (const std::exception& e) {
        // Log error if needed
        return 1;
    }
}
