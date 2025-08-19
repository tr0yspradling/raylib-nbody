#include <algorithm>
#include <array>
#include <cmath>
#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>

#include "components/Components.hpp"
#include "core/Config.hpp"
#include "core/Constants.hpp"

// New header-only systems
#include "systems/Camera.hpp"
#include "systems/Interaction.hpp"
#include "systems/Physics.hpp"
#include "systems/UI.hpp"
#include "systems/WorldRenderer.hpp"

namespace scenario {
void create_initial_bodies(const flecs::world& world) {
    // Create entities with both original and new interaction components
    auto makeBody = [&](const raylib::Vector2 pos, const raylib::Vector2 vel, const float mass, const raylib::Color col,
                        const bool pinned) {
        world.entity()
            .set<Position>({dvec2(pos)})
            .set<Velocity>({dvec2(vel)})
            .set<Acceleration>({DVec2{0.0, 0.0}})
            .set<PrevAcceleration>({DVec2{0.0, 0.0}})
            .set<Mass>({mass})
            .set<Pinned>({pinned})
            .set<Tint>({col})
            .set<Trail>({{}})
            .add<Selectable>()  // Make all bodies selectable
            .set<Draggable>({.can_drag_velocity = true, .drag_scale = nbody::constants::drag_vel_scale});  // Make all bodies draggable
    };

    makeBody({static_cast<float>(nbody::constants::seed_center_x), static_cast<float>(nbody::constants::seed_center_y)},
             {0.0F, 0.0F}, static_cast<float>(nbody::constants::seed_central_mass), RED, false);

    const auto* cfg = world.get<Config>();
    const double radius = nbody::constants::seed_offset_x;
    const float orbitalSpeed =
        (cfg != nullptr) ? static_cast<float>(std::sqrt(cfg->g * nbody::constants::seed_central_mass / radius)) : 0.0F;

    makeBody({static_cast<float>(nbody::constants::seed_center_x + nbody::constants::seed_offset_x),
              static_cast<float>(nbody::constants::seed_center_y)},
             {0.0F, orbitalSpeed}, static_cast<float>(nbody::constants::seed_small_mass), BLUE, false);
    makeBody({static_cast<float>(nbody::constants::seed_center_x - nbody::constants::seed_offset_x),
              static_cast<float>(nbody::constants::seed_center_y)},
             {0.0F, -orbitalSpeed}, static_cast<float>(nbody::constants::seed_small_mass), GREEN, false);
}
}  // namespace scenario

class Application {
public:
    Application() {
        SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
        // Initialize window after setting flags
        InitWindow(nbody::constants::window_width, nbody::constants::window_height, "N-Body Gravity Simulation • ECS");
        SetTargetFPS(nbody::constants::target_fps);
        rlImGuiSetup(true);

        initialize_world();
    }

    ~Application() {
        rlImGuiShutdown();
        CloseWindow();
    }

    void run() {
        while (!WindowShouldClose()) {
            update();
            render();
        }
    }

private:
    flecs::world world_;

    void initialize_world() const {
        // Initialize singleton components
        world_.set<Config>({});

        // Register all systems
        nbody::Physics::register_systems(world_);
        nbody::Camera::register_systems(world_);
        nbody::Interaction::register_systems(world_);

        // Create initial scenario
        scenario::create_initial_bodies(world_);

        // Center camera to initial COM
        nbody::Camera::center_on_center_of_mass(world_);
    }

    void update() const {
        const double frameStart = GetTime();

        // Get camera and configuration
        raylib::Camera2D* camera = nbody::Camera::get(world_);
        auto* cfg = world_.get_mut<Config>();

        if (cfg == nullptr || camera == nullptr) {
            return;
        }

        // UI first (this sets up ImGui state)
        nbody::UI::begin();
        nbody::UI::draw(world_, *camera);

        // Check if UI wants to capture mouse
        const ImGuiIO& imguiIO = ImGui::GetIO();
        const bool ui_blocks_mouse = imguiIO.WantCaptureMouse &&
            (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

        // Zoom at mouse when UI not captured
        if (!ui_blocks_mouse) {
            if (const float wheel = GetMouseWheelMove(); wheel != 0.0F) {
                nbody::Camera::zoom_at_mouse(*camera, wheel);
            }
        }

        // Calculate unscaled delta time for physics; Physics system applies timeScale
        const float deltaTime = (cfg->use_fixed_dt ? cfg->fixed_dt : GetFrameTime());

        // Process interaction input every frame so it can always
        // detect right-button release even if UI captures the mouse.
        // Internally, it early-returns for most actions when UI blocks.
        nbody::Interaction::process_input(world_, *camera);

        // Progress ECS world (runs physics and other systems)
        if (!cfg->paused) {
            [[maybe_unused]] auto progress = world_.progress(deltaTime);
        }

        // Track frame timing
        constexpr double kMsPerSec = 1000.0;
        cfg->last_step_ms = (GetTime() - frameStart) * kMsPerSec;
    }

    void render() {
        BeginDrawing();
        ClearBackground(nbody::constants::background);

        // Get camera for rendering
        raylib::Camera2D* camera = nbody::Camera::get(world_);
        if (camera == nullptr) {
            nbody::UI::end();
            EndDrawing();
            return;
        }

        // Get configuration for rendering
        if (const auto* cfg = world_.get<Config>()) {
            // Render the physics scene
            nbody::systems::WorldRenderer::render_scene(world_, *cfg, *camera);
        }

        // Render interaction overlays (selection rings, drag visuals)
        nbody::Interaction::render_overlay(world_, *camera);

        // Debug HUD for camera/DPI diagnostics
        render_debug_hud(*camera);

        // End UI frame and drawing (UI was started in Update)
        nbody::UI::end();
        EndDrawing();
    }

    static void render_debug_hud(const raylib::Camera2D& cam) {
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
                 screenW, screenH, renderW, renderH, x, y, cam.zoom, cam.offset.x, cam.offset.y, cam.target.x,
                 cam.target.y, imguiIO.DisplaySize.x, imguiIO.DisplaySize.y, imguiIO.DisplayFramebufferScale.x,
                 imguiIO.DisplayFramebufferScale.y);
        constexpr int kHudX = 10;
        constexpr int kHudY = 10;
        constexpr int kHudFont = 12;
        DrawText(buf.data(), kHudX, kHudY, kHudFont, RAYWHITE);
    }
};

auto main() -> int {
    try {
        Application app;
        app.run();
        return 0;
    } catch (const std::exception& e) {
        TraceLog(LOG_ERROR, "Exception: %s", e.what());
        return 1;
    }
}
