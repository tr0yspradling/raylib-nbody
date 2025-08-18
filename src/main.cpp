#include <algorithm>
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
#include "systems/Render.hpp"
#include "systems/UI.hpp"

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
                .add<Selectable>()  // Make all bodies selectable
                .set<Draggable>({true, nbody::constants::dragVelScale});  // Make all bodies draggable
        };

        mk({nbody::constants::seedCenterX, nbody::constants::seedCenterY}, {0.0f, 0.0f},
           nbody::constants::seedCentralMass, RED, false);
        mk({nbody::constants::seedCenterX + nbody::constants::seedOffsetX, nbody::constants::seedCenterY},
           {0.0f, nbody::constants::seedSpeed}, nbody::constants::seedSmallMass, BLUE, false);
        mk({nbody::constants::seedCenterX - nbody::constants::seedOffsetX, nbody::constants::seedCenterY},
           {0.0f, -nbody::constants::seedSpeed}, nbody::constants::seedSmallMass, GREEN, false);
    }
}  // namespace scenario

class Application {
public:
    Application() {
        SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
        // Initialize window after setting flags
        InitWindow(nbody::constants::windowWidth, nbody::constants::windowHeight, "N-Body Gravity Simulation • ECS");
        SetTargetFPS(nbody::constants::targetFps);
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
        nbody::Physics::Register(world_);
        nbody::Camera::Register(world_);
        nbody::Interaction::Register(world_);

        // Create initial scenario
        scenario::CreateInitialBodies(world_);

        // Center camera to initial COM
        nbody::Camera::CenterOnCenterOfMass(world_);
    }

    void Update() const {
        const double frameStart = GetTime();

        // Get camera and configuration
        raylib::Camera2D* camera = nbody::Camera::Get(world_);
        auto* cfg = world_.get_mut<Config>();

        if (!cfg || !camera) return;

        // UI first (this sets up ImGui state)
        nbody::UI::Begin();
        nbody::UI::Draw(world_, *camera);

        // Check if UI wants to capture mouse
        const ImGuiIO& io = ImGui::GetIO();
        const bool ui_blocks_mouse =
            io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

        // Zoom at mouse when UI not captured
        if (!ui_blocks_mouse) {
            if (const float wheel = GetMouseWheelMove(); wheel != 0.0F) {
                nbody::Camera::ZoomAtMouse(*camera, wheel);
            }
        }

        // Calculate delta time for physics
        const float dt = (cfg->useFixedDt ? cfg->fixedDt : GetFrameTime()) * std::max(0.0f, cfg->timeScale);

        // Process interaction input (mouse handling, selection, etc.)
        if (!ui_blocks_mouse) {
            nbody::Interaction::ProcessInput(world_, *camera);
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
        ClearBackground(nbody::constants::background);

        // Get camera for rendering
        raylib::Camera2D* camera = nbody::Camera::Get(world_);
        if (!camera) {
            nbody::UI::End();
            EndDrawing();
            return;
        }

        // Get configuration for rendering
        if (const auto* cfg = world_.get<Config>()) {
            // Render the physics scene
            nbody::Renderer::RenderScene(world_, *cfg, *camera);
        }

        // Render interaction overlays (selection rings, drag visuals)
        nbody::Interaction::RenderOverlay(world_, *camera);

        // Debug HUD for camera/DPI diagnostics
        RenderDebugHUD(*camera);

        // End UI frame and drawing (UI was started in Update)
        nbody::UI::End();
        EndDrawing();
    }

    static void RenderDebugHUD(const raylib::Camera2D& cam) {
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
                 io.DisplaySize.x, io.DisplaySize.y, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        DrawText(buf, 10, 10, 12, RAYWHITE);
    }
};

auto main() -> int {
    try {
        Application app;
        app.Run();
        return 0;
    } catch (const std::exception& e) {
        TraceLog(LOG_ERROR, "Exception: %s", e.what());
        return 1;
    }
}
