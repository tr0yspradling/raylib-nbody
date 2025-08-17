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
}  // namespace constants

namespace ecs {
    void register_physics_systems(const flecs::world&);
    void render_scene(const flecs::world&, const Config&, raylib::Camera2D&);
    void draw_ui(const flecs::world&, raylib::Camera2D&);
    void ui_begin();
    void ui_end();
}  // namespace ecs

namespace ecs {
    // Utility: pick the nearest entity within a radius
    static flecs::entity PickEntity(const flecs::world& world, const raylib::Vector2& worldPos, const float radius) {
        flecs::entity best = flecs::entity::null();
        float bestDist2 = radius * radius;
        world.each([&](const flecs::entity ent, const Position& pos, const Mass& mass) {
            const raylib::Vector2 delta = worldPos - pos.value;
            const float dist2 = (delta.x * delta.x) + (delta.y * delta.y);
            const double safeMass = std::max(1.0, static_cast<double>(mass.value));
            const float bodyRadius = std::max(6.0F, static_cast<float>(std::cbrt(safeMass)));
            if (const float pickRadius = (radius + bodyRadius); dist2 <= pickRadius * pickRadius && dist2 < bestDist2) {
                best = ent;
                bestDist2 = dist2;
            }
        });
        return best;
    }
}  // namespace ecs

auto main() -> int {
    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT);
    raylib::Window window(constants::window_width, constants::window_height, "N-Body Gravity Simulation • ECS");
    SetTargetFPS(constants::target_fps);
    rlImGuiSetup(true);

    // Camera
    raylib::Camera2D cam{};
    InitCamera(cam);

    // World + systems
    flecs::world world;
    world.set<Config>({});
    world.set<Selection>({0});
    ecs::register_physics_systems(world);

    // Initial scenario: inline seed (3 bodies)
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
            .set<Trail>({{}});
    };
    mk({constants::seed_center_x, constants::seed_center_y}, {0.0f, 0.0f}, constants::seed_central_mass, RED, false);
    mk({constants::seed_center_x + constants::seed_offset_x, constants::seed_center_y}, {0.0f, constants::seed_speed},
       constants::seed_small_mass, BLUE, false);
    mk({constants::seed_center_x - constants::seed_offset_x, constants::seed_center_y}, {0.0f, -constants::seed_speed},
       constants::seed_small_mass, GREEN, false);

    // Center camera to initial COM for robust centering regardless of DPI
    {
        double Px = 0, Py = 0, M = 0, Cx = 0, Cy = 0;
        world.each([&](const Position& p, const Mass& m) {
            Px += static_cast<double>(m.value) * 0.0;  // not used
            Py += static_cast<double>(m.value) * 0.0;  // not used
            Cx += static_cast<double>(m.value) * static_cast<double>(p.value.x);
            Cy += static_cast<double>(m.value) * static_cast<double>(p.value.y);
            M += static_cast<double>(m.value);
        });
        if (M > 0.0) cam.target = {static_cast<float>(Cx / M), static_cast<float>(Cy / M)};
    }

    // Input/drag state
    bool draggingVel = false;
    raylib::Vector2 dragWorld{0, 0};
    bool showDrag = false;
    constexpr float dragVelScale = constants::drag_vel_scale;
    bool lmbPanning = false;
    flecs::entity lmbPickCandidate = flecs::entity::null();
    float lmbDragDistSq = 0.0f;
    constexpr float selectThresholdSq = constants::select_threshold_sq;

    while (!raylib::Window::ShouldClose()) {
        const double frameStart = GetTime();

        BeginDrawing();
        ClearBackground(constants::background);

        // UI & controls
        ecs::ui_begin();
        ecs::draw_ui(world, cam);

        auto* cfg = world.get_mut<Config>();
        auto* sel = world.get_mut<Selection>();
        const ImGuiIO& io = ImGui::GetIO();
        const bool ui_blocks_mouse =
            io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

        const float dt = (cfg->useFixedDt ? cfg->fixedDt : GetFrameTime()) * std::max(0.0f, cfg->timeScale);
        if (!cfg->paused) {
            [[maybe_unused]] const auto world_progressed = world.progress(dt);
        }

        // Zoom at mouse when UI not captured
        if (const float wheel = GetMouseWheelMove(); !ui_blocks_mouse && wheel != 0.0F) {
            ZoomCameraAtMouse(cam, wheel);
        }

        // Pan/select
        if (!ui_blocks_mouse) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                raylib::Vector2 wpos = GetScreenToWorld2D(GetMousePosition(), cam);
                lmbPickCandidate = ecs::PickEntity(world, wpos, constants::pick_radius_px / cam.zoom);
                lmbPanning = !lmbPickCandidate.is_alive();
                lmbDragDistSq = 0.0f;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                raylib::Vector2 dpx = GetMouseDelta();
                lmbDragDistSq += dpx.x * dpx.x + dpx.y * dpx.y;
                if (lmbPanning) {
                    const raylib::Vector2 delta = dpx * (-1.0f / cam.zoom);
                    cam.target = Vector2Add(cam.target, delta);
                }
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                if (!lmbPanning && lmbPickCandidate.is_alive() && lmbDragDistSq <= selectThresholdSq) {
                    sel->id = lmbPickCandidate.id();
                }
                lmbPanning = false;
                lmbPickCandidate = flecs::entity::null();
                lmbDragDistSq = 0.0f;
            }
        }

        // Right-drag to set velocity of selected
        showDrag = false;
        flecs::entity selected = (sel->id != 0 ? world.entity(sel->id) : flecs::entity::null());
        if (!ui_blocks_mouse && selected.is_alive()) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                draggingVel = true;
            }
            if (draggingVel && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                dragWorld = GetScreenToWorld2D(GetMousePosition(), cam);
                showDrag = true;
                cfg->paused = true;
                const auto p = selected.get<Position>();
                if (const auto v = selected.get_mut<Velocity>(); p != nullptr && v != nullptr) {
                    v->value = (dragWorld - p->value) * dragVelScale;
                }
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
                draggingVel = false;
            }
        }

        // Render scene
        ecs::render_scene(world, *cfg, cam);

        // Overlay selection and velocity-drag visuals (draw in world-space)
        BeginMode2D(cam);
        if (selected.is_alive()) {
            const auto p = selected.get<Position>();
            if (const auto m = selected.get<Mass>(); p != nullptr && m != nullptr) {
                const float r = static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(m->value)))) +
                    constants::ring_extra_radius;
                DrawRing(p->value, r, r + constants::ring_thickness, 0, 360, 32, YELLOW);
            }
        }
        if (showDrag && selected.is_alive()) {
            if (const auto p = selected.get<Position>(); p != nullptr) {
                DrawLineEx(p->value, dragWorld, constants::drag_line_width, YELLOW);
                DrawCircleV(dragWorld, constants::drag_circle_radius, YELLOW);
            }
        }
        EndMode2D();

        // Debug HUD for DPI/camera diagnostics (top-left)
        {
            auto [x, y] = GetWindowScaleDPI();
            const int sw = GetScreenWidth();
            const int sh = GetScreenHeight();
            const int rw = GetRenderWidth();
            const int rh = GetRenderHeight();
            const ImGuiIO& io2 = ImGui::GetIO();
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "SWxSH=%dx%d RWxRH=%dx%d DPI=(%.2f,%.2f) cam.zoom=%.3f off=(%.1f,%.1f) tgt=(%.1f,%.1f) "
                     "io.Display=(%.0f,%.0f) FBScale=(%.2f,%.2f)",
                     sw, sh, rw, rh, x, y, cam.zoom, cam.offset.x, cam.offset.y, cam.target.x, cam.target.y,
                     io2.DisplaySize.x, io2.DisplaySize.y, io2.DisplayFramebufferScale.x,
                     io2.DisplayFramebufferScale.y);
            DrawText(buf, 10, 10, 12, RAYWHITE);
        }

        // End frame
        cfg->lastStepMs = (GetTime() - frameStart) * 1000.0;
        ecs::ui_end();
        EndDrawing();
    }

    rlImGuiShutdown();
    return 0;
}
