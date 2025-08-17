#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Camera.hpp"
#include "../core/Config.hpp"

namespace ecs {

    // Input state components
    struct InputState {
        bool draggingVel = false;
        raylib::Vector2 dragWorld{0, 0};
        bool showDrag = false;
        bool lmbPanning = false;
        flecs::entity lmbPickCandidate = flecs::entity::null();
        float lmbDragDistSq = 0.0f;
    };

    // Constants for input handling
    namespace input_constants {
        constexpr float pick_radius_px = 24.0F;
        constexpr float select_threshold_sq = 9.0F;
        constexpr float drag_vel_scale = 0.01F;
        constexpr float drag_line_width = 2.0F;
        constexpr float drag_circle_radius = 3.0F;
        constexpr float ring_extra_radius = 4.0F;
        constexpr float ring_thickness = 3.5F;
    }

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

    class InputSystem {
    public:
        static void ProcessMouseInput(const flecs::world& world, raylib::Camera2D& cam) {
            auto* cfg = world.get_mut<Config>();
            auto* sel = world.get_mut<Selection>();
            auto* inputState = world.get_mut<InputState>();
            
            const ImGuiIO& io = ImGui::GetIO();
            const bool ui_blocks_mouse = 
                io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

            // Zoom at mouse when UI not captured
            if (const float wheel = GetMouseWheelMove(); !ui_blocks_mouse && wheel != 0.0F) {
                ZoomCameraAtMouse(cam, wheel);
            }

            // Pan/select logic
            if (!ui_blocks_mouse) {
                ProcessPanAndSelect(world, cam, *inputState, *sel);
            }

            // Right-drag to set velocity of selected
            ProcessVelocityDrag(world, cam, *inputState, *sel, *cfg, ui_blocks_mouse);
        }

        static void RenderInputOverlay(const flecs::world& world, const raylib::Camera2D& cam) {
            const auto* sel = world.get<Selection>();
            const auto* inputState = world.get<InputState>();

            if (!sel || !inputState) return;

            BeginMode2D(cam);
            
            // Render selection ring
            if (flecs::entity selected = (sel->id != 0 ? world.entity(sel->id) : flecs::entity::null()); 
                selected.is_alive()) {
                const auto p = selected.get<Position>();
                if (const auto m = selected.get<Mass>(); p != nullptr && m != nullptr) {
                    const float r = static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(m->value)))) +
                        input_constants::ring_extra_radius;
                    DrawRing(p->value, r, r + input_constants::ring_thickness, 0, 360, 32, YELLOW);
                }

                // Render velocity drag visual
                if (inputState->showDrag) {
                    if (p != nullptr) {
                        DrawLineEx(p->value, inputState->dragWorld, input_constants::drag_line_width, YELLOW);
                        DrawCircleV(inputState->dragWorld, input_constants::drag_circle_radius, YELLOW);
                    }
                }
            }
            
            EndMode2D();
        }

    private:
        static void ProcessPanAndSelect(const flecs::world& world, raylib::Camera2D& cam, 
                                      InputState& inputState, Selection& sel) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                raylib::Vector2 wpos = GetScreenToWorld2D(GetMousePosition(), cam);
                inputState.lmbPickCandidate = PickEntity(world, wpos, input_constants::pick_radius_px / cam.zoom);
                inputState.lmbPanning = !inputState.lmbPickCandidate.is_alive();
                inputState.lmbDragDistSq = 0.0f;
            }
            
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                raylib::Vector2 dpx = GetMouseDelta();
                inputState.lmbDragDistSq += dpx.x * dpx.x + dpx.y * dpx.y;
                if (inputState.lmbPanning) {
                    const raylib::Vector2 delta = dpx * (-1.0f / cam.zoom);
                    cam.target = Vector2Add(cam.target, delta);
                }
            }
            
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                if (!inputState.lmbPanning && inputState.lmbPickCandidate.is_alive() && 
                    inputState.lmbDragDistSq <= input_constants::select_threshold_sq) {
                    sel.id = inputState.lmbPickCandidate.id();
                }
                inputState.lmbPanning = false;
                inputState.lmbPickCandidate = flecs::entity::null();
                inputState.lmbDragDistSq = 0.0f;
            }
        }

        static void ProcessVelocityDrag(const flecs::world& world, const raylib::Camera2D& cam,
                                      InputState& inputState, const Selection& sel, Config& cfg, bool ui_blocks_mouse) {
            inputState.showDrag = false;
            flecs::entity selected = (sel.id != 0 ? world.entity(sel.id) : flecs::entity::null());
            
            if (!ui_blocks_mouse && selected.is_alive()) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                    inputState.draggingVel = true;
                }
                
                if (inputState.draggingVel && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                    inputState.dragWorld = GetScreenToWorld2D(GetMousePosition(), cam);
                    inputState.showDrag = true;
                    cfg.paused = true;
                    
                    const auto p = selected.get<Position>();
                    if (const auto v = selected.get_mut<Velocity>(); p != nullptr && v != nullptr) {
                        v->value = (inputState.dragWorld - p->value) * input_constants::drag_vel_scale;
                    }
                }
                
                if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
                    inputState.draggingVel = false;
                }
            }
        }
    };

    void register_input_system(const flecs::world& world) {
        // Register InputState as singleton
        world.set<InputState>({});

        // Input processing system (runs every frame)
        world.system<>().kind(flecs::PreUpdate).iter([&](flecs::iter&) {
            // Input system is now handled directly in the main loop
            // This system is kept for future expansion
        });
    }

    void render_input_overlay(const flecs::world& world, const raylib::Camera2D& cam) {
        InputSystem::RenderInputOverlay(world, cam);
    }

} // namespace ecs