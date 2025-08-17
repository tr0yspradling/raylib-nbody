#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raymath.h>
#include <cmath>

#include "../components/Components.hpp"
#include "../core/Config.hpp"

namespace ecs {
    // Forward declaration to access the ECS camera
    raylib::Camera2D* get_camera(const flecs::world&);

    namespace interaction_constants {
        constexpr float pick_radius_px = 24.0F;
        constexpr float select_threshold_sq = 9.0F;
        constexpr float drag_line_width = 2.0F;
        constexpr float drag_circle_radius = 3.0F;
        constexpr float ring_extra_radius = 4.0F;
        constexpr float ring_thickness = 3.5F;
    }

    // Interaction state singleton
    struct InteractionState {
        bool isDraggingVelocity = false;
        bool isDraggingSelected = false;
        bool isPanning = false;
        float dragDistancePixels = 0.0f;
        raylib::Vector2 dragStartWorld{0, 0};
        raylib::Vector2 currentDragWorld{0, 0};
        raylib::Vector2 selectedDragOffset{0, 0};
        flecs::entity panCandidate = flecs::entity::null();
        flecs::entity hoveredEntity = flecs::entity::null();
        flecs::entity selectedEntity = flecs::entity::null();
    };

    class InteractionSystem {
    public:
        static flecs::entity FindEntityAtPosition(const flecs::world& world, const raylib::Vector2& worldPos, 
                                                float pickRadius) {
            flecs::entity best = flecs::entity::null();
            float bestDist2 = pickRadius * pickRadius;
            
            world.each([&](const flecs::entity ent, const Position& pos, const Mass& mass, const Selectable& selectable) {
                if (!selectable.canSelect) return;
                
                const raylib::Vector2 delta = worldPos - pos.value;
                const float dist2 = (delta.x * delta.x) + (delta.y * delta.y);
                const double safeMass = std::max(1.0, static_cast<double>(mass.value));
                const float bodyRadius = std::max(6.0F, static_cast<float>(std::cbrt(safeMass)));
                const float totalPickRadius = pickRadius + bodyRadius;
                
                if (dist2 <= totalPickRadius * totalPickRadius && dist2 < bestDist2) {
                    best = ent;
                    bestDist2 = dist2;
                }
            });
            
            return best;
        }

        static void SelectEntity(const flecs::world& world, flecs::entity entity) {
            auto* state = world.get_mut<InteractionState>();
            if (!state) return;

            // Clear previous selection
            if (state->selectedEntity.is_alive()) {
                state->selectedEntity.remove<Selected>();
            }

            // Set new selection
            state->selectedEntity = entity;
            if (entity.is_alive()) {
                entity.add<Selected>();
            }
        }

        static void StartVelocityDrag(const flecs::world& world, const raylib::Vector2& worldPos) {
            auto* state = world.get_mut<InteractionState>();
            auto* cfg = world.get_mut<Config>();
            if (!state || !cfg || !state->selectedEntity.is_alive()) return;

            const auto* draggable = state->selectedEntity.get<Draggable>();
            if (!draggable || !draggable->canDragVelocity) return;

            state->isDraggingVelocity = true;
            state->dragStartWorld = worldPos;
            state->currentDragWorld = worldPos;
            cfg->paused = true; // Pause simulation during velocity drag
        }

        static void UpdateVelocityDrag(const flecs::world& world, const raylib::Vector2& worldPos) {
            auto* state = world.get_mut<InteractionState>();
            if (!state || !state->isDraggingVelocity || !state->selectedEntity.is_alive()) return;

            state->currentDragWorld = worldPos;

            // Update entity velocity based on drag
            const auto* position = state->selectedEntity.get<Position>();
            const auto* draggable = state->selectedEntity.get<Draggable>();
            auto* velocity = state->selectedEntity.get_mut<Velocity>();
            
            if (position && draggable && velocity) {
                const raylib::Vector2 dragVector = worldPos - position->value;
                velocity->value = dragVector * draggable->dragScale;
            }
        }

        static void EndVelocityDrag(const flecs::world& world) {
            auto* state = world.get_mut<InteractionState>();
            if (!state) return;

            state->isDraggingVelocity = false;
        }

        static void ProcessMouseInput(const flecs::world& world, const raylib::Camera2D& camera) {
            auto* state = world.get_mut<InteractionState>();
            if (!state) return;

            const ImGuiIO& io = ImGui::GetIO();
            const bool ui_blocks_mouse = 
                io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());

            if (ui_blocks_mouse) return;

            const raylib::Vector2 mouseScreen = GetMousePosition();
            const raylib::Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
            const float pickRadius = interaction_constants::pick_radius_px / camera.zoom;

            // Left mouse button - selection, panning, and dragging selected when paused
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                flecs::entity entityAtMouse = FindEntityAtPosition(world, mouseWorld, pickRadius);

                // If paused and clicking on the selected entity, start drag-move
                if (const auto* cfg = world.get<Config>(); cfg && cfg->paused &&
                    state->selectedEntity.is_alive() && entityAtMouse.is_alive() &&
                    entityAtMouse.id() == state->selectedEntity.id()) {
                    if (const auto* pos = state->selectedEntity.get<Position>()) {
                        state->isDraggingSelected = true;
                        state->isPanning = false;
                        state->panCandidate = flecs::entity::null();
                        state->selectedDragOffset = pos->value - mouseWorld;
                    }
                } else {
                    if (entityAtMouse.is_alive()) {
                        state->panCandidate = entityAtMouse;
                        state->isPanning = false;
                    } else {
                        state->isPanning = true;
                        state->panCandidate = flecs::entity::null();
                    }
                }
                
                state->dragDistancePixels = 0.0f;
            }

            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                raylib::Vector2 mouseDelta = GetMouseDelta();
                state->dragDistancePixels += std::sqrt(mouseDelta.x * mouseDelta.x + mouseDelta.y * mouseDelta.y);
                
                if (state->isDraggingSelected && state->selectedEntity.is_alive()) {
                    // Move selected entity to follow mouse (with initial offset), only while paused
                    if (const auto* cfg = world.get<Config>(); cfg && cfg->paused) {
                        if (auto* pos = state->selectedEntity.get_mut<Position>()) {
                            pos->value = mouseWorld + state->selectedDragOffset;
                        }
                    }
                }

                if (state->isPanning) {
                    // Apply panning by moving camera target opposite to mouse delta (scaled by zoom)
                    if (raylib::Camera2D* cam = get_camera(world)) {
                        const raylib::Vector2 worldDelta = mouseDelta * (-1.0f / cam->zoom);
                        cam->target = Vector2Add(cam->target, worldDelta);
                    }
                }
            }

            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                // Finish drag-move if active
                if (state->isDraggingSelected) {
                    state->isDraggingSelected = false;
                    state->panCandidate = flecs::entity::null();
                    state->dragDistancePixels = 0.0f;
                } else if (!state->isPanning && state->panCandidate.is_alive() && 
                    state->dragDistancePixels <= std::sqrt(interaction_constants::select_threshold_sq)) {
                    SelectEntity(world, state->panCandidate);
                }
                
                state->isPanning = false;
                state->panCandidate = flecs::entity::null();
                state->dragDistancePixels = 0.0f;
            }

            // Right mouse button - velocity dragging
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                StartVelocityDrag(world, mouseWorld);
            }

            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && state->isDraggingVelocity) {
                UpdateVelocityDrag(world, mouseWorld);
            }

            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
                EndVelocityDrag(world);
            }

            // Update hover state
            state->hoveredEntity = FindEntityAtPosition(world, mouseWorld, pickRadius);
        }

        static void RenderSelectionOverlay(const flecs::world& world, const raylib::Camera2D& camera) {
            const auto* state = world.get<InteractionState>();
            if (!state) return;

            BeginMode2D(camera);

            // Render selection ring
            if (state->selectedEntity.is_alive()) {
                const auto* pos = state->selectedEntity.get<Position>();
                const auto* mass = state->selectedEntity.get<Mass>();
                
                if (pos && mass) {
                    const float bodyRadius = std::max(6.0F, static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(mass->value)))));
                    const float ringRadius = bodyRadius + interaction_constants::ring_extra_radius;
                    DrawRing(pos->value, ringRadius, ringRadius + interaction_constants::ring_thickness, 0, 360, 32, YELLOW);
                }

                // Render velocity drag visualization
                if (state->isDraggingVelocity && pos) {
                    DrawLineEx(pos->value, state->currentDragWorld, interaction_constants::drag_line_width, YELLOW);
                    DrawCircleV(state->currentDragWorld, interaction_constants::drag_circle_radius, YELLOW);
                }
            }

            // Render hover highlight
            if (state->hoveredEntity.is_alive() && state->hoveredEntity != state->selectedEntity) {
                const auto* pos = state->hoveredEntity.get<Position>();
                const auto* mass = state->hoveredEntity.get<Mass>();
                
                if (pos && mass) {
                    const float bodyRadius = std::max(6.0F, static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(mass->value)))));
                    DrawCircleLines(static_cast<int>(pos->value.x), static_cast<int>(pos->value.y), 
                                  bodyRadius + 2.0f, ColorAlpha(WHITE, 0.5f));
                }
            }

            EndMode2D();
        }
    };

    void register_interaction_system(const flecs::world& world) {
        // Initialize interaction state
        world.set<InteractionState>({});

        // No periodic systems needed; input handled from main loop
    }

    void process_interaction_input(const flecs::world& world, const raylib::Camera2D& camera) {
        InteractionSystem::ProcessMouseInput(world, camera);
    }

    void render_interaction_overlay(const flecs::world& world, const raylib::Camera2D& camera) {
        InteractionSystem::RenderSelectionOverlay(world, camera);
    }

    flecs::entity get_selected_entity(const flecs::world& world) {
        if (const auto* state = world.get<InteractionState>()) {
            return state->selectedEntity;
        }
        return flecs::entity::null();
    }

    void select_entity(const flecs::world& world, flecs::entity entity) {
        InteractionSystem::SelectEntity(world, entity);
    }

    flecs::entity get_hovered_entity(const flecs::world& world) {
        if (const auto* state = world.get<InteractionState>()) {
            return state->hoveredEntity;
        }
        return flecs::entity::null();
    }

} // namespace ecs
