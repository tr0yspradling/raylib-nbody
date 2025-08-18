#pragma once

#include <cmath>
#include <flecs.h>
#include <imgui.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Config.hpp"
#include "../core/Constants.hpp"
#include "Camera.hpp"

namespace nbody {

    class Interaction {
    public:
        struct State {
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

        static void Register(const flecs::world& world) { world.set<State>({}); }

        static flecs::entity GetSelected(const flecs::world& world) {
            if (const auto* s = world.get<State>()) return s->selectedEntity;
            return flecs::entity::null();
        }

        static flecs::entity GetHovered(const flecs::world& world) {
            if (const auto* s = world.get<State>()) return s->hoveredEntity;
            return flecs::entity::null();
        }

        static void Select(const flecs::world& world, flecs::entity entity) {
            auto* state = world.get_mut<State>();
            if (!state) return;
            if (state->selectedEntity.is_alive()) state->selectedEntity.remove<Selected>();
            state->selectedEntity = entity;
            if (entity.is_alive()) entity.add<Selected>();
        }

        static void ProcessInput(const flecs::world& world, const raylib::Camera2D& camera) {
            auto* state = world.get_mut<State>();
            if (!state) return;

            const ImGuiIO& io = ImGui::GetIO();
            const bool ui_blocks_mouse = io.WantCaptureMouse &&
                (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());
            if (ui_blocks_mouse) return;

            const raylib::Vector2 mouseScreen = GetMousePosition();
            const raylib::Vector2 mouseWorld = GetScreenToWorld2D(mouseScreen, camera);
            const float pickRadius = nbody::constants::pickRadiusPx / camera.zoom;
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) HandleMousePress(world, state, mouseWorld, pickRadius);

            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) HandleMouseDrag(world, state, mouseWorld);

            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) HandleMouseRelease(world, state);
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) StartVelocityDrag(world, mouseWorld);
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && state->isDraggingVelocity)
                UpdateVelocityDrag(world, mouseWorld);
            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) EndVelocityDrag(world);

            state->hoveredEntity = FindEntityAtPosition(world, mouseWorld, pickRadius);
        }

        static void RenderOverlay(const flecs::world& world, const raylib::Camera2D& camera) {
            const auto* state = world.get<State>();
            if (!state) return;
            BeginMode2D(camera);
            if (state->selectedEntity.is_alive()) {
                const auto* pos = state->selectedEntity.get<Position>();
                const auto* mass = state->selectedEntity.get<Mass>();
                if (pos && mass) {
                    const float bodyRadius =
                        std::max(nbody::constants::minBodyRadius,
                                 static_cast<float>(std::cbrt(std::max(1.0, static_cast<double>(mass->value)))));
                    const float ringRadius = bodyRadius + nbody::constants::ringExtraRadius;
                    DrawRing(pos->value, ringRadius, ringRadius + nbody::constants::ringThickness,
                             nbody::constants::ringStartAngle, nbody::constants::ringEndAngle,
                             nbody::constants::ringSegments, YELLOW);
                    DrawCircleLines(static_cast<int>(pos->value.x), static_cast<int>(pos->value.y),
                                    bodyRadius + nbody::constants::ringInnerOffset,
                                    ColorAlpha(WHITE, nbody::constants::selectedCircleAlpha));
                }
            }
            EndMode2D();
        }

    private:
        static flecs::entity FindEntityAtPosition(const flecs::world& world, const raylib::Vector2& worldPos,
                                                  float pickRadius) {
            flecs::entity best = flecs::entity::null();
            float bestDist2 = pickRadius * pickRadius;
            world.each(
                [&](const flecs::entity ent, const Position& pos, const Mass& mass, const Selectable& selectable) {
                    if (!selectable.canSelect) return;
                    const raylib::Vector2 delta = worldPos - pos.value;
                    const float dist2 = (delta.x * delta.x) + (delta.y * delta.y);
                    const double safeMass = std::max(1.0, static_cast<double>(mass.value));
                    const float bodyRadius =
                        std::max(nbody::constants::minBodyRadius, static_cast<float>(std::cbrt(safeMass)));
                    const float totalPickRadius = pickRadius + bodyRadius;
                    if (dist2 <= totalPickRadius * totalPickRadius && dist2 < bestDist2) {
                        best = ent;
                        bestDist2 = dist2;
                    }
                });
            return best;
        }

        static void HandleMousePress(const flecs::world& world, State* state, const raylib::Vector2& mouseWorld, float pickRadius) {
            flecs::entity entityAtMouse = FindEntityAtPosition(world, mouseWorld, pickRadius);
            if (const auto* cfg = world.get<Config>(); cfg && cfg->paused && state->selectedEntity.is_alive() &&
                entityAtMouse.is_alive() && entityAtMouse.id() == state->selectedEntity.id()) {
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

        static void HandleMouseDrag(const flecs::world& world, State* state, const raylib::Vector2& mouseWorld) {
            const raylib::Vector2 mouseDelta = GetMouseDelta();
            state->dragDistancePixels += Vector2Length(mouseDelta);
            if (state->isDraggingSelected && state->selectedEntity.is_alive()) {
                if (const auto* cfg = world.get<Config>(); cfg && cfg->paused) {
                    if (auto* pos = state->selectedEntity.get_mut<Position>()) {
                        pos->value = mouseWorld + state->selectedDragOffset;
                    }
                }
            }
            if (state->isPanning) {
                if (auto* cam = Camera::Get(world)) {
                    const raylib::Vector2 worldDelta = mouseDelta * (-1.0f / cam->zoom);
                    cam->target = Vector2Add(cam->target, worldDelta);
                }
            }
        }

        static void HandleMouseRelease(const flecs::world& world, State* state) {
            if (state->isDraggingSelected) {
                state->isDraggingSelected = false;
                state->panCandidate = flecs::entity::null();
                state->dragDistancePixels = 0.0f;
            } else if (!state->isPanning && state->panCandidate.is_alive() &&
                       state->dragDistancePixels * state->dragDistancePixels <= nbody::constants::selectThresholdSq) {
                Select(world, state->panCandidate);
            }
            state->isPanning = false;
            state->panCandidate = flecs::entity::null();
            state->dragDistancePixels = 0.0f;
        }

        static void StartVelocityDrag(const flecs::world& world, const raylib::Vector2& worldPos) {
            auto* state = world.get_mut<State>();
            auto* cfg = world.get_mut<Config>();
            if (!state || !cfg || !state->selectedEntity.is_alive()) return;
            const auto* draggable = state->selectedEntity.get<Draggable>();
            if (!draggable || !draggable->canDragVelocity) return;
            state->isDraggingVelocity = true;
            state->dragStartWorld = worldPos;
            state->currentDragWorld = worldPos;
            cfg->paused = true;
        }

        static void UpdateVelocityDrag(const flecs::world& world, const raylib::Vector2& worldPos) {
            auto* state = world.get_mut<State>();
            if (!state || !state->isDraggingVelocity || !state->selectedEntity.is_alive()) return;
            state->currentDragWorld = worldPos;
            const auto* position = state->selectedEntity.get<Position>();
            const auto* draggable = state->selectedEntity.get<Draggable>();
            auto* velocity = state->selectedEntity.get_mut<Velocity>();
            if (position && draggable && velocity) {
                const raylib::Vector2 dragVector = worldPos - position->value;
                velocity->value = dragVector * draggable->dragScale;
            }
        }

        static void EndVelocityDrag(const flecs::world& world) {
            if (auto* state = world.get_mut<State>()) state->isDraggingVelocity = false;
        }
    };

}  // namespace nbody
