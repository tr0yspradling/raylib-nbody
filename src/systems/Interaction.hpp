#pragma once

#include <cmath>
#include <flecs.h>
#include <imgui.h>
#include <numbers>
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
        bool is_dragging_velocity = false;
        bool is_dragging_selected = false;
        bool is_panning = false;
        float drag_distance_pixels = 0.0f;
        DVec2 drag_start_world{0.0, 0.0};
        DVec2 current_drag_world{0.0, 0.0};
        DVec2 selected_drag_offset{0.0, 0.0};
        flecs::entity pan_candidate = flecs::entity::null();
        flecs::entity hovered_entity = flecs::entity::null();
        flecs::entity selected_entity = flecs::entity::null();
    };

    static void register_systems(const flecs::world& world) { world.set<State>({}); }

    static flecs::entity get_selected(const flecs::world& world) {
        if (const auto* s = world.get<State>()) return s->selected_entity;
        return flecs::entity::null();
    }

    static flecs::entity get_hovered(const flecs::world& world) {
        if (const auto* s = world.get<State>()) return s->hovered_entity;
        return flecs::entity::null();
    }

    static void select(const flecs::world& world, flecs::entity entity) {
        auto* state = world.get_mut<State>();
        if (!state) return;
        if (state->selected_entity.is_alive()) state->selected_entity.remove<Selected>();
        state->selected_entity = entity;
        if (entity.is_alive()) entity.add<Selected>();
    }

    static void process_input(const flecs::world& world, const raylib::Camera2D& camera) {
        auto* state = world.get_mut<State>();
        if (!state) return;

        // Always end velocity drag on right-button release, even if UI captures mouse
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) end_velocity_drag(world);

        const ImGuiIO& io = ImGui::GetIO();
        const bool ui_blocks_mouse =
            io.WantCaptureMouse && (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) || ImGui::IsAnyItemHovered());
        if (ui_blocks_mouse) return;

        const raylib::Vector2 mouseScreen = GetMousePosition();
        const DVec2 mouseWorld = dvec2(GetScreenToWorld2D(mouseScreen, camera));
        const float pickRadius = nbody::constants::pick_radius_px / camera.zoom;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) handle_mouse_press(world, state, mouseWorld, pickRadius);

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) handle_mouse_drag(world, state, mouseWorld);

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) handle_mouse_release(world, state);
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) start_velocity_drag(world, mouseWorld);
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && state->is_dragging_velocity) update_velocity_drag(world, mouseWorld);
        // Right-button release handled above to avoid stuck state when UI captures mouse

        state->hovered_entity = find_entity_at_position(world, mouseWorld, pickRadius);
    }

    static void render_overlay(const flecs::world& world, const raylib::Camera2D& camera) {
        const auto* state = world.get<State>();
        if (!state) return;
        // Defensive: if drag state somehow persisted but the right button is no longer held,
        // end the drag so the preview line disappears.
        if (state->is_dragging_velocity && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            end_velocity_drag(world);
        }
        BeginMode2D(camera);
        if (state->is_dragging_velocity) {
            const raylib::Vector2 a = fvec2(state->drag_start_world);
            const raylib::Vector2 b = fvec2(state->current_drag_world);
            DrawLineEx(a, b, nbody::constants::drag_line_width / camera.zoom, WHITE);
            DrawCircleV(a, nbody::constants::drag_circle_radius / camera.zoom, WHITE);
            DrawCircleV(b, nbody::constants::drag_circle_radius / camera.zoom, WHITE);
        }
        if (state->selected_entity.is_alive()) {
            const auto* pos = state->selected_entity.get<Position>();
            const auto* mass = state->selected_entity.get<Mass>();
            const auto* rad = state->selected_entity.get<Radius>();
            if (pos && mass) {
                double rMeters = rad ? rad->value : 0.0;
                if (!rad) {
                    const double safeMass = std::max(1.0, static_cast<double>(mass->value));
                    rMeters = std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::body_density));
                }
                const auto* cfg = world.get<Config>();
                const float minRadiusWorld = nbody::constants::min_body_radius / camera.zoom;
                const float bodyRadius = std::max(minRadiusWorld,
                                                  (cfg ? cfg->radius_scale : nbody::constants::default_radius_scale) *
                                                      static_cast<float>(rMeters));
                const float ringRadius = bodyRadius + nbody::constants::ring_extra_radius / camera.zoom;
                DrawRing(fvec2(pos->value), ringRadius, ringRadius + nbody::constants::ring_thickness / camera.zoom,
                         nbody::constants::ring_start_angle, nbody::constants::ring_end_angle,
                         nbody::constants::ring_segments, YELLOW);
                DrawCircleLines(static_cast<int>(pos->value.x), static_cast<int>(pos->value.y),
                                bodyRadius + nbody::constants::ring_inner_offset / camera.zoom,
                                ColorAlpha(WHITE, nbody::constants::selected_circle_alpha));
            }
        }
        EndMode2D();
    }

private:
    static flecs::entity find_entity_at_position(const flecs::world& world, const DVec2& worldPos,
                                                 float pickRadius) {
        flecs::entity best = flecs::entity::null();
        float bestDist2 = pickRadius * pickRadius;
        const auto* cam = nbody::Camera::get(world);
        const float zoom = cam ? cam->zoom : 1.0f;
        world.each([&](const flecs::entity ent, const Position& pos, const Mass& mass, const Selectable& selectable) {
            if (!selectable.canSelect) return;
            const DVec2 delta = worldPos - pos.value;
            const double dist2d = delta.x * delta.x + delta.y * delta.y;
            const float dist2 = static_cast<float>(dist2d);
            double rMeters = 0.0;
            if (const auto* rad = ent.get<Radius>()) {
                rMeters = rad->value;
            } else {
                const double safeMass = std::max(1.0, static_cast<double>(mass.value));
                rMeters = std::cbrt((3.0 * safeMass) / (4.0 * std::numbers::pi * nbody::constants::body_density));
            }
            const Config* cfg = world.get<Config>();
            // pickRadius is already in world units (pixels/zoom). Use same min radius rule as rendering.
            const float minRadiusWorld = nbody::constants::min_body_radius / zoom;
            const float bodyRadius = std::max(minRadiusWorld,
                                              (cfg ? cfg->radius_scale : nbody::constants::default_radius_scale) *
                                                  static_cast<float>(rMeters));
            const float totalPickRadius = pickRadius + bodyRadius;
            if (dist2 <= totalPickRadius * totalPickRadius && dist2 < bestDist2) {
                best = ent;
                bestDist2 = dist2;
            }
        });
        return best;
    }

    static void handle_mouse_press(const flecs::world& world, State* state, const DVec2& mouseWorld,
                                   float pickRadius) {
        flecs::entity entityAtMouse = find_entity_at_position(world, mouseWorld, pickRadius);
        if (const auto* cfg = world.get<Config>(); cfg && cfg->paused && state->selected_entity.is_alive() &&
            entityAtMouse.is_alive() && entityAtMouse.id() == state->selected_entity.id()) {
            if (const auto* pos = state->selected_entity.get<Position>()) {
                state->is_dragging_selected = true;
                state->is_panning = false;
                state->pan_candidate = flecs::entity::null();
                state->selected_drag_offset = pos->value - mouseWorld;
            }
        } else {
            if (entityAtMouse.is_alive()) {
                state->pan_candidate = entityAtMouse;
                state->is_panning = false;
            } else {
                state->is_panning = true;
                state->pan_candidate = flecs::entity::null();
            }
        }
        state->drag_distance_pixels = 0.0f;
    }

    static void handle_mouse_drag(const flecs::world& world, State* state, const DVec2& mouseWorld) {
        const raylib::Vector2 mouseDelta = GetMouseDelta();
        state->drag_distance_pixels += Vector2Length(mouseDelta);
        if (state->is_dragging_selected && state->selected_entity.is_alive()) {
            if (auto* pos = state->selected_entity.get_mut<Position>()) {
                pos->value = mouseWorld + state->selected_drag_offset;
            }
        }
        if (state->is_panning) {
            if (auto* cam = Camera::get(world)) {
                const raylib::Vector2 worldDelta = mouseDelta * (-1.0f / cam->zoom);
                cam->target = Vector2Add(cam->target, worldDelta);
            }
        }
    }

    static void handle_mouse_release(const flecs::world& world, State* state) {
        if (state->is_dragging_selected) {
            state->is_dragging_selected = false;
            state->pan_candidate = flecs::entity::null();
            state->drag_distance_pixels = 0.0f;
        } else if (!state->is_panning && state->pan_candidate.is_alive() &&
                   state->drag_distance_pixels * state->drag_distance_pixels <= nbody::constants::select_threshold_sq) {
            select(world, state->pan_candidate);
        }
        state->is_panning = false;
        state->pan_candidate = flecs::entity::null();
        state->drag_distance_pixels = 0.0f;
    }

    static void start_velocity_drag(const flecs::world& world, const DVec2& worldPos) {
        auto* state = world.get_mut<State>();
        auto* cfg = world.get_mut<Config>();
        if (!state || !cfg || !state->selected_entity.is_alive()) return;
        const auto* draggable = state->selected_entity.get<Draggable>();
        const auto* position = state->selected_entity.get<Position>();
        if (!draggable || !draggable->can_drag_velocity || !position) return;
        state->is_dragging_velocity = true;
        state->drag_start_world = position->value;
        state->current_drag_world = worldPos;
        cfg->paused = true;
    }

    static void update_velocity_drag(const flecs::world& world, const DVec2& worldPos) {
        auto* state = world.get_mut<State>();
        auto* cfg = world.get_mut<Config>();
        if (!state || !cfg || !state->is_dragging_velocity || !state->selected_entity.is_alive()) return;
        state->current_drag_world = worldPos;
        const auto* position = state->selected_entity.get<Position>();
        const auto* draggable = state->selected_entity.get<Draggable>();
        auto* velocity = state->selected_entity.get_mut<Velocity>();
        if (position && draggable && velocity) {
            // World-space drag vector is in meters. Convert to a velocity that is stable across timeScale
            // by scaling with the effective dt used by physics.
            const float baseDt = cfg->use_fixed_dt ? cfg->fixed_dt : GetFrameTime();
            const float dtEff = std::max(1e-6f, baseDt * std::max(0.0f, cfg->time_scale));
            const DVec2 dragVector = worldPos - position->value;
            // draggable->drag_scale is interpreted as a fraction of the drag line per physics step.
            const float fractionPerStep = std::max(0.0f, draggable->drag_scale);
            const DVec2 newVel = dragVector * static_cast<double>(fractionPerStep / dtEff);
            velocity->value = newVel;
            // Respect optional velocity cap
            if (cfg->max_speed > 0.0f) {
                const double vlen = std::sqrt(velocity->value.x * velocity->value.x + velocity->value.y * velocity->value.y);
                if (vlen > static_cast<double>(cfg->max_speed))
                    velocity->value = velocity->value * (static_cast<double>(cfg->max_speed) / vlen);
            }
        }
    }

    static void end_velocity_drag(const flecs::world& world) {
        if (auto* state = world.get_mut<State>()) {
            state->is_dragging_velocity = false;
            state->drag_start_world = DVec2{0.0, 0.0};
            state->current_drag_world = DVec2{0.0, 0.0};
        }
    }

public:
    // Snake_case API only
};

}  // namespace nbody
