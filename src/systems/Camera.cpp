#include <flecs.h>
#include <raylib-cpp.hpp>
#include <raymath.h>

#include "../components/Components.hpp"
#include "../core/Camera.hpp"
#include "../core/Config.hpp"

namespace ecs {

    // Camera component that wraps raylib::Camera2D with additional state
    struct CameraComponent {
        raylib::Camera2D camera;
        
        CameraComponent() {
            InitCamera(camera);
        }
    };

    class CameraSystem {
    public:
        static void InitializeCamera(const flecs::world& world) {
            world.set<CameraComponent>({});
            
            // Center camera to initial center of mass
            CenterOnCenterOfMass(world);
        }

        static void CenterOnCenterOfMass(const flecs::world& world) {
            auto* camComp = world.get_mut<CameraComponent>();
            if (!camComp) return;

            double Cx = 0, Cy = 0, M = 0;
            world.each([&](const Position& p, const Mass& m) {
                Cx += static_cast<double>(m.value) * static_cast<double>(p.value.x);
                Cy += static_cast<double>(m.value) * static_cast<double>(p.value.y);
                M += static_cast<double>(m.value);
            });
            
            if (M > 0.0) {
                camComp->camera.target = {static_cast<float>(Cx / M), static_cast<float>(Cy / M)};
            }
        }

        // Removed unused zoom/pan smoothing and helpers

        static raylib::Camera2D* GetCamera(const flecs::world& world) {
            if (auto* camComp = world.get_mut<CameraComponent>()) {
                return &camComp->camera;
            }
            return nullptr;
        }

        static void FocusOnEntity(const flecs::world& world, flecs::entity entity) {
            auto* camComp = world.get_mut<CameraComponent>();
            if (!camComp) return;

            if (const auto* pos = entity.get<Position>()) {
                camComp->camera.target = pos->value;
            }
        }

        // Removed unused SetZoom/GetZoom and conversion helpers
    };

    void register_camera_system(const flecs::world& world) {
        // Initialize camera component
        CameraSystem::InitializeCamera(world);

        // No periodic camera updates needed currently
    }

    raylib::Camera2D* get_camera(const flecs::world& world) {
        return CameraSystem::GetCamera(world);
    }

    void center_camera_on_com(const flecs::world& world) {
        CameraSystem::CenterOnCenterOfMass(world);
    }

    void focus_camera_on_entity(const flecs::world& world, flecs::entity entity) {
        CameraSystem::FocusOnEntity(world, entity);
    }

} // namespace ecs
