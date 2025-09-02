#pragma once

#include <flecs.h>
#include "../core/Math.hpp"

namespace nbody::events {

struct SimulationPaused {
    bool paused;
};

struct EntitySelected {
    flecs::entity entity;
};

struct EntityDeselected {
    flecs::entity entity;
};

struct ConfigChanged {
    enum class Parameter {
        TimeScale,
        Integrator,
        Gravity,
        Softening,
        MaxSpeed,
        Paused
    };
    Parameter parameter;
};

struct CameraChanged {
    DVec2 target;
    float zoom;
};

struct AddBodyRequested {
    DVec2 position;
    DVec2 velocity;
    float mass;
    bool pinned;
};

struct ResetScenarioRequested {};

struct ResetAllRequested {};

}  // namespace nbody::events