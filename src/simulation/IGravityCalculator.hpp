#pragma once

#include <flecs.h>

namespace nbody::simulation {

class IGravityCalculator {
public:
    virtual ~IGravityCalculator() = default;
    virtual void computeGravity(const flecs::world& world) = 0;
};

}  // namespace nbody::simulation