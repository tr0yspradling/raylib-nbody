#pragma once

#include <flecs.h>

namespace nbody::simulation {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    
    virtual void integrate(const flecs::world& world, float deltaTime) = 0;
    virtual const char* getName() const = 0;
    virtual int getId() const = 0;
};

}  // namespace nbody::simulation