#pragma once

#include <memory>
#include "IIntegrator.hpp"
#include "integrators/SemiImplicitEulerIntegrator.hpp"
#include "integrators/VelocityVerletIntegrator.hpp"

namespace nbody::simulation {

class IntegratorFactory {
public:
    static std::shared_ptr<IIntegrator> create(int integratorId) {
        switch (integratorId) {
            case 0:
                return std::make_shared<integrators::SemiImplicitEulerIntegrator>();
            case 1:
                return std::make_shared<integrators::VelocityVerletIntegrator>();
            default:
                return std::make_shared<integrators::VelocityVerletIntegrator>(); // Default
        }
    }

    static std::shared_ptr<IIntegrator> createDefault() {
        return create(1); // Velocity Verlet as default
    }
};

}  // namespace nbody::simulation