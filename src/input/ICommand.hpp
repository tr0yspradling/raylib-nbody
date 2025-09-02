#pragma once

#include <flecs.h>

namespace nbody::input {

class ICommand {
public:
    virtual ~ICommand() = default;
    virtual void execute(const flecs::world& world) = 0;
    virtual const char* getName() const = 0;
};

}  // namespace nbody::input