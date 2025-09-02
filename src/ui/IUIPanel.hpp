#pragma once

#include <flecs.h>
#include <raylib-cpp.hpp>

namespace nbody::ui {

class IUIPanel {
public:
    virtual ~IUIPanel() = default;
    
    virtual void update(const flecs::world& world, const raylib::Camera2D& camera) = 0;
    virtual bool isVisible() const = 0;
    virtual void setVisible(bool visible) = 0;
    virtual const char* getName() const = 0;
};

}  // namespace nbody::ui