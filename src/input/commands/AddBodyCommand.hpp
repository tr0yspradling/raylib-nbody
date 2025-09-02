#pragma once

#include "../ICommand.hpp"
#include "../../components/Components.hpp"
#include "../../core/Colors.hpp"
#include "../../core/Config.hpp"
#include "../../core/Constants.hpp"
#include "../../core/Math.hpp"

namespace nbody::input::commands {

class AddBodyCommand : public ICommand {
public:
    explicit AddBodyCommand(DVec2 position) : position_(position) {}

    void execute(const flecs::world& world) override {
        const auto* cfg = world.get<Config>();
        if (!cfg) return;

        world.entity()
            .set<Position>({position_})
            .set<Velocity>({dvec2(cfg->add_spawn_velocity)})
            .set<Acceleration>({DVec2{0.0, 0.0}})
            .set<PrevAcceleration>({DVec2{0.0, 0.0}})
            .set<Mass>({std::max(nbody::constants::spawn_mass_min, cfg->add_spawn_mass)})
            .set<Pinned>({cfg->add_spawn_pinned})
            .set<Tint>({random_nice_color()})
            .set<Trail>({{}})
            .add<Selectable>()
            .set<Draggable>({true, cfg->add_drag_vel_scale});
    }

    const char* getName() const override {
        return "AddBody";
    }

private:
    DVec2 position_;
};

}  // namespace nbody::input::commands