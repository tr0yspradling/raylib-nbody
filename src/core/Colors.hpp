#pragma once

#include <raylib-cpp.hpp>

#include "Constants.hpp"

namespace nbody {

inline auto random_nice_color() -> raylib::Color {
    return {static_cast<unsigned char>(GetRandomValue(constants::random_color_min, constants::random_color_max)),
            static_cast<unsigned char>(GetRandomValue(constants::random_color_min, constants::random_color_max)),
            static_cast<unsigned char>(GetRandomValue(constants::random_color_min, constants::random_color_max)),
            constants::alpha_opaque};
}

}  // namespace nbody
