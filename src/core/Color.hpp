#pragma once

#include <raylib-cpp.hpp>

#include "Constants.hpp"

namespace nbody {

    inline raylib::Color RandomNiceColor() {
        return {static_cast<unsigned char>(
                    GetRandomValue(nbody::constants::randomColorMin, nbody::constants::randomColorMax)),
                static_cast<unsigned char>(
                    GetRandomValue(nbody::constants::randomColorMin, nbody::constants::randomColorMax)),
                static_cast<unsigned char>(
                    GetRandomValue(nbody::constants::randomColorMin, nbody::constants::randomColorMax)),
                255};
    }

}  // namespace nbody
