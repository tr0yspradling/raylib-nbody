#pragma once

#include <raylib-cpp.hpp>
#include <vector>

#include "Constants.hpp"

namespace nbody {

    class TrailPool {
    public:
        static std::vector<raylib::Vector2> Acquire() {
            auto& pool = get_pool();
            if (!pool.empty()) {
                auto pts = std::move(pool.back());
                pool.pop_back();
                pts.clear();
                return pts;
            }
            std::vector<raylib::Vector2> pts;
            pts.reserve(constants::trailLengthMax);
            return pts;
        }

        static void Release(std::vector<raylib::Vector2>&& pts) {
            pts.clear();
            get_pool().push_back(std::move(pts));
        }

    private:
        static std::vector<std::vector<raylib::Vector2>>& get_pool() {
            static std::vector<std::vector<raylib::Vector2>> pool;
            return pool;
        }
    };

}  // namespace nbody
