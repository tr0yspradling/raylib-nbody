#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <array>
#include <raylib-cpp.hpp>
#include <vector>

namespace nbody {

class SpatialPartition {
public:
    struct Body {
        raylib::Vector2 pos;
        float mass;
        int index;
    };

private:
    struct Node {
        raylib::Vector2 center{};
        float halfSize = 0.0F;
        float mass = 0.0F;
        raylib::Vector2 com{0.0F, 0.0F};
        Body* body = nullptr;
        std::array<std::unique_ptr<Node>, 4> children{};

        Node(const raylib::Vector2& centerPos, float halfSizeVal) : center(centerPos), halfSize(halfSizeVal) {}
        [[nodiscard]] auto is_leaf() const -> bool { return !children[0]; }
    };

    std::unique_ptr<Node> root;

public:
    void build(std::vector<Body>& bodies) {
        if (bodies.empty()) {
            return;
        }
        float minX = bodies[0].pos.x;
        float maxX = bodies[0].pos.x;
        float minY = bodies[0].pos.y;
        float maxY = bodies[0].pos.y;
        for (const auto& body : bodies) {
            minX = std::min(minX, body.pos.x);
            maxX = std::max(maxX, body.pos.x);
            minY = std::min(minY, body.pos.y);
            maxY = std::max(maxY, body.pos.y);
        }
        constexpr float kHalf = 0.5F;
        float size = std::max(maxX - minX, maxY - minY) * kHalf;
        if (size <= 0.0F) {
            size = 1.0F;
        }
        raylib::Vector2 center{(minX + maxX) * kHalf, (minY + maxY) * kHalf};
        root = std::make_unique<Node>(center, size);
        for (auto& body : bodies) {
            insert_iterative(root.get(), &body);
        }
        aggregate_mass_com_iterative();
    }

    void compute_force(const Body& target, double theta, double gravConst, double eps2, raylib::Vector2& acc) const {
        if (!root) return;
        // Explicit stack-based traversal to avoid recursion
        std::vector<const Node*> stack;
        stack.reserve(64);
        stack.push_back(root.get());
        while (!stack.empty()) {
            const Node* node = stack.back();
            stack.pop_back();
            if (!node || node->mass <= 0.0F) continue;

            if (node->is_leaf()) {
                if (!node->body || node->body->index == target.index) continue;
                const double dx = static_cast<double>(node->body->pos.x) - static_cast<double>(target.pos.x);
                const double dy = static_cast<double>(node->body->pos.y) - static_cast<double>(target.pos.y);
                const double r2 = (dx * dx) + (dy * dy) + eps2;
                const double invR = 1.0 / std::sqrt(r2);
                const double invR3 = invR * invR * invR;
                const double ax = gravConst * static_cast<double>(node->body->mass) * dx * invR3;
                const double ay = gravConst * static_cast<double>(node->body->mass) * dy * invR3;
                acc.x += static_cast<float>(ax);
                acc.y += static_cast<float>(ay);
                continue;
            }

            const double dx = static_cast<double>(node->com.x) - static_cast<double>(target.pos.x);
            const double dy = static_cast<double>(node->com.y) - static_cast<double>(target.pos.y);
            const double dist = std::sqrt((dx * dx) + (dy * dy));
            if ((static_cast<double>(node->halfSize) * 2.0) / dist < theta) {
                const double r2 = (dx * dx) + (dy * dy) + eps2;
                const double invR = 1.0 / std::sqrt(r2);
                const double invR3 = invR * invR * invR;
                const double ax = gravConst * static_cast<double>(node->mass) * dx * invR3;
                const double ay = gravConst * static_cast<double>(node->mass) * dy * invR3;
                acc.x += static_cast<float>(ax);
                acc.y += static_cast<float>(ay);
            } else {
                // Traverse children
                for (const auto& child : node->children) {
                    if (child) stack.push_back(child.get());
                }
            }
        }
    }

private:
    void insert_iterative(Node* node, Body* bodyPtr) {
        // Insert a body into the tree without recursion. When a collision occurs
        // (leaf already has a body), subdivide and reinsert the existing body,
        // then continue inserting the new one.
        while (true) {
            if (node->is_leaf()) {
                if (node->body == nullptr) {
                    node->body = bodyPtr;
                    // mass/com aggregated later in a separate pass
                    return;
                }

                // Subdivide and reinsert the existing body down the appropriate branch
                Body* existing = node->body;
                subdivide(node);
                node->body = nullptr;

                // Place the existing body first
                Node* cur = node->children[static_cast<std::size_t>(get_quadrant(node, existing->pos))].get();
                while (true) {
                    if (cur->is_leaf()) {
                        if (cur->body == nullptr) {
                            cur->body = existing;
                            break;
                        } else {
                            // Need to subdivide further
                            Body* ex2 = cur->body;
                            subdivide(cur);
                            cur->body = nullptr;
                            cur = cur->children[static_cast<std::size_t>(get_quadrant(cur, ex2->pos))].get();
                            existing = ex2;
                            continue;
                        }
                    } else {
                        cur = cur->children[static_cast<std::size_t>(get_quadrant(cur, existing->pos))].get();
                    }
                }

                // Now continue inserting the new body starting from current internal node
                node = node->children[static_cast<std::size_t>(get_quadrant(node, bodyPtr->pos))].get();
                continue;
            }

            // Not a leaf, descend towards the quadrant of the new body
            const int quadrant = get_quadrant(node, bodyPtr->pos);
            node = node->children[static_cast<std::size_t>(quadrant)].get();
        }
    }

    void subdivide(Node* node) {
        constexpr float kHalf = 0.5F;
        const float hs = node->halfSize * kHalf;
        const float cx = node->center.x;
        const float cy = node->center.y;
        node->children[0] = std::make_unique<Node>(raylib::Vector2{cx - hs, cy - hs}, hs);  // NW
        node->children[1] = std::make_unique<Node>(raylib::Vector2{cx + hs, cy - hs}, hs);  // NE
        node->children[2] = std::make_unique<Node>(raylib::Vector2{cx - hs, cy + hs}, hs);  // SW
        node->children[3] = std::make_unique<Node>(raylib::Vector2{cx + hs, cy + hs}, hs);  // SE
    }

    static auto get_quadrant(const Node* node, const raylib::Vector2& point) -> int {
        const bool east = point.x > node->center.x;
        const bool south = point.y > node->center.y;
        if (east) {
            return south ? 3 : 1;
        }
        return south ? 2 : 0;
    }

    void aggregate_mass_com_iterative() {
        if (!root) return;
        // Post-order traversal using an explicit stack
        struct Frame { Node* node; bool visited; };
        std::vector<Frame> stack;
        stack.reserve(128);
        stack.push_back(Frame{root.get(), false});

        while (!stack.empty()) {
            Frame f = stack.back();
            stack.pop_back();
            Node* node = f.node;
            if (!node) continue;

            if (f.visited || node->is_leaf()) {
                if (node->is_leaf()) {
                    if (node->body) {
                        node->mass = node->body->mass;
                        node->com = node->body->pos;
                    } else {
                        node->mass = 0.0F;
                        node->com = raylib::Vector2{0.0F, 0.0F};
                    }
                } else {
                    float mass_sum = 0.0F;
                    raylib::Vector2 com_sum{0.0F, 0.0F};
                    for (const auto& child : node->children) {
                        if (child && child->mass > 0.0F) {
                            mass_sum += child->mass;
                            com_sum += child->com * child->mass;
                        }
                    }
                    node->mass = mass_sum;
                    node->com = (mass_sum > 0.0F) ? (com_sum * (1.0F / mass_sum)) : raylib::Vector2{0.0F, 0.0F};
                }
            } else {
                stack.push_back(Frame{node, true});
                for (auto& child : node->children) {
                    if (child) stack.push_back(Frame{child.get(), false});
                }
            }
        }
    }
};

}  // namespace nbody
