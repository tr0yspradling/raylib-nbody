#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
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
            float halfSize = 0.0f;
            float mass = 0.0f;
            raylib::Vector2 com{0.0f, 0.0f};
            Body* body = nullptr;
            std::unique_ptr<Node> children[4];

            Node(const raylib::Vector2& c, float hs) : center(c), halfSize(hs) {}
            bool IsLeaf() const { return !children[0]; }
        };

        std::unique_ptr<Node> root;

    public:
        void Build(std::vector<Body>& bodies) {
            if (bodies.empty()) return;
            float minX = bodies[0].pos.x, maxX = bodies[0].pos.x;
            float minY = bodies[0].pos.y, maxY = bodies[0].pos.y;
            for (const auto& b : bodies) {
                minX = std::min(minX, b.pos.x);
                maxX = std::max(maxX, b.pos.x);
                minY = std::min(minY, b.pos.y);
                maxY = std::max(maxY, b.pos.y);
            }
            float size = std::max(maxX - minX, maxY - minY) * 0.5f;
            if (size <= 0.0f) size = 1.0f;
            raylib::Vector2 center{(minX + maxX) * 0.5f, (minY + maxY) * 0.5f};
            root = std::make_unique<Node>(center, size);
            for (auto& b : bodies) Insert(root.get(), &b);
        }

        void ComputeForce(const Body& target, double theta, double G, double eps2, raylib::Vector2& acc) const {
            ComputeForceRec(root.get(), target, theta, G, eps2, acc);
        }

    private:
        void Insert(Node* node, Body* b) {
            if (node->IsLeaf()) {
                if (!node->body) {
                    node->body = b;
                    node->mass = b->mass;
                    node->com = b->pos;
                    return;
                }
                Subdivide(node);
                Insert(node, node->body);
                node->body = nullptr;
            }
            const int q = GetQuadrant(node, b->pos);
            Insert(node->children[q].get(), b);
            node->mass = 0.0f;
            node->com = raylib::Vector2{0.0f, 0.0f};
            for (const auto& ch : node->children) {
                if (ch && ch->mass > 0.0f) {
                    node->mass += ch->mass;
                    node->com += ch->com * ch->mass;
                }
            }
            if (node->mass > 0.0f) node->com *= (1.0f / node->mass);
        }

        void Subdivide(Node* node) {
            const float hs = node->halfSize * 0.5f;
            const float cx = node->center.x;
            const float cy = node->center.y;
            node->children[0] = std::make_unique<Node>(raylib::Vector2{cx - hs, cy - hs}, hs);  // NW
            node->children[1] = std::make_unique<Node>(raylib::Vector2{cx + hs, cy - hs}, hs);  // NE
            node->children[2] = std::make_unique<Node>(raylib::Vector2{cx - hs, cy + hs}, hs);  // SW
            node->children[3] = std::make_unique<Node>(raylib::Vector2{cx + hs, cy + hs}, hs);  // SE
        }

        int GetQuadrant(const Node* node, const raylib::Vector2& p) const {
            const bool east = p.x > node->center.x;
            const bool south = p.y > node->center.y;
            if (east) return south ? 3 : 1;
            return south ? 2 : 0;
        }

        void ComputeForceRec(const Node* node, const Body& target, double theta, double G, double eps2,
                             raylib::Vector2& acc) const {
            if (!node || node->mass <= 0.0f) return;
            if (node->IsLeaf()) {
                if (!node->body || node->body->index == target.index) return;
                const double dx = static_cast<double>(node->body->pos.x) - static_cast<double>(target.pos.x);
                const double dy = static_cast<double>(node->body->pos.y) - static_cast<double>(target.pos.y);
                const double r2 = dx * dx + dy * dy + eps2;
                const double invR = 1.0 / std::sqrt(r2);
                const double invR3 = invR * invR * invR;
                const double ax = G * static_cast<double>(node->body->mass) * dx * invR3;
                const double ay = G * static_cast<double>(node->body->mass) * dy * invR3;
                acc.x += static_cast<float>(ax);
                acc.y += static_cast<float>(ay);
                return;
            }
            const double dx = static_cast<double>(node->com.x) - static_cast<double>(target.pos.x);
            const double dy = static_cast<double>(node->com.y) - static_cast<double>(target.pos.y);
            const double dist = std::sqrt(dx * dx + dy * dy);
            if ((static_cast<double>(node->halfSize) * 2.0) / dist < theta) {
                const double r2 = dx * dx + dy * dy + eps2;
                const double invR = 1.0 / std::sqrt(r2);
                const double invR3 = invR * invR * invR;
                const double ax = G * static_cast<double>(node->mass) * dx * invR3;
                const double ay = G * static_cast<double>(node->mass) * dy * invR3;
                acc.x += static_cast<float>(ax);
                acc.y += static_cast<float>(ay);
            } else {
                for (const auto& ch : node->children)
                    if (ch) ComputeForceRec(ch.get(), target, theta, G, eps2, acc);
            }
        }
    };

}  // namespace nbody
