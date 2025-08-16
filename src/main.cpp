// src/main.cpp
/**
 * Purpose: Real-time, debuggable N-body gravity simulation with raylib + rlImGui + ImGui.
 * Provides split control windows, per-body list selection, robust input gating so UI scroll does not zoom,
 * integrator selection, softening, diagnostics, camera pan/zoom, click-to-select, right-drag velocity set, trails.
 * Update: Left-click drag anywhere on empty space pans the viewport. Small left-click on a body selects it.
 */

#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>
#include <imgui.h>
#include <vector>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <string>

struct Body {
    Vector2 position;
    Vector2 velocity;
    float mass;
    Color color;
    bool pinned;
};

/* Purpose: Return a random high-contrast color for new bodies. */
static Color RandomNiceColor() {
    return {
        static_cast<unsigned char>(GetRandomValue(64, 255)),
        static_cast<unsigned char>(GetRandomValue(64, 255)),
        static_cast<unsigned char>(GetRandomValue(64, 255)),
        255
    };
}

/* Purpose: Check the finiteness of a numeric value. */
template <typename T>
static bool IsFinite(T v) { return std::isfinite(static_cast<double>(v)); }

/* Purpose: Compute accelerations via symmetric gravity with softening; returns false on non-finite. */
static bool ComputeAccelerations(const std::vector<Body>& bodies, std::vector<Vector2>& a, double G, double eps2) {
    const size_t n = bodies.size();
    a.assign(n, Vector2{0.0f, 0.0f});
    if (n == 0) return true;

    for (size_t i = 0; i < n; ++i) {
        if (!IsFinite(bodies[i].position.x) || !IsFinite(bodies[i].position.y)) return false;
        if (!IsFinite(bodies[i].velocity.x) || !IsFinite(bodies[i].velocity.y)) return false;
        if (!(bodies[i].mass > 0.0f) || !IsFinite(bodies[i].mass)) return false;
    }

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dx = static_cast<double>(bodies[j].position.x) - static_cast<double>(bodies[i].position.x);
            double dy = static_cast<double>(bodies[j].position.y) - static_cast<double>(bodies[i].position.y);
            double r2 = dx*dx + dy*dy + eps2;
            double invR = 1.0 / std::sqrt(r2);
            double invR3 = invR * invR * invR;

            double ax_i = G * static_cast<double>(bodies[j].mass) * dx * invR3;
            double ay_i = G * static_cast<double>(bodies[j].mass) * dy * invR3;
            double ax_j = -G * static_cast<double>(bodies[i].mass) * dx * invR3;
            double ay_j = -G * static_cast<double>(bodies[i].mass) * dy * invR3;

            if (!bodies[i].pinned) {
                a[i].x += static_cast<float>(ax_i);
                a[i].y += static_cast<float>(ay_i);
            }
            if (!bodies[j].pinned) {
                a[j].x += static_cast<float>(ax_j);
                a[j].y += static_cast<float>(ay_j);
            }
        }
    }

    for (size_t i = 0; i < n; ++i) {
        if (!IsFinite(a[i].x) || !IsFinite(a[i].y)) return false;
    }
    return true;
}

static bool StepSemiImplicitEuler(std::vector<Body>& bodies, std::vector<Vector2>& a, float dt, double G, double eps2, float maxSpeed) {
    if (!ComputeAccelerations(bodies, a, G, eps2)) return false;
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].pinned) continue;
        bodies[i].velocity = Vector2Add(bodies[i].velocity, Vector2Scale(a[i], dt));
        if (maxSpeed > 0.0f) {
            float vlen = Vector2Length(bodies[i].velocity);
            if (vlen > maxSpeed) bodies[i].velocity = Vector2Scale(bodies[i].velocity, maxSpeed / vlen);
        }
        bodies[i].position = Vector2Add(bodies[i].position, Vector2Scale(bodies[i].velocity, dt));
    }
    return true;
}

static bool StepVelocityVerlet(std::vector<Body>& bodies, std::vector<Vector2>& a, float dt, double G, double eps2, float maxSpeed) {
    if (!ComputeAccelerations(bodies, a, G, eps2)) return false;

    const size_t n = bodies.size();
    std::vector a_new(n, Vector2{0,0});

    for (size_t i = 0; i < n; ++i) {
        if (bodies[i].pinned) continue;
        Vector2 dx = Vector2Add(Vector2Scale(bodies[i].velocity, dt), Vector2Scale(a[i], 0.5f * dt * dt));
        bodies[i].position = Vector2Add(bodies[i].position, dx);
    }

    if (!ComputeAccelerations(bodies, a_new, G, eps2)) return false;

    for (size_t i = 0; i < n; ++i) {
        if (bodies[i].pinned) continue;
        Vector2 avg = Vector2Scale(Vector2Add(a[i], a_new[i]), 0.5f);
        bodies[i].velocity = Vector2Add(bodies[i].velocity, Vector2Scale(avg, dt));
        if (maxSpeed > 0.0f) {
            float vlen = Vector2Length(bodies[i].velocity);
            if (vlen > maxSpeed) bodies[i].velocity = Vector2Scale(bodies[i].velocity, maxSpeed / vlen);
        }
    }

    a.swap(a_new);
    return true;
}

struct Diagnostics {
    double kinetic;
    double potential;
    double energy;
    Vector2 momentum;
    Vector2 com;
    double totalMass;
};

static bool ComputeDiagnostics(const std::vector<Body>& bodies, double G, double eps2, Diagnostics& out) {
    const size_t n = bodies.size();
    out = Diagnostics{};
    if (n == 0) return true;

    double KE = 0.0;
    double PE = 0.0;
    double M = 0.0;
    double Px = 0.0, Py = 0.0;
    double Cx = 0.0, Cy = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double vx = static_cast<double>(bodies[i].velocity.x);
        double vy = static_cast<double>(bodies[i].velocity.y);
        double m = static_cast<double>(bodies[i].mass);
        KE += 0.5 * m * (vx*vx + vy*vy);
        Px += m * vx;
        Py += m * vy;
        Cx += m * static_cast<double>(bodies[i].position.x);
        Cy += m * static_cast<double>(bodies[i].position.y);
        M += m;
    }

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            double dx = static_cast<double>(bodies[j].position.x) - static_cast<double>(bodies[i].position.x);
            double dy = static_cast<double>(bodies[j].position.y) - static_cast<double>(bodies[i].position.y);
            double r2 = dx*dx + dy*dy + eps2;
            double r = std::sqrt(r2);
            PE += -G * static_cast<double>(bodies[i].mass) * static_cast<double>(bodies[j].mass) / r;
        }
    }

    out.kinetic = KE;
    out.potential = PE;
    out.energy = KE + PE;
    out.momentum = Vector2{static_cast<float>(Px), static_cast<float>(Py)};
    out.totalMass = M;
    out.com = (M > 0.0) ? Vector2{static_cast<float>(Cx / M), static_cast<float>(Cy / M)} : Vector2{0,0};

    if (!IsFinite(out.kinetic) || !IsFinite(out.potential) || !IsFinite(out.energy)) return false;
    if (!IsFinite(out.momentum.x) || !IsFinite(out.momentum.y)) return false;
    if (!IsFinite(out.com.x) || !IsFinite(out.com.y)) return false;
    return true;
}

static void ResetScenario(std::vector<Body>& bodies) {
    bodies.clear();
    bodies.push_back(Body{ Vector2{640, 360}, Vector2{ 0.0f,  0.0f}, 4000.0f, RED,   false });
    bodies.push_back(Body{ Vector2{840, 360}, Vector2{ 0.0f,  1.20f},   12.0f, BLUE,  false });
    bodies.push_back(Body{ Vector2{440, 360}, Vector2{ 0.0f, -1.20f},   12.0f, GREEN, false });
}

static void ZeroNetMomentum(std::vector<Body>& bodies) {
    double Px = 0.0, Py = 0.0, M = 0.0;
    for (auto& b : bodies) {
        Px += static_cast<double>(b.mass) * static_cast<double>(b.velocity.x);
        Py += static_cast<double>(b.mass) * static_cast<double>(b.velocity.y);
        M += static_cast<double>(b.mass);
    }
    if (M <= 0.0) return;
    Vector2 v0 = Vector2{ static_cast<float>(Px / M), static_cast<float>(Py / M) };
    for (auto& b : bodies) {
        if (b.pinned) continue;
        b.velocity = Vector2Subtract(b.velocity, v0);
    }
}

static void UpdateTrails(std::vector<std::vector<Vector2>>& trails, const std::vector<Body>& bodies, int maxLen) {
    if (static_cast<int>(trails.size()) != static_cast<int>(bodies.size())) trails.assign(bodies.size(), {});
    for (size_t i = 0; i < bodies.size(); ++i) {
        auto& t = trails[i];
        t.push_back(bodies[i].position);
        if (static_cast<int>(t.size()) > maxLen) t.erase(t.begin());
    }
}

static int PickBody(const std::vector<Body>& bodies, const Vector2& worldPos, float radius) {
    int best = -1;
    float bestD2 = radius * radius;
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        float d2 = Vector2LengthSqr(Vector2Subtract(worldPos, bodies[i].position));
        double safeMass = std::max(1.0, static_cast<double>(bodies[i].mass));
        float r = std::max(6.0f, static_cast<float>(std::cbrt(safeMass)));
        float pick = (radius + r);
        if (d2 <= pick * pick && d2 < bestD2) { best = i; bestD2 = d2; }
    }
    return best;
}

static void DrawWorldGrid(const Camera2D& cam, float spacing) {
    Vector2 tl = GetScreenToWorld2D(Vector2{0,0}, cam);
    Vector2 br = GetScreenToWorld2D(Vector2{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())}, cam);

    float startX = std::floor(tl.x / spacing) * spacing;
    float endX   = std::ceil (br.x / spacing) * spacing;
    float startY = std::floor(tl.y / spacing) * spacing;
    float endY   = std::ceil (br.y / spacing) * spacing;

    Color gridC = {40, 40, 40, 255};
    Color axisC = {80, 80, 80, 255};

    for (float x = startX; x <= endX; x += spacing) DrawLineV(Vector2{x, startY}, Vector2{x, endY}, (std::abs(x) < 1e-4f) ? axisC : gridC);
    for (float y = startY; y <= endY; y += spacing) DrawLineV(Vector2{startX, y}, Vector2{endX, y}, (std::abs(y) < 1e-4f) ? axisC : gridC);
}

enum class Integrator { SemiImplicitEuler = 0, VelocityVerlet = 1 };

int main() {
    InitWindow(1280, 720, "N-Body Gravity Simulation • Split Controls");
    SetTargetFPS(120);
    rlImGuiSetup(true);

    Camera2D cam{};
    cam.zoom = 1.0f;
    cam.offset = Vector2{ static_cast<float>(GetScreenWidth()) * 0.5f, static_cast<float>(GetScreenHeight()) * 0.5f };
    cam.target = Vector2{ 640.0f, 360.0f };

    std::vector<Body> bodies;
    ResetScenario(bodies);
    ZeroNetMomentum(bodies);

    double G = 6.67430e-3;
    float softening = 4.0f;
    float timeScale = 1.0f;
    bool useFixedDt = false;
    float fixedDt = 1.0f / 120.0f;
    float maxSpeed = 0.0f;

    Integrator integrator = Integrator::VelocityVerlet;
    std::vector<Vector2> accel;
    std::vector<std::vector<Vector2>> trails;
    int trailMax = 200;

    int selected = -1;
    float spawnMass = 12.0f;
    auto spawnVel = Vector2{0,0};
    bool spawnPinned = false;
    float dragVelScale = 0.01f;

    bool paused = false;
    bool drawVelocity = true;
    bool drawAcceleration = false;
    bool drawTrails = true;

    double lastStepMs = 0.0;
    bool draggingVel = false;
    auto dragWorld = Vector2{0,0};
    bool showDrag = false;

    bool lmbPanning = false;
    int  lmbPickCandidate = -1;
    float lmbDragDistSq = 0.0f;
    constexpr float selectThresholdSq = 9.0f; // 3px threshold

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(Color{10, 10, 14, 255});

        rlImGuiBegin();

        bool requestStep = false;

        ImGui::Begin("Time & Integrator");
        ImGui::Checkbox("Paused", &paused);
        ImGui::SameLine();
        if (ImGui::Button("Step")) requestStep = true;
        ImGui::Checkbox("Use Fixed dt", &useFixedDt);
        ImGui::SliderFloat("Fixed dt", &fixedDt, 1e-4f, 0.05f, "%.6f");
        ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 5.0f, "%.3f");
        int integ = (integrator == Integrator::VelocityVerlet) ? 1 : 0;
        ImGui::RadioButton("Semi-Implicit Euler", &integ, 0); ImGui::SameLine();
        ImGui::RadioButton("Velocity Verlet", &integ, 1);
        integrator = (integ == 1) ? Integrator::VelocityVerlet : Integrator::SemiImplicitEuler;
        ImGui::Text("Last step: %.3f ms", lastStepMs);
        ImGui::End();

        ImGui::Begin("Physics");
        float Gf = static_cast<float>(G);
        ImGui::SliderFloat("G", &Gf, 0.0f, 0.02f, "%.6f");
        G = Gf;
        ImGui::SliderFloat("Softening (epsilon)", &softening, 0.0f, 20.0f, "%.3f");
        ImGui::SliderFloat("Velocity Cap", &maxSpeed, 0.0f, 200.0f, "%.1f");
        if (ImGui::Button("Zero Net Momentum")) ZeroNetMomentum(bodies);
        ImGui::SameLine();
        if (ImGui::Button("Reset Scenario")) {
            ResetScenario(bodies);
            ZeroNetMomentum(bodies);
            trails.clear();
            UpdateTrails(trails, bodies, trailMax);
            selected = -1;
            paused = false;
        }
        ImGui::End();

        ImGui::Begin("Visuals");
        ImGui::Checkbox("Trails", &drawTrails);
        ImGui::SameLine();
        ImGui::Checkbox("Velocity Vectors", &drawVelocity);
        ImGui::SameLine();
        ImGui::Checkbox("Acceleration Vectors", &drawAcceleration);
        ImGui::SliderInt("Trail Length", &trailMax, 0, 2000);
        ImGui::End();

        ImGui::Begin("Add / Edit");
        ImGui::SliderFloat("Spawn Mass", &spawnMass, 1.0f, 5000.0f, "%.1f");
        ImGui::SliderFloat2("Spawn Velocity", &spawnVel.x, -5.0f, 5.0f, "%.3f");
        ImGui::Checkbox("Spawn Pinned", &spawnPinned);
        if (ImGui::Button("Add Body At Mouse")) {
            Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), cam);
            bodies.push_back(Body{ mouseWorld, spawnVel, std::max(1.0f, spawnMass), RandomNiceColor(), spawnPinned });
            if (drawTrails) UpdateTrails(trails, bodies, trailMax);
        }
        ImGui::SliderFloat("Right-Drag Vel Scale", &dragVelScale, 0.001f, 0.2f, "%.3f");
        if (selected >= 0 && selected < static_cast<int>(bodies.size())) {
            ImGui::SeparatorText("Selected Body");
            ImGui::Text("Index: %d", selected);
            ImGui::Checkbox("Pinned", &bodies[selected].pinned);
            ImGui::SliderFloat("Mass", &bodies[selected].mass, 1.0f, 10000.0f, "%.1f");
            ImGui::SliderFloat2("Velocity", &bodies[selected].velocity.x, -200.0f, 200.0f, "%.3f");
            if (ImGui::Button("Zero Velocity")) bodies[selected].velocity = Vector2{0,0};
            ImGui::SameLine();
            if (ImGui::Button("Remove Body")) {
                bodies.erase(bodies.begin() + selected);
                selected = -1;
                trails.clear();
                UpdateTrails(trails, bodies, trailMax);
            }
            ImGui::SameLine();
            if (ImGui::Button("Focus Camera")) cam.target = bodies[std::max(0, selected)].position;
        }
        ImGui::End();

        ImGui::Begin("Bodies");
        if (ImGui::BeginListBox("##BodyList", ImVec2(-FLT_MIN, 300))) {
            for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
                const auto& b = bodies[i];
                std::string label = std::to_string(i) + "  m=" + std::to_string(static_cast<int>(b.mass)) +
                                    "  pos=(" + std::to_string(static_cast<int>(b.position.x)) + "," + std::to_string(static_cast<int>(b.position.y)) + ")" +
                                    (b.pinned ? "  [P]" : "");
                ImVec4 col = ImVec4(b.color.r/255.0f, b.color.g/255.0f, b.color.b/255.0f, 1.0f);
                ImGui::PushID(i);
                ImGui::ColorButton("##c", col, ImGuiColorEditFlags_NoTooltip, ImVec2(16,16));
                ImGui::SameLine();
                if (ImGui::Selectable(label.c_str(), selected == i)) selected = i;
                ImGui::PopID();
            }
            ImGui::EndListBox();
        }
        if (ImGui::Button("Duplicate Selected") && selected >= 0 && selected < static_cast<int>(bodies.size())) {
            Body b = bodies[selected];
            b.position = Vector2Add(b.position, Vector2{20.0f, 0.0f});
            bodies.push_back(b);
            UpdateTrails(trails, bodies, trailMax);
        }
        ImGui::SameLine();
        if (ImGui::Button("Recenter to COM")) {
            Diagnostics d{};
            if (ComputeDiagnostics(bodies, G, static_cast<double>(softening) * static_cast<double>(softening), d)) cam.target = d.com;
        }
        ImGui::End();

        Diagnostics d{};
        bool okDiag = ComputeDiagnostics(bodies, G, static_cast<double>(softening) * static_cast<double>(softening), d);
        ImGui::Begin("Diagnostics");
        ImGui::Text("Kinetic: %.6g", d.kinetic);
        ImGui::Text("Potential: %.6g", d.potential);
        ImGui::Text("Total: %.6g", d.energy);
        ImGui::Text("Momentum: (%.6g, %.6g)", d.momentum.x, d.momentum.y);
        ImGui::Text("COM: (%.3f, %.3f)  Mass: %.3f", d.com.x, d.totalMass ? d.com.y : 0.0f, d.totalMass);
        if (!okDiag) ImGui::TextColored(ImVec4(1,0.3f,0.3f,1), "Non-finite detected; auto-paused.");
        ImGui::End();

        ImGuiIO& io = ImGui::GetIO();
        bool uiWantsMouse = io.WantCaptureMouse;

        float dt = (useFixedDt ? fixedDt : GetFrameTime()) * std::max(0.0f, timeScale);
        if (!paused) {
            double t0 = GetTime();
            bool ok = true;
            switch (integrator) {
                case Integrator::SemiImplicitEuler:
                    ok = StepSemiImplicitEuler(bodies, accel, dt, G, static_cast<double>(softening) * static_cast<double>(softening), maxSpeed);
                    break;
                case Integrator::VelocityVerlet:
                    ok = StepVelocityVerlet(bodies, accel, dt, G, static_cast<double>(softening) * static_cast<double>(softening), maxSpeed);
                    break;
            }
            double t1 = GetTime();
            lastStepMs = (t1 - t0) * 1000.0;
            if (!ok) paused = true;
            if (drawTrails) UpdateTrails(trails, bodies, trailMax);
        }
        if (requestStep) {
            double t0 = GetTime();
            bool ok = true;
            switch (integrator) {
                case Integrator::SemiImplicitEuler:
                    ok = StepSemiImplicitEuler(bodies, accel, fixedDt, G, static_cast<double>(softening) * static_cast<double>(softening), maxSpeed);
                    break;
                case Integrator::VelocityVerlet:
                    ok = StepVelocityVerlet(bodies, accel, fixedDt, G, static_cast<double>(softening) * static_cast<double>(softening), maxSpeed);
                    break;
            }
            double t1 = GetTime();
            lastStepMs = (t1 - t0) * 1000.0;
            if (!ok) paused = true;
            if (drawTrails) UpdateTrails(trails, bodies, trailMax);
        }

        float wheel = GetMouseWheelMove();
        if (!uiWantsMouse && wheel != 0.0f) {
            Vector2 mouse = GetMousePosition();
            Vector2 worldBefore = GetScreenToWorld2D(mouse, cam);
            float newZoom = std::clamp(cam.zoom * (1.0f + wheel * 0.1f), 0.05f, 10.0f);
            cam.zoom = newZoom;
            Vector2 worldAfter = GetScreenToWorld2D(mouse, cam);
            cam.target = Vector2Add(cam.target, Vector2Subtract(worldBefore, worldAfter));
        }

        if (!uiWantsMouse) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                Vector2 w = GetScreenToWorld2D(GetMousePosition(), cam);
                lmbPickCandidate = PickBody(bodies, w, 24.0f / cam.zoom);
                lmbPanning = (lmbPickCandidate == -1);
                lmbDragDistSq = 0.0f;
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                Vector2 dpx = GetMouseDelta();
                lmbDragDistSq += dpx.x*dpx.x + dpx.y*dpx.y;
                if (lmbPanning) {
                    Vector2 delta = Vector2Scale(dpx, -1.0f / cam.zoom);
                    cam.target = Vector2Add(cam.target, delta);
                }
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                if (!lmbPanning && lmbPickCandidate >= 0 && lmbDragDistSq <= selectThresholdSq) {
                    selected = lmbPickCandidate;
                }
                lmbPanning = false;
                lmbPickCandidate = -1;
                lmbDragDistSq = 0.0f;
            }
        }

        showDrag = false;
        if (!uiWantsMouse && selected >= 0 && selected < static_cast<int>(bodies.size())) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) draggingVel = true;
            if (draggingVel && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                dragWorld = GetScreenToWorld2D(GetMousePosition(), cam);
                showDrag = true;
                paused = true;
                bodies[selected].velocity = Vector2Scale(Vector2Subtract(dragWorld, bodies[selected].position), dragVelScale);
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) draggingVel = false;
        }

        BeginMode2D(cam);

        DrawWorldGrid(cam, 50.0f);

        if (drawTrails) {
            for (size_t i = 0; i < trails.size(); ++i) {
                const auto& t = trails[i];
                for (size_t k = 1; k < t.size(); ++k) {
                    Color c = bodies[i].color;
                    c.a = static_cast<unsigned char>(
                        std::clamp(20 + static_cast<int>(230.0 * static_cast<double>(k) /
                                                       std::max(1, static_cast<int>(t.size()))),
                                   20, 250));
                    DrawLineV(t[k-1], t[k], c);
                }
            }
        }

        for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
            float r = static_cast<float>(
                std::cbrt(std::max(1.0, static_cast<double>(bodies[i].mass))));
            DrawCircleV(bodies[i].position, r, bodies[i].color);
            if (drawVelocity) {
                Vector2 tip = Vector2Add(bodies[i].position, Vector2Scale(bodies[i].velocity, 10.0f));
                DrawLineEx(bodies[i].position, tip, 1.5f, WHITE);
            }
            if (drawAcceleration && i < static_cast<int>(accel.size())) {
                Vector2 tip = Vector2Add(bodies[i].position, Vector2Scale(accel[i], 500.0f));
                DrawLineEx(bodies[i].position, tip, 1.0f, ORANGE);
            }
        }

        if (selected >= 0 && selected < static_cast<int>(bodies.size())) {
            float r = static_cast<float>(
                          std::cbrt(std::max(1.0, static_cast<double>(bodies[selected].mass)))) +
                      4.0f;
            DrawRing(bodies[selected].position, r, r + 3.5f, 0, 360, 32, YELLOW);
        }

        if (showDrag && selected >= 0 && selected < static_cast<int>(bodies.size())) {
            DrawLineEx(bodies[selected].position, dragWorld, 2.0f, YELLOW);
            DrawCircleV(dragWorld, 3.0f, YELLOW);
        }

        EndMode2D();

        rlImGuiEnd();
        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
