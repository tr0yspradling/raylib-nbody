# N-Body Gravity Simulation

Real-time, interactive N-body gravity simulation built with raylib, flecs, and ImGui.

## Features

- **Real-time Physics**: Two integration methods (Semi-Implicit Euler, Velocity Verlet)
- **Interactive Controls**: Pan, zoom, select bodies, drag to set velocities
- **Visual Elements**: Particle trails, velocity/acceleration vectors, grid overlay
- **Live Diagnostics**: Energy conservation monitoring, momentum tracking
- **Body Management**: Add, remove, edit masses and velocities via UI
- **Scenarios**: Built-in three-body system with momentum zeroing

## Controls

- **Mouse Wheel**: Zoom in/out
- **Left Click + Drag**: Pan viewport (empty space) or select body (on particle)
- **Right Click + Drag**: Set velocity for selected body
- **UI Panels**: Complete control over simulation parameters

## Building

Requires the raylib development package installed on your system. Other dependencies are
vendored as Git submodules.

```bash
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/raylib_nbody
```

## Dependencies

- raylib (graphics and windowing)
- raylib-cpp (C++ wrapper for raylib)
- ImGui (UI framework)
- rlImGui (raylib-ImGui bridge)
- flecs (entity-component system)
