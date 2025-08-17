# N-Body Gravity Simulation

Real-time, interactive N-body gravity simulation built with raylib and ImGui.

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

Requires raylib installed on your system.

```bash
cmake --build cmake-build-debug
./cmake-build-debug/raylib_nbody
```

## Dependencies

- raylib (graphics and windowing)
- ImGui (UI framework, fetched automatically)
- rlImGui (raylib-ImGui integration, fetched automatically)

## TODO

High Priority (Correctness/Maintainability):
1. Remove dead code (Selection, CameraState, unused components)
2. Consolidate duplicate constants into shared header
3. Add proper null checks and error handling

Medium Priority (Code Quality):
1. Extract constants for magic numbers
2. Break down large methods (draw_ui, ProcessMouseInput)
3. Move static functions into appropriate classes/namespaces

Low Priority (Performance):
1. Optimize data access patterns in physics
2. Consider object pooling for frequent allocations
3. Add spatial partitioning for large N-body simulations
