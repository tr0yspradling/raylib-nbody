# AGENTS

Single source of truth for OpenAI agents and Claude Code. Link or symlink `CLAUDE.md` to this file.

---

## Build and Run

**Canonical out-of-source flow**

- Configure: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`
- Build: `cmake --build build -j`
- Run: `./build/raylib_nbody`
- Clean: `cmake --build build --target clean`

**IDE sandbox or existing in-source build**

- Build: `cmake --build cmake-build-debug`
- Run: `./cmake-build-debug/raylib_nbody`
- Clean: `cmake --build cmake-build-debug --target clean`

**Notes**

- Works with Ninja and default CMake generators. Prefer the canonical out-of-source flow when possible.
- Ensure submodules are present: `git submodule update --init --recursive`.
- raylib development files must be available locally. Other deps (raylib-cpp, ImGui, rlImGui, flecs) are vendored.

---

## Project Structure

- Source entry: `src/main.cpp` (C++23).
- ECS components in `src/components/`, shared config/constants in `src/core/`.
- Systems (physics, camera, interaction, rendering, UI) live under `src/systems/`.
- Dependencies are managed via vendored submodules in `external/`.

---

## Architecture Overview

**Libraries and language**

- raylib for windowing and rendering (via raylib-cpp wrapper).
- flecs for the ECS world.
- ImGui + rlImGui for UI.
- C++23 with STL containers.

**Core simulation components**

- ECS components: `Position`, `Velocity`, `Acceleration`, `PrevAcceleration`, `Mass`,
  `Pinned`, `Tint`, `Trail`, `Selectable`, `Selected`, `Draggable`.
- Systems: physics (gravity, integration, trails), camera, interaction, rendering, and UI.
- Integrators: Semi-Implicit Euler and Velocity Verlet.
- Diagnostics: energy/momentum checks with auto-pause on non-finite values.

**Key patterns**

- flecs systems drive the update loop.
- Physics loop: acceleration computation → integration step → trail update.
- Immediate-mode UI state in separate control windows.
- Input routing between UI and world space.
- Rendering: 2D camera for world; overlay UI outside camera.

---

## Code Style and Naming

- Format with `.clang-format` (LLVM base, 4 spaces, 120 columns).
- Run: `clang-format -i src/*.cpp src/*.h*`
- Naming:
  - Types and ECS components: `PascalCase` (e.g., `Position`, `WorldRenderer`).
  - Functions: `CamelCase` (e.g., `Register`, `CenterOnCenterOfMass`).
  - General variables: `lowerCamelCase`.
  - Constants and Config fields: `snake_case` (e.g., `default_time_scale`, `radius_scale`, `min_body_radius`).
- Keep physics, UI, and rendering concerns in separate files.

---

## Testing

- No automated tests yet; validate by building and running.
- Manual checks: switch integrator, pause/step, adjust `G` and softening, pan/zoom, add/remove bodies, confirm diagnostics remain finite.
- If adding tests, prefer lightweight CTest targets via CMake.

---

## Contribution

- Commits: imperative mood, concise subject, optional scope. Example: `physics: cap velocity in Verlet step`.
- Group related changes and keep diffs focused; include brief rationale if behavior changes.
- PRs: include build/run steps, platform notes, and screenshots or gifs for UI changes.
- Ensure code is formatted, builds cleanly, and runs without runtime errors before review.

---

## Platform Notes

- Install raylib. Examples: macOS `brew install raylib`; Ubuntu `sudo apt install libraylib-dev`.
- macOS frameworks are handled in CMake. Linux links `m`, `pthread`, `GL`, `dl`, and `X11`.
