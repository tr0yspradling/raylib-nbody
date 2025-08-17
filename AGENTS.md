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
- raylib must be installed locally; ImGui and rlImGui are fetched during configure.

---

## Project Structure

- Source entry: `src/main.cpp` (C++23).
- Build system: `CMakeLists.txt` uses FetchContent for ImGui/rlImGui.
- Add new modules under `src/` (e.g., `src/physics.*`, `src/ui.*`).

---

## Architecture Overview

**Libraries and language**

- raylib for windowing and rendering.
- ImGui + rlImGui for UI.
- C++23 with STL containers.

**Core simulation components**

- `Body` struct: position, velocity, mass, color, pinned state.
- Physics integrators: Semi-Implicit Euler and Velocity Verlet.
- UI: time control, physics parameters, visuals, body editing.
- Camera: pan, zoom, body selection, velocity dragging.
- Diagnostics: energy checks with auto-pause on non-finite values.

**Key patterns**

- Physics loop: acceleration computation → integration step → trail update.
- Immediate-mode UI state in separate control windows.
- Input routing between UI and world space.
- Rendering: 2D camera for world; overlay UI outside camera.

---

## Code Style and Naming

- Format with `.clang-format` (LLVM base, 4 spaces, 120 columns).
- Run: `clang-format -i src/*.cpp src/*.h*`
- Naming: Types `PascalCase`, functions `CamelCase`, variables `lowerCamelCase`.
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