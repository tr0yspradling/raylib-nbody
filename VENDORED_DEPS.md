Vendored Dependencies (Git Submodules)

This repo vendors third‑party libraries under `external/` via Git submodules. No network fetch is performed at CMake time.

Submodules
- `external/raylib` → https://github.com/raysan5/raylib.git (branch: master)
- `external/imgui` → https://github.com/ocornut/imgui.git (branch: docking)
- `external/rlImGui` → https://github.com/raylib-extras/rlImGui.git (branch: main)
- `external/flecs` → https://github.com/SanderMertens/flecs.git (branch: v3)

Setup
- Initial clone: `git clone <repo> && cd <repo>`
- If submodules are not yet added (fresh fork), add them:
  - `git submodule add -b master https://github.com/raysan5/raylib.git external/raylib`
  - `git submodule add -b docking https://github.com/ocornut/imgui.git external/imgui`
  - `git submodule add -b main https://github.com/raylib-extras/rlImGui.git external/rlImGui`
  - `git submodule add -b v3 https://github.com/SanderMertens/flecs.git external/flecs`
  - `git commit -m "chore: add vendored submodules"`
- Initialize submodules: `git submodule update --init --recursive`
- Update submodules: `git submodule update --remote --recursive`

Notes
- ImGui does not ship a CMake project; our build compiles its core .cpp files directly from `external/imgui`.
- rlImGui is used either via its own CMake (if present) or as a single `rlImGui.cpp` fallback.
- flecs is built from its submodule CMake. If needed, you can swap to the amalgamated pair (`flecs.c`/`flecs.h`).
- raylib is required as a submodule by default. To use a system raylib instead, configure with `-DUSE_VENDORED_DEPS=OFF`.

Build
- Configure: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_VENDORED_DEPS=ON`
- Build: `cmake --build build -j`
- Run: `./build/raylib_nbody`
