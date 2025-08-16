# Repository Guidelines

## Project Structure & Module Organization
- Source: `src/main.cpp` (C++23) with raylib + ImGui + rlImGui.
- Build config: `CMakeLists.txt` (FetchContent pulls ImGui/rlImGui).
- Assets/tests: none yet. Add future modules under `src/` (e.g., `src/physics.*`, `src/ui.*`).

## Build, Test, and Development Commands
- Configure (out-of-source): `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`
- Build: `cmake --build build -j`
- Run: `./build/raylib_nbody` (or from CLion: `cmake-build-debug/raylib_nbody`)
- Clean: `cmake --build build --target clean`
Notes: raylib must be installed on your system; ImGui/rlImGui are fetched automatically during configure.

## Coding Style & Naming Conventions
- Language: C++23. Format with `.clang-format` (LLVM base, 4 spaces, 120 cols).
- Run formatter: `clang-format -i src/*.cpp src/*.h*`
- Naming: Types `PascalCase` (e.g., `Body`), functions `CamelCase` (e.g., `ComputeDiagnostics`), variables `lowerCamelCase`.
- Structure: Keep physics, UI, and rendering concerns in separate files when adding code.

## Testing Guidelines
- No automated tests yet. Validate changes by building and running the app.
- Quick manual checks: switch integrator, pause/step, adjust `G`/softening, pan/zoom, add/remove bodies, confirm diagnostics remain finite.
- If adding a test harness, prefer lightweight CTest targets wired via CMake.

## Commit & Pull Request Guidelines
- Commits: Imperative mood, concise subject, optional scope. Example: `physics: cap velocity in Verlet step`.
- Group related changes; keep diffs focused. Include brief rationale in body if behavior changes.
- PRs: clear description, how to run/build, platform notes (macOS/Linux), and screenshots/gifs for UI changes.
- Ensure code is formatted, builds cleanly, and runs without runtime errors before requesting review.

## Security & Configuration Tips
- Dependencies: install raylib (e.g., macOS `brew install raylib`; Ubuntu `sudo apt install libraylib-dev`).
- macOS linking frameworks are handled in CMake; Linux links `m`, `pthread`, `GL`, `dl`, `X11` automatically.
