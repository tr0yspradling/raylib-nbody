# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

The project uses CMake with Ninja generator:

- **Build executable**: `cmake --build cmake-build-debug` (builds the `raylib_nbody` executable)
- **Clean**: `cmake --build cmake-build-debug --target clean`
- **Run application**: `./cmake-build-debug/raylib_nbody`

## Code Architecture

This is a real-time N-body gravity simulation built with:

- **Core Libraries**: raylib (graphics/window), ImGui + rlImGui (UI)
- **Language**: C++23 with standard library containers
- **Main Components**:
  - `Body` struct: position, velocity, mass, color, pinned state
  - Physics integrators: Semi-Implicit Euler and Velocity Verlet
  - Interactive UI windows for time control, physics parameters, visuals, body editing
  - Camera system with pan/zoom, body selection, velocity dragging

## Key Architecture Patterns

- **Physics Loop**: Acceleration computation → integration step → trail updates
- **UI State Management**: ImGui immediate mode with separate control windows
- **Input Handling**: Mouse interactions differentiated between UI and world space
- **Rendering Pipeline**: 2D camera mode for world objects, overlay UI outside camera
- **Diagnostics**: Real-time energy conservation monitoring with auto-pause on non-finite values

## Code Style

The project follows `.clang-format` configuration with LLVM base style, 4-space indentation, and 120-column limit.