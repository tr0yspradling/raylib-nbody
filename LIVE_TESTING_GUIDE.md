# Live Testing & Development Guide

## Quick Start

### 🚀 **Immediate Testing**
```bash
# Build and test both versions
./dev.sh

# Or run individual versions:
./dev.sh original      # Original monolithic version
./dev.sh refactored    # New clean architecture version
```

### 🔨 **Development Workflow**

1. **Make changes to refactored code** in `src/core/`, `src/simulation/`, `src/input/`, or `src/ui/`
2. **Quick build & test:**
   ```bash
   ./dev.sh refactored
   ```
3. **Build both versions:**
   ```bash
   cmake --build build -j
   ```

## Available Executables

| Executable | Description | Architecture |
|------------|-------------|--------------|
| `./build/raylib_nbody` | Original version | Monolithic |
| `./build/raylib_nbody_refactored` | New version | Clean Architecture |

## Testing Both Versions

### Automated Sequential Testing
```bash
./test_versions.sh
```
Launches original version first, then refactored version after you close the first window.

### Manual Testing
```bash
# Original
./build/raylib_nbody

# Refactored  
./build/raylib_nbody_refactored
```

## Build System

### Full Build
```bash
cmake --build build -j
```

### Individual Targets
```bash
cmake --build build -j --target raylib_nbody           # Original only
cmake --build build -j --target raylib_nbody_refactored # Refactored only
```

### Clean Build
```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

## Development Features

### Live Comparison
- Both versions share the same physics simulation
- Same UI controls and features
- Identical visual output
- Performance comparison possible

### Hot Reload Workflow
1. Edit refactored code
2. Run `./dev.sh refactored`
3. Test changes immediately
4. Compare with original if needed

## Code Structure

### Original (Monolithic)
```
src/main.cpp
src/systems/Physics.hpp    (340+ lines, multiple responsibilities)
src/systems/UI.hpp         (500+ lines, all UI in one place)
src/systems/Interaction.hpp (mixed concerns)
```

### Refactored (Clean Architecture)
```
src/main_refactored.cpp
src/core/Application.hpp      (orchestration)
src/simulation/PhysicsEngine.hpp (focused physics)
src/input/InputManager.hpp    (command pattern)
src/ui/UIManager.hpp          (modular panels)
```

## Testing Checklist

### ✅ **Basic Functionality**
- [ ] Window opens without crashes
- [ ] Initial 3-body system displays
- [ ] Physics simulation runs
- [ ] UI panels appear

### ✅ **Physics**
- [ ] Bodies orbit correctly
- [ ] Gravity simulation works
- [ ] Time controls function
- [ ] Integrator switching works

### ✅ **Interaction**
- [ ] Mouse selection works
- [ ] Camera pan/zoom works
- [ ] Body addition works
- [ ] Keyboard shortcuts work

### ✅ **UI**
- [ ] All panels display
- [ ] Controls are responsive
- [ ] Settings changes apply
- [ ] No UI crashes

## Performance Monitoring

### Frame Rate
Both versions should maintain similar FPS. The refactored version may have slight overhead due to indirection but should be negligible.

### Memory Usage
Monitor for memory leaks during extended runs:
```bash
# macOS
leaks raylib_nbody_refactored

# Linux
valgrind ./build/raylib_nbody_refactored
```

## Troubleshooting

### Common Issues

#### Build Failures
- **Missing dependencies**: Run `git submodule update --init --recursive`
- **CMake cache issues**: Delete `build/` and reconfigure
- **Compiler errors**: Check C++23 support

#### Runtime Issues
- **Flecs errors**: Usually component registration issues
- **Graphics issues**: Check OpenGL/Metal drivers
- **UI crashes**: ImGui state problems

#### Performance Issues
- **Slow physics**: Check debug vs release build
- **UI lag**: Too many entities for rendering
- **Memory leaks**: Entity cleanup problems

### Debug Builds
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

### Verbose Output
```bash
cmake --build build -j --verbose
```

## Next Steps

### Phase 1: Current (✅ Complete)
- [x] Both versions compile and run
- [x] Basic functionality preserved
- [x] Architecture separation achieved

### Phase 2: Enhancement
- [ ] Complete UI panel separation
- [ ] Full Strategy pattern integration
- [ ] Event system implementation
- [ ] Command pattern completion

### Phase 3: Advanced
- [ ] Unit testing framework
- [ ] Performance benchmarking
- [ ] Additional integrators
- [ ] Plugin architecture

## Quick Reference

### Essential Commands
```bash
# Quick development cycle
./dev.sh refactored

# Compare versions
./test_versions.sh

# Clean rebuild
rm -rf build && cmake -S . -B build && cmake --build build -j

# Run specific version
./build/raylib_nbody_refactored
```

### Key Files to Edit
- **Core**: `src/core/Application.hpp`
- **Physics**: `src/simulation/PhysicsEngine.hpp`
- **Input**: `src/input/InputManager.hpp`
- **UI**: `src/ui/panels/TimeControlPanel.hpp`

The refactored architecture is ready for live development and testing! 🎉