# CMakeLists.txt
cmake_minimum_required(VERSION 3.23)
# Enable both C and C++ since vendored deps (raylib, flecs amalgamation) use C
project(raylib_nbody LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress developer warning about OpenGL deprecation on macOS (raylib dependency)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE INTERNAL "")

# We'll add strict warnings only to our own target later

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external)
option(USE_VENDORED_DEPS "Use dependencies from external directory" ON)

# raylib: prefer vendored; if missing and a system package exists, use it
set(RAYLIB_TARGET raylib)
if (USE_VENDORED_DEPS)
    if (EXISTS ${EXTERNAL_DIR}/raylib/CMakeLists.txt)
        message(STATUS "Using vendored raylib at ${EXTERNAL_DIR}/raylib")
        add_subdirectory(${EXTERNAL_DIR}/raylib)
    else()
        find_package(raylib QUIET)
        if (NOT raylib_FOUND)
            message(FATAL_ERROR "Vendored raylib not found and no system raylib available. Initialize submodule or set USE_VENDORED_DEPS=OFF.")
        endif()
    endif()
else()
    find_package(raylib REQUIRED)
endif()
if (TARGET raylib::raylib)
    set(RAYLIB_TARGET raylib::raylib)
endif()

# raylib-cpp: header-only C++ wrapper; prefer vendored, else allow RAYLIB_CPP_DIR override
set(RAYLIB_CPP_DIR ${EXTERNAL_DIR}/raylib-cpp)
set(RAYLIB_CPP_HEADER_PATH)
if (EXISTS ${RAYLIB_CPP_DIR}/include/raylib-cpp.hpp)
    set(RAYLIB_CPP_HEADER_PATH ${RAYLIB_CPP_DIR}/include)
elseif (EXISTS ${RAYLIB_CPP_DIR}/raylib-cpp.hpp)
    # Some checkouts keep header at repo root
    set(RAYLIB_CPP_HEADER_PATH ${RAYLIB_CPP_DIR})
elseif(DEFINED RAYLIB_CPP_DIR AND EXISTS ${RAYLIB_CPP_DIR}/include/raylib-cpp.hpp)
    set(RAYLIB_CPP_HEADER_PATH ${RAYLIB_CPP_DIR}/include)
elseif(DEFINED RAYLIB_CPP_DIR AND EXISTS ${RAYLIB_CPP_DIR}/raylib-cpp.hpp)
    set(RAYLIB_CPP_HEADER_PATH ${RAYLIB_CPP_DIR})
else()
    message(FATAL_ERROR "raylib-cpp not found. Provide vendored external/raylib-cpp or set RAYLIB_CPP_DIR to a local checkout.")
endif()

add_library(raylib_cpp INTERFACE)
target_include_directories(raylib_cpp INTERFACE ${RAYLIB_CPP_HEADER_PATH})
target_link_libraries(raylib_cpp INTERFACE ${RAYLIB_TARGET})

# imgui: prefer vendored; otherwise allow specifying IMGUI_DIR to a system/local install
set(IMGUI_DIR ${EXTERNAL_DIR}/imgui)
if (EXISTS ${IMGUI_DIR}/imgui.cpp)
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
    )
    target_include_directories(imgui PUBLIC ${IMGUI_DIR})
    if (EXISTS ${IMGUI_DIR}/imgui_demo.cpp)
        target_sources(imgui PRIVATE ${IMGUI_DIR}/imgui_demo.cpp)
    endif()
elseif(DEFINED IMGUI_DIR AND EXISTS ${IMGUI_DIR}/imgui.cpp)
    message(STATUS "Using system/local ImGui at ${IMGUI_DIR}")
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
    )
    target_include_directories(imgui PUBLIC ${IMGUI_DIR})
    if (EXISTS ${IMGUI_DIR}/imgui_demo.cpp)
        target_sources(imgui PRIVATE ${IMGUI_DIR}/imgui_demo.cpp)
    endif()
else()
    message(FATAL_ERROR "ImGui not found. Provide vendored external/imgui or set IMGUI_DIR to a local install.")
endif()

# rlImGui: prefer vendored; otherwise allow specifying RLIMGUI_DIR
set(RLIMGUI_DIR ${EXTERNAL_DIR}/rlImGui)
if (EXISTS ${RLIMGUI_DIR}/rlImGui.cpp)
    add_library(rlImGui STATIC ${RLIMGUI_DIR}/rlImGui.cpp)
    target_include_directories(rlImGui PUBLIC ${RLIMGUI_DIR} ${IMGUI_DIR})
    target_link_libraries(rlImGui PUBLIC ${RAYLIB_TARGET} imgui)
elseif(DEFINED RLIMGUI_DIR AND EXISTS ${RLIMGUI_DIR}/rlImGui.cpp)
    message(STATUS "Using system/local rlImGui at ${RLIMGUI_DIR}")
    add_library(rlImGui STATIC ${RLIMGUI_DIR}/rlImGui.cpp)
    target_include_directories(rlImGui PUBLIC ${RLIMGUI_DIR} ${IMGUI_DIR})
    target_link_libraries(rlImGui PUBLIC ${RAYLIB_TARGET} imgui)
else()
    message(FATAL_ERROR "rlImGui not found. Provide vendored external/rlImGui or set RLIMGUI_DIR to a local checkout.")
endif()

# flecs: prefer amalgamated source (flecs.c/flecs.h) in external/flecs, else submodule CMake, else system package
set(FLECS_DIR ${EXTERNAL_DIR}/flecs)
if (USE_VENDORED_DEPS)
    if (EXISTS ${FLECS_DIR}/flecs.c AND EXISTS ${FLECS_DIR}/flecs.h)
        add_library(flecs STATIC ${FLECS_DIR}/flecs.c)
        target_include_directories(flecs PUBLIC ${FLECS_DIR})
    elseif (EXISTS ${FLECS_DIR}/CMakeLists.txt)
        add_subdirectory(${FLECS_DIR})
    else()
        find_package(flecs CONFIG QUIET)
        if (NOT flecs_FOUND)
            message(FATAL_ERROR "flecs not found. Provide vendored external/flecs or install a system flecs package.")
        endif()
    endif()
endif()

# Normalize flecs target name
set(FLECS_LIB flecs)
if (TARGET flecs::flecs)
    set(FLECS_LIB flecs::flecs)
elseif (TARGET flecs_static)
    set(FLECS_LIB flecs_static)
endif()

add_executable(raylib_nbody
    src/main.cpp
    src/core/Config.hpp
    src/components/Components.hpp
    src/systems/Physics.hpp
    src/systems/WorldRenderer.hpp
    src/systems/UI.hpp
    src/systems/Camera.hpp
    src/systems/Interaction.hpp
    src/systems/Collision.hpp
    src/core/Scenario.hpp
)

target_include_directories(raylib_nbody
    PRIVATE
        ${IMGUI_DIR}
        ${RLIMGUI_DIR}
        ${FLECS_DIR}
        ${FLECS_DIR}/include
        ${FLECS_DIR}/include/flecs/addons/cpp
        src
)

target_link_libraries(raylib_nbody
    PRIVATE
        raylib_cpp
        rlImGui
        ${FLECS_LIB}
)

# Add strict compiler warnings only to our own code
if(MSVC)
    target_compile_options(raylib_nbody PRIVATE /W4)
else()
    target_compile_options(raylib_nbody PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

if (APPLE)
    target_link_libraries(raylib_nbody PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
elseif (UNIX AND NOT APPLE)
    target_link_libraries(raylib_nbody PRIVATE m pthread GL dl X11)
endif()
