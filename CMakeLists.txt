# CMakeLists.txt
cmake_minimum_required(VERSION 3.23)
project(raylib_nbody CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external)
option(USE_VENDORED_DEPS "Use dependencies from external directory" ON)

# raylib: require vendored when USE_VENDORED_DEPS=ON
set(RAYLIB_TARGET raylib)
if (USE_VENDORED_DEPS)
    if (EXISTS ${EXTERNAL_DIR}/raylib/CMakeLists.txt)
        message(STATUS "Using vendored raylib at ${EXTERNAL_DIR}/raylib")
        add_subdirectory(${EXTERNAL_DIR}/raylib)
    else()
        message(FATAL_ERROR "Vendored raylib not found at ${EXTERNAL_DIR}/raylib. Initialize submodule or set USE_VENDORED_DEPS=OFF.")
    endif()
else()
    find_package(raylib REQUIRED)
endif()

# imgui: vendored sources
set(IMGUI_DIR ${EXTERNAL_DIR}/imgui)
if (USE_VENDORED_DEPS)
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
    else()
        message(FATAL_ERROR "Vendored imgui not found at ${IMGUI_DIR}. Please place imgui sources there.")
    endif()
endif()

# rlImGui: vendored single cpp
set(RLIMGUI_DIR ${EXTERNAL_DIR}/rlImGui)
if (USE_VENDORED_DEPS)
    if (EXISTS ${RLIMGUI_DIR}/rlImGui.cpp)
        add_library(rlImGui STATIC ${RLIMGUI_DIR}/rlImGui.cpp)
        target_include_directories(rlImGui PUBLIC ${RLIMGUI_DIR} ${IMGUI_DIR})
        target_link_libraries(rlImGui PUBLIC ${RAYLIB_TARGET} imgui)
    else()
        message(FATAL_ERROR "Vendored rlImGui not found at ${RLIMGUI_DIR}. Please place rlImGui there.")
    endif()
endif()

# flecs: prefer amalgamated source (flecs.c/flecs.h) in external/flecs, else submodule CMake
set(FLECS_DIR ${EXTERNAL_DIR}/flecs)
if (USE_VENDORED_DEPS)
    if (EXISTS ${FLECS_DIR}/flecs.c AND EXISTS ${FLECS_DIR}/flecs.h)
        add_library(flecs STATIC ${FLECS_DIR}/flecs.c)
        target_include_directories(flecs PUBLIC ${FLECS_DIR})
    elseif (EXISTS ${FLECS_DIR}/CMakeLists.txt)
        add_subdirectory(${FLECS_DIR})
    else()
        message(FATAL_ERROR "Vendored flecs not found. Place flecs.c/flecs.h or the full project under ${FLECS_DIR}.")
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
    src/core/Camera.hpp
    src/components/Components.hpp
    src/systems/Physics.cpp
    src/systems/Render.cpp
    src/systems/UI.cpp
)

target_include_directories(raylib_nbody
    PRIVATE
        ${IMGUI_DIR}
        ${RLIMGUI_DIR}
        ${FLECS_DIR}
        src
)

target_link_libraries(raylib_nbody
    PRIVATE
        ${RAYLIB_TARGET}
        imgui
        rlImGui
        ${FLECS_LIB}
)

if (APPLE)
    target_link_libraries(raylib_nbody PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
elseif (UNIX AND NOT APPLE)
    target_link_libraries(raylib_nbody PRIVATE m pthread GL dl X11)
endif()
