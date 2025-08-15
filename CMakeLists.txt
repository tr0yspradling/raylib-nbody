# CMakeLists.txt
cmake_minimum_required(VERSION 3.23)
project(raylib_nbody CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(raylib REQUIRED)

include(FetchContent)

# ImGui (docking branch to match rlImGui main API)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)
FetchContent_MakeAvailable(imgui)

# rlImGui
FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG main
)
FetchContent_MakeAvailable(rlimgui)

add_executable(raylib_nbody
    src/main.cpp

    # ImGui core
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp

    # Optional demo (good for testing UI, safe to remove later)
    ${imgui_SOURCE_DIR}/imgui_demo.cpp

    # rlImGui backend
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)

target_include_directories(raylib_nbody
    PRIVATE
        ${imgui_SOURCE_DIR}
        ${rlimgui_SOURCE_DIR}
)

target_link_libraries(raylib_nbody
    PRIVATE
        raylib
)

if (APPLE)
    target_link_libraries(raylib_nbody PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
elseif (UNIX AND NOT APPLE)
    target_link_libraries(raylib_nbody PRIVATE m pthread GL dl X11)
endif()