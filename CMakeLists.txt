cmake_minimum_required(VERSION 3.14)
project(imgui_robot_viewer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Build type
#
# An empty CMAKE_BUILD_TYPE means no optimisation flags at all.
# Eigen expands into deep template code that only collapses under
# an optimiser, so an unconfigured build runs an order of magnitude
# slower than it should. Default to Release; -DCMAKE_BUILD_TYPE=Debug
# still works.
# ============================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                 Debug Release RelWithDebInfo MinSizeRel)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
# Packages
# ============================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
pkg_check_modules(GL REQUIRED gl)
pkg_check_modules(EIGEN REQUIRED eigen3)

# ============================================================
# ImGui (fetched from GitHub)
# ============================================================
include(FetchContent)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8-docking
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_DIR ${imgui_SOURCE_DIR})
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
# SYSTEM: third-party headers must not trip our own warning flags.
target_include_directories(imgui SYSTEM PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC ${GLFW_LIBRARIES} ${GL_LIBRARIES})

# ============================================================
# Robot viewer executable
# ============================================================
add_executable(robot_viewer
    src/main.cpp
    src/camera.cpp
    src/gl_mesh.cpp
    src/gl_shader.cpp
    src/robot.cpp
)
target_include_directories(robot_viewer PRIVATE src)
target_include_directories(robot_viewer SYSTEM PRIVATE
    ${EIGEN_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
)
target_compile_options(robot_viewer PRIVATE
    -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion
)
target_link_libraries(robot_viewer PRIVATE
    imgui
    ${GLFW_LIBRARIES}
    ${GL_LIBRARIES}
    dl
    pthread
)
