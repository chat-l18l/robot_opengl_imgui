cmake_minimum_required(VERSION 3.14)
project(imgui_robot_viewer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Packages
# ============================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
pkg_check_modules(GL REQUIRED gl)
pkg_check_modules(EIGEN REQUIRED eigen3)

# ============================================================
# ImGui (fetch from GitHub)
# ============================================================
include(FetchContent)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
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
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC ${GLFW_LIBRARIES} ${GL_LIBRARIES})

# ============================================================
# Robot Viewer executable
# ============================================================
add_executable(robot_viewer
    src/main.cpp
    src/robot.cpp
)
target_include_directories(robot_viewer PRIVATE
    src
    ${EIGEN_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
)
target_link_libraries(robot_viewer PRIVATE
    imgui
    ${GLFW_LIBRARIES}
    ${GL_LIBRARIES}
    dl
    pthread
)
