# ============================================================
# ui_input — headless pointer routing test
#
# Links the real camera implementation against Dear ImGui and drives
# both from synthetic mouse events. No window, no GL context.
# ============================================================
add_executable(ui_input_test
    ui_input_test.cpp
    ${CMAKE_SOURCE_DIR}/src/camera.cpp
    ${CMAKE_SOURCE_DIR}/src/ui_layout.cpp
)
target_include_directories(ui_input_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(ui_input_test SYSTEM PRIVATE ${EIGEN_INCLUDE_DIRS})
target_compile_options(ui_input_test PRIVATE
    -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion
)
target_link_libraries(ui_input_test PRIVATE imgui)

add_test(NAME ui_input COMMAND ui_input_test)
