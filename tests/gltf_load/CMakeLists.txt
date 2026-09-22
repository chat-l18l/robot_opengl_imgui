# ============================================================
# gltf_load — headless loader test
#
# Loads the generated test files and checks structure, kinematics and
# repairs against values known in advance. Nothing is uploaded, so it
# needs no window and no GL context; the GL library is only linked
# because the mesh and shader code reference its entry points.
# ============================================================
add_executable(gltf_load_test
    gltf_load_test.cpp
    ${CMAKE_SOURCE_DIR}/src/gltf_load.cpp
    ${CMAKE_SOURCE_DIR}/src/robot.cpp
    ${CMAKE_SOURCE_DIR}/src/gl_mesh.cpp
    ${CMAKE_SOURCE_DIR}/src/gl_shader.cpp
)
target_include_directories(gltf_load_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(gltf_load_test SYSTEM PRIVATE
    ${cgltf_SOURCE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
)
target_compile_definitions(gltf_load_test PRIVATE
    RBT_MODELS_DIR="${CMAKE_SOURCE_DIR}/models"
    RBT_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(gltf_load_test PRIVATE
    -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion
)
target_link_libraries(gltf_load_test PRIVATE ${GL_LIBRARIES})

add_test(NAME gltf_load COMMAND gltf_load_test)
