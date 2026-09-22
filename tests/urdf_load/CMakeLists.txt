# ============================================================
# urdf_load — headless URDF loader test
#
# Loads a hand-written test robot and checks structure, kinematics,
# geometry, colours and the COLLADA up axis against values worked out
# in advance. Nothing is uploaded, so it needs no window and no GL
# context; the GL library is only linked because the mesh and shader
# code reference its entry points.
# ============================================================
add_executable(urdf_load_test
    urdf_load_test.cpp
    ${CMAKE_SOURCE_DIR}/src/urdf_load.cpp
    ${CMAKE_SOURCE_DIR}/src/mesh_import.cpp
    ${CMAKE_SOURCE_DIR}/src/robot.cpp
    ${CMAKE_SOURCE_DIR}/src/gl_mesh.cpp
    ${CMAKE_SOURCE_DIR}/src/gl_shader.cpp
)
target_include_directories(urdf_load_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(urdf_load_test SYSTEM PRIVATE
    ${EIGEN_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
    ${TINYXML2_INCLUDE_DIRS}
)
target_compile_definitions(urdf_load_test PRIVATE
    RBT_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(urdf_load_test PRIVATE
    -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion
)
target_link_libraries(urdf_load_test PRIVATE
    ${GL_LIBRARIES}
    ${TINYXML2_LINK_LIBRARIES}
    assimp::assimp
)

add_test(NAME urdf_load COMMAND urdf_load_test)
