/**
 * @file gl_shader.h
 * @brief Shader program with uniform locations resolved once at link time.
 *
 * Why: glGetUniformLocation is a string lookup into the driver's reflection
 * data. Calling it per draw call, per frame, for values that are fixed the
 * moment the program links, is pure waste. Locations are resolved once here
 * and the per-frame state is split from the per-object state so view and
 * projection travel to the GPU once per frame instead of once per mesh.
 */

#pragma once

#include "gl_core.h"
#include "gl_math.h"

/**
 * @brief A linked GL program plus its cached uniform locations.
 *
 * A location of -1 means the uniform is absent or was optimised away by the
 * GLSL compiler; writes to it are skipped rather than sent to the driver.
 */
typedef struct {
    GLuint program;          /**< GL program name, 0 when not built. */
    GLint  u_model;          /**< mat4 model matrix. */
    GLint  u_view;           /**< mat4 view matrix. */
    GLint  u_projection;     /**< mat4 projection matrix. */
    GLint  u_normal_matrix;  /**< mat3 inverse-transpose of the model matrix. */
    GLint  u_color;          /**< vec3 base colour. */
    GLint  u_light_pos;      /**< vec3 key light position in world space. */
    GLint  u_fill_light_pos; /**< vec3 fill light position in world space. */
    GLint  u_view_pos;       /**< vec3 camera position in world space. */
} rbt_shader_t;

/**
 * @brief Compile and link a program, then resolve its uniform locations.
 *
 * Compile and link failures are logged to stderr and reported through the
 * return value; the caller decides whether that is fatal.
 *
 * @param shader       Caller-owned storage, overwritten on success.
 * @param vertex_src   Vertex shader source, NUL-terminated.
 * @param fragment_src Fragment shader source, NUL-terminated.
 * @return true when the program linked and is ready to use.
 */
bool rbt_shader_build(rbt_shader_t *shader, const char *vertex_src, const char *fragment_src);

/** @brief Delete the GL program and zero the handle. Safe on an unbuilt shader. */
void rbt_shader_destroy(rbt_shader_t *shader);

/**
 * @brief Bind the program and upload the state that is constant for a frame.
 *
 * Pre: a GL context is current and @p shader was built successfully.
 */
void rbt_shader_set_frame(const rbt_shader_t *shader,
                          const Matrix4f &view,
                          const Matrix4f &projection,
                          const Vector3f &eye_pos,
                          const Vector3f &light_pos,
                          const Vector3f &fill_light_pos);

/**
 * @brief Upload the per-object state: model matrix, normal matrix and colour.
 *
 * Pre: rbt_shader_set_frame was called for this frame.
 *
 * @param model Model matrix; may contain non-uniform scale, which is why the
 *              normal matrix is derived from it rather than reused.
 * @param color Three floats, RGB in 0..1.
 */
void rbt_shader_set_object(const rbt_shader_t *shader, const Matrix4f &model, const float color[3]);
