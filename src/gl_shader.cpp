/**
 * @file gl_shader.cpp
 * @brief Shader program construction and uniform upload.
 */

#include "gl_shader.h"

#include <assert.h>
#include <stdio.h>

/** Size of the stack buffer used for GL info logs. */
#define RBT_SHADER_LOG_SIZE 1024

/**
 * @brief Compile one shader stage.
 * @return The shader name, or 0 when compilation failed.
 */
static GLuint s_compile_stage(GLenum type, const char *source)
{
    assert(source != NULL);

    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_FALSE) {
        char log[RBT_SHADER_LOG_SIZE];
        glGetShaderInfoLog(shader, (GLsizei)sizeof(log), NULL, log);
        fprintf(stderr, "shader: %s stage failed to compile:\n%s\n",
                type == GL_VERTEX_SHADER ? "vertex" : "fragment", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

/**
 * @brief Link a vertex and fragment stage into a program.
 *
 * The stages are detached and deleted either way, so no shader objects
 * survive this call.
 *
 * @return The program name, or 0 when linking failed.
 */
static GLuint s_link_program(GLuint vertex, GLuint fragment)
{
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);

    glDetachShader(program, vertex);
    glDetachShader(program, fragment);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked == GL_FALSE) {
        char log[RBT_SHADER_LOG_SIZE];
        glGetProgramInfoLog(program, (GLsizei)sizeof(log), NULL, log);
        fprintf(stderr, "shader: program failed to link:\n%s\n", log);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

bool rbt_shader_build(rbt_shader_t *shader, const char *vertex_src, const char *fragment_src)
{
    assert(shader != NULL);
    assert(vertex_src != NULL);
    assert(fragment_src != NULL);

    const GLuint vertex = s_compile_stage(GL_VERTEX_SHADER, vertex_src);
    if (vertex == 0) {
        return false;
    }
    const GLuint fragment = s_compile_stage(GL_FRAGMENT_SHADER, fragment_src);
    if (fragment == 0) {
        glDeleteShader(vertex);
        return false;
    }

    const GLuint program = s_link_program(vertex, fragment);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    if (program == 0) {
        return false;
    }

    shader->program         = program;
    shader->u_model         = glGetUniformLocation(program, "u_model");
    shader->u_view          = glGetUniformLocation(program, "u_view");
    shader->u_projection    = glGetUniformLocation(program, "u_projection");
    shader->u_normal_matrix = glGetUniformLocation(program, "u_normal_matrix");
    shader->u_color         = glGetUniformLocation(program, "u_color");
    shader->u_light_pos     = glGetUniformLocation(program, "u_light_pos");
    shader->u_fill_light_pos = glGetUniformLocation(program, "u_fill_light_pos");
    shader->u_view_pos      = glGetUniformLocation(program, "u_view_pos");
    return true;
}

void rbt_shader_destroy(rbt_shader_t *shader)
{
    assert(shader != NULL);

    if (shader->program != 0) {
        glDeleteProgram(shader->program);
        shader->program = 0;
    }
}

void rbt_shader_set_frame(const rbt_shader_t *shader,
                          const Matrix4f &view,
                          const Matrix4f &projection,
                          const Vector3f &eye_pos,
                          const Vector3f &light_pos,
                          const Vector3f &fill_light_pos)
{
    assert(shader != NULL);
    assert(shader->program != 0);

    glUseProgram(shader->program);

    if (shader->u_view >= 0) {
        glUniformMatrix4fv(shader->u_view, 1, GL_FALSE, view.data());
    }
    if (shader->u_projection >= 0) {
        glUniformMatrix4fv(shader->u_projection, 1, GL_FALSE, projection.data());
    }
    if (shader->u_view_pos >= 0) {
        glUniform3f(shader->u_view_pos, eye_pos.x(), eye_pos.y(), eye_pos.z());
    }
    if (shader->u_light_pos >= 0) {
        glUniform3f(shader->u_light_pos, light_pos.x(), light_pos.y(), light_pos.z());
    }
    if (shader->u_fill_light_pos >= 0) {
        glUniform3f(shader->u_fill_light_pos, fill_light_pos.x(), fill_light_pos.y(), fill_light_pos.z());
    }
}

void rbt_shader_set_object(const rbt_shader_t *shader, const Matrix4f &model, const float color[3])
{
    assert(shader != NULL);
    assert(shader->program != 0);
    assert(color != NULL);

    if (shader->u_model >= 0) {
        glUniformMatrix4fv(shader->u_model, 1, GL_FALSE, model.data());
    }
    if (shader->u_normal_matrix >= 0) {
        /* Non-uniform scale makes the plain rotation part wrong for normals. */
        const Matrix3f normal_matrix = model.block<3, 3>(0, 0).inverse().transpose();
        glUniformMatrix3fv(shader->u_normal_matrix, 1, GL_FALSE, normal_matrix.data());
    }
    if (shader->u_color >= 0) {
        glUniform3f(shader->u_color, color[0], color[1], color[2]);
    }
}
