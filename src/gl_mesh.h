/**
 * @file gl_mesh.h
 * @brief GPU-resident triangle meshes and the primitives this viewer draws.
 */

#pragma once

#include "gl_core.h"
#include "gl_math.h"

#include <stdint.h>
#include <vector>

/** @brief One vertex: position and normal, both in model space. */
typedef struct {
    Vector3f position;
    Vector3f normal;
} rbt_vertex_t;

/**
 * @brief A triangle mesh that owns its VAO and VBO.
 *
 * Move-only on purpose: a copy would leave two objects holding the same GL
 * names, and whichever destructor ran second would delete a buffer the other
 * still believes it owns.
 *
 * Indices are optional: the generated primitives are plain triangle lists,
 * while loaded models share vertices between triangles and come indexed.
 *
 * The CPU-side arrays are released on upload; only the counts and the bounding
 * box survive, because nothing here reads geometry back.
 */
struct rbt_mesh_t {
    std::vector<rbt_vertex_t> vertices;  /**< Staging data, empty after upload. */
    std::vector<uint32_t>     indices;   /**< Staging indices; empty for a plain triangle list. */
    GLuint   vao = 0;                    /**< Vertex array object, 0 until uploaded. */
    GLuint   vbo = 0;                    /**< Vertex buffer object, 0 until uploaded. */
    GLuint   ebo = 0;                    /**< Element buffer object, 0 when not indexed. */
    GLsizei  vertex_count = 0;           /**< Valid after upload. */
    GLsizei  index_count = 0;            /**< Valid after upload; 0 draws the vertices in order. */
    Vector3f bounds_min = Vector3f::Zero();  /**< Model-space box, valid after upload. */
    Vector3f bounds_max = Vector3f::Zero();

    rbt_mesh_t() = default;
    ~rbt_mesh_t();

    rbt_mesh_t(const rbt_mesh_t &) = delete;
    rbt_mesh_t &operator=(const rbt_mesh_t &) = delete;

    rbt_mesh_t(rbt_mesh_t &&other) noexcept;
    rbt_mesh_t &operator=(rbt_mesh_t &&other) noexcept;
};

/**
 * @brief Upload the staged vertices and indices, record the bounds, free the CPU copy.
 *
 * Pre: a GL context is current. Uploading twice is a no-op.
 */
void rbt_mesh_upload(rbt_mesh_t *mesh);

/**
 * @brief Draw the mesh with the currently bound program.
 *
 * Leaves the mesh's VAO bound; call rbt_mesh_unbind() once the pass is done.
 * Pre: the mesh was uploaded.
 */
void rbt_mesh_draw(const rbt_mesh_t *mesh);

/** @brief Unbind whatever VAO is current, at the end of a draw pass. */
void rbt_mesh_unbind(void);

/**
 * @brief Cylinder along +Y, base at y = 0, top at y = @p height.
 *
 * A different bottom and top radius gives a truncated cone; the side normals
 * are tilted to match the slope so the shading stays correct.
 */
rbt_mesh_t rbt_mesh_make_cylinder(float radius_bottom, float radius_top, float height, int segments);

/** @brief Axis-aligned box centred on the origin. */
rbt_mesh_t rbt_mesh_make_box(float size_x, float size_y, float size_z);

/** @brief Sphere centred on the origin. */
rbt_mesh_t rbt_mesh_make_sphere(float radius, int rings, int segments);

/**
 * @brief Flat grid in the XZ plane, drawn as thin quads rather than GL_LINES.
 *
 * Quads keep a constant world-space width under perspective, which lines do
 * not, and they go through the same lit shader as everything else.
 */
rbt_mesh_t rbt_mesh_make_grid(float size, int divisions);
