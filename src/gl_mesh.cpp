/**
 * @file gl_mesh.cpp
 * @brief Mesh lifetime, GPU upload and primitive generation.
 */

#include "gl_mesh.h"

#include <assert.h>
#include <stddef.h>

rbt_mesh_t::~rbt_mesh_t()
{
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
    }
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
    }
}

rbt_mesh_t::rbt_mesh_t(rbt_mesh_t &&other) noexcept
    : vertices(std::move(other.vertices)),
      vao(other.vao),
      vbo(other.vbo),
      vertex_count(other.vertex_count)
{
    other.vao          = 0;
    other.vbo          = 0;
    other.vertex_count = 0;
}

rbt_mesh_t &rbt_mesh_t::operator=(rbt_mesh_t &&other) noexcept
{
    if (this == &other) {
        return *this;
    }
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
    }
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
    }

    vertices     = std::move(other.vertices);
    vao          = other.vao;
    vbo          = other.vbo;
    vertex_count = other.vertex_count;

    other.vao          = 0;
    other.vbo          = 0;
    other.vertex_count = 0;
    return *this;
}

void rbt_mesh_upload(rbt_mesh_t *mesh)
{
    assert(mesh != NULL);

    if (mesh->vao != 0) {
        return;
    }
    assert(!mesh->vertices.empty());

    glGenVertexArrays(1, &mesh->vao);
    glGenBuffers(1, &mesh->vbo);

    glBindVertexArray(mesh->vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(mesh->vertices.size() * sizeof(rbt_vertex_t)),
                 mesh->vertices.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(rbt_vertex_t), (void *)offsetof(rbt_vertex_t, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          sizeof(rbt_vertex_t), (void *)offsetof(rbt_vertex_t, normal));

    glBindVertexArray(0);

    mesh->vertex_count = (GLsizei)mesh->vertices.size();

    /* The geometry now lives on the GPU and is never read back. */
    std::vector<rbt_vertex_t>().swap(mesh->vertices);
}

void rbt_mesh_draw(const rbt_mesh_t *mesh)
{
    assert(mesh != NULL);
    assert(mesh->vao != 0);

    glBindVertexArray(mesh->vao);
    glDrawArrays(GL_TRIANGLES, 0, mesh->vertex_count);
}

void rbt_mesh_unbind(void)
{
    glBindVertexArray(0);
}

/* ============================================================
 * Primitive generation
 * ============================================================ */

rbt_mesh_t rbt_mesh_make_cylinder(float radius_bottom, float radius_top, float height, int segments)
{
    assert(segments >= 3);
    assert(height > 0.0f);

    rbt_mesh_t mesh;
    std::vector<rbt_vertex_t> &v = mesh.vertices;
    v.reserve((size_t)segments * 12);

    /* A side normal points outward plus a bit along Y for a tapered wall. */
    const float slope     = (radius_bottom - radius_top) / height;
    const float normalise = 1.0f / std::sqrt(1.0f + slope * slope);
    const float normal_y  = slope * normalise;

    for (int i = 0; i < segments; i++) {
        const float a0 = 2.0f * RBT_PI * (float)i / (float)segments;
        const float a1 = 2.0f * RBT_PI * (float)(i + 1) / (float)segments;

        const float c0 = std::cos(a0), s0 = std::sin(a0);
        const float c1 = std::cos(a1), s1 = std::sin(a1);

        const Vector3f n0(c0 * normalise, normal_y, s0 * normalise);
        const Vector3f n1(c1 * normalise, normal_y, s1 * normalise);

        const Vector3f b0(radius_bottom * c0, 0.0f, radius_bottom * s0);
        const Vector3f b1(radius_bottom * c1, 0.0f, radius_bottom * s1);
        const Vector3f t0(radius_top * c0, height, radius_top * s0);
        const Vector3f t1(radius_top * c1, height, radius_top * s1);

        v.push_back({b0, n0});
        v.push_back({t0, n0});
        v.push_back({b1, n1});

        v.push_back({b1, n1});
        v.push_back({t0, n0});
        v.push_back({t1, n1});
    }

    const Vector3f down(0.0f, -1.0f, 0.0f);
    const Vector3f up(0.0f, 1.0f, 0.0f);

    for (int i = 0; i < segments; i++) {
        const float a0 = 2.0f * RBT_PI * (float)i / (float)segments;
        const float a1 = 2.0f * RBT_PI * (float)(i + 1) / (float)segments;

        v.push_back({{0.0f, 0.0f, 0.0f}, down});
        v.push_back({{radius_bottom * std::cos(a1), 0.0f, radius_bottom * std::sin(a1)}, down});
        v.push_back({{radius_bottom * std::cos(a0), 0.0f, radius_bottom * std::sin(a0)}, down});

        v.push_back({{0.0f, height, 0.0f}, up});
        v.push_back({{radius_top * std::cos(a0), height, radius_top * std::sin(a0)}, up});
        v.push_back({{radius_top * std::cos(a1), height, radius_top * std::sin(a1)}, up});
    }

    return mesh;
}

rbt_mesh_t rbt_mesh_make_box(float size_x, float size_y, float size_z)
{
    const float hx = size_x * 0.5f;
    const float hy = size_y * 0.5f;
    const float hz = size_z * 0.5f;

    /* Face centre normal plus the four corners in counter-clockwise order. */
    const struct {
        Vector3f normal;
        Vector3f corner[4];
    } faces[6] = {
        {{ 0,  1,  0}, {{-hx,  hy, -hz}, { hx,  hy, -hz}, { hx,  hy,  hz}, {-hx,  hy,  hz}}},
        {{ 0, -1,  0}, {{-hx, -hy,  hz}, { hx, -hy,  hz}, { hx, -hy, -hz}, {-hx, -hy, -hz}}},
        {{ 1,  0,  0}, {{ hx, -hy, -hz}, { hx,  hy, -hz}, { hx,  hy,  hz}, { hx, -hy,  hz}}},
        {{-1,  0,  0}, {{-hx, -hy,  hz}, {-hx,  hy,  hz}, {-hx,  hy, -hz}, {-hx, -hy, -hz}}},
        {{ 0,  0,  1}, {{-hx, -hy,  hz}, { hx, -hy,  hz}, { hx,  hy,  hz}, {-hx,  hy,  hz}}},
        {{ 0,  0, -1}, {{ hx, -hy, -hz}, {-hx, -hy, -hz}, {-hx,  hy, -hz}, { hx,  hy, -hz}}},
    };

    rbt_mesh_t mesh;
    mesh.vertices.reserve(36);
    for (int f = 0; f < 6; f++) {
        mesh.vertices.push_back({faces[f].corner[0], faces[f].normal});
        mesh.vertices.push_back({faces[f].corner[1], faces[f].normal});
        mesh.vertices.push_back({faces[f].corner[2], faces[f].normal});
        mesh.vertices.push_back({faces[f].corner[0], faces[f].normal});
        mesh.vertices.push_back({faces[f].corner[2], faces[f].normal});
        mesh.vertices.push_back({faces[f].corner[3], faces[f].normal});
    }
    return mesh;
}

rbt_mesh_t rbt_mesh_make_sphere(float radius, int rings, int segments)
{
    assert(rings >= 2);
    assert(segments >= 3);

    rbt_mesh_t mesh;
    mesh.vertices.reserve((size_t)rings * (size_t)segments * 6);

    for (int r = 0; r < rings; r++) {
        const float phi0 = RBT_PI * (float)r / (float)rings;
        const float phi1 = RBT_PI * (float)(r + 1) / (float)rings;

        for (int s = 0; s < segments; s++) {
            const float theta0 = 2.0f * RBT_PI * (float)s / (float)segments;
            const float theta1 = 2.0f * RBT_PI * (float)(s + 1) / (float)segments;

            /* On a unit sphere the position doubles as the normal. */
            const auto unit = [](float phi, float theta) -> Vector3f {
                return {std::sin(phi) * std::cos(theta),
                        std::cos(phi),
                        std::sin(phi) * std::sin(theta)};
            };

            const Vector3f n00 = unit(phi0, theta0), n01 = unit(phi0, theta1);
            const Vector3f n10 = unit(phi1, theta0), n11 = unit(phi1, theta1);

            mesh.vertices.push_back({n00 * radius, n00});
            mesh.vertices.push_back({n10 * radius, n10});
            mesh.vertices.push_back({n11 * radius, n11});

            mesh.vertices.push_back({n00 * radius, n00});
            mesh.vertices.push_back({n11 * radius, n11});
            mesh.vertices.push_back({n01 * radius, n01});
        }
    }
    return mesh;
}

rbt_mesh_t rbt_mesh_make_grid(float size, int divisions)
{
    assert(divisions >= 1);

    /* Line width in world units; thin enough to read as a line at any zoom. */
    const float thickness = 0.005f;
    const float step      = size / (float)divisions;
    const float half      = size * 0.5f;
    const Vector3f up(0.0f, 1.0f, 0.0f);

    rbt_mesh_t mesh;
    std::vector<rbt_vertex_t> &v = mesh.vertices;
    v.reserve((size_t)(divisions + 1) * 12);

    for (int i = 0; i <= divisions; i++) {
        const float p = -half + (float)i * step;

        /* Line parallel to Z. */
        v.push_back({{p, 0.0f, -half}, up});
        v.push_back({{p, 0.0f,  half}, up});
        v.push_back({{p + thickness, 0.0f,  half}, up});
        v.push_back({{p, 0.0f, -half}, up});
        v.push_back({{p + thickness, 0.0f,  half}, up});
        v.push_back({{p + thickness, 0.0f, -half}, up});

        /* Line parallel to X. */
        v.push_back({{-half, 0.0f, p}, up});
        v.push_back({{ half, 0.0f, p}, up});
        v.push_back({{ half, 0.0f, p + thickness}, up});
        v.push_back({{-half, 0.0f, p}, up});
        v.push_back({{ half, 0.0f, p + thickness}, up});
        v.push_back({{-half, 0.0f, p + thickness}, up});
    }
    return mesh;
}
