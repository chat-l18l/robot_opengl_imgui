// ============================================================
// gl_utils.h — OpenGL helper: shaders, mesh generatie, tekenen
// ============================================================
#pragma once

#include "gl_core.h"
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>

using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;
using Vector3f = Eigen::Vector3f;
using Vector2f = Eigen::Vector2f;

// ============================================================
// Shader utilities
// ============================================================

inline GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        std::cerr << "Shader compile error:\n" << log << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

inline GLuint linkProgram(GLuint vert, GLuint frag) {
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, nullptr, log);
        std::cerr << "Program link error:\n" << log << std::endl;
        return 0;
    }
    return prog;
}

inline GLuint createProgramFromSource(const char* vertSrc, const char* fragSrc) {
    GLuint vert = compileShader(GL_VERTEX_SHADER, vertSrc);
    GLuint frag = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    if (!vert || !frag) return 0;
    return linkProgram(vert, frag);
}

// ============================================================
// Math utilities (Eigen-based, geen glm nodig)
// ============================================================

inline Matrix4f perspectiveMatrix(float fovDeg, float aspect, float near, float far) {
    float fov = fovDeg * M_PI / 180.0f;
    float f = 1.0f / tanf(fov / 2.0f);

    Matrix4f m = Matrix4f::Zero();
    m(0, 0) = f / aspect;
    m(1, 1) = f;
    m(2, 2) = -(far + near) / (far - near);
    m(2, 3) = -(2.0f * far * near) / (far - near);
    m(3, 2) = -1.0f;
    return m;
}

inline Matrix4f lookAtMatrix(const Vector3f& eye, const Vector3f& center, const Vector3f& up) {
    Vector3f f = (center - eye).normalized();
    Vector3f r = f.cross(up).normalized();
    Vector3f u = r.cross(f).normalized();

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) =  r.x();  m(0, 1) =  r.y();  m(0, 2) =  r.z();  m(0, 3) = -r.dot(eye);
    m(1, 0) =  u.x();  m(1, 1) =  u.y();  m(1, 2) =  u.z();  m(1, 3) = -u.dot(eye);
    m(2, 0) = -f.x();  m(2, 1) = -f.y();  m(2, 2) = -f.z();  m(2, 3) =  f.dot(eye);
    m(3, 3) = 1.0f;
    return m;
}

inline Matrix4f translationMatrix(float x, float y, float z) {
    Matrix4f m = Matrix4f::Identity();
    m(0, 3) = x;
    m(1, 3) = y;
    m(2, 3) = z;
    return m;
}

inline Matrix4f scaleMatrix(float sx, float sy, float sz) {
    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = sx;
    m(1, 1) = sy;
    m(2, 2) = sz;
    return m;
}

inline Matrix4f rotationX(float rad) {
    Matrix4f m = Matrix4f::Identity();
    float c = cosf(rad), s = sinf(rad);
    m(1, 1) = c;  m(1, 2) = -s;
    m(2, 1) = s;  m(2, 2) = c;
    return m;
}

inline Matrix4f rotationY(float rad) {
    Matrix4f m = Matrix4f::Identity();
    float c = cosf(rad), s = sinf(rad);
    m(0, 0) = c;   m(0, 2) = s;
    m(2, 0) = -s;  m(2, 2) = c;
    return m;
}

inline Matrix4f rotationZ(float rad) {
    Matrix4f m = Matrix4f::Identity();
    float c = cosf(rad), s = sinf(rad);
    m(0, 0) = c;  m(0, 1) = -s;
    m(1, 0) = s;  m(1, 1) = c;
    return m;
}

/// Converteer Eigen Matrix4f naar OpenGL column-major float array
inline const float* eigenToGL(const Matrix4f& m) {
    // Eigen is column-major by default, net als OpenGL — perfect!
    return m.data();
}

// ============================================================
// Primitieve mesh data (triangle meshes)
// ============================================================

struct Vertex {
    Vector3f position;
    Vector3f normal;
};

struct Mesh {
    std::vector<Vertex> vertices;
    GLuint vao = 0;
    GLuint vbo = 0;
    bool uploaded = false;

    void upload() {
        if (uploaded) return;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     vertices.size() * sizeof(Vertex),
                     vertices.data(),
                     GL_STATIC_DRAW);

        // position (location = 0)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(Vertex), (void*)offsetof(Vertex, position));

        // normal (location = 1)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                              sizeof(Vertex), (void*)offsetof(Vertex, normal));

        glBindVertexArray(0);
        uploaded = true;
    }

    void draw() {
        if (!uploaded) upload();
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertices.size());
        glBindVertexArray(0);
    }
};

// ============================================================
// Mesh generatoren
// ============================================================

/// Genereer een cilinder mesh (oriented along Y-axis)
inline Mesh makeCylinder(float radiusBottom, float radiusTop, float height,
                         int segments = 24) {
    Mesh mesh;

    // Zijvlak
    for (int i = 0; i < segments; i++) {
        float a0 = 2.0f * M_PI * i / segments;
        float a1 = 2.0f * M_PI * (i + 1) / segments;

        // Normal berekening (voor getaperde cilinder)
        float slope = (radiusBottom - radiusTop) / height;
        float nx0 = cosf(a0) / sqrtf(1 + slope * slope);
        float nz0 = sinf(a0) / sqrtf(1 + slope * slope);
        float nx1 = cosf(a1) / sqrtf(1 + slope * slope);
        float nz1 = sinf(a1) / sqrtf(1 + slope * slope);

        // Onderste quad → 2 triangles
        mesh.vertices.push_back({{radiusBottom * cosf(a0), 0.0f, radiusBottom * sinf(a0)},
                                 {nx0, slope / sqrtf(1 + slope * slope), nz0}});
        mesh.vertices.push_back({{radiusTop * cosf(a0), height, radiusTop * sinf(a0)},
                                 {nx0, slope / sqrtf(1 + slope * slope), nz0}});
        mesh.vertices.push_back({{radiusBottom * cosf(a1), 0.0f, radiusBottom * sinf(a1)},
                                 {nx1, slope / sqrtf(1 + slope * slope), nz1}});

        mesh.vertices.push_back({{radiusBottom * cosf(a1), 0.0f, radiusBottom * sinf(a1)},
                                 {nx1, slope / sqrtf(1 + slope * slope), nz1}});
        mesh.vertices.push_back({{radiusTop * cosf(a0), height, radiusTop * sinf(a0)},
                                 {nx0, slope / sqrtf(1 + slope * slope), nz0}});
        mesh.vertices.push_back({{radiusTop * cosf(a1), height, radiusTop * sinf(a1)},
                                 {nx1, slope / sqrtf(1 + slope * slope), nz1}});
    }

    // Onderkant (cap)
    Vector3f downNormal(0, -1, 0);
    for (int i = 0; i < segments; i++) {
        float a0 = 2.0f * M_PI * i / segments;
        float a1 = 2.0f * M_PI * (i + 1) / segments;
        mesh.vertices.push_back({{0, 0, 0}, downNormal});
        mesh.vertices.push_back({{radiusBottom * cosf(a1), 0, radiusBottom * sinf(a1)}, downNormal});
        mesh.vertices.push_back({{radiusBottom * cosf(a0), 0, radiusBottom * sinf(a0)}, downNormal});
    }

    // Bovenkant (cap)
    Vector3f upNormal(0, 1, 0);
    for (int i = 0; i < segments; i++) {
        float a0 = 2.0f * M_PI * i / segments;
        float a1 = 2.0f * M_PI * (i + 1) / segments;
        mesh.vertices.push_back({{0, height, 0}, upNormal});
        mesh.vertices.push_back({{radiusTop * cosf(a0), height, radiusTop * sinf(a0)}, upNormal});
        mesh.vertices.push_back({{radiusTop * cosf(a1), height, radiusTop * sinf(a1)}, upNormal});
    }

    return mesh;
}

/// Genereer een box (axis-aligned, gecentreerd op oorsprong)
inline Mesh makeBox(float sx, float sy, float sz) {
    float hx = sx / 2.0f, hy = sy / 2.0f, hz = sz / 2.0f;
    Mesh mesh;
    auto& v = mesh.vertices;

    // +Y (boven)
    v.push_back({{-hx, hy, -hz}, {0, 1, 0}}); v.push_back({{ hx, hy, -hz}, {0, 1, 0}}); v.push_back({{ hx, hy,  hz}, {0, 1, 0}});
    v.push_back({{-hx, hy, -hz}, {0, 1, 0}}); v.push_back({{ hx, hy,  hz}, {0, 1, 0}}); v.push_back({{-hx, hy,  hz}, {0, 1, 0}});
    // -Y (onder)
    v.push_back({{-hx, -hy,  hz}, {0,-1, 0}}); v.push_back({{ hx, -hy,  hz}, {0,-1, 0}}); v.push_back({{ hx, -hy, -hz}, {0,-1, 0}});
    v.push_back({{-hx, -hy,  hz}, {0,-1, 0}}); v.push_back({{ hx, -hy, -hz}, {0,-1, 0}}); v.push_back({{-hx, -hy, -hz}, {0,-1, 0}});
    // +X (rechts)
    v.push_back({{ hx, -hy, -hz}, {1, 0, 0}}); v.push_back({{ hx,  hy, -hz}, {1, 0, 0}}); v.push_back({{ hx,  hy,  hz}, {1, 0, 0}});
    v.push_back({{ hx, -hy, -hz}, {1, 0, 0}}); v.push_back({{ hx,  hy,  hz}, {1, 0, 0}}); v.push_back({{ hx, -hy,  hz}, {1, 0, 0}});
    // -X (links)
    v.push_back({{-hx, -hy,  hz}, {-1, 0, 0}}); v.push_back({{-hx,  hy,  hz}, {-1, 0, 0}}); v.push_back({{-hx,  hy, -hz}, {-1, 0, 0}});
    v.push_back({{-hx, -hy,  hz}, {-1, 0, 0}}); v.push_back({{-hx,  hy, -hz}, {-1, 0, 0}}); v.push_back({{-hx, -hy, -hz}, {-1, 0, 0}});
    // +Z (voor)
    v.push_back({{-hx, -hy,  hz}, {0, 0, 1}}); v.push_back({{ hx, -hy,  hz}, {0, 0, 1}}); v.push_back({{ hx,  hy,  hz}, {0, 0, 1}});
    v.push_back({{-hx, -hy,  hz}, {0, 0, 1}}); v.push_back({{ hx,  hy,  hz}, {0, 0, 1}}); v.push_back({{-hx,  hy,  hz}, {0, 0, 1}});
    // -Z (achter)
    v.push_back({{ hx, -hy, -hz}, {0, 0,-1}}); v.push_back({{-hx, -hy, -hz}, {0, 0,-1}}); v.push_back({{-hx,  hy, -hz}, {0, 0,-1}});
    v.push_back({{ hx, -hy, -hz}, {0, 0,-1}}); v.push_back({{-hx,  hy, -hz}, {0, 0,-1}}); v.push_back({{ hx,  hy, -hz}, {0, 0,-1}});

    return mesh;
}

/// Genereer een sphere
inline Mesh makeSphere(float radius, int rings = 16, int segments = 24) {
    Mesh mesh;
    for (int r = 0; r < rings; r++) {
        float phi0 = M_PI * r / rings;
        float phi1 = M_PI * (r + 1) / rings;
        for (int s = 0; s < segments; s++) {
            float theta0 = 2.0f * M_PI * s / segments;
            float theta1 = 2.0f * M_PI * (s + 1) / segments;

            auto p = [&](float phi, float theta) -> Vector3f {
                return {radius * sinf(phi) * cosf(theta),
                        radius * cosf(phi),
                        radius * sinf(phi) * sinf(theta)};
            };

            Vector3f p00 = p(phi0, theta0), p01 = p(phi0, theta1);
            Vector3f p10 = p(phi1, theta0), p11 = p(phi1, theta1);

            Vector3f n00 = p00.normalized(), n01 = p01.normalized();
            Vector3f n10 = p10.normalized(), n11 = p11.normalized();

            mesh.vertices.push_back({p00, n00});
            mesh.vertices.push_back({p10, n10});
            mesh.vertices.push_back({p11, n11});

            mesh.vertices.push_back({p00, n00});
            mesh.vertices.push_back({p11, n11});
            mesh.vertices.push_back({p01, n01});
        }
    }
    return mesh;
}

// ============================================================
// Grid floor rendering
// ============================================================

inline Mesh makeGrid(float size, int divisions) {
    Mesh mesh;
    Vector3f yNormal(0, 1, 0);
    float step = size / divisions;
    float half = size / 2.0f;

    // Dunne quads voor gridlijnen
    float thickness = 0.005f;
    for (int i = 0; i <= divisions; i++) {
        float pos = -half + i * step;
        // X-lijn
        mesh.vertices.push_back({{pos, 0, -half}, yNormal});
        mesh.vertices.push_back({{pos, 0,  half}, yNormal});
        mesh.vertices.push_back({{pos + thickness, 0,  half}, yNormal});
        mesh.vertices.push_back({{pos, 0, -half}, yNormal});
        mesh.vertices.push_back({{pos + thickness, 0,  half}, yNormal});
        mesh.vertices.push_back({{pos + thickness, 0, -half}, yNormal});
        // Z-lijn
        mesh.vertices.push_back({{-half, 0, pos}, yNormal});
        mesh.vertices.push_back({{ half, 0, pos}, yNormal});
        mesh.vertices.push_back({{ half, 0, pos + thickness}, yNormal});
        mesh.vertices.push_back({{-half, 0, pos}, yNormal});
        mesh.vertices.push_back({{ half, 0, pos + thickness}, yNormal});
        mesh.vertices.push_back({{-half, 0, pos + thickness}, yNormal});
    }
    return mesh;
}

// ============================================================
// Shader uniform helper
// ============================================================

inline void setUniforms(GLuint program, const Matrix4f& model,
                        const Matrix4f& view, const Matrix4f& proj,
                        float r, float g, float b) {
    Matrix4f mvp = proj * view * model;
    Matrix3f normalMat = model.block<3, 3>(0, 0).inverse().transpose();

    GLuint loc;
    loc = glGetUniformLocation(program, "uModel");
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, eigenToGL(model));
    loc = glGetUniformLocation(program, "uView");
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, eigenToGL(view));
    loc = glGetUniformLocation(program, "uProjection");
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, eigenToGL(proj));
    loc = glGetUniformLocation(program, "uMVP");
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, eigenToGL(mvp));
    loc = glGetUniformLocation(program, "uNormalMatrix");
    if (loc >= 0) glUniformMatrix3fv(loc, 1, GL_FALSE, normalMat.data());
    loc = glGetUniformLocation(program, "uColor");
    if (loc >= 0) glUniform3f(loc, r, g, b);
}
