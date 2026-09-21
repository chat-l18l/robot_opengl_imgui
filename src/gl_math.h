/**
 * @file gl_math.h
 * @brief Matrix and vector helpers for the render pipeline.
 *
 * Eigen stores matrices column-major, which is exactly the layout
 * glUniformMatrix*fv expects, so a Matrix4f goes to the GPU through
 * .data() without a transpose. Eigen's own type names are kept as-is;
 * only symbols owned by this project use the rbt_ prefix.
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>

using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;
using Vector3f = Eigen::Vector3f;

/** Pi as a float, so degree conversions never promote to double. */
constexpr float RBT_PI = 3.14159265358979323846f;

/** Convert degrees to radians. */
inline float rbt_deg_to_rad(float degrees)
{
    return degrees * (RBT_PI / 180.0f);
}

/**
 * @brief Right-handed perspective projection mapping depth to [-1, 1].
 *
 * @param fov_deg Vertical field of view in degrees.
 * @param aspect  Viewport width divided by height; must be > 0.
 * @param z_near  Near plane distance, > 0.
 * @param z_far   Far plane distance, > z_near.
 */
inline Matrix4f rbt_perspective(float fov_deg, float aspect, float z_near, float z_far)
{
    const float f = 1.0f / std::tan(rbt_deg_to_rad(fov_deg) * 0.5f);

    Matrix4f m = Matrix4f::Zero();
    m(0, 0) = f / aspect;
    m(1, 1) = f;
    m(2, 2) = -(z_far + z_near) / (z_far - z_near);
    m(2, 3) = -(2.0f * z_far * z_near) / (z_far - z_near);
    m(3, 2) = -1.0f;
    return m;
}

/** @brief View matrix looking from @p eye at @p center, with @p up as roll reference. */
inline Matrix4f rbt_look_at(const Vector3f &eye, const Vector3f &center, const Vector3f &up)
{
    const Vector3f forward = (center - eye).normalized();
    const Vector3f right   = forward.cross(up).normalized();
    const Vector3f true_up = right.cross(forward).normalized();

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) =  right.x();    m(0, 1) =  right.y();    m(0, 2) =  right.z();    m(0, 3) = -right.dot(eye);
    m(1, 0) =  true_up.x();  m(1, 1) =  true_up.y();  m(1, 2) =  true_up.z();  m(1, 3) = -true_up.dot(eye);
    m(2, 0) = -forward.x();  m(2, 1) = -forward.y();  m(2, 2) = -forward.z();  m(2, 3) =  forward.dot(eye);
    return m;
}

/** @brief Translation matrix. */
inline Matrix4f rbt_translation(float x, float y, float z)
{
    Matrix4f m = Matrix4f::Identity();
    m(0, 3) = x;
    m(1, 3) = y;
    m(2, 3) = z;
    return m;
}

/** @brief Non-uniform scale matrix. */
inline Matrix4f rbt_scale(float sx, float sy, float sz)
{
    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = sx;
    m(1, 1) = sy;
    m(2, 2) = sz;
    return m;
}

/** @brief Rotation of @p radians about the X axis. */
inline Matrix4f rbt_rotation_x(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    Matrix4f m = Matrix4f::Identity();
    m(1, 1) = c;  m(1, 2) = -s;
    m(2, 1) = s;  m(2, 2) =  c;
    return m;
}

/** @brief Rotation of @p radians about the Y axis. */
inline Matrix4f rbt_rotation_y(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) =  c;  m(0, 2) = s;
    m(2, 0) = -s;  m(2, 2) = c;
    return m;
}

/** @brief Rotation of @p radians about the Z axis. */
inline Matrix4f rbt_rotation_z(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = c;  m(0, 1) = -s;
    m(1, 0) = s;  m(1, 1) =  c;
    return m;
}
