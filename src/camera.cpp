/**
 * @file camera.cpp
 * @brief Orbit camera input handling.
 */

#include "camera.h"

#include <algorithm>
#include <assert.h>

/* Framing defaults, chosen so the whole arm fits on screen at startup. */
static const float    s_default_distance  = 8.0f;
static const float    s_default_yaw_deg   = 45.0f;
static const float    s_default_pitch_deg = 25.0f;
static const Vector3f s_default_target(0.0f, 1.5f, 0.0f);

/* Input scaling. */
static const float s_orbit_deg_per_pixel = 0.4f;
static const float s_pan_per_pixel       = 0.001f;  /**< Scaled by distance, so panning feels the same at any zoom. */
static const float s_zoom_per_notch      = 0.8f;
static const float s_default_min_distance = 1.0f;
static const float s_default_max_distance = 50.0f;
static const float s_default_z_near       = 0.1f;
static const float s_default_z_far        = 100.0f;

/* Framing: breathing room around the model, and the limits as fractions and
 * multiples of the framed distance. */
static const float s_frame_margin        = 1.1f;
static const float s_frame_min_factor    = 0.05f;
static const float s_frame_max_factor    = 20.0f;
static const float s_frame_near_factor   = 0.01f;
static const float s_frame_far_factor    = 40.0f;  /**< Beyond max_distance plus the model itself. */
static const float s_max_pitch_deg       = 89.0f;

void rbt_camera_reset(rbt_camera_t *camera)
{
    assert(camera != NULL);

    camera->distance  = s_default_distance;
    camera->yaw_deg   = s_default_yaw_deg;
    camera->pitch_deg = s_default_pitch_deg;
    camera->target    = s_default_target;

    camera->min_distance = s_default_min_distance;
    camera->max_distance = s_default_max_distance;
    camera->z_near       = s_default_z_near;
    camera->z_far        = s_default_z_far;
    camera->orbiting  = false;
    camera->panning   = false;

    /* Never read before a drag sets them, but leaving them undefined would
     * make a stack-allocated camera depend on that argument staying true. */
    camera->last_mouse_x = 0.0f;
    camera->last_mouse_y = 0.0f;
}

void rbt_camera_frame_box(rbt_camera_t *camera, const Vector3f &box_min, const Vector3f &box_max, float fov_deg)
{
    assert(camera != NULL);
    assert(fov_deg > 0.0f && fov_deg < 180.0f);

    /* Fit the bounding sphere into the vertical field of view; the view is
     * wider than it is tall, so the vertical fit is the one that binds. */
    float radius = (box_max - box_min).norm() * 0.5f;
    if (radius < 1e-4f) {
        radius = 1e-4f;
    }
    const float distance = radius / std::sin(rbt_deg_to_rad(fov_deg) * 0.5f) * s_frame_margin;

    camera->target       = (box_min + box_max) * 0.5f;
    camera->distance     = distance;
    camera->min_distance = distance * s_frame_min_factor;
    camera->max_distance = distance * s_frame_max_factor;
    camera->z_near       = distance * s_frame_near_factor;
    camera->z_far        = distance * s_frame_far_factor;
}

Vector3f rbt_camera_eye(const rbt_camera_t *camera)
{
    assert(camera != NULL);

    const float yaw   = rbt_deg_to_rad(camera->yaw_deg);
    const float pitch = rbt_deg_to_rad(camera->pitch_deg);
    const float ring  = camera->distance * std::cos(pitch);

    return {camera->target.x() + ring * std::sin(yaw),
            camera->target.y() + camera->distance * std::sin(pitch),
            camera->target.z() + ring * std::cos(yaw)};
}

Matrix4f rbt_camera_view(const rbt_camera_t *camera)
{
    assert(camera != NULL);

    return rbt_look_at(rbt_camera_eye(camera), camera->target, Vector3f(0.0f, 1.0f, 0.0f));
}

void rbt_camera_input(rbt_camera_t *camera, const rbt_camera_input_t *input)
{
    assert(camera != NULL);
    assert(input != NULL);

    if (input->wheel != 0.0f) {
        camera->distance = std::clamp(camera->distance - input->wheel * s_zoom_per_notch,
                                      camera->min_distance, camera->max_distance);
    }

    /* One drag at a time: the button that claimed the view keeps it. */
    const bool orbit = input->orbit && !camera->panning;
    const bool pan   = input->pan && !orbit && !camera->orbiting;

    const bool started = (orbit && !camera->orbiting) || (pan && !camera->panning);
    if (started) {
        camera->last_mouse_x = input->mouse_x;
        camera->last_mouse_y = input->mouse_y;
    }

    const float dx = input->mouse_x - camera->last_mouse_x;
    const float dy = input->mouse_y - camera->last_mouse_y;

    if (orbit && !started) {
        camera->yaw_deg  -= dx * s_orbit_deg_per_pixel;
        camera->pitch_deg = std::clamp(camera->pitch_deg + dy * s_orbit_deg_per_pixel,
                                       -s_max_pitch_deg, s_max_pitch_deg);
    } else if (pan && !started) {
        /* Slide in the camera's screen plane: right follows yaw, up is world up. */
        const float    yaw = rbt_deg_to_rad(camera->yaw_deg);
        const Vector3f right(std::cos(yaw), 0.0f, -std::sin(yaw));
        const float    amount = s_pan_per_pixel * camera->distance;

        camera->target -= right * (dx * amount);
        camera->target += Vector3f(0.0f, 1.0f, 0.0f) * (dy * amount);
    }

    if (orbit || pan) {
        camera->last_mouse_x = input->mouse_x;
        camera->last_mouse_y = input->mouse_y;
    }

    camera->orbiting = orbit;
    camera->panning  = pan;
}
