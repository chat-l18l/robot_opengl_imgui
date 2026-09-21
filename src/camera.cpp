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
static const float s_min_distance        = 1.0f;
static const float s_max_distance        = 50.0f;
static const float s_max_pitch_deg       = 89.0f;

void rbt_camera_reset(rbt_camera_t *camera)
{
    assert(camera != NULL);

    camera->distance  = s_default_distance;
    camera->yaw_deg   = s_default_yaw_deg;
    camera->pitch_deg = s_default_pitch_deg;
    camera->target    = s_default_target;
    camera->orbiting  = false;
    camera->panning   = false;

    /* Never read before a drag sets them, but leaving them undefined would
     * make a stack-allocated camera depend on that argument staying true. */
    camera->last_mouse_x = 0.0f;
    camera->last_mouse_y = 0.0f;
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

    if (input->over_view && input->wheel != 0.0f) {
        camera->distance = std::clamp(camera->distance - input->wheel * s_zoom_per_notch,
                                      s_min_distance, s_max_distance);
    }

    /* A drag begins only over the viewport; releasing the button ends it
     * wherever the pointer is, so re-entering never resumes a stale drag. */
    const bool orbit_active = camera->orbiting ? input->orbit_down
                                               : (input->orbit_down && input->over_view && !camera->panning);
    const bool pan_active   = camera->panning ? input->pan_down
                                              : (input->pan_down && input->over_view && !camera->orbiting);

    const bool orbit_started = orbit_active && !camera->orbiting;
    const bool pan_started   = pan_active && !camera->panning;

    if (orbit_started || pan_started) {
        camera->last_mouse_x = input->mouse_x;
        camera->last_mouse_y = input->mouse_y;
    }

    const float dx = input->mouse_x - camera->last_mouse_x;
    const float dy = input->mouse_y - camera->last_mouse_y;

    if (orbit_active && !orbit_started) {
        camera->yaw_deg   -= dx * s_orbit_deg_per_pixel;
        camera->pitch_deg  = std::clamp(camera->pitch_deg + dy * s_orbit_deg_per_pixel,
                                        -s_max_pitch_deg, s_max_pitch_deg);
    } else if (pan_active && !pan_started) {
        /* Slide in the camera's screen plane: right follows yaw, up is world up. */
        const float    yaw = rbt_deg_to_rad(camera->yaw_deg);
        const Vector3f right(std::cos(yaw), 0.0f, -std::sin(yaw));
        const float    amount = s_pan_per_pixel * camera->distance;

        camera->target -= right * (dx * amount);
        camera->target += Vector3f(0.0f, 1.0f, 0.0f) * (dy * amount);
    }

    if (orbit_active || pan_active) {
        camera->last_mouse_x = input->mouse_x;
        camera->last_mouse_y = input->mouse_y;
    }

    camera->orbiting = orbit_active;
    camera->panning  = pan_active;
}
