/**
 * @file camera.h
 * @brief Orbit camera: yaw/pitch/distance around a target point.
 */

#pragma once

#include "gl_math.h"

/** @brief Camera state. Angles are in degrees; pitch is clamped to avoid gimbal flip. */
typedef struct {
    float    distance;   /**< Distance from target to eye, in world units. */
    float    yaw_deg;    /**< Rotation about +Y. */
    float    pitch_deg;  /**< Elevation, clamped to (-90, 90). */
    Vector3f target;     /**< Point the camera looks at. */

    /* Which drag ran last frame, so the first frame of a new one seeds the
     * reference position instead of jumping by a stale delta. */
    bool  orbiting;      /**< An orbit drag was active last frame. */
    bool  panning;       /**< A pan drag was active last frame. */
    float last_mouse_x;  /**< Pointer X at the previous input step. */
    float last_mouse_y;  /**< Pointer Y at the previous input step. */
} rbt_camera_t;

/**
 * @brief One frame of pointer input, in screen pixels.
 *
 * The caller decides whether the viewport owns the pointer; the camera does
 * not second-guess it. Letting the camera track "a drag started over the view,
 * so keep going until the button comes up" duplicated state the UI toolkit
 * already owns, and a release the toolkit saw but the camera did not would
 * have left it orbiting forever.
 */
typedef struct {
    float mouse_x;
    float mouse_y;
    float wheel;   /**< Scroll delta; the caller zeroes it when out of range. */
    bool  orbit;   /**< The viewport holds the pointer for the orbit button. */
    bool  pan;     /**< The viewport holds the pointer for the pan button. */
} rbt_camera_input_t;

/** @brief Restore the default framing. */
void rbt_camera_reset(rbt_camera_t *camera);

/** @brief Eye position in world space, derived from yaw, pitch and distance. */
Vector3f rbt_camera_eye(const rbt_camera_t *camera);

/** @brief View matrix for the current state. */
Matrix4f rbt_camera_view(const rbt_camera_t *camera);

/**
 * @brief Fold one frame of pointer input into the camera state.
 *
 * Call it every frame. Orbit and pan are mutually exclusive: whichever claimed
 * the view first keeps it until it is released.
 */
void rbt_camera_input(rbt_camera_t *camera, const rbt_camera_input_t *input);
