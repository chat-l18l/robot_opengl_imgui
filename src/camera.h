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

    /* Drag state. A drag starts only over the viewport but continues until
     * the button is released, wherever the pointer goes. */
    bool  orbiting;      /**< An orbit drag is in progress. */
    bool  panning;       /**< A pan drag is in progress. */
    float last_mouse_x;  /**< Pointer X at the previous input step. */
    float last_mouse_y;  /**< Pointer Y at the previous input step. */
} rbt_camera_t;

/** @brief One frame of pointer input, in screen pixels. */
typedef struct {
    float mouse_x;
    float mouse_y;
    float wheel;       /**< Scroll delta this frame; positive zooms in. */
    bool  orbit_down;  /**< Orbit button (left) is held. */
    bool  pan_down;    /**< Pan button (right) is held. */
    bool  over_view;   /**< Pointer is over the 3D viewport. */
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
 * Must be called every frame, including frames where the pointer is not over
 * the viewport: that is how a drag notices the button was released elsewhere.
 * Starting a drag and zooming both require @ref rbt_camera_input_t::over_view.
 */
void rbt_camera_input(rbt_camera_t *camera, const rbt_camera_input_t *input);
