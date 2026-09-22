/**
 * @file robot.h
 * @brief Robot arm: joint array, forward kinematics and rendering.
 */

#pragma once

#include "gl_mesh.h"
#include "gl_shader.h"

#include <vector>

/** @brief Rotation axis of a revolute joint, in its own frame. */
typedef enum {
    RBT_AXIS_X = 0,
    RBT_AXIS_Y,
    RBT_AXIS_Z,
} rbt_axis_t;

/** @brief Parent index of a joint that has none. Only the root carries it. */
#define RBT_NO_PARENT (-1)

/**
 * @brief One revolute joint and the link that reaches it from its parent.
 *
 * Joints live in one flat array and name their parent by index, the usual
 * representation in robotics. A parent always sits earlier in the array, so
 * forward kinematics is a single forward pass: no recursion, no pointers to
 * keep valid, and the whole chain walks contiguous memory.
 *
 * The link is not stored separately: its length is the Y component of
 * @ref offset, so there is exactly one place that says how long it is.
 */
typedef struct {
    const char *name;             /**< Static string; joints do not own their name. */
    rbt_axis_t  axis;             /**< Rotation axis. */
    int         parent;           /**< Index of the parent joint, or RBT_NO_PARENT. */
    int         child_count;      /**< Filled by rbt_robot_build; 0 marks a tool tip. */

    float       min_angle_deg;    /**< Lower travel limit. */
    float       max_angle_deg;    /**< Upper travel limit. */
    float       default_angle_deg;/**< Pose restored by rbt_robot_reset_joints. */
    float       angle_deg;        /**< Live angle, driven by the UI. */

    Vector3f    offset;           /**< Parent joint to this joint, in the parent frame. */
    float       link_radius;      /**< Radius of the link drawn along that offset. */
    float       color[3];         /**< RGB in 0..1, for the link and the joint marker. */

    Matrix4f    world_transform;  /**< Written by rbt_robot_update_fk. */
} rbt_joint_t;

/**
 * @brief The arm: its joints and the primitives they are drawn with.
 *
 * The topology is fixed once rbt_robot_build has run. @ref rbt_joint_t::parent
 * and @ref rbt_joint_t::child_count describe it, and nothing recomputes them.
 */
typedef struct {
    std::vector<rbt_joint_t> joints;  /**< Root first; every parent precedes its children. */

    rbt_mesh_t cylinder;              /**< Unit cylinder: radius 1, height 1, along +Y. */
    rbt_mesh_t sphere;                /**< Unit sphere: radius 1. */
    rbt_mesh_t box;                   /**< Unit cube. */
} rbt_robot_t;

/**
 * @brief Build the joint array and generate the primitive meshes on the CPU.
 *
 * No GL calls happen here, so this may run before a context exists.
 * Post: every joint sits at its default angle, and parent and child counts
 * describe the arm.
 */
void rbt_robot_build(rbt_robot_t *robot);

/** @brief Upload the primitives. Pre: a GL context is current. */
void rbt_robot_upload_meshes(rbt_robot_t *robot);

/** @brief Restore every joint to its default angle. */
void rbt_robot_reset_joints(rbt_robot_t *robot);

/** @brief Recompute every world transform from the current joint angles. */
void rbt_robot_update_fk(rbt_robot_t *robot);

/**
 * @brief Draw the arm.
 *
 * Pre: rbt_robot_update_fk ran this frame and rbt_shader_set_frame was called
 * on @p shader.
 */
void rbt_robot_draw(const rbt_robot_t *robot, const rbt_shader_t *shader);

/** @brief Short label for a rotation axis, for the UI. */
const char *rbt_axis_label(rbt_axis_t axis);
