/**
 * @file robot.h
 * @brief Robot arm: joint tree, forward kinematics and rendering.
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

/**
 * @brief One revolute joint and the link that reaches it from its parent.
 *
 * The link is not stored separately: its length is the Y component of
 * @ref offset, so there is exactly one place that says how long it is.
 */
typedef struct rbt_joint_t {
    const char *name;             /**< Static string; joints do not own their name. */
    rbt_axis_t  axis;             /**< Rotation axis. */
    float       min_angle_deg;    /**< Lower travel limit. */
    float       max_angle_deg;    /**< Upper travel limit. */
    float       default_angle_deg;/**< Pose restored by rbt_robot_reset_joints. */
    float       angle_deg;        /**< Live angle, driven by the UI. */
    Vector3f    offset;           /**< Parent joint to this joint, in the parent frame. */
    float       link_radius;      /**< Radius of the link drawn along that offset. */
    float       color[3];         /**< RGB in 0..1, used for the link and the joint marker. */

    Matrix4f    world_transform;  /**< Written by rbt_robot_update_fk. */
    std::vector<rbt_joint_t> children;
} rbt_joint_t;

/**
 * @brief The arm: its joint tree, a flat view of it, and the shared primitives.
 *
 * @ref joints points into @ref base. The tree must not be modified after
 * rbt_robot_build, or those pointers dangle.
 */
typedef struct {
    rbt_joint_t base;                 /**< Root of the kinematic tree. */
    std::vector<rbt_joint_t *> joints;/**< Depth-first view, built once for the UI. */

    rbt_mesh_t cylinder;              /**< Unit cylinder: radius 1, height 1, along +Y. */
    rbt_mesh_t sphere;                /**< Unit sphere: radius 1. */
    rbt_mesh_t box;                   /**< Unit cube. */
} rbt_robot_t;

/**
 * @brief Build the joint tree and generate the primitive meshes on the CPU.
 *
 * No GL calls happen here, so this may run before a context exists.
 * Post: every joint sits at its default angle and @ref rbt_robot_t::joints is valid.
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
