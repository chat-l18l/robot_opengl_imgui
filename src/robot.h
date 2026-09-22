/**
 * @file robot.h
 * @brief Robot: joints for the kinematics, visuals for the geometry.
 *
 * The two are kept apart on purpose. A joint says where a frame is and how it
 * may move; a visual says what to draw in that frame. That is the split URDF
 * makes between joints and links, and it lets the built-in arm and a loaded
 * model go through exactly the same forward kinematics and the same draw loop.
 */

#pragma once

#include "gl_mesh.h"
#include "gl_shader.h"

#include <stddef.h>
#include <vector>

/** Room for a joint or model name, terminator included; longer names are cut. */
#define RBT_NAME_SIZE 64

/** @brief Parent index of a joint that has none. */
#define RBT_NO_PARENT (-1)

/** @brief How a joint moves relative to its parent. */
typedef enum {
    RBT_JOINT_FIXED = 0,  /**< Rigidly attached: carries geometry, never moves. */
    RBT_JOINT_REVOLUTE,   /**< Rotates about @ref rbt_joint_t::axis between two limits. */
} rbt_joint_type_t;

/**
 * @brief One frame in the kinematic tree.
 *
 * Joints live in one flat array and name their parent by index. A parent
 * always sits earlier in the array, so forward kinematics is a single forward
 * pass: no recursion, no pointers to keep valid, contiguous memory.
 */
typedef struct {
    char             name[RBT_NAME_SIZE];
    rbt_joint_type_t type;
    int              parent;             /**< Index of the parent joint, or RBT_NO_PARENT. */
    int              child_count;        /**< Filled by rbt_robot_finalize; 0 marks a tip. */

    Matrix4f         origin;             /**< Rest pose in the parent frame: translation and rotation. */
    Vector3f         axis;               /**< Unit rotation axis in this joint's frame. */

    float            min_angle_deg;      /**< Lower travel limit. */
    float            max_angle_deg;      /**< Upper travel limit. */
    float            default_angle_deg;  /**< Pose restored by rbt_robot_reset_joints. */
    float            angle_deg;          /**< Live angle, driven by the UI. */
    float            color[3];           /**< Colour key shown next to the joint in the panel. */

    Matrix4f         world_transform;    /**< Written by rbt_robot_update_fk. */
} rbt_joint_t;

/** @brief A mesh drawn in a joint's frame. */
typedef struct {
    int      joint;     /**< Index into rbt_robot_t::joints. */
    int      mesh;      /**< Index into rbt_robot_t::meshes. */
    Matrix4f local;     /**< Placement within the joint frame. */
    float    color[3];  /**< RGB in 0..1. */
} rbt_visual_t;

/**
 * @brief A robot: its kinematic tree, its meshes, and what is drawn where.
 *
 * Visuals refer to joints and meshes by index rather than by pointer, so the
 * arrays can grow while a model is being assembled. The topology is fixed
 * once rbt_robot_finalize has run.
 */
typedef struct {
    char                      name[RBT_NAME_SIZE];
    std::vector<rbt_joint_t>  joints;   /**< Every parent precedes its children. */
    std::vector<rbt_mesh_t>   meshes;
    std::vector<rbt_visual_t> visuals;  /**< Drawn in this order. */
} rbt_robot_t;

/**
 * @brief Build the built-in six-axis arm, on the CPU only.
 *
 * No GL calls happen here, so this may run before a context exists.
 * Post: every joint sits at its default angle and the robot is finalised.
 */
void rbt_robot_build_builtin(rbt_robot_t *robot);

/**
 * @brief Derive the per-joint bookkeeping once the joint array is complete.
 *
 * Pre: every joint's parent precedes it. That ordering is what the rest of the
 * code relies on, so a violation is a programmer error and asserts.
 */
void rbt_robot_finalize(rbt_robot_t *robot);

/** @brief Upload every mesh. Pre: a GL context is current. */
void rbt_robot_upload_meshes(rbt_robot_t *robot);

/** @brief Restore every joint to its default angle. */
void rbt_robot_reset_joints(rbt_robot_t *robot);

/** @brief Recompute every world transform from the current joint angles. */
void rbt_robot_update_fk(rbt_robot_t *robot);

/**
 * @brief Draw every visual.
 *
 * Pre: rbt_robot_update_fk ran this frame and rbt_shader_set_frame was called
 * on @p shader.
 */
void rbt_robot_draw(const rbt_robot_t *robot, const rbt_shader_t *shader);

/**
 * @brief World-space bounding box of everything drawn, in the current pose.
 *
 * Pre: the meshes are uploaded, which is when their bounds are recorded, and
 * rbt_robot_update_fk has run.
 * @return false when there is nothing to draw.
 */
bool rbt_robot_bounds(const rbt_robot_t *robot, Vector3f *box_min, Vector3f *box_max);

/**
 * @brief Describe an axis for the UI: "X", "-Z", or the vector itself.
 * @param out Caller-owned buffer, always terminated.
 */
void rbt_axis_format(const Vector3f &axis, char *out, size_t out_size);

/** @brief Copy a name into a fixed buffer, cutting it if it does not fit. */
void rbt_name_copy(char out[RBT_NAME_SIZE], const char *name);
