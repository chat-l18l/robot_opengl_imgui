/**
 * @file robot.cpp
 * @brief Arm definition, forward kinematics and rendering.
 */

#include "robot.h"

#include <assert.h>
#include <stddef.h>

/* ============================================================
 * Arm definition
 *
 * A 6-DOF industrial-style manipulator on a fixed pedestal:
 *
 *   Base          fixed pedestal, rotates about Y
 *   Shoulder Pan  Y  — slews the whole arm
 *   Shoulder Lift Z  — raises the upper arm
 *   Elbow         Z  — bends the forearm
 *   Wrist Pitch   Z
 *   Wrist Roll    Y
 *   Tool          X  — carries the gripper
 *
 * The arm is data, not code: one table and one loop. Each row names its
 * parent by index, so a second arm hanging off the same shoulder is another
 * row rather than another data structure.
 * ============================================================ */

/** @brief Declarative description of one joint. */
typedef struct {
    const char *name;
    int         parent;         /**< Row index of the parent, or RBT_NO_PARENT. */
    rbt_axis_t  axis;
    float       min_angle_deg;
    float       max_angle_deg;
    float       default_angle_deg;
    float       offset[3];      /**< Plain floats keep this table a POD. */
    float       link_radius;
    float       color[3];
} rbt_joint_spec_t;

/** Pedestal dimensions, drawn under the root joint. */
static const float s_pedestal_radius = 0.18f;
static const float s_pedestal_height = 0.24f;

/** Joint markers are drawn as spheres this much wider than the link. */
static const float s_joint_marker_scale = 1.4f;

/** Mesh tessellation. Enough to look round, small enough to stay cheap. */
static const int s_cylinder_segments = 24;
static const int s_sphere_rings      = 16;
static const int s_sphere_segments   = 24;

static const rbt_joint_spec_t s_arm_joints[] = {
    /* name             parent          axis          min     max     default  offset (x, y, z)     radius  color (r, g, b)      */
    {"Base",            RBT_NO_PARENT, RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.00f, 0.0f}, 0.180f, {0.45f, 0.45f, 0.45f}},
    {"Shoulder Pan",                0, RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.100f, {0.85f, 0.35f, 0.15f}},
    {"Shoulder Lift",               1, RBT_AXIS_Z,  -90.0f, 120.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.080f, {0.20f, 0.55f, 0.90f}},
    {"Elbow",                       2, RBT_AXIS_Z, -135.0f, 135.0f,  -45.0f, {0.0f, 1.20f, 0.0f}, 0.070f, {0.20f, 0.70f, 0.40f}},
    {"Wrist Pitch",                 3, RBT_AXIS_Z, -180.0f, 180.0f,    0.0f, {0.0f, 1.00f, 0.0f}, 0.055f, {0.80f, 0.75f, 0.15f}},
    {"Wrist Roll",                  4, RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.30f, 0.0f}, 0.040f, {0.70f, 0.30f, 0.70f}},
    {"Tool",                        5, RBT_AXIS_X, -180.0f, 180.0f,    0.0f, {0.0f, 0.15f, 0.0f}, 0.035f, {0.90f, 0.90f, 0.20f}},
};

static const size_t s_arm_joint_count = sizeof(s_arm_joints) / sizeof(s_arm_joints[0]);

/** @brief Copy a spec into a joint. */
static void s_joint_from_spec(rbt_joint_t *joint, const rbt_joint_spec_t *spec)
{
    assert(joint != NULL);
    assert(spec != NULL);

    joint->name              = spec->name;
    joint->axis              = spec->axis;
    joint->parent            = spec->parent;
    joint->child_count       = 0;
    joint->min_angle_deg     = spec->min_angle_deg;
    joint->max_angle_deg     = spec->max_angle_deg;
    joint->default_angle_deg = spec->default_angle_deg;
    joint->angle_deg         = spec->default_angle_deg;
    joint->offset            = Vector3f(spec->offset[0], spec->offset[1], spec->offset[2]);
    joint->link_radius       = spec->link_radius;
    joint->color[0]          = spec->color[0];
    joint->color[1]          = spec->color[1];
    joint->color[2]          = spec->color[2];
    joint->world_transform   = Matrix4f::Identity();
}

void rbt_robot_build(rbt_robot_t *robot)
{
    assert(robot != NULL);
    assert(s_arm_joint_count > 0);
    assert(s_arm_joints[0].parent == RBT_NO_PARENT);

    robot->joints.clear();
    robot->joints.resize(s_arm_joint_count);

    for (size_t i = 0; i < s_arm_joint_count; i++) {
        const rbt_joint_spec_t *spec = &s_arm_joints[i];

        /* A parent must already exist. That single rule is what lets forward
         * kinematics and drawing run as one forward pass over the array. */
        assert(spec->parent == RBT_NO_PARENT
               || (spec->parent >= 0 && (size_t)spec->parent < i));

        s_joint_from_spec(&robot->joints[i], spec);
    }

    for (const rbt_joint_t &joint : robot->joints) {
        if (joint.parent != RBT_NO_PARENT) {
            robot->joints[(size_t)joint.parent].child_count++;
        }
    }

    robot->cylinder = rbt_mesh_make_cylinder(1.0f, 1.0f, 1.0f, s_cylinder_segments);
    robot->sphere   = rbt_mesh_make_sphere(1.0f, s_sphere_rings, s_sphere_segments);
    robot->box      = rbt_mesh_make_box(1.0f, 1.0f, 1.0f);
}

void rbt_robot_upload_meshes(rbt_robot_t *robot)
{
    assert(robot != NULL);

    rbt_mesh_upload(&robot->cylinder);
    rbt_mesh_upload(&robot->sphere);
    rbt_mesh_upload(&robot->box);
}

void rbt_robot_reset_joints(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (rbt_joint_t &joint : robot->joints) {
        joint.angle_deg = joint.default_angle_deg;
    }
}

/** @brief Rotation matrix for a joint's current angle about its own axis. */
static Matrix4f s_joint_rotation(const rbt_joint_t *joint)
{
    const float radians = rbt_deg_to_rad(joint->angle_deg);

    switch (joint->axis) {
    case RBT_AXIS_X: return rbt_rotation_x(radians);
    case RBT_AXIS_Y: return rbt_rotation_y(radians);
    case RBT_AXIS_Z: return rbt_rotation_z(radians);
    }

    assert(false && "unknown rotation axis");
    return Matrix4f::Identity();
}

void rbt_robot_update_fk(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (size_t i = 0; i < robot->joints.size(); i++) {
        rbt_joint_t &joint = robot->joints[i];

        const Matrix4f local =
            rbt_translation(joint.offset.x(), joint.offset.y(), joint.offset.z())
            * s_joint_rotation(&joint);

        /* The parent is earlier in the array, so its transform is already the
         * one for this frame. */
        joint.world_transform = (joint.parent == RBT_NO_PARENT)
                              ? local
                              : robot->joints[(size_t)joint.parent].world_transform * local;
    }
}

/**
 * @brief Draw the gripper at the tip of a joint that carries no children.
 *
 * Geometry is expressed in the joint's own frame: a palm block at the flange
 * with two fingers reaching along +Y.
 */
static void s_draw_gripper(const rbt_robot_t *robot, const rbt_shader_t *shader, const Matrix4f &tool)
{
    static const float palm_color[3]   = {0.70f, 0.70f, 0.70f};
    static const float finger_color[3] = {0.95f, 0.95f, 0.25f};

    rbt_shader_set_object(shader, tool * rbt_translation(0.0f, 0.03f, 0.0f)
                                       * rbt_scale(0.10f, 0.04f, 0.06f), palm_color);
    rbt_mesh_draw(&robot->box);

    rbt_shader_set_object(shader, tool * rbt_translation(-0.05f, 0.09f, 0.0f)
                                       * rbt_scale(0.02f, 0.10f, 0.035f), finger_color);
    rbt_mesh_draw(&robot->box);

    rbt_shader_set_object(shader, tool * rbt_translation(0.05f, 0.09f, 0.0f)
                                       * rbt_scale(0.02f, 0.10f, 0.035f), finger_color);
    rbt_mesh_draw(&robot->box);
}

void rbt_robot_draw(const rbt_robot_t *robot, const rbt_shader_t *shader)
{
    assert(robot != NULL);
    assert(shader != NULL);
    assert(!robot->joints.empty());
    assert(robot->joints[0].parent == RBT_NO_PARENT);

    /* Pedestal: a property of the machine, not of any joint, so it is drawn
     * here rather than being special-cased inside the walk. The unit cylinder
     * already spans y = 0..1, so scaling alone stands it on the floor. */
    const rbt_joint_t &root = robot->joints[0];
    rbt_shader_set_object(shader,
                          root.world_transform
                              * rbt_scale(s_pedestal_radius, s_pedestal_height, s_pedestal_radius),
                          root.color);
    rbt_mesh_draw(&robot->cylinder);

    for (const rbt_joint_t &joint : robot->joints) {
        if (joint.parent != RBT_NO_PARENT) {
            /* The link is the unit cylinder stretched from the parent's origin
             * to this joint's offset, so it always spans exactly the distance
             * forward kinematics uses. The marker sphere carries this joint's
             * own colour, matching the panel's colour key. */
            const rbt_joint_t &parent = robot->joints[(size_t)joint.parent];
            const float length = joint.offset.y();
            const float radius = joint.link_radius;

            rbt_shader_set_object(shader,
                                  parent.world_transform * rbt_scale(radius, length, radius),
                                  parent.color);
            rbt_mesh_draw(&robot->cylinder);

            const float marker = radius * s_joint_marker_scale;
            rbt_shader_set_object(shader,
                                  parent.world_transform
                                      * rbt_translation(0.0f, length, 0.0f)
                                      * rbt_scale(marker, marker, marker),
                                  joint.color);
            rbt_mesh_draw(&robot->sphere);
        }

        if (joint.child_count == 0) {
            s_draw_gripper(robot, shader, joint.world_transform);
        }
    }
}

const char *rbt_axis_label(rbt_axis_t axis)
{
    switch (axis) {
    case RBT_AXIS_X: return "X";
    case RBT_AXIS_Y: return "Y";
    case RBT_AXIS_Z: return "Z";
    }
    return "?";
}
