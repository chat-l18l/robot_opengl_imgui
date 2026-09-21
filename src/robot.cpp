/**
 * @file robot.cpp
 * @brief Arm definition, forward kinematics and rendering.
 */

#include "robot.h"

#include <assert.h>

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
 * The chain is data, not code: one table and one loop, so adding or
 * retuning a joint is a single line and no reference juggling.
 * ============================================================ */

/** @brief Declarative description of one joint in the chain. */
typedef struct {
    const char *name;
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

static const rbt_joint_spec_t s_arm_chain[] = {
    /* name              axis          min     max     default  offset (x, y, z)         radius  color (r, g, b)       */
    {"Base",             RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.00f, 0.0f}, 0.180f, {0.45f, 0.45f, 0.45f}},
    {"Shoulder Pan",     RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.100f, {0.85f, 0.35f, 0.15f}},
    {"Shoulder Lift",    RBT_AXIS_Z,  -90.0f, 120.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.080f, {0.20f, 0.55f, 0.90f}},
    {"Elbow",            RBT_AXIS_Z, -135.0f, 135.0f,  -45.0f, {0.0f, 1.20f, 0.0f}, 0.070f, {0.20f, 0.70f, 0.40f}},
    {"Wrist Pitch",      RBT_AXIS_Z, -180.0f, 180.0f,    0.0f, {0.0f, 1.00f, 0.0f}, 0.055f, {0.80f, 0.75f, 0.15f}},
    {"Wrist Roll",       RBT_AXIS_Y, -180.0f, 180.0f,    0.0f, {0.0f, 0.30f, 0.0f}, 0.040f, {0.70f, 0.30f, 0.70f}},
    {"Tool",             RBT_AXIS_X, -180.0f, 180.0f,    0.0f, {0.0f, 0.15f, 0.0f}, 0.035f, {0.90f, 0.90f, 0.20f}},
};

static const size_t s_arm_chain_count = sizeof(s_arm_chain) / sizeof(s_arm_chain[0]);

/** @brief Copy a spec into a joint. */
static void s_joint_from_spec(rbt_joint_t *joint, const rbt_joint_spec_t *spec)
{
    assert(joint != NULL);
    assert(spec != NULL);

    joint->name              = spec->name;
    joint->axis              = spec->axis;
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

/** @brief Append every joint under @p joint to @p out, depth first, parents before children. */
static void s_collect_joints(rbt_joint_t *joint, std::vector<rbt_joint_t *> *out)
{
    assert(joint != NULL);
    assert(out != NULL);

    out->push_back(joint);
    for (rbt_joint_t &child : joint->children) {
        s_collect_joints(&child, out);
    }
}

void rbt_robot_build(rbt_robot_t *robot)
{
    assert(robot != NULL);

    s_joint_from_spec(&robot->base, &s_arm_chain[0]);
    robot->base.children.clear();

    rbt_joint_t *parent = &robot->base;
    for (size_t i = 1; i < s_arm_chain_count; i++) {
        /* The table describes a single chain. Appending a second child would
         * invalidate `parent`, so the invariant is checked rather than assumed;
         * a branching arm needs a parent index in the spec, not this loop. */
        assert(parent->children.empty());

        parent->children.emplace_back();
        s_joint_from_spec(&parent->children.back(), &s_arm_chain[i]);
        parent = &parent->children.back();
    }

    /* Valid from here on because the tree is never modified again. */
    robot->joints.clear();
    robot->joints.reserve(s_arm_chain_count);
    s_collect_joints(&robot->base, &robot->joints);
    assert(robot->joints.size() == s_arm_chain_count);

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

    for (rbt_joint_t *joint : robot->joints) {
        joint->angle_deg = joint->default_angle_deg;
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

/** @brief Propagate the parent transform down the subtree at @p joint. */
static void s_update_fk(rbt_joint_t *joint, const Matrix4f &parent_transform)
{
    assert(joint != NULL);

    joint->world_transform = parent_transform
                           * rbt_translation(joint->offset.x(), joint->offset.y(), joint->offset.z())
                           * s_joint_rotation(joint);

    for (rbt_joint_t &child : joint->children) {
        s_update_fk(&child, joint->world_transform);
    }
}

void rbt_robot_update_fk(rbt_robot_t *robot)
{
    assert(robot != NULL);

    s_update_fk(&robot->base, Matrix4f::Identity());
}

/**
 * @brief Draw the gripper at the tip of a leaf joint.
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

/**
 * @brief Draw the links leaving @p joint, then recurse.
 *
 * A link is the unit cylinder stretched from this joint's origin to the child's
 * offset, so it always spans exactly the distance forward kinematics uses. The
 * marker sphere carries the child's own colour, matching the UI's colour key.
 */
static void s_draw_joint(const rbt_robot_t *robot, const rbt_shader_t *shader, const rbt_joint_t *joint)
{
    assert(joint != NULL);

    for (const rbt_joint_t &child : joint->children) {
        const float length = child.offset.y();
        const float radius = child.link_radius;

        rbt_shader_set_object(shader,
                              joint->world_transform * rbt_scale(radius, length, radius),
                              joint->color);
        rbt_mesh_draw(&robot->cylinder);

        const float marker = radius * s_joint_marker_scale;
        rbt_shader_set_object(shader,
                              joint->world_transform
                                  * rbt_translation(0.0f, length, 0.0f)
                                  * rbt_scale(marker, marker, marker),
                              child.color);
        rbt_mesh_draw(&robot->sphere);

        s_draw_joint(robot, shader, &child);
    }

    if (joint->children.empty()) {
        s_draw_gripper(robot, shader, joint->world_transform);
    }
}

void rbt_robot_draw(const rbt_robot_t *robot, const rbt_shader_t *shader)
{
    assert(robot != NULL);
    assert(shader != NULL);

    /* Pedestal: a property of the machine, not of any joint, so it is drawn
     * here instead of being special-cased inside the tree walk. The unit
     * cylinder already spans y = 0..1, so scaling alone stands it on the
     * floor; translating first would lift it clear of the grid. */
    rbt_shader_set_object(shader,
                          robot->base.world_transform
                              * rbt_scale(s_pedestal_radius, s_pedestal_height, s_pedestal_radius),
                          robot->base.color);
    rbt_mesh_draw(&robot->cylinder);

    s_draw_joint(robot, shader, &robot->base);
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
