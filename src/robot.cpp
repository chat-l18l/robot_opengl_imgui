/**
 * @file robot.cpp
 * @brief The built-in arm, forward kinematics and drawing.
 */

#include "robot.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * Built-in arm
 *
 * A 6-DOF industrial-style manipulator on a fixed pedestal:
 *
 *   Base          Y  — the pedestal's own turntable
 *   Shoulder Pan  Y  — slews the whole arm
 *   Shoulder Lift Z  — raises the upper arm
 *   Elbow         Z  — bends the forearm
 *   Wrist Pitch   Z
 *   Wrist Roll    Y
 *   Tool          X  — carries the gripper
 *
 * The arm is data, not code: one table for the joints, and its geometry is
 * generated from the same table as ordinary visuals. Each row names its
 * parent by index, so a branch is another row, not another data structure.
 * ============================================================ */

/** @brief Declarative description of one joint of the built-in arm. */
typedef struct {
    const char *name;
    int         parent;         /**< Row index of the parent, or RBT_NO_PARENT. */
    float       axis[3];        /**< Unit rotation axis. */
    float       min_angle_deg;
    float       max_angle_deg;
    float       default_angle_deg;
    float       offset[3];      /**< Parent to this joint; the link runs along +Y of the parent. */
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

/* Indices of the built-in primitives in rbt_robot_t::meshes. */
static const int s_mesh_cylinder = 0;
static const int s_mesh_sphere   = 1;
static const int s_mesh_box      = 2;

static const rbt_joint_spec_t s_arm_joints[] = {
    /* name             parent         axis             min     max     default  offset (x, y, z)     radius  color (r, g, b)      */
    {"Base",            RBT_NO_PARENT, {0.0f, 1.0f, 0.0f}, -180.0f, 180.0f,    0.0f, {0.0f, 0.00f, 0.0f}, 0.180f, {0.45f, 0.45f, 0.45f}},
    {"Shoulder Pan",                0, {0.0f, 1.0f, 0.0f}, -180.0f, 180.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.100f, {0.85f, 0.35f, 0.15f}},
    {"Shoulder Lift",               1, {0.0f, 0.0f, 1.0f},  -90.0f, 120.0f,    0.0f, {0.0f, 0.25f, 0.0f}, 0.080f, {0.20f, 0.55f, 0.90f}},
    {"Elbow",                       2, {0.0f, 0.0f, 1.0f}, -135.0f, 135.0f,  -45.0f, {0.0f, 1.20f, 0.0f}, 0.070f, {0.20f, 0.70f, 0.40f}},
    {"Wrist Pitch",                 3, {0.0f, 0.0f, 1.0f}, -180.0f, 180.0f,    0.0f, {0.0f, 1.00f, 0.0f}, 0.055f, {0.80f, 0.75f, 0.15f}},
    {"Wrist Roll",                  4, {0.0f, 1.0f, 0.0f}, -180.0f, 180.0f,    0.0f, {0.0f, 0.30f, 0.0f}, 0.040f, {0.70f, 0.30f, 0.70f}},
    {"Tool",                        5, {1.0f, 0.0f, 0.0f}, -180.0f, 180.0f,    0.0f, {0.0f, 0.15f, 0.0f}, 0.035f, {0.90f, 0.90f, 0.20f}},
};

static const size_t s_arm_joint_count = sizeof(s_arm_joints) / sizeof(s_arm_joints[0]);

void rbt_name_copy(char out[RBT_NAME_SIZE], const char *name)
{
    assert(out != NULL);

    if (name == NULL) {
        out[0] = '\0';
        return;
    }
    strncpy(out, name, RBT_NAME_SIZE - 1);
    out[RBT_NAME_SIZE - 1] = '\0';
}

/** @brief Append one visual. */
static void s_add_visual(rbt_robot_t *robot, int joint, int mesh, const Matrix4f &local, const float color[3])
{
    rbt_visual_t visual;
    visual.joint    = joint;
    visual.mesh     = mesh;
    visual.local    = local;
    visual.color[0] = color[0];
    visual.color[1] = color[1];
    visual.color[2] = color[2];
    robot->visuals.push_back(visual);
}

/**
 * @brief Generate the arm's geometry as visuals.
 *
 * The order matches the order this geometry was drawn in before visuals were
 * data, which is what keeps the rendered image byte for byte the same.
 */
static void s_add_builtin_visuals(rbt_robot_t *robot)
{
    static const float palm_color[3]   = {0.70f, 0.70f, 0.70f};
    static const float finger_color[3] = {0.95f, 0.95f, 0.25f};

    /* Pedestal: the unit cylinder already spans y = 0..1, so scaling alone
     * stands it on the floor. */
    s_add_visual(robot, 0, s_mesh_cylinder,
                 rbt_scale(s_pedestal_radius, s_pedestal_height, s_pedestal_radius),
                 robot->joints[0].color);

    for (size_t i = 0; i < robot->joints.size(); i++) {
        const rbt_joint_t      &joint = robot->joints[i];
        const rbt_joint_spec_t *spec  = &s_arm_joints[i];

        if (joint.parent != RBT_NO_PARENT) {
            /* The link is the unit cylinder stretched from the parent's origin
             * to this joint, in the parent's frame. The marker sphere at its
             * far end carries this joint's own colour, matching the panel. */
            const float length = spec->offset[1];
            const float radius = spec->link_radius;
            const float marker = radius * s_joint_marker_scale;

            s_add_visual(robot, joint.parent, s_mesh_cylinder,
                         rbt_scale(radius, length, radius),
                         robot->joints[(size_t)joint.parent].color);
            s_add_visual(robot, joint.parent, s_mesh_sphere,
                         rbt_translation(0.0f, length, 0.0f) * rbt_scale(marker, marker, marker),
                         joint.color);
        }

        if (joint.child_count == 0) {
            /* Gripper: a palm block at the flange, two fingers along +Y. */
            s_add_visual(robot, (int)i, s_mesh_box,
                         rbt_translation(0.0f, 0.03f, 0.0f) * rbt_scale(0.10f, 0.04f, 0.06f),
                         palm_color);
            s_add_visual(robot, (int)i, s_mesh_box,
                         rbt_translation(-0.05f, 0.09f, 0.0f) * rbt_scale(0.02f, 0.10f, 0.035f),
                         finger_color);
            s_add_visual(robot, (int)i, s_mesh_box,
                         rbt_translation(0.05f, 0.09f, 0.0f) * rbt_scale(0.02f, 0.10f, 0.035f),
                         finger_color);
        }
    }
}

void rbt_robot_build_builtin(rbt_robot_t *robot)
{
    assert(robot != NULL);
    assert(s_arm_joint_count > 0);

    rbt_name_copy(robot->name, "Built-in 6-DOF arm");
    robot->joints.clear();
    robot->meshes.clear();
    robot->visuals.clear();

    robot->joints.resize(s_arm_joint_count);
    for (size_t i = 0; i < s_arm_joint_count; i++) {
        const rbt_joint_spec_t *spec  = &s_arm_joints[i];
        rbt_joint_t            *joint = &robot->joints[i];

        rbt_name_copy(joint->name, spec->name);
        joint->type              = RBT_JOINT_REVOLUTE;
        joint->parent            = spec->parent;
        joint->child_count       = 0;
        joint->origin            = rbt_translation(spec->offset[0], spec->offset[1], spec->offset[2]);
        joint->axis              = Vector3f(spec->axis[0], spec->axis[1], spec->axis[2]);
        joint->min_angle_deg     = spec->min_angle_deg;
        joint->max_angle_deg     = spec->max_angle_deg;
        joint->default_angle_deg = spec->default_angle_deg;
        joint->angle_deg         = spec->default_angle_deg;
        joint->color[0]          = spec->color[0];
        joint->color[1]          = spec->color[1];
        joint->color[2]          = spec->color[2];
        joint->world_transform   = Matrix4f::Identity();
    }
    rbt_robot_finalize(robot);

    /* Pushed in the order of the s_mesh_* indices above. */
    robot->meshes.reserve(3);
    robot->meshes.push_back(rbt_mesh_make_cylinder(1.0f, 1.0f, 1.0f, s_cylinder_segments));
    robot->meshes.push_back(rbt_mesh_make_sphere(1.0f, s_sphere_rings, s_sphere_segments));
    robot->meshes.push_back(rbt_mesh_make_box(1.0f, 1.0f, 1.0f));

    s_add_builtin_visuals(robot);
}

void rbt_robot_finalize(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (size_t i = 0; i < robot->joints.size(); i++) {
        rbt_joint_t &joint = robot->joints[i];

        /* A parent must already exist. That single rule is what lets forward
         * kinematics run as one forward pass over the array. */
        assert(joint.parent == RBT_NO_PARENT
               || (joint.parent >= 0 && (size_t)joint.parent < i));

        joint.child_count = 0;
        if (joint.parent != RBT_NO_PARENT) {
            robot->joints[(size_t)joint.parent].child_count++;
        }
    }
}

void rbt_robot_upload_meshes(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (rbt_mesh_t &mesh : robot->meshes) {
        rbt_mesh_upload(&mesh);
    }
}

void rbt_robot_reset_joints(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (rbt_joint_t &joint : robot->joints) {
        joint.angle_deg = joint.default_angle_deg;
    }
}

/**
 * @brief Rotation of @p radians about a unit axis.
 *
 * The principal axes take the direct path: it is exact, cheaper than the
 * general form, and covers most real joints. Anything else goes through
 * Rodrigues' formula.
 */
static Matrix4f s_axis_rotation(const Vector3f &axis, float radians)
{
    if (axis == Vector3f::UnitX()) {
        return rbt_rotation_x(radians);
    }
    if (axis == Vector3f::UnitY()) {
        return rbt_rotation_y(radians);
    }
    if (axis == Vector3f::UnitZ()) {
        return rbt_rotation_z(radians);
    }

    Matrix4f m = Matrix4f::Identity();
    m.block<3, 3>(0, 0) = Eigen::AngleAxisf(radians, axis).toRotationMatrix();
    return m;
}

void rbt_robot_update_fk(rbt_robot_t *robot)
{
    assert(robot != NULL);

    for (size_t i = 0; i < robot->joints.size(); i++) {
        rbt_joint_t &joint = robot->joints[i];

        const Matrix4f local = (joint.type == RBT_JOINT_REVOLUTE)
                             ? joint.origin * s_axis_rotation(joint.axis, rbt_deg_to_rad(joint.angle_deg))
                             : joint.origin;

        /* The parent is earlier in the array, so its transform is already the
         * one for this frame. */
        joint.world_transform = (joint.parent == RBT_NO_PARENT)
                              ? local
                              : robot->joints[(size_t)joint.parent].world_transform * local;
    }
}

void rbt_robot_draw(const rbt_robot_t *robot, const rbt_shader_t *shader)
{
    assert(robot != NULL);
    assert(shader != NULL);

    for (const rbt_visual_t &visual : robot->visuals) {
        assert(visual.joint >= 0 && (size_t)visual.joint < robot->joints.size());
        assert(visual.mesh >= 0 && (size_t)visual.mesh < robot->meshes.size());

        rbt_shader_set_object(shader,
                              robot->joints[(size_t)visual.joint].world_transform * visual.local,
                              visual.color);
        rbt_mesh_draw(&robot->meshes[(size_t)visual.mesh]);
    }
}

bool rbt_robot_bounds(const rbt_robot_t *robot, Vector3f *box_min, Vector3f *box_max)
{
    assert(robot != NULL);
    assert(box_min != NULL);
    assert(box_max != NULL);

    bool any = false;
    for (const rbt_visual_t &visual : robot->visuals) {
        const rbt_mesh_t &mesh  = robot->meshes[(size_t)visual.mesh];
        const Matrix4f    model = robot->joints[(size_t)visual.joint].world_transform * visual.local;

        /* All eight corners: a rotated box's extent is not its min and max
         * corners transformed. */
        for (int corner = 0; corner < 8; corner++) {
            const Vector3f local((corner & 1) ? mesh.bounds_max.x() : mesh.bounds_min.x(),
                                 (corner & 2) ? mesh.bounds_max.y() : mesh.bounds_min.y(),
                                 (corner & 4) ? mesh.bounds_max.z() : mesh.bounds_min.z());
            const Vector3f world = (model * local.homogeneous()).head<3>();

            if (!any) {
                *box_min = world;
                *box_max = world;
                any = true;
            } else {
                *box_min = box_min->cwiseMin(world);
                *box_max = box_max->cwiseMax(world);
            }
        }
    }
    return any;
}

void rbt_axis_format(const Vector3f &axis, char *out, size_t out_size)
{
    assert(out != NULL);
    assert(out_size > 0);

    static const struct {
        Vector3f    axis;
        const char *label;
    } s_named[] = {
        {Vector3f( 1.0f,  0.0f,  0.0f), "X"},  {Vector3f(-1.0f,  0.0f,  0.0f), "-X"},
        {Vector3f( 0.0f,  1.0f,  0.0f), "Y"},  {Vector3f( 0.0f, -1.0f,  0.0f), "-Y"},
        {Vector3f( 0.0f,  0.0f,  1.0f), "Z"},  {Vector3f( 0.0f,  0.0f, -1.0f), "-Z"},
    };

    for (size_t i = 0; i < sizeof(s_named) / sizeof(s_named[0]); i++) {
        if (axis == s_named[i].axis) {
            snprintf(out, out_size, "%s", s_named[i].label);
            return;
        }
    }
    snprintf(out, out_size, "%.2f, %.2f, %.2f", (double)axis.x(), (double)axis.y(), (double)axis.z());
}
