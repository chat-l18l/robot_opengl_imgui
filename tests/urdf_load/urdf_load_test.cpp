/**
 * @file urdf_load_test.cpp
 * @brief Headless check of the URDF loader against a robot with known answers.
 *
 * Why: the loader has several places where a mistake still produces a robot
 * that looks plausible. rpy composed in the wrong order, a cylinder left on
 * the wrong axis, degrees taken for radians, a colour rule turned round, a
 * package not found. test_pkg/urdf/test_robot.urdf is small enough to work
 * out by hand, and the derivations are in the comments below rather than
 * computed by the code under test.
 *
 * What: structure, kinematics, geometry placement, colours, the COLLADA up
 * axis with and without --honor-up-axis, and files that must be refused. When
 * example-robot-data is installed, as it is through pixi, two real robots are
 * loaded as well; without it that part is skipped, not failed.
 *
 * Throughout: the root frame turns ROS Z-up into this viewer's Y-up, so a ROS
 * point (x, y, z) lands in the world at (x, z, -y).
 */

#include "urdf_load.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>

#ifndef RBT_TEST_DATA_DIR
#error "RBT_TEST_DATA_DIR must name this test's directory"
#endif

#define RBT_TEST_ROBOT RBT_TEST_DATA_DIR "/test_pkg/urdf/test_robot.urdf"

static const float s_tolerance = 1e-4f;
static const float s_rad_to_deg = 180.0f / RBT_PI;

static int s_failures = 0;

static void s_check(bool ok, const char *what)
{
    printf("  %-60s %s\n", what, ok ? "pass" : "FAIL");
    if (!ok) {
        s_failures++;
    }
}

static int s_find(const rbt_robot_t *robot, const char *name)
{
    for (size_t i = 0; i < robot->joints.size(); i++) {
        if (strcmp(robot->joints[i].name, name) == 0) {
            return (int)i;
        }
    }
    return -1;
}

/** @brief The @p nth visual drawn in the named frame, or NULL. */
static const rbt_visual_t *s_visual(const rbt_robot_t *robot, const char *frame, int nth)
{
    const int index = s_find(robot, frame);
    for (const rbt_visual_t &visual : robot->visuals) {
        if (visual.joint == index && nth-- == 0) {
            return &visual;
        }
    }
    return NULL;
}

/** @brief World position of a visual's local origin. */
static Vector3f s_visual_origin(const rbt_robot_t *robot, const rbt_visual_t *visual)
{
    const Matrix4f m = robot->joints[(size_t)visual->joint].world_transform * visual->local;
    return m.block<3, 1>(0, 3);
}

static void s_pose(rbt_robot_t *robot, const char *moved, float degrees)
{
    for (rbt_joint_t &joint : robot->joints) {
        joint.angle_deg = (moved != NULL && strcmp(joint.name, moved) == 0) ? degrees : 0.0f;
    }
    rbt_robot_update_fk(robot);
}

static bool s_near(const Vector3f &a, const Vector3f &b)
{
    return (a - b).norm() < s_tolerance;
}

static bool s_color_is(const float c[3], float r, float g, float b)
{
    return fabsf(c[0] - r) < 1e-6f && fabsf(c[1] - g) < 1e-6f && fabsf(c[2] - b) < 1e-6f;
}

/** @brief Bounds of a mesh's staged vertices; meshes are never uploaded here. */
static void s_mesh_bounds(const rbt_mesh_t &mesh, Vector3f *lo, Vector3f *hi)
{
    *lo = mesh.vertices[0].position;
    *hi = *lo;
    for (const rbt_vertex_t &v : mesh.vertices) {
        *lo = lo->cwiseMin(v.position);
        *hi = hi->cwiseMax(v.position);
    }
}

static void s_case_structure(void)
{
    printf("test_robot.urdf, structure:\n");

    rbt_robot_t robot;
    const bool loaded = rbt_urdf_load(&robot, RBT_TEST_ROBOT, false);
    s_check(loaded, "loads");
    if (!loaded) {
        return;
    }
    s_check(strcmp(robot.name, "test_robot") == 0, "named after <robot name>");

    static const char *const order[] = {"base_link", "yaw", "pitch", "slide", "follower"};
    bool order_ok = robot.joints.size() == 5;
    for (size_t i = 0; order_ok && i < 5; i++) {
        order_ok = strcmp(robot.joints[i].name, order[i]) == 0;
    }
    s_check(order_ok, "one frame per link, root by link name, the rest by joint");
    s_check(robot.joints[1].parent == 0 && robot.joints[2].parent == 1
            && robot.joints[3].parent == 2 && robot.joints[4].parent == 2,
            "parents follow the joint tree");
    s_check(s_find(&robot, "commented_out") < 0, "a commented-out link is not there");

    const rbt_joint_t &yaw      = robot.joints[1];
    const rbt_joint_t &pitch    = robot.joints[2];
    const rbt_joint_t &slide    = robot.joints[3];
    const rbt_joint_t &follower = robot.joints[4];
    s_check(robot.joints[0].type == RBT_JOINT_FIXED, "the root is fixed");
    s_check(yaw.type == RBT_JOINT_REVOLUTE && yaw.min_angle_deg == -180.0f && yaw.max_angle_deg == 180.0f,
            "continuous moves, a full turn each way");
    /* -1 and 2 radians. */
    s_check(pitch.type == RBT_JOINT_REVOLUTE && fabsf(pitch.min_angle_deg - (-1.0f * s_rad_to_deg)) < 1e-3f
            && fabsf(pitch.max_angle_deg - 2.0f * s_rad_to_deg) < 1e-3f,
            "revolute limits converted from radians");
    s_check(pitch.axis == Vector3f::UnitY() && yaw.axis == Vector3f::UnitZ(), "axes as written");
    s_check(slide.type == RBT_JOINT_FIXED, "prismatic is shown fixed");
    s_check(follower.type == RBT_JOINT_REVOLUTE, "a mimic joint still moves, on its own");
}

static void s_case_geometry(void)
{
    printf("test_robot.urdf, geometry and colour:\n");

    rbt_robot_t robot;
    if (!rbt_urdf_load(&robot, RBT_TEST_ROBOT, false)) {
        s_check(false, "loads");
        return;
    }

    const rbt_visual_t *box      = s_visual(&robot, "base_link", 0);
    const rbt_visual_t *cylinder = s_visual(&robot, "yaw", 0);
    const rbt_visual_t *stl      = s_visual(&robot, "pitch", 0);
    const rbt_visual_t *dae      = s_visual(&robot, "pitch", 1);
    const rbt_visual_t *sphere   = s_visual(&robot, "slide", 0);
    if (box == NULL || cylinder == NULL || stl == NULL || dae == NULL || sphere == NULL) {
        s_check(false, "every visual is present");
        return;
    }
    s_check(robot.visuals.size() == 5, "five visuals, none from the commented-out link");

    s_check(box->local(0, 0) == 0.4f && box->local(1, 1) == 0.2f && box->local(2, 2) == 0.1f,
            "a box is scaled to its size");

    /* URDF cylinders run along Z, centred: the unit cylinder's axis, 0 to 1
     * along Y, must land from z = -0.3 to z = +0.3 for length 0.6. */
    const Vector3f bottom = (cylinder->local * Vector3f(0.0f, 0.0f, 0.0f).homogeneous()).head<3>();
    const Vector3f top    = (cylinder->local * Vector3f(0.0f, 1.0f, 0.0f).homogeneous()).head<3>();
    s_check(s_near(bottom, Vector3f(0.0f, 0.0f, -0.3f)) && s_near(top, Vector3f(0.0f, 0.0f, 0.3f)),
            "a cylinder runs along Z, centred on its origin");
    s_check(fabsf(stl->local(0, 0) - 2.0f) < 1e-6f, "a mesh's scale attribute is applied");

    s_check(s_color_is(box->color, 0.0f, 0.0f, 1.0f), "a named material's colour");
    s_check(s_color_is(cylinder->color, 1.0f, 0.0f, 0.0f), "an inline material's colour");
    s_check(s_color_is(stl->color, 0.0f, 0.0f, 1.0f), "a mesh without colour takes the URDF's");
    s_check(s_color_is(dae->color, 0.0f, 1.0f, 0.0f), "a mesh's own colour wins over the URDF's");
    s_check(s_color_is(sphere->color, 0.70f, 0.70f, 0.72f), "no colour at all gives the default");
    s_check(robot.joints[0].color[2] == 1.0f, "a frame's panel colour is its first visual's");
}

static void s_case_kinematics(void)
{
    printf("test_robot.urdf, kinematics:\n");

    rbt_robot_t robot;
    if (!rbt_urdf_load(&robot, RBT_TEST_ROBOT, false)) {
        s_check(false, "loads");
        return;
    }
    const rbt_visual_t *box = s_visual(&robot, "base_link", 0);
    const rbt_visual_t *stl = s_visual(&robot, "pitch", 0);
    if (box == NULL || stl == NULL) {
        s_check(false, "the box and the STL visual are present");
        return;
    }

    s_pose(&robot, NULL, 0.0f);
    /* The box sits at ROS (0, 0, 0.05), so at world (0, 0.05, 0). */
    s_check(s_near(s_visual_origin(&robot, box), Vector3f(0.0f, 0.05f, 0.0f)), "the root turns Z-up into Y-up");

    /* The STL visual is (0, 0, 0.2) in the pitch frame. The frame's rpy is a
     * quarter turn about Y, (x, y, z) -> (z, y, -x), giving (0.2, 0, 0) in the
     * upper frame, which is 0.1 + 0.3 = 0.4 up: ROS (0.2, 0, 0.4), world
     * (0.2, 0.4, 0). */
    s_check(s_near(s_visual_origin(&robot, stl), Vector3f(0.2f, 0.4f, 0.0f)), "an origin's rpy turns its frame");

    /* Yaw 90 degrees about ROS Z, at height 0.1: (0.2, 0, 0.3) above it turns
     * to (0, 0.2, 0.3), so ROS (0, 0.2, 0.4), world (0, 0.4, -0.2). */
    s_pose(&robot, "yaw", 90.0f);
    s_check(s_near(s_visual_origin(&robot, stl), Vector3f(0.0f, 0.4f, -0.2f)), "a joint turns what hangs off it");

    /* Pitch 90 degrees about Y, after the frame's own quarter turn about Y:
     * half a turn in all, (0, 0, 0.2) -> (0, 0, -0.2), so ROS (0, 0, 0.2),
     * world (0, 0.2, 0). */
    s_pose(&robot, "pitch", 90.0f);
    s_check(s_near(s_visual_origin(&robot, stl), Vector3f(0.0f, 0.2f, 0.0f)),
            "the joint's rotation comes after its origin's");
}

/**
 * @brief rpy composes as Rz(yaw) * Ry(pitch) * Rx(roll), about fixed axes.
 *
 * rpy.urdf turns roll 90 and yaw 90. Taking (0, 1, 0): roll first gives
 * (0, 0, 1), which yaw about Z leaves alone, so (0, 0, 1). The opposite order
 * would give yaw first, (-1, 0, 0), then roll, still (-1, 0, 0).
 */
static void s_case_rpy_order(void)
{
    printf("rpy.urdf, rotation order:\n");

    rbt_robot_t robot;
    const rbt_visual_t *visual = NULL;
    if (!rbt_urdf_load(&robot, RBT_TEST_DATA_DIR "/rpy.urdf", false) || (visual = s_visual(&robot, "base", 0)) == NULL) {
        s_check(false, "loads");
        return;
    }
    const Vector3f turned = visual->local.block<3, 3>(0, 0) * Vector3f::UnitY();
    s_check(s_near(turned, Vector3f::UnitZ()), "roll, then pitch, then yaw, about fixed axes");
}

static void s_case_up_axis(void)
{
    printf("COLLADA up axis:\n");

    /* marker_y_up.dae reaches from 0 to 1 along +Y as written, and declares Y_UP. */
    Vector3f lo;
    Vector3f hi;

    rbt_robot_t ignored;
    const rbt_visual_t *ignored_dae = NULL;
    if (!rbt_urdf_load(&ignored, RBT_TEST_ROBOT, false) || (ignored_dae = s_visual(&ignored, "pitch", 1)) == NULL) {
        s_check(false, "loads, with the COLLADA visual");
        return;
    }
    s_mesh_bounds(ignored.meshes[(size_t)ignored_dae->mesh], &lo, &hi);
    s_check(fabsf(hi.y() - 1.0f) < 1e-5f && fabsf(hi.z()) < 1e-5f, "ignored by default: taken as written");

    /* Honoured, Y_UP turns +90 degrees about X, (x, y, z) -> (x, -z, y): the
     * tip moves from +Y to +Z. */
    rbt_robot_t honored;
    const rbt_visual_t *honored_dae = NULL;
    const rbt_visual_t *honored_stl = NULL;
    if (!rbt_urdf_load(&honored, RBT_TEST_ROBOT, true) || (honored_dae = s_visual(&honored, "pitch", 1)) == NULL
        || (honored_stl = s_visual(&honored, "pitch", 0)) == NULL) {
        s_check(false, "loads with the up axis honoured, with both mesh visuals");
        return;
    }
    s_mesh_bounds(honored.meshes[(size_t)honored_dae->mesh], &lo, &hi);
    s_check(fabsf(hi.z() - 1.0f) < 1e-5f && fabsf(hi.y()) < 1e-5f, "honoured on request: turned to Z-up");

    s_mesh_bounds(honored.meshes[(size_t)honored_stl->mesh], &lo, &hi);
    s_check(fabsf(hi.y() - 0.1f) < 1e-5f && fabsf(hi.z()) < 1e-5f, "STL declares nothing and is never turned");
}

static void s_case_refusals(void)
{
    printf("Refused files:\n");

    rbt_robot_t robot;
    rbt_robot_build_builtin(&robot);
    const size_t joints_before = robot.joints.size();

    s_check(!rbt_urdf_load(&robot, RBT_TEST_DATA_DIR "/does_not_exist.urdf", false), "a missing file");
    s_check(!rbt_urdf_load(&robot, RBT_TEST_DATA_DIR "/urdf_load_test.cpp", false), "a file that is not XML");
    s_check(!rbt_urdf_load(&robot, RBT_TEST_DATA_DIR "/empty.urdf", false), "a robot without links");
    s_check(!rbt_urdf_load(&robot, RBT_TEST_DATA_DIR "/loop.urdf", false), "joints that form a loop");
    s_check(robot.joints.size() == joints_before && strcmp(robot.name, "Built-in 6-DOF arm") == 0,
            "and the robot is left as it was");
}

/** @brief Two real robots from example-robot-data, when it is installed. */
static void s_case_real_robots(void)
{
    printf("example-robot-data:\n");

    const char *prefix = getenv("CONDA_PREFIX");
    const std::string root = std::string(prefix != NULL ? prefix : "") + "/share/example-robot-data/robots";
    if (prefix == NULL || access(root.c_str(), R_OK) != 0) {
        printf("  %-60s skipped\n", "not installed (run through pixi to include it)");
        return;
    }

    static const struct {
        const char *path;
        size_t      movable;
    } robots[] = {
        {"/panda_description/urdf/panda.urdf", 7},
        {"/ur_description/urdf/ur5_robot.urdf", 6},
    };
    for (const auto &r : robots) {
        rbt_robot_t robot;
        const bool  loaded = rbt_urdf_load(&robot, (root + r.path).c_str(), false);
        size_t      movable = 0;
        for (const rbt_joint_t &joint : robot.joints) {
            movable += (joint.type == RBT_JOINT_REVOLUTE) ? 1 : 0;
        }
        char what[96];
        snprintf(what, sizeof(what), "%s: %zu movable joints, meshes found", strrchr(r.path, '/') + 1, r.movable);
        s_check(loaded && movable == r.movable && !robot.meshes.empty(), what);
    }
}

int main(void)
{
    s_case_structure();
    s_case_geometry();
    s_case_kinematics();
    s_case_rpy_order();
    s_case_up_axis();
    s_case_refusals();
    s_case_real_robots();

    printf("\n%s\n", s_failures == 0 ? "urdf_load: all checks passed" : "urdf_load: FAILURES above");
    return s_failures == 0 ? 0 : 1;
}
