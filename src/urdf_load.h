/**
 * @file urdf_load.h
 * @brief Build a robot from a URDF file.
 *
 * URDF describes a tree of links joined by joints. The mapping onto the
 * viewer's flat joint array is one frame per link:
 *
 * - the root link becomes a fixed frame, turned upright: ROS is Z-up, this
 *   viewer Y-up, so the root carries a -90 degree turn about X;
 * - every other link's frame is its parent's, moved by the incoming joint's
 *   origin and then by the joint itself. It is named after that joint,
 *   because the joint is what the panel moves;
 * - every <visual> becomes a visual in its link's frame. Meshes are read
 *   through rbt_mesh_import, boxes, cylinders and spheres come from the
 *   built-in primitives.
 *
 * revolute and continuous joints move; fixed joints do not. prismatic,
 * floating and planar joints are shown fixed, and joints that <mimic>
 * another move on their own; both are reported. Limits are converted from
 * radians to the panel's degrees.
 *
 * COLLADA up axes are ignored unless asked for; see mesh_import.h for why.
 *
 * A mesh's own material colour is kept. A URDF <material> colour applies to
 * meshes without one, such as STL, and to the primitives.
 *
 * Mesh paths may be package://, file:// or relative to the URDF. A package
 * is found by walking up from the URDF's directory for a directory of that
 * name, then under ROS_PACKAGE_PATH, AMENT_PREFIX_PATH/share and
 * CONDA_PREFIX/share. That covers a ROS workspace, an installed package and
 * example-robot-data from pixi alike.
 */

#pragma once

#include "robot.h"

/**
 * @brief Replace @p robot with the robot described in @p path.
 *
 * On failure the file is reported on stderr and @p robot is left untouched.
 * Missing or unreadable mesh files do not fail the load; they are reported
 * and their visuals left out. Pre: a GL context is current, because the
 * robot being replaced may own GPU meshes.
 *
 * @param honor_up_axis Passed on to rbt_mesh_import for every mesh.
 * @return true when the robot was loaded.
 */
bool rbt_urdf_load(rbt_robot_t *robot, const char *path, bool honor_up_axis);
