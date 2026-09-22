/**
 * @file gltf_load.h
 * @brief Build a robot from a glTF 2.0 file (.gltf or .glb), through cgltf.
 *
 * glTF describes geometry and a tree of transformed nodes, but has no notion
 * of a joint that moves. The mapping is therefore:
 *
 * - every node becomes a joint, fixed unless its extras say otherwise, with
 *   the node's local transform as the joint's rest pose;
 * - every triangle primitive becomes a mesh, drawn as a visual in the frame
 *   of the node that references it. Nodes sharing a mesh share its GPU data.
 *
 * A node becomes a revolute joint through flat keys in its extras, which is
 * what Blender writes for custom properties:
 *
 *     "extras": {
 *         "rbt_joint":       "revolute",
 *         "rbt_axis":        [0, 0, 1],
 *         "rbt_min_deg":     -90,
 *         "rbt_max_deg":     120,
 *         "rbt_default_deg": 0
 *     }
 *
 * Only rbt_joint is required. The axis defaults to +Z and is normalised, the
 * limits default to +-180 degrees. Angles are in degrees, which the key names
 * spell out, because glTF itself uses radians everywhere else.
 *
 * Materials contribute their base colour factor only; textures, skins and
 * animations are ignored. Coordinates are taken as they are: glTF is Y-up in
 * metres, which is what this viewer uses too.
 */

#pragma once

#include "robot.h"

/**
 * @brief Replace @p robot with the model in @p path.
 *
 * On failure the file is reported on stderr and @p robot is left untouched: a
 * missing or malformed model is an operational failure, not a programmer
 * error. Pre: a GL context is current, because the robot being replaced may
 * own GPU meshes. The new meshes are not uploaded yet.
 *
 * @return true when the model was loaded.
 */
bool rbt_gltf_load(rbt_robot_t *robot, const char *path);
