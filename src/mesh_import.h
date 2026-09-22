/**
 * @file mesh_import.h
 * @brief Read the mesh files robot descriptions point at, through assimp.
 *
 * URDFs name their geometry as COLLADA, STL or OBJ files. assimp reads all
 * of them into one shape, and this module turns that into rbt_mesh_t, the
 * same staged meshes the rest of the viewer draws.
 *
 * A COLLADA file declares an up axis, and robot meshes use that declaration
 * inconsistently. By default it is ignored and the vertices are taken as
 * link-frame coordinates as written, which is what RViz does; the file's unit
 * scale is always honoured. Of the six robots in example-robot-data that draw
 * Y_UP meshes, five assemble only this way: Baxter, Go1, asr_twodof, Borinot
 * and Hextilt. The sixth, iCub, was authored for tools that convert the
 * declared axis to Z-up, and falls apart unless that conversion is asked for.
 * STL and OBJ declare nothing and are always taken as written.
 */

#pragma once

#include "gl_mesh.h"

#include <vector>

/** @brief The colour a mesh's material gives it, if it gives one at all. */
typedef struct {
    float rgb[3];
    bool  present;  /**< False for files without materials, such as STL. */
} rbt_mesh_color_t;

/**
 * @brief Append every triangle mesh in a file to @p meshes, staged, not uploaded.
 *
 * Node transforms inside the file are baked into the vertices, so each mesh
 * is ready to be drawn in the frame that references the file. Points and
 * lines are dropped. A file that cannot be read is reported on stderr and
 * appends nothing: an operational failure, not a programmer error.
 *
 * @param honor_up_axis Turn a COLLADA file from its declared up axis to Z-up,
 *                      as Gazebo does, instead of taking it as written.
 * @param colors        Receives one entry per appended mesh.
 * @return The number of meshes appended; 0 on failure.
 */
size_t rbt_mesh_import(const char *path, bool honor_up_axis,
                       std::vector<rbt_mesh_t> *meshes, std::vector<rbt_mesh_color_t> *colors);
