/**
 * @file gltf_load.cpp
 * @brief glTF 2.0 import: node tree to joints, primitives to meshes.
 */

/* cgltf is a single-header library; exactly one translation unit carries its
 * implementation, and this is it. */
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#include "gltf_load.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <utility>
#include <vector>

/** Colour for geometry that names no material, and for joints that carry none. */
static const float s_default_color[3] = {0.80f, 0.80f, 0.80f};

/** A revolute joint without limits in its extras may turn this far each way. */
static const float s_default_limit_deg = 180.0f;

/** @brief Human-readable name for a cgltf result, for error messages. */
static const char *s_result_name(cgltf_result result)
{
    switch (result) {
    case cgltf_result_success:         return "success";
    case cgltf_result_data_too_short:  return "file is truncated";
    case cgltf_result_unknown_format:  return "not a glTF file";
    case cgltf_result_invalid_json:    return "invalid JSON";
    case cgltf_result_invalid_gltf:    return "invalid glTF";
    case cgltf_result_invalid_options: return "invalid options";
    case cgltf_result_file_not_found:  return "file not found";
    case cgltf_result_io_error:        return "I/O error";
    case cgltf_result_out_of_memory:   return "out of memory";
    case cgltf_result_legacy_gltf:     return "glTF 1.0, only 2.0 is supported";
    default:                           return "unknown error";
    }
}

/* ============================================================
 * Extras
 *
 * cgltf hands extras over as the raw JSON text of the object. The schema is
 * flat on purpose, so a small scanner does: it looks for a key followed by a
 * colon, which a string value with the same text can never be.
 * ============================================================ */

static bool s_is_space(char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

/** @brief Text just after "key": in a flat JSON object, or NULL when absent. */
static const char *s_json_value(const char *json, const char *key)
{
    assert(json != NULL);
    assert(key != NULL);

    const size_t key_len = strlen(key);
    for (const char *quote = strchr(json, '"'); quote != NULL; quote = strchr(quote + 1, '"')) {
        if (strncmp(quote + 1, key, key_len) != 0 || quote[1 + key_len] != '"') {
            continue;
        }
        const char *cursor = quote + 2 + key_len;
        while (s_is_space(*cursor)) {
            cursor++;
        }
        if (*cursor != ':') {
            continue;
        }
        cursor++;
        while (s_is_space(*cursor)) {
            cursor++;
        }
        return cursor;
    }
    return NULL;
}

/** @brief Read a number; leaves @p out alone when the key is absent or not a number. */
static bool s_json_float(const char *json, const char *key, float *out)
{
    const char *value = s_json_value(json, key);
    if (value == NULL) {
        return false;
    }
    char *end = NULL;
    const float number = strtof(value, &end);
    if (end == value) {
        return false;
    }
    *out = number;
    return true;
}

/** @brief Read a three-element number array. */
static bool s_json_vec3(const char *json, const char *key, Vector3f *out)
{
    const char *cursor = s_json_value(json, key);
    if (cursor == NULL || *cursor != '[') {
        return false;
    }
    cursor++;

    float component[3];
    for (int i = 0; i < 3; i++) {
        while (s_is_space(*cursor)) {
            cursor++;
        }
        char *end = NULL;
        component[i] = strtof(cursor, &end);
        if (end == cursor) {
            return false;
        }
        cursor = end;
        while (s_is_space(*cursor)) {
            cursor++;
        }
        if (*cursor != ((i < 2) ? ',' : ']')) {
            return false;
        }
        cursor++;
    }
    *out = Vector3f(component[0], component[1], component[2]);
    return true;
}

/** @brief True when the key holds exactly the string @p expected. */
static bool s_json_string_is(const char *json, const char *key, const char *expected)
{
    const char *value = s_json_value(json, key);
    if (value == NULL || *value != '"') {
        return false;
    }
    const size_t length = strlen(expected);
    return strncmp(value + 1, expected, length) == 0 && value[1 + length] == '"';
}

/* ============================================================
 * Nodes to joints
 * ============================================================ */

/**
 * @brief Turn one node into a joint.
 *
 * Malformed articulation data is repaired and reported rather than rejected:
 * a model with a bad limit is still worth looking at.
 */
static void s_joint_from_node(rbt_joint_t *joint, const cgltf_node *node, size_t node_index, int parent)
{
    assert(joint != NULL);
    assert(node != NULL);

    if (node->name != NULL && node->name[0] != '\0') {
        rbt_name_copy(joint->name, node->name);
    } else {
        snprintf(joint->name, RBT_NAME_SIZE, "node %zu", node_index);
    }

    /* cgltf composes translation, rotation and scale, or takes the matrix,
     * into column-major floats: Eigen's own layout. */
    float local[16];
    cgltf_node_transform_local(node, local);

    joint->type              = RBT_JOINT_FIXED;
    joint->parent            = parent;
    joint->child_count       = 0;
    joint->origin            = Eigen::Map<const Matrix4f>(local);
    joint->axis              = Vector3f::UnitZ();
    joint->min_angle_deg     = -s_default_limit_deg;
    joint->max_angle_deg     = s_default_limit_deg;
    joint->default_angle_deg = 0.0f;
    joint->color[0]          = s_default_color[0];
    joint->color[1]          = s_default_color[1];
    joint->color[2]          = s_default_color[2];
    joint->world_transform   = Matrix4f::Identity();

    const char *extras = node->extras.data;
    if (extras != NULL && s_json_value(extras, "rbt_joint") != NULL
        && !s_json_string_is(extras, "rbt_joint", "revolute")
        && !s_json_string_is(extras, "rbt_joint", "fixed")) {
        fprintf(stderr, "model: joint '%s': rbt_joint must be \"revolute\" or \"fixed\", treating it as fixed\n",
                joint->name);
    }
    if (extras != NULL && s_json_string_is(extras, "rbt_joint", "revolute")) {
        joint->type = RBT_JOINT_REVOLUTE;

        Vector3f axis;
        if (s_json_vec3(extras, "rbt_axis", &axis) && axis.norm() > 1e-6f) {
            joint->axis = axis.normalized();
        } else if (s_json_value(extras, "rbt_axis") != NULL) {
            fprintf(stderr, "model: joint '%s': rbt_axis is not a usable vector, using +Z\n", joint->name);
        }

        s_json_float(extras, "rbt_min_deg", &joint->min_angle_deg);
        s_json_float(extras, "rbt_max_deg", &joint->max_angle_deg);
        s_json_float(extras, "rbt_default_deg", &joint->default_angle_deg);

        if (joint->min_angle_deg > joint->max_angle_deg) {
            fprintf(stderr, "model: joint '%s': limits are the wrong way round, swapping them\n", joint->name);
            std::swap(joint->min_angle_deg, joint->max_angle_deg);
        }
        if (joint->default_angle_deg < joint->min_angle_deg) {
            joint->default_angle_deg = joint->min_angle_deg;
        }
        if (joint->default_angle_deg > joint->max_angle_deg) {
            joint->default_angle_deg = joint->max_angle_deg;
        }
    }
    joint->angle_deg = joint->default_angle_deg;
}

/* ============================================================
 * Primitives to meshes
 * ============================================================ */

/** @brief Why a primitive was left out; counted so each reason is reported once. */
typedef enum {
    RBT_SKIP_NOT_TRIANGLES = 0,
    RBT_SKIP_COMPRESSED,
    RBT_SKIP_NO_POSITIONS,
    RBT_SKIP_BAD_INDICES,
    RBT_SKIP_COUNT,
} rbt_skip_reason_t;

static const char *s_skip_text[RBT_SKIP_COUNT] = {
    "not a triangle list (points, lines, strips and fans are not drawn)",
    "compressed geometry (Draco or meshopt is not supported)",
    "no usable POSITION attribute",
    "indices out of range or not a multiple of three",
};

/**
 * @brief Smooth normals for a primitive that brought none.
 *
 * Area-weighted: each face adds its unnormalised cross product to its three
 * vertices, so large faces count for more, as they should.
 */
static void s_compute_normals(rbt_mesh_t *mesh)
{
    for (rbt_vertex_t &vertex : mesh->vertices) {
        vertex.normal = Vector3f::Zero();
    }

    const bool   indexed = !mesh->indices.empty();
    const size_t corners = indexed ? mesh->indices.size() : mesh->vertices.size();
    for (size_t i = 0; i + 2 < corners; i += 3) {
        const size_t a = indexed ? mesh->indices[i + 0] : i + 0;
        const size_t b = indexed ? mesh->indices[i + 1] : i + 1;
        const size_t c = indexed ? mesh->indices[i + 2] : i + 2;

        const Vector3f face = (mesh->vertices[b].position - mesh->vertices[a].position)
                              .cross(mesh->vertices[c].position - mesh->vertices[a].position);
        mesh->vertices[a].normal += face;
        mesh->vertices[b].normal += face;
        mesh->vertices[c].normal += face;
    }

    for (rbt_vertex_t &vertex : mesh->vertices) {
        const float length = vertex.normal.norm();
        vertex.normal = (length > 1e-12f) ? Vector3f(vertex.normal / length) : Vector3f::UnitY();
    }
}

/**
 * @brief Convert one primitive into a staged, not yet uploaded, mesh.
 * @return false with @p reason set when the primitive cannot be drawn.
 */
static bool s_mesh_from_primitive(rbt_mesh_t *mesh, const cgltf_primitive *primitive, rbt_skip_reason_t *reason)
{
    assert(mesh != NULL);
    assert(primitive != NULL);
    assert(reason != NULL);

    if (primitive->type != cgltf_primitive_type_triangles) {
        *reason = RBT_SKIP_NOT_TRIANGLES;
        return false;
    }
    if (primitive->has_draco_mesh_compression) {
        *reason = RBT_SKIP_COMPRESSED;
        return false;
    }

    const cgltf_accessor *positions = cgltf_find_accessor(primitive, cgltf_attribute_type_position, 0);
    if (positions == NULL || positions->count == 0 || positions->type != cgltf_type_vec3
        || (positions->buffer_view == NULL && !positions->is_sparse)) {
        *reason = RBT_SKIP_NO_POSITIONS;
        return false;
    }
    const size_t vertex_count = positions->count;

    std::vector<float> xyz(vertex_count * 3);
    cgltf_accessor_unpack_floats(positions, xyz.data(), xyz.size());

    std::vector<float> normals;
    const cgltf_accessor *normal_accessor = cgltf_find_accessor(primitive, cgltf_attribute_type_normal, 0);
    if (normal_accessor != NULL && normal_accessor->count == vertex_count
        && normal_accessor->type == cgltf_type_vec3 && normal_accessor->buffer_view != NULL) {
        normals.resize(vertex_count * 3);
        cgltf_accessor_unpack_floats(normal_accessor, normals.data(), normals.size());
    }

    /* Without indices the vertices already form a triangle list, and the mesh
     * stays unindexed rather than carrying a 0, 1, 2, ... index buffer. */
    mesh->indices.clear();
    if (primitive->indices != NULL) {
        const size_t index_count = primitive->indices->count;
        if (index_count % 3 != 0) {
            *reason = RBT_SKIP_BAD_INDICES;
            return false;
        }
        mesh->indices.resize(index_count);
        for (size_t i = 0; i < index_count; i++) {
            const size_t index = cgltf_accessor_read_index(primitive->indices, i);
            if (index >= vertex_count) {
                *reason = RBT_SKIP_BAD_INDICES;
                return false;
            }
            mesh->indices[i] = (uint32_t)index;
        }
    } else if (vertex_count % 3 != 0) {
        *reason = RBT_SKIP_BAD_INDICES;
        return false;
    }

    mesh->vertices.resize(vertex_count);
    for (size_t v = 0; v < vertex_count; v++) {
        mesh->vertices[v].position = Vector3f(xyz[v * 3 + 0], xyz[v * 3 + 1], xyz[v * 3 + 2]);
        mesh->vertices[v].normal   = normals.empty()
                                   ? Vector3f::Zero()
                                   : Vector3f(normals[v * 3 + 0], normals[v * 3 + 1], normals[v * 3 + 2]);
    }
    if (normals.empty()) {
        s_compute_normals(mesh);
    }
    return true;
}

/**
 * @brief The colour a primitive is drawn in.
 *
 * glTF colour factors are linear. This renderer writes its colours to the
 * screen as they are, without a gamma stage, so the factor is encoded to sRGB
 * here; otherwise every loaded model would come out darker than its author
 * saw it.
 */
static void s_primitive_color(const cgltf_primitive *primitive, float out[3])
{
    if (primitive->material == NULL || !primitive->material->has_pbr_metallic_roughness) {
        out[0] = s_default_color[0];
        out[1] = s_default_color[1];
        out[2] = s_default_color[2];
        return;
    }

    const float *linear = primitive->material->pbr_metallic_roughness.base_color_factor;
    for (int i = 0; i < 3; i++) {
        const float c = linear[i] < 0.0f ? 0.0f : (linear[i] > 1.0f ? 1.0f : linear[i]);
        out[i] = (c <= 0.0031308f) ? c * 12.92f : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
    }
}

/* ============================================================
 * Assembly
 * ============================================================ */

/** @brief One loaded primitive of a glTF mesh, ready to be referenced by a visual. */
typedef struct {
    int   mesh;       /**< Index into rbt_robot_t::meshes. */
    float color[3];
} rbt_loaded_primitive_t;

/** @brief File name without its directories, for the model's display name. */
static const char *s_basename(const char *path)
{
    const char *slash = strrchr(path, '/');
    return (slash != NULL) ? slash + 1 : path;
}

/**
 * @brief Walk the node tree depth first and fill @p robot.
 *
 * Depth first, parent before children, is exactly the order the joint array
 * requires, so no sorting pass is needed.
 */
static bool s_build_robot(rbt_robot_t *robot, const cgltf_data *data, const char *path)
{
    const cgltf_scene *scene = (data->scene != NULL) ? data->scene
                             : (data->scenes_count > 0) ? &data->scenes[0]
                             : NULL;

    std::vector<const cgltf_node *> stack;
    if (scene != NULL) {
        for (size_t i = scene->nodes_count; i > 0; i--) {
            stack.push_back(scene->nodes[i - 1]);
        }
    } else {
        /* No scene: every node without a parent is a root. */
        for (size_t i = data->nodes_count; i > 0; i--) {
            if (data->nodes[i - 1].parent == NULL) {
                stack.push_back(&data->nodes[i - 1]);
            }
        }
    }
    if (stack.empty()) {
        fprintf(stderr, "model: %s: the file has no nodes to show\n", path);
        return false;
    }

    std::vector<int>  joint_of_node(data->nodes_count, -1);
    std::vector<bool> mesh_seen(data->meshes_count, false);
    std::vector<std::vector<rbt_loaded_primitive_t>> primitives_of_mesh(data->meshes_count);
    size_t skipped[RBT_SKIP_COUNT] = {0};
    bool   has_skin = false;

    while (!stack.empty()) {
        const cgltf_node *node = stack.back();
        stack.pop_back();

        const size_t node_index = (size_t)(node - data->nodes);
        if (joint_of_node[node_index] != -1) {
            continue;  /* listed twice: a malformed file, keep the first */
        }

        /* A parent that is not placed yet can only mean the scene lists a
         * non-root node; treat it as a root so the order still holds. */
        int parent = RBT_NO_PARENT;
        if (node->parent != NULL) {
            parent = joint_of_node[(size_t)(node->parent - data->nodes)];
            if (parent < 0) {
                parent = RBT_NO_PARENT;
            }
        }

        const int joint_index = (int)robot->joints.size();
        joint_of_node[node_index] = joint_index;
        robot->joints.emplace_back();
        s_joint_from_node(&robot->joints.back(), node, node_index, parent);
        has_skin = has_skin || (node->skin != NULL);

        if (node->mesh != NULL) {
            const size_t mesh_index = (size_t)(node->mesh - data->meshes);
            if (!mesh_seen[mesh_index]) {
                mesh_seen[mesh_index] = true;
                for (size_t p = 0; p < node->mesh->primitives_count; p++) {
                    const cgltf_primitive *primitive = &node->mesh->primitives[p];

                    rbt_mesh_t        mesh;
                    rbt_skip_reason_t reason = RBT_SKIP_NOT_TRIANGLES;
                    if (!s_mesh_from_primitive(&mesh, primitive, &reason)) {
                        skipped[reason]++;
                        continue;
                    }

                    rbt_loaded_primitive_t loaded;
                    loaded.mesh = (int)robot->meshes.size();
                    s_primitive_color(primitive, loaded.color);
                    robot->meshes.push_back(std::move(mesh));
                    primitives_of_mesh[mesh_index].push_back(loaded);
                }
            }

            rbt_joint_t &joint = robot->joints[(size_t)joint_index];
            for (const rbt_loaded_primitive_t &loaded : primitives_of_mesh[mesh_index]) {
                rbt_visual_t visual;
                visual.joint    = joint_index;
                visual.mesh     = loaded.mesh;
                visual.local    = Matrix4f::Identity();
                visual.color[0] = loaded.color[0];
                visual.color[1] = loaded.color[1];
                visual.color[2] = loaded.color[2];
                robot->visuals.push_back(visual);
            }
            /* The panel's colour key for this joint: what it carries. */
            if (!primitives_of_mesh[mesh_index].empty()) {
                const float *color = primitives_of_mesh[mesh_index][0].color;
                joint.color[0] = color[0];
                joint.color[1] = color[1];
                joint.color[2] = color[2];
            }
        }

        for (size_t c = node->children_count; c > 0; c--) {
            stack.push_back(node->children[c - 1]);
        }
    }

    for (int reason = 0; reason < RBT_SKIP_COUNT; reason++) {
        if (skipped[reason] > 0) {
            fprintf(stderr, "model: %s: skipped %zu primitive(s): %s\n", path, skipped[reason], s_skip_text[reason]);
        }
    }
    if (has_skin) {
        fprintf(stderr, "model: %s: skinned meshes are shown in their bind pose\n", path);
    }
    if (robot->visuals.empty()) {
        fprintf(stderr, "model: %s: nothing drawable in the file\n", path);
        return false;
    }

    rbt_name_copy(robot->name, s_basename(path));
    rbt_robot_finalize(robot);
    return true;
}

bool rbt_gltf_load(rbt_robot_t *robot, const char *path)
{
    assert(robot != NULL);
    assert(path != NULL);

    cgltf_options options;
    memset(&options, 0, sizeof(options));

    cgltf_data  *data   = NULL;
    cgltf_result result = cgltf_parse_file(&options, path, &data);
    if (result != cgltf_result_success) {
        fprintf(stderr, "model: %s: %s\n", path, s_result_name(result));
        return false;
    }

    result = cgltf_load_buffers(&options, data, path);
    if (result == cgltf_result_success) {
        result = cgltf_validate(data);
    }

    bool ok = false;
    if (result != cgltf_result_success) {
        fprintf(stderr, "model: %s: %s\n", path, s_result_name(result));
    } else {
        /* Built aside and swapped in whole, so a failure leaves the current
         * robot as it was. */
        rbt_robot_t loaded;
        ok = s_build_robot(&loaded, data, path);
        if (ok) {
            *robot = std::move(loaded);
        }
    }

    cgltf_free(data);
    return ok;
}
