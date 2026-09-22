/**
 * @file urdf_load.cpp
 * @brief URDF to joints and visuals, through tinyxml2 and assimp.
 */

#include "urdf_load.h"

#include "mesh_import.h"

#include <tinyxml2.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using tinyxml2::XMLElement;

/** Colour for geometry that has none of its own and names no URDF material. */
static const float s_default_color[3] = {0.70f, 0.70f, 0.72f};

/** A continuous joint, or a revolute one without limits, turns this far each way in the panel. */
static const float s_free_limit_deg = 180.0f;

/** Missing mesh files listed by name before the rest are only counted. */
static const int s_missing_listed = 5;

/* ============================================================
 * Small parsers
 * ============================================================ */

/** @brief Parse whitespace-separated floats; false unless exactly @p count are present. */
static bool s_parse_floats(const char *text, float *out, int count)
{
    if (text == NULL) {
        return false;
    }
    const char *cursor = text;
    for (int i = 0; i < count; i++) {
        char *end = NULL;
        out[i] = strtof(cursor, &end);
        if (end == cursor) {
            return false;
        }
        cursor = end;
    }
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
        cursor++;
    }
    return *cursor == '\0';
}

/**
 * @brief An <origin xyz rpy> element as a transform; identity when absent.
 *
 * rpy is roll, pitch, yaw about the fixed X, Y and Z axes, which composes as
 * Rz(yaw) * Ry(pitch) * Rx(roll).
 */
static Matrix4f s_origin(const XMLElement *parent)
{
    const XMLElement *origin = (parent != NULL) ? parent->FirstChildElement("origin") : NULL;
    if (origin == NULL) {
        return Matrix4f::Identity();
    }

    float xyz[3] = {0.0f, 0.0f, 0.0f};
    float rpy[3] = {0.0f, 0.0f, 0.0f};
    s_parse_floats(origin->Attribute("xyz"), xyz, 3);
    s_parse_floats(origin->Attribute("rpy"), rpy, 3);

    return rbt_translation(xyz[0], xyz[1], xyz[2])
         * rbt_rotation_z(rpy[2]) * rbt_rotation_y(rpy[1]) * rbt_rotation_x(rpy[0]);
}

/* ============================================================
 * Paths
 * ============================================================ */

static bool s_readable(const std::string &path)
{
    return !path.empty() && access(path.c_str(), R_OK) == 0;
}

static std::string s_dirname(const std::string &path)
{
    const size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) {
        return ".";
    }
    return (slash == 0) ? std::string("/") : path.substr(0, slash);
}

static std::string s_basename(const std::string &path)
{
    const size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? path : path.substr(slash + 1);
}

/** @brief Look for package @p package holding @p rest under every root in a colon-separated variable. */
static std::string s_search_env(const char *variable, const char *suffix, const std::string &package,
                                const std::string &rest)
{
    const char *value = getenv(variable);
    if (value == NULL) {
        return std::string();
    }

    std::string list(value);
    size_t start = 0;
    while (start <= list.size()) {
        size_t colon = list.find(':', start);
        if (colon == std::string::npos) {
            colon = list.size();
        }
        const std::string root = list.substr(start, colon - start) + suffix;
        if (!root.empty()) {
            if (s_basename(root) == package && s_readable(root + "/" + rest)) {
                return root + "/" + rest;
            }
            if (s_readable(root + "/" + package + "/" + rest)) {
                return root + "/" + package + "/" + rest;
            }
        }
        start = colon + 1;
    }
    return std::string();
}

/**
 * @brief Turn a URDF filename attribute into a readable path, or "" when none is found.
 */
static std::string s_resolve(const char *uri, const std::string &urdf_dir)
{
    const std::string text(uri);

    if (text.compare(0, 10, "package://") == 0) {
        const std::string tail  = text.substr(10);
        const size_t      slash = tail.find('/');
        if (slash == std::string::npos) {
            return std::string();
        }
        const std::string package = tail.substr(0, slash);
        const std::string rest    = tail.substr(slash + 1);

        /* Up from the URDF: a directory with the package's name, or one that
         * holds it, as in a workspace's src/. */
        for (std::string dir = urdf_dir; !dir.empty(); ) {
            if (s_basename(dir) == package && s_readable(dir + "/" + rest)) {
                return dir + "/" + rest;
            }
            if (s_readable(dir + "/" + package + "/" + rest)) {
                return dir + "/" + package + "/" + rest;
            }
            const std::string parent = s_dirname(dir);
            if (parent == dir) {
                break;
            }
            dir = parent;
        }

        std::string found = s_search_env("ROS_PACKAGE_PATH", "", package, rest);
        if (found.empty()) {
            found = s_search_env("AMENT_PREFIX_PATH", "/share", package, rest);
        }
        if (found.empty()) {
            found = s_search_env("CONDA_PREFIX", "/share", package, rest);
        }
        return found;
    }

    if (text.compare(0, 7, "file://") == 0) {
        const std::string local = text.substr(7);
        return s_readable(local) ? local : std::string();
    }

    const std::string local = (!text.empty() && text[0] == '/') ? text : urdf_dir + "/" + text;
    return s_readable(local) ? local : std::string();
}

/* ============================================================
 * Assembly
 * ============================================================ */

/** @brief Everything found along the way that deserves one line on stderr. */
typedef struct {
    int shown_fixed_prismatic;
    int shown_fixed_other;     /**< floating, planar or unknown types */
    int mimic;
    int revolute_without_limit;
    int zero_axis;
    int unsupported_geometry;
    int missing_meshes;
    int unreadable_meshes;
} rbt_urdf_report_t;

/** @brief Shared state while one file is being turned into a robot. */
typedef struct {
    rbt_robot_t                      *robot;
    std::string                       urdf_dir;
    std::unordered_map<std::string, int> materials;       /**< Top-level <material> name to index in colors. */
    std::vector<std::vector<float>>   material_colors;
    std::unordered_map<std::string, std::pair<int, int>> mesh_files;  /**< Resolved path to first mesh, count. */
    std::vector<rbt_mesh_color_t>     mesh_colors;        /**< Parallel to robot->meshes. */
    int                               box_mesh;
    int                               cylinder_mesh;
    int                               sphere_mesh;
    bool                              honor_up_axis;
    rbt_urdf_report_t                 report;
} rbt_urdf_build_t;

/** @brief The colour a <visual>'s <material> gives it, inline or by name. */
static bool s_material_color(const rbt_urdf_build_t *build, const XMLElement *visual, float out[3])
{
    const XMLElement *material = visual->FirstChildElement("material");
    if (material == NULL) {
        return false;
    }

    float rgba[4];
    const XMLElement *color = material->FirstChildElement("color");
    if (color != NULL && s_parse_floats(color->Attribute("rgba"), rgba, 4)) {
        out[0] = rgba[0];
        out[1] = rgba[1];
        out[2] = rgba[2];
        return true;
    }

    const char *name = material->Attribute("name");
    if (name != NULL) {
        const auto found = build->materials.find(name);
        if (found != build->materials.end()) {
            const std::vector<float> &c = build->material_colors[(size_t)found->second];
            out[0] = c[0];
            out[1] = c[1];
            out[2] = c[2];
            return true;
        }
    }
    return false;
}

/** @brief Index of a built-in primitive, generated the first time it is needed. */
static int s_primitive(rbt_urdf_build_t *build, int *slot, rbt_mesh_t (*make)(void))
{
    if (*slot < 0) {
        *slot = (int)build->robot->meshes.size();
        build->robot->meshes.push_back(make());
        build->mesh_colors.push_back(rbt_mesh_color_t{{0.0f, 0.0f, 0.0f}, false});
    }
    return *slot;
}

static rbt_mesh_t s_make_box(void)      { return rbt_mesh_make_box(1.0f, 1.0f, 1.0f); }
static rbt_mesh_t s_make_cylinder(void) { return rbt_mesh_make_cylinder(1.0f, 1.0f, 1.0f, 24); }
static rbt_mesh_t s_make_sphere(void)   { return rbt_mesh_make_sphere(1.0f, 16, 24); }

static void s_add_visual(rbt_robot_t *robot, int frame, int mesh, const Matrix4f &local, const float color[3])
{
    rbt_visual_t visual;
    visual.joint    = frame;
    visual.mesh     = mesh;
    visual.local    = local;
    visual.color[0] = color[0];
    visual.color[1] = color[1];
    visual.color[2] = color[2];
    robot->visuals.push_back(visual);
}

/** @brief Turn one <visual> into visuals on @p frame. */
static void s_add_link_visual(rbt_urdf_build_t *build, const XMLElement *visual, int frame)
{
    const XMLElement *geometry = visual->FirstChildElement("geometry");
    const XMLElement *shape    = (geometry != NULL) ? geometry->FirstChildElement() : NULL;
    if (shape == NULL) {
        build->report.unsupported_geometry++;
        return;
    }

    const Matrix4f origin = s_origin(visual);
    float urdf_color[3];
    const bool has_urdf_color = s_material_color(build, visual, urdf_color);
    const float *fallback = has_urdf_color ? urdf_color : s_default_color;

    const char *kind = shape->Name();
    float size[3];

    if (strcmp(kind, "mesh") == 0) {
        const char *filename = shape->Attribute("filename");
        if (filename == NULL) {
            build->report.unsupported_geometry++;
            return;
        }

        const std::string path = s_resolve(filename, build->urdf_dir);
        if (path.empty()) {
            if (build->report.missing_meshes < s_missing_listed) {
                fprintf(stderr, "model: mesh not found: %s\n", filename);
            }
            build->report.missing_meshes++;
            return;
        }

        auto cached = build->mesh_files.find(path);
        if (cached == build->mesh_files.end()) {
            const int    first = (int)build->robot->meshes.size();
            const size_t count = rbt_mesh_import(path.c_str(), build->honor_up_axis,
                                                 &build->robot->meshes, &build->mesh_colors);
            if (count == 0) {
                build->report.unreadable_meshes++;
            }
            cached = build->mesh_files.emplace(path, std::make_pair(first, (int)count)).first;
        }

        float scale[3] = {1.0f, 1.0f, 1.0f};
        s_parse_floats(shape->Attribute("scale"), scale, 3);
        const Matrix4f local = origin * rbt_scale(scale[0], scale[1], scale[2]);

        for (int i = 0; i < cached->second.second; i++) {
            const int mesh = cached->second.first + i;
            const rbt_mesh_color_t &own = build->mesh_colors[(size_t)mesh];
            s_add_visual(build->robot, frame, mesh, local, own.present ? own.rgb : fallback);
        }
    } else if (strcmp(kind, "box") == 0 && s_parse_floats(shape->Attribute("size"), size, 3)) {
        s_add_visual(build->robot, frame, s_primitive(build, &build->box_mesh, s_make_box),
                     origin * rbt_scale(size[0], size[1], size[2]), fallback);
    } else if (strcmp(kind, "cylinder") == 0
               && s_parse_floats(shape->Attribute("radius"), &size[0], 1)
               && s_parse_floats(shape->Attribute("length"), &size[1], 1)) {
        /* URDF cylinders run along Z, centred on the origin; the unit
         * cylinder here runs along +Y from 0 to 1. Centre it, then turn +Y
         * onto +Z. */
        const float radius = size[0];
        const float length = size[1];
        s_add_visual(build->robot, frame, s_primitive(build, &build->cylinder_mesh, s_make_cylinder),
                     origin * rbt_rotation_x(0.5f * RBT_PI)
                            * rbt_translation(0.0f, -0.5f * length, 0.0f)
                            * rbt_scale(radius, length, radius),
                     fallback);
    } else if (strcmp(kind, "sphere") == 0 && s_parse_floats(shape->Attribute("radius"), &size[0], 1)) {
        s_add_visual(build->robot, frame, s_primitive(build, &build->sphere_mesh, s_make_sphere),
                     origin * rbt_scale(size[0], size[0], size[0]), fallback);
    } else {
        build->report.unsupported_geometry++;
    }
}

/** @brief Fill a frame from the joint that leads into it, or as a root when @p joint is NULL. */
static void s_frame_from_joint(rbt_urdf_build_t *build, rbt_joint_t *frame, const XMLElement *joint,
                               const char *link_name, int parent)
{
    rbt_name_copy(frame->name, (joint != NULL && joint->Attribute("name") != NULL) ? joint->Attribute("name")
                                                                                   : link_name);
    frame->type              = RBT_JOINT_FIXED;
    frame->parent            = parent;
    frame->child_count       = 0;
    frame->axis              = Vector3f::UnitX();
    frame->min_angle_deg     = -s_free_limit_deg;
    frame->max_angle_deg     = s_free_limit_deg;
    frame->default_angle_deg = 0.0f;
    frame->angle_deg         = 0.0f;
    frame->color[0]          = s_default_color[0];
    frame->color[1]          = s_default_color[1];
    frame->color[2]          = s_default_color[2];
    frame->world_transform   = Matrix4f::Identity();

    if (joint == NULL) {
        /* The root: ROS is Z-up, this viewer Y-up. A -90 degree turn about X
         * takes +Z onto +Y and stands the robot upright. */
        frame->origin = rbt_rotation_x(-0.5f * RBT_PI);
        return;
    }
    frame->origin = s_origin(joint);

    const char *type = joint->Attribute("type");
    type = (type != NULL) ? type : "";
    if (joint->FirstChildElement("mimic") != NULL) {
        build->report.mimic++;
    }

    if (strcmp(type, "revolute") == 0 || strcmp(type, "continuous") == 0) {
        frame->type = RBT_JOINT_REVOLUTE;

        float axis[3];
        const XMLElement *axis_element = joint->FirstChildElement("axis");
        if (axis_element != NULL && s_parse_floats(axis_element->Attribute("xyz"), axis, 3)) {
            const Vector3f v(axis[0], axis[1], axis[2]);
            if (v.norm() > 1e-6f) {
                frame->axis = v.normalized();
            } else {
                build->report.zero_axis++;
            }
        }

        const XMLElement *limit = joint->FirstChildElement("limit");
        float lower = 0.0f;
        float upper = 0.0f;
        if (strcmp(type, "revolute") == 0) {
            if (limit != NULL && limit->QueryFloatAttribute("lower", &lower) == tinyxml2::XML_SUCCESS
                && limit->QueryFloatAttribute("upper", &upper) == tinyxml2::XML_SUCCESS) {
                if (lower > upper) {
                    std::swap(lower, upper);
                }
                frame->min_angle_deg = lower * (180.0f / RBT_PI);
                frame->max_angle_deg = upper * (180.0f / RBT_PI);
            } else {
                build->report.revolute_without_limit++;
            }
        }
        /* Rest at zero where the limits allow it, else at the nearer limit. */
        frame->default_angle_deg = fminf(fmaxf(0.0f, frame->min_angle_deg), frame->max_angle_deg);
        frame->angle_deg         = frame->default_angle_deg;
    } else if (strcmp(type, "prismatic") == 0) {
        build->report.shown_fixed_prismatic++;
    } else if (strcmp(type, "fixed") != 0) {
        build->report.shown_fixed_other++;
    }
}

static void s_print_report(const char *path, const rbt_urdf_report_t *r)
{
    if (r->shown_fixed_prismatic > 0) {
        fprintf(stderr, "model: %s: %d prismatic joint(s) shown fixed\n", path, r->shown_fixed_prismatic);
    }
    if (r->shown_fixed_other > 0) {
        fprintf(stderr, "model: %s: %d floating, planar or unknown joint(s) shown fixed\n", path, r->shown_fixed_other);
    }
    if (r->mimic > 0) {
        fprintf(stderr, "model: %s: %d mimic joint(s) move on their own here\n", path, r->mimic);
    }
    if (r->revolute_without_limit > 0) {
        fprintf(stderr, "model: %s: %d revolute joint(s) without limits, given +-180 degrees\n", path,
                r->revolute_without_limit);
    }
    if (r->zero_axis > 0) {
        fprintf(stderr, "model: %s: %d joint(s) with a zero axis, using +X\n", path, r->zero_axis);
    }
    if (r->unsupported_geometry > 0) {
        fprintf(stderr, "model: %s: %d visual(s) with geometry that is not mesh, box, cylinder or sphere\n", path,
                r->unsupported_geometry);
    }
    if (r->missing_meshes > s_missing_listed) {
        fprintf(stderr, "model: %s: %d more mesh(es) not found\n", path, r->missing_meshes - s_missing_listed);
    }
    if (r->unreadable_meshes > 0) {
        fprintf(stderr, "model: %s: %d mesh file(s) could not be read\n", path, r->unreadable_meshes);
    }
}

static bool s_build_robot(rbt_robot_t *robot, const XMLElement *root, const char *path, bool honor_up_axis)
{
    rbt_urdf_build_t build;
    build.robot         = robot;
    build.honor_up_axis = honor_up_axis;
    build.urdf_dir      = s_dirname(path);
    build.box_mesh      = -1;
    build.cylinder_mesh = -1;
    build.sphere_mesh   = -1;
    memset(&build.report, 0, sizeof(build.report));

    for (const XMLElement *m = root->FirstChildElement("material"); m != NULL; m = m->NextSiblingElement("material")) {
        float rgba[4];
        const XMLElement *color = m->FirstChildElement("color");
        if (m->Attribute("name") != NULL && color != NULL && s_parse_floats(color->Attribute("rgba"), rgba, 4)) {
            build.materials[m->Attribute("name")] = (int)build.material_colors.size();
            build.material_colors.push_back({rgba[0], rgba[1], rgba[2]});
        }
    }

    /* Links by name, and for each link the joints that leave it. */
    std::vector<const XMLElement *>      links;
    std::unordered_map<std::string, int> link_of_name;
    for (const XMLElement *l = root->FirstChildElement("link"); l != NULL; l = l->NextSiblingElement("link")) {
        const char *name = l->Attribute("name");
        if (name != NULL && link_of_name.emplace(name, (int)links.size()).second) {
            links.push_back(l);
        }
    }
    if (links.empty()) {
        fprintf(stderr, "model: %s: no links\n", path);
        return false;
    }

    std::vector<std::vector<const XMLElement *>> leaving(links.size());
    std::vector<const XMLElement *>              entering(links.size(), NULL);
    for (const XMLElement *j = root->FirstChildElement("joint"); j != NULL; j = j->NextSiblingElement("joint")) {
        const XMLElement *parent = j->FirstChildElement("parent");
        const XMLElement *child  = j->FirstChildElement("child");
        const char *p = (parent != NULL) ? parent->Attribute("link") : NULL;
        const char *c = (child != NULL) ? child->Attribute("link") : NULL;
        const auto pi = (p != NULL) ? link_of_name.find(p) : link_of_name.end();
        const auto ci = (c != NULL) ? link_of_name.find(c) : link_of_name.end();
        if (pi == link_of_name.end() || ci == link_of_name.end()) {
            fprintf(stderr, "model: %s: joint '%s' names a link that does not exist, ignoring it\n", path,
                    j->Attribute("name") ? j->Attribute("name") : "?");
            continue;
        }
        if (entering[(size_t)ci->second] != NULL) {
            fprintf(stderr, "model: %s: link '%s' has two parents, keeping the first\n", path, c);
            continue;
        }
        entering[(size_t)ci->second] = j;
        leaving[(size_t)pi->second].push_back(j);
    }

    /* Depth first from every root, parents before children, which is the
     * order the joint array needs. */
    std::vector<int>                     frame_of_link(links.size(), -1);
    std::vector<std::pair<int, int>>     stack;  /* link, parent frame */
    for (size_t l = links.size(); l > 0; l--) {
        if (entering[l - 1] == NULL) {
            stack.push_back(std::make_pair((int)(l - 1), RBT_NO_PARENT));
        }
    }
    if (stack.empty()) {
        fprintf(stderr, "model: %s: every link has a parent, so there is no root; the joints form a loop\n", path);
        return false;
    }
    if (stack.size() > 1) {
        fprintf(stderr, "model: %s: %zu links have no parent; showing each as its own root\n", path, stack.size());
    }

    while (!stack.empty()) {
        const int link   = stack.back().first;
        const int parent = stack.back().second;
        stack.pop_back();
        if (frame_of_link[(size_t)link] != -1) {
            continue;
        }

        const int frame = (int)robot->joints.size();
        frame_of_link[(size_t)link] = frame;
        robot->joints.emplace_back();
        s_frame_from_joint(&build, &robot->joints.back(), (parent == RBT_NO_PARENT) ? NULL : entering[(size_t)link],
                           links[(size_t)link]->Attribute("name"), parent);

        const size_t visuals_before = robot->visuals.size();
        for (const XMLElement *v = links[(size_t)link]->FirstChildElement("visual"); v != NULL;
             v = v->NextSiblingElement("visual")) {
            s_add_link_visual(&build, v, frame);
        }
        if (robot->visuals.size() > visuals_before) {
            const float *c = robot->visuals[visuals_before].color;
            rbt_joint_t &j = robot->joints[(size_t)frame];
            j.color[0] = c[0];
            j.color[1] = c[1];
            j.color[2] = c[2];
        }

        const std::vector<const XMLElement *> &out = leaving[(size_t)link];
        for (size_t i = out.size(); i > 0; i--) {
            const char *child = out[i - 1]->FirstChildElement("child")->Attribute("link");
            stack.push_back(std::make_pair(link_of_name.find(child)->second, frame));
        }
    }

    s_print_report(path, &build.report);
    if (robot->visuals.empty()) {
        fprintf(stderr, "model: %s: nothing drawable in the file\n", path);
        return false;
    }

    const char *name = root->Attribute("name");
    rbt_name_copy(robot->name, (name != NULL && name[0] != '\0') ? name : s_basename(path).c_str());
    rbt_robot_finalize(robot);
    return true;
}

bool rbt_urdf_load(rbt_robot_t *robot, const char *path, bool honor_up_axis)
{
    assert(robot != NULL);
    assert(path != NULL);

    tinyxml2::XMLDocument document;
    if (document.LoadFile(path) != tinyxml2::XML_SUCCESS) {
        fprintf(stderr, "model: %s: %s\n", path, document.ErrorStr());
        return false;
    }
    const XMLElement *root = document.FirstChildElement("robot");
    if (root == NULL) {
        fprintf(stderr, "model: %s: no <robot> element, so not a URDF\n", path);
        return false;
    }

    /* Built aside and swapped in whole, so a failure leaves the current robot
     * as it was. */
    rbt_robot_t loaded;
    if (!s_build_robot(&loaded, root, path, honor_up_axis)) {
        return false;
    }
    *robot = std::move(loaded);
    return true;
}
