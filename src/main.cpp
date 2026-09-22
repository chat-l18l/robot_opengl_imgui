/**
 * @file main.cpp
 * @brief Dear ImGui + OpenGL 3.3 viewer for a 6-DOF robot arm.
 *
 * The panels live in an ImGui dock space, so they can be resized, tabbed or
 * torn loose and the arrangement survives a restart.
 *
 * The 3D scene renders into its own framebuffer and reaches the panel as a
 * texture, so it is an ordinary entry in ImGui's draw list and composites in
 * draw order like any other widget. That is what keeps it visible when the
 * panel floats above another one or above the dock space's empty central node,
 * both of which paint over the default framebuffer after the scene would have
 * been drawn into it.
 *
 * Dependencies: Eigen3, GLFW3, OpenGL 3.3+, Dear ImGui docking branch.
 */

#include "gl_core.h"

#include "camera.h"
#include "gl_math.h"
#include "gl_mesh.h"
#include "gl_shader.h"
#include "gl_target.h"
#include "robot.h"
#include "screenshot.h"
#include "ui_layout.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GLFW/glfw3.h>

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* ============================================================
 * Shaders
 *
 * Blinn-Phong with a key light, a fixed fill light from behind and a
 * subtle rim term, which is what gives the parts their moulded look.
 * ============================================================ */

static const char *s_vertex_shader = R"(
#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat3 u_normal_matrix;

out vec3 v_world_pos;
out vec3 v_normal;

void main()
{
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    v_world_pos = world_pos.xyz;
    v_normal    = normalize(u_normal_matrix * a_normal);
    gl_Position = u_projection * u_view * world_pos;
}
)";

static const char *s_fragment_shader = R"(
#version 330 core

in vec3 v_world_pos;
in vec3 v_normal;

uniform vec3 u_color;
uniform vec3 u_light_pos;
uniform vec3 u_view_pos;

out vec4 frag_color;

const vec3  FILL_LIGHT_POS = vec3(-3.0, 5.0, -5.0);
const float AMBIENT        = 0.35;
const float KEY_STRENGTH   = 0.80;
const float FILL_STRENGTH  = 0.25;
const float SPECULAR       = 0.30;
const float SHININESS      = 32.0;

void main()
{
    vec3 normal   = normalize(v_normal);
    vec3 view_dir = normalize(u_view_pos - v_world_pos);

    vec3 key_dir  = normalize(u_light_pos - v_world_pos);
    vec3 fill_dir = normalize(FILL_LIGHT_POS - v_world_pos);

    vec3 lit = u_color * (AMBIENT
                        + KEY_STRENGTH  * max(dot(normal, key_dir),  0.0)
                        + FILL_STRENGTH * max(dot(normal, fill_dir), 0.0));

    vec3 half_dir = normalize(key_dir + view_dir);
    lit += vec3(SPECULAR) * pow(max(dot(normal, half_dir), 0.0), SHININESS);

    // Rim term: darkens surfaces turning away from the camera.
    float rim = 1.0 - abs(dot(normal, view_dir));
    lit += vec3(0.1, 0.1, 0.15) * pow(rim, 3.0) * 0.5;

    frag_color = vec4(lit, 1.0);
}
)";

/* ============================================================
 * Application state
 * ============================================================ */

/** Window background, behind the panels. */
static const float s_window_clear[3] = {0.12f, 0.13f, 0.16f};
/** Viewport background, behind the 3D scene. */
static const float s_viewport_clear[3] = {0.09f, 0.10f, 0.13f};
/** Key light position in world space. */
static const Vector3f s_light_pos(5.0f, 8.0f, 5.0f);
/** Grid colour, dim enough to stay behind the arm. */
static const float s_grid_color[3] = {0.25f, 0.25f, 0.28f};

static const float s_grid_size      = 10.0f;
static const int   s_grid_divisions = 20;
static const float s_fov_deg        = 45.0f;
static const float s_z_near         = 0.1f;
static const float s_z_far          = 100.0f;

/** Inset of the hint text from the viewport's top-left corner, in pixels. */
static const float s_overlay_margin = 10.0f;
/**
 * Multisampling for the scene. 1 turns it off, which is worth doing on a
 * software rasteriser: this machine reports llvmpipe, where every extra
 * sample is CPU work rather than free silicon.
 */
static const GLint s_scene_samples = 4;

/** Directory and file the window layout is persisted in. */
static const char *s_config_dir_name  = "robot_viewer";
static const char *s_layout_file_name = "imgui.ini";

/** Backing store for io.IniFilename, which ImGui holds a pointer to all run. */
static char s_layout_path[PATH_MAX];

static rbt_shader_t s_shader;
static rbt_robot_t  s_robot;
static rbt_camera_t s_camera;
static rbt_mesh_t   s_grid;
static rbt_target_t s_target;

static bool s_show_grid = true;
static bool s_wireframe = false;


/* ============================================================
 * Layout persistence
 * ============================================================ */

/**
 * @brief Join two path components with a separator.
 * @return false when the result would not fit in @p out.
 */
static bool s_path_join(char *out, size_t out_size, const char *base, const char *leaf)
{
    assert(out != NULL);
    assert(base != NULL);
    assert(leaf != NULL);

    const size_t base_len = strlen(base);
    const size_t leaf_len = strlen(leaf);

    if (base_len + 1 + leaf_len + 1 > out_size) {
        return false;
    }

    memcpy(out, base, base_len);
    out[base_len] = '/';
    memcpy(out + base_len + 1, leaf, leaf_len);
    out[base_len + 1 + leaf_len] = '\0';
    return true;
}

/**
 * @brief Create a directory, treating "it already exists" as success.
 *
 * A missing config directory is an operational failure, not a programmer
 * error: it is logged and the caller degrades to running without persistence.
 */
static bool s_make_directory(const char *path)
{
    assert(path != NULL);

    if (mkdir(path, 0755) == 0 || errno == EEXIST) {
        return true;
    }
    fprintf(stderr, "layout: cannot create %s: %s\n", path, strerror(errno));
    return false;
}

/**
 * @brief Resolve where the dock layout is stored.
 *
 * ImGui opens io.IniFilename relative to the working directory, so launching
 * from the source tree and from build/ used to leave two unrelated layouts on
 * disk. Anchor it to $XDG_CONFIG_HOME, or ~/.config when that is unset.
 *
 * @return Path to hand to ImGui, or NULL to run without persistence.
 */
static const char *s_resolve_layout_path(void)
{
    char config_root[PATH_MAX];
    char config_dir[PATH_MAX];

    const char *xdg = getenv("XDG_CONFIG_HOME");
    if (xdg != NULL && xdg[0] != '\0') {
        if (strlen(xdg) + 1 > sizeof(config_root)) {
            fprintf(stderr, "layout: XDG_CONFIG_HOME is too long to use\n");
            return NULL;
        }
        memcpy(config_root, xdg, strlen(xdg) + 1);
    } else {
        const char *home = getenv("HOME");
        if (home == NULL || home[0] == '\0') {
            fprintf(stderr, "layout: neither XDG_CONFIG_HOME nor HOME is set\n");
            return NULL;
        }
        if (!s_path_join(config_root, sizeof(config_root), home, ".config")) {
            fprintf(stderr, "layout: HOME is too long to use\n");
            return NULL;
        }
    }

    if (!s_make_directory(config_root)) {
        return NULL;
    }
    if (!s_path_join(config_dir, sizeof(config_dir), config_root, s_config_dir_name)) {
        fprintf(stderr, "layout: config path is too long to use\n");
        return NULL;
    }
    if (!s_make_directory(config_dir)) {
        return NULL;
    }
    if (!s_path_join(s_layout_path, sizeof(s_layout_path), config_dir, s_layout_file_name)) {
        fprintf(stderr, "layout: config path is too long to use\n");
        return NULL;
    }
    return s_layout_path;
}

/* ============================================================
 * Command line
 * ============================================================ */

/** @brief Options. Capturing is off unless --shot is given. */
typedef struct {
    const char *shot_path;      /**< PNG to write, NULL to run interactively. */
    const char *pose;           /**< Comma-separated joint angles in degrees. */
    int         width;
    int         height;
    int         warmup_frames;  /**< Frames to settle before capturing. */
    float       yaw_deg;
    float       pitch_deg;
    float       distance;
    float       target[3];      /**< Point the camera looks at, in world units. */
    bool        has_camera;     /**< A camera placement override was given. */
    bool        has_target;     /**< A look-at override was given. */
    bool        crop_to_view;   /**< Capture the 3D view alone, without panels. */
} rbt_options_t;

/** Enough frames for the dock layout to be built and then applied. */
static const int s_default_warmup = 6;

static void s_print_usage(const char *program)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "\n"
            "  --shot FILE      render one frame to a PNG and exit\n"
            "  --size WxH       window size (default 1280x800)\n"
            "  --view YAW,PITCH,DISTANCE   camera placement, degrees and units\n"
            "  --target X,Y,Z   point the camera looks at (default 0,1.5,0)\n"
            "  --pose A,B,C,... joint angles in degrees, in panel order\n"
            "  --bare           capture the 3D view only, without the panels\n"
            "  --warmup N       frames to settle before capturing (default %d)\n"
            "  --help           this message\n"
            "\n"
            "With no options the viewer runs normally.\n",
            program, s_default_warmup);
}

/** @brief Parse "WxH". */
static bool s_parse_size(const char *text, int *width, int *height)
{
    char *end = NULL;
    const long w = strtol(text, &end, 10);
    if (end == text || *end != 'x') {
        return false;
    }
    const char *rest = end + 1;
    const long h = strtol(rest, &end, 10);
    if (end == rest || *end != '\0' || w < 1 || h < 1) {
        return false;
    }
    *width  = (int)w;
    *height = (int)h;
    return true;
}

/** @brief Parse three comma-separated floats, as both --view and --target take. */
static bool s_parse_triple(const char *text, float out[3])
{
    const char *cursor = text;
    for (int i = 0; i < 3; i++) {
        char *end = NULL;
        out[i] = strtof(cursor, &end);
        if (end == cursor) {
            return false;
        }
        const char expected = (i < 2) ? ',' : '\0';
        if (*end != expected) {
            return false;
        }
        cursor = end + 1;
    }
    return true;
}

/**
 * @brief Read the command line.
 *
 * @param help_requested Set when the caller asked for usage, which is not an
 *                       error and should exit successfully.
 * @return false when an option was unknown or malformed.
 */
static bool s_parse_options(rbt_options_t *options, int argc, char **argv, bool *help_requested)
{
    *help_requested = false;

    for (int i = 1; i < argc; i++) {
        const char *arg  = argv[i];
        const bool has_value = (i + 1) < argc;

        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            s_print_usage(argv[0]);
            *help_requested = true;
            return true;
        } else if (strcmp(arg, "--bare") == 0) {
            options->crop_to_view = true;
        } else if (strcmp(arg, "--shot") == 0 && has_value) {
            options->shot_path = argv[++i];
        } else if (strcmp(arg, "--pose") == 0 && has_value) {
            options->pose = argv[++i];
        } else if (strcmp(arg, "--warmup") == 0 && has_value) {
            options->warmup_frames = (int)strtol(argv[++i], NULL, 10);
        } else if (strcmp(arg, "--size") == 0 && has_value) {
            if (!s_parse_size(argv[++i], &options->width, &options->height)) {
                fprintf(stderr, "bad --size, expected WxH\n");
                return false;
            }
        } else if (strcmp(arg, "--view") == 0 && has_value) {
            float view[3];
            if (!s_parse_triple(argv[++i], view)) {
                fprintf(stderr, "bad --view, expected YAW,PITCH,DISTANCE\n");
                return false;
            }
            options->yaw_deg    = view[0];
            options->pitch_deg  = view[1];
            options->distance   = view[2];
            options->has_camera = true;
        } else if (strcmp(arg, "--target") == 0 && has_value) {
            if (!s_parse_triple(argv[++i], options->target)) {
                fprintf(stderr, "bad --target, expected X,Y,Z\n");
                return false;
            }
            options->has_target = true;
        } else {
            fprintf(stderr, "unknown or incomplete option: %s\n\n", arg);
            s_print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

/** @brief Apply comma-separated angles to the joints, clamped to their limits. */
static void s_apply_pose(const char *text)
{
    assert(text != NULL);

    const char *cursor = text;
    for (size_t i = 0; i < s_robot.joints.size() && *cursor != '\0'; i++) {
        char *end = NULL;
        const float degrees = strtof(cursor, &end);
        if (end == cursor) {
            break;
        }

        rbt_joint_t &joint = s_robot.joints[i];
        float angle = degrees;
        if (angle < joint.min_angle_deg) {
            angle = joint.min_angle_deg;
        }
        if (angle > joint.max_angle_deg) {
            angle = joint.max_angle_deg;
        }
        joint.angle_deg = angle;

        cursor = (*end == ',') ? end + 1 : end;
    }
}

/* ============================================================
 * Rendering
 * ============================================================ */

static void s_glfw_error(int error, const char *description)
{
    fprintf(stderr, "glfw: error %d: %s\n", error, description);
}

/**
 * @brief Render the scene into its texture and place that texture in the panel.
 *
 * @return The panel's geometry and pointer state, for routing camera input.
 */
static rbt_viewport_t s_draw_viewport(void)
{
    const rbt_viewport_t view = rbt_viewport_begin();
    if (!view.visible) {
        rbt_viewport_end();
        return view;
    }

    /* The panel is measured in ImGui's screen coordinates; the texture is
     * allocated in framebuffer pixels, which differ on a HiDPI display. */
    const ImGuiIO &io = ImGui::GetIO();
    const GLsizei pixel_w = (GLsizei)(view.size.x * io.DisplayFramebufferScale.x);
    const GLsizei pixel_h = (GLsizei)(view.size.y * io.DisplayFramebufferScale.y);

    if (!rbt_target_resize(&s_target, pixel_w, pixel_h, s_scene_samples)) {
        rbt_viewport_end();
        return view;
    }

    rbt_target_begin(&s_target);

    glClearColor(s_viewport_clear[0], s_viewport_clear[1], s_viewport_clear[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const Matrix4f projection  = rbt_perspective(s_fov_deg, view.size.x / view.size.y, s_z_near, s_z_far);
    const Matrix4f camera_view = rbt_camera_view(&s_camera);

    rbt_shader_set_frame(&s_shader, camera_view, projection, rbt_camera_eye(&s_camera), s_light_pos);

    glPolygonMode(GL_FRONT_AND_BACK, s_wireframe ? GL_LINE : GL_FILL);

    if (s_show_grid) {
        /* A hair below the floor plane: the pedestal stands at exactly y = 0,
         * and coplanar geometry z-fights. */
        rbt_shader_set_object(&s_shader, rbt_translation(0.0f, -0.002f, 0.0f), s_grid_color);
        rbt_mesh_draw(&s_grid);
    }

    rbt_robot_draw(&s_robot, &s_shader);

    /* Leave GL as ImGui's backend expects to find it. */
    rbt_mesh_unbind();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    rbt_target_end(&s_target);

    /* Straight into the window's draw list rather than through ImGui::Image,
     * so the image does not become a second item competing with the button
     * above for the pointer. V is flipped because GL textures start at the
     * bottom row. */
    ImGui::GetWindowDrawList()->AddImage((ImTextureID)s_target.texture,
                                         view.pos,
                                         ImVec2(view.pos.x + view.size.x, view.pos.y + view.size.y),
                                         ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

    /* Screen coordinates, not window coordinates: SetCursorPos is measured
     * from the panel origin including its title bar, which put this text
     * underneath the title. */
    ImGui::SetCursorScreenPos(ImVec2(view.pos.x + s_overlay_margin, view.pos.y + s_overlay_margin));
    ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 0.8f), "LMB: orbit | RMB: pan | scroll: zoom");

    rbt_viewport_end();
    return view;
}

/** @brief Joint sliders, display options and a small readout. */
static void s_draw_control_panel(void)
{
    const ImGuiIO &io = ImGui::GetIO();

    ImGui::Begin(RBT_CONTROLS_TITLE, NULL, 0);

    if (ImGui::CollapsingHeader("Robot", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Reset All Joints")) {
            rbt_robot_reset_joints(&s_robot);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Camera")) {
            rbt_camera_reset(&s_camera);
        }
    }

    if (ImGui::CollapsingHeader("Joints", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (size_t i = 0; i < s_robot.joints.size(); i++) {
            rbt_joint_t &joint = s_robot.joints[i];

            ImGui::PushID((int)i);
            ImGui::Separator();

            ImGui::TextColored(ImVec4(joint.color[0], joint.color[1], joint.color[2], 1.0f),
                               "%s [%s]", joint.name, rbt_axis_label(joint.axis));

            ImGui::SliderFloat("##angle", &joint.angle_deg,
                               joint.min_angle_deg, joint.max_angle_deg, "%.1f deg");
            ImGui::SameLine();
            if (ImGui::SmallButton("R")) {
                joint.angle_deg = joint.default_angle_deg;
            }
            ImGui::PopID();
        }
    }

    if (ImGui::CollapsingHeader("Display")) {
        ImGui::Checkbox("Show Grid", &s_show_grid);
        ImGui::Checkbox("Wireframe", &s_wireframe);

        ImGui::Separator();
        ImGui::Text("Camera");
        ImGui::SliderFloat("Distance", &s_camera.distance, 2.0f, 20.0f);
        ImGui::SliderFloat("Yaw", &s_camera.yaw_deg, -180.0f, 180.0f);
        ImGui::SliderFloat("Pitch", &s_camera.pitch_deg, -89.0f, 89.0f);
    }

    if (ImGui::CollapsingHeader("Info")) {
        ImGui::Text("6-DOF manipulator, %zu joints", s_robot.joints.size());
        ImGui::Separator();
        ImGui::Text("%.1f FPS (%.2f ms)", io.Framerate, 1000.0f / io.Framerate);
    }

    ImGui::End();
}

/* ============================================================
 * Main
 * ============================================================ */

int main(int argc, char **argv)
{
    rbt_options_t options = {NULL, NULL, 1280, 800, s_default_warmup,
                             0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 0.0f}, false, false, false};
    bool help_requested = false;
    if (!s_parse_options(&options, argc, argv, &help_requested)) {
        return 1;
    }
    if (help_requested) {
        return 0;
    }
    const bool capturing = options.shot_path != NULL;

    glfwSetErrorCallback(s_glfw_error);
    if (glfwInit() == GLFW_FALSE) {
        fprintf(stderr, "glfw: init failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    /* No GLFW_SAMPLES: the default framebuffer carries only UI, which ImGui
     * antialiases itself. The scene is multisampled in its own target. */
    if (capturing) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }

    GLFWwindow *window = glfwCreateWindow(options.width, options.height, "Robot Viewer", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "glfw: could not create a window or an OpenGL 3.3 context\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    rbt_ui_configure_io();
    /* Multi-viewport stays off: a panel in its own OS window would not share
     * the framebuffer the scene is rendered into. */
    /* A capture must not depend on how the user happens to have arranged the
     * panels, so it starts from the built-in layout every time. */
    io.IniFilename = capturing ? NULL : s_resolve_layout_path();

    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding  = 3.0f;
    style.GrabRounding   = 2.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.11f, 0.14f, 1.00f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    printf("OpenGL %s on %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
    printf("layout: %s\n", io.IniFilename != NULL ? io.IniFilename : "(not persisted)");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    if (!rbt_shader_build(&s_shader, s_vertex_shader, s_fragment_shader)) {
        fprintf(stderr, "fatal: scene shader did not build\n");
        return 1;
    }

    rbt_camera_reset(&s_camera);
    if (options.has_camera) {
        s_camera.yaw_deg   = options.yaw_deg;
        s_camera.pitch_deg = options.pitch_deg;
        s_camera.distance  = options.distance;
    }
    if (options.has_target) {
        s_camera.target = Vector3f(options.target[0], options.target[1], options.target[2]);
    }

    rbt_robot_build(&s_robot);
    rbt_robot_upload_meshes(&s_robot);
    if (options.pose != NULL) {
        s_apply_pose(options.pose);
    }

    s_grid = rbt_mesh_make_grid(s_grid_size, s_grid_divisions);
    rbt_mesh_upload(&s_grid);

    int frame_index = 0;
    int exit_code = 0;

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int framebuffer_w = 0;
        int framebuffer_h = 0;
        glfwGetFramebufferSize(window, &framebuffer_w, &framebuffer_h);
        glViewport(0, 0, framebuffer_w, framebuffer_h);
        glClearColor(s_window_clear[0], s_window_clear[1], s_window_clear[2], 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        rbt_ui_dockspace(rbt_ui_default_layout);

        rbt_robot_update_fk(&s_robot);
        const rbt_viewport_t view = s_draw_viewport();

        const rbt_camera_input_t camera_input = rbt_viewport_camera_input(&view);
        rbt_camera_input(&s_camera, &camera_input);

        s_draw_control_panel();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        frame_index++;
        if (capturing && frame_index >= options.warmup_frames) {
            /* The back buffer holds the finished frame; reading it before the
             * swap avoids depending on what the window system keeps in front. */
            GLint   crop_x = 0;
            GLint   crop_y = 0;
            GLsizei crop_w = (GLsizei)framebuffer_w;
            GLsizei crop_h = (GLsizei)framebuffer_h;

            if (options.crop_to_view && view.visible) {
                crop_x = (GLint)(view.pos.x * io.DisplayFramebufferScale.x);
                crop_y = (GLint)((io.DisplaySize.y - view.pos.y - view.size.y) * io.DisplayFramebufferScale.y);
                crop_w = (GLsizei)(view.size.x * io.DisplayFramebufferScale.x);
                crop_h = (GLsizei)(view.size.y * io.DisplayFramebufferScale.y);
            }

            glReadBuffer(GL_BACK);
            if (!rbt_screenshot_capture(options.shot_path, crop_x, crop_y, crop_w, crop_h)) {
                exit_code = 1;
            }
            break;
        }

        glfwSwapBuffers(window);
    }

    rbt_target_destroy(&s_target);
    rbt_shader_destroy(&s_shader);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return exit_code;
}
