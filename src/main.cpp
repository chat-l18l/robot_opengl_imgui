/**
 * @file main.cpp
 * @brief Dear ImGui + OpenGL 3.3 viewer for a 6-DOF robot arm.
 *
 * One window, two panels: a 3D viewport and a joint control panel. The 3D is
 * rendered straight into the default framebuffer under a scissor rectangle
 * matching the viewport panel, before ImGui's own draw data is submitted, so
 * the UI composites over it. That avoids a render target and a resolve, at the
 * cost of only ever supporting one 3D view.
 *
 * Dependencies: Eigen3, GLFW3, OpenGL 3.3+, Dear ImGui (fetched by CMake).
 */

#include "gl_core.h"

#include "camera.h"
#include "gl_math.h"
#include "gl_mesh.h"
#include "gl_shader.h"
#include "robot.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GLFW/glfw3.h>

#include <stdio.h>

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

/** Window background, behind the ImGui panels. */
static const float s_window_clear[3] = {0.12f, 0.13f, 0.16f};
/** Viewport background, behind the 3D scene. */
static const float s_viewport_clear[3] = {0.09f, 0.10f, 0.13f};
/** Key light position in world space. */
static const Vector3f s_light_pos(5.0f, 8.0f, 5.0f);
/** Grid colour, dim enough to stay behind the arm. */
static const float s_grid_color[3] = {0.25f, 0.25f, 0.28f};

static const float s_grid_size     = 10.0f;
static const int   s_grid_divisions = 20;
static const float s_fov_deg       = 45.0f;
static const float s_z_near        = 0.1f;
static const float s_z_far         = 100.0f;

static const int s_control_panel_width = 310;

static rbt_shader_t s_shader;
static rbt_robot_t  s_robot;
static rbt_camera_t s_camera;
static rbt_mesh_t   s_grid;

static bool s_show_grid = true;
static bool s_wireframe = false;

/** @brief Where the 3D viewport panel ended up this frame. */
typedef struct {
    ImVec2 pos;        /**< Top-left in screen coordinates. */
    ImVec2 size;       /**< Size in screen coordinates. */
    bool   hovered;    /**< Pointer is over the panel. */
} rbt_viewport_t;

/* ============================================================
 * Rendering
 * ============================================================ */

static void s_glfw_error(int error, const char *description)
{
    fprintf(stderr, "glfw: error %d: %s\n", error, description);
}

/**
 * @brief Draw the 3D scene inside an ImGui panel.
 *
 * The panel is borderless and background-less; the GL scissor rectangle is
 * what actually confines the scene to it.
 *
 * @return The panel's geometry, so the caller can route pointer input to it.
 */
static rbt_viewport_t s_draw_viewport(void)
{
    rbt_viewport_t view = {ImVec2(0, 0), ImVec2(0, 0), false};

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar
                                 | ImGuiWindowFlags_NoScrollWithMouse
                                 | ImGuiWindowFlags_NoBackground;

    ImGui::Begin("Robot 3D View", NULL, flags);
    view.pos     = ImGui::GetCursorScreenPos();
    view.size    = ImGui::GetContentRegionAvail();
    view.hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);

    if (view.size.x < 1.0f || view.size.y < 1.0f) {
        ImGui::End();
        return view;
    }

    /* ImGui reports screen coordinates with Y down; GL wants framebuffer
     * pixels with Y up. DisplayFramebufferScale bridges the two on HiDPI. */
    const ImGuiIO &io = ImGui::GetIO();
    const GLint gl_x = (GLint)(view.pos.x * io.DisplayFramebufferScale.x);
    const GLint gl_y = (GLint)((io.DisplaySize.y - view.pos.y - view.size.y) * io.DisplayFramebufferScale.y);
    const GLint gl_w = (GLint)(view.size.x * io.DisplayFramebufferScale.x);
    const GLint gl_h = (GLint)(view.size.y * io.DisplayFramebufferScale.y);

    glViewport(gl_x, gl_y, gl_w, gl_h);
    glScissor(gl_x, gl_y, gl_w, gl_h);
    glEnable(GL_SCISSOR_TEST);

    glClearColor(s_viewport_clear[0], s_viewport_clear[1], s_viewport_clear[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const Matrix4f projection = rbt_perspective(s_fov_deg, view.size.x / view.size.y, s_z_near, s_z_far);
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
    glDisable(GL_SCISSOR_TEST);

    ImGui::SetCursorPos(ImVec2(10.0f, 10.0f));
    ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 0.8f), "LMB: orbit | RMB: pan | scroll: zoom");

    ImGui::End();
    return view;
}

/** @brief Joint sliders, display options and a small readout. */
static void s_draw_control_panel(void)
{
    const ImGuiIO &io = ImGui::GetIO();

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - (float)s_control_panel_width - 10.0f, 0.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2((float)s_control_panel_width, io.DisplaySize.y),
                             ImGuiCond_FirstUseEver);

    ImGui::Begin("Joint Controls", NULL, 0);

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
            rbt_joint_t *joint = s_robot.joints[i];

            ImGui::PushID((int)i);
            ImGui::Separator();

            ImGui::TextColored(ImVec4(joint->color[0], joint->color[1], joint->color[2], 1.0f),
                               "%s [%s]", joint->name, rbt_axis_label(joint->axis));

            ImGui::SliderFloat("##angle", &joint->angle_deg,
                               joint->min_angle_deg, joint->max_angle_deg, "%.1f deg");
            ImGui::SameLine();
            if (ImGui::SmallButton("R")) {
                joint->angle_deg = joint->default_angle_deg;
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

int main(void)
{
    glfwSetErrorCallback(s_glfw_error);
    if (glfwInit() == GLFW_FALSE) {
        fprintf(stderr, "glfw: init failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow *window = glfwCreateWindow(1280, 800, "Robot Viewer", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "glfw: could not create a window or an OpenGL 3.3 context\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding  = 3.0f;
    style.GrabRounding   = 2.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.11f, 0.14f, 1.00f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    printf("OpenGL %s on %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    if (!rbt_shader_build(&s_shader, s_vertex_shader, s_fragment_shader)) {
        fprintf(stderr, "fatal: scene shader did not build\n");
        return 1;
    }

    rbt_camera_reset(&s_camera);
    rbt_robot_build(&s_robot);
    rbt_robot_upload_meshes(&s_robot);

    s_grid = rbt_mesh_make_grid(s_grid_size, s_grid_divisions);
    rbt_mesh_upload(&s_grid);

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

        rbt_robot_update_fk(&s_robot);
        const rbt_viewport_t view = s_draw_viewport();

        /* Fed every frame, hovered or not, so an ongoing drag sees the button
         * come up even when the pointer left the viewport. */
        const ImGuiIO &io = ImGui::GetIO();
        const rbt_camera_input_t camera_input = {
            io.MousePos.x,
            io.MousePos.y,
            io.MouseWheel,
            ImGui::IsMouseDown(ImGuiMouseButton_Left),
            ImGui::IsMouseDown(ImGuiMouseButton_Right),
            view.hovered,
        };
        rbt_camera_input(&s_camera, &camera_input);

        s_draw_control_panel();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    rbt_shader_destroy(&s_shader);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
