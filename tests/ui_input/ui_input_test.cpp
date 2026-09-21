/**
 * @file ui_input_test.cpp
 * @brief Headless check that pointer input reaches the camera only when it should.
 *
 * Why: the viewport panel and the camera both want the mouse, and every bug
 * this test covers was found by hand, in the running application, by noticing
 * the arm turning when it had no business turning. Dear ImGui needs no window
 * or GL context to decide hover and active state, so the whole interaction can
 * be replayed from synthetic events and asserted on.
 *
 * What: drives the real rbt_camera_input() through the same ImGui structure
 * main.cpp builds, for a panel that is docked and for one that floats.
 *
 * The ImGui calls here mirror s_draw_viewport() in src/main.cpp. They are a
 * deliberate copy, because that function also renders; if its Begin,
 * InvisibleButton or End sequence changes, change it here too.
 */

#include "camera.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <math.h>
#include <stdio.h>

/* Panel geometry for the floating cases, in screen pixels. */
static const ImVec2 s_float_pos(300.0f, 200.0f);
static const ImVec2 s_float_size(600.0f, 400.0f);

/* Where the controls panel sits when it is not docked. */
static const ImVec2 s_controls_float_pos(950.0f, 100.0f);
static const ImVec2 s_controls_float_size(300.0f, 500.0f);

static const ImVec2 s_display_size(1280.0f, 800.0f);
static const float  s_control_panel_fraction = 0.24f;

/** Pointer travel per simulated frame, in pixels. */
static const float s_drag_step = 15.0f;
/** Frames of pointer travel per simulated drag. */
static const int s_drag_frames = 6;
/** Yaw change below this counts as "the camera did not turn". */
static const float s_yaw_epsilon = 0.01f;

static int s_failures = 0;

/** State the panel reported during the last simulated frame. */
static bool   s_hovered;
static bool   s_active;
static ImVec2 s_panel_pos;
static ImVec2 s_content_origin;
static ImVec2 s_controls_pos;
static ImVec2 s_controls_content_origin;
static bool   s_dock_the_view;
static bool   s_dock_the_controls;

/** @brief Record one expectation; a failure is reported and counted, never fatal. */
static void s_check(bool ok, const char *what)
{
    printf("  %-58s %s\n", what, ok ? "pass" : "FAIL");
    if (!ok) {
        s_failures++;
    }
}

/**
 * @brief Run one simulated frame.
 *
 * @param mouse  Pointer position in screen pixels.
 * @param button Mouse button to report, or -1 for none held.
 * @param wheel  Scroll delta for this frame.
 */
static void s_frame(ImVec2 mouse, int button, float wheel)
{
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, button == ImGuiMouseButton_Left);
    io.AddMouseButtonEvent(ImGuiMouseButton_Right, button == ImGuiMouseButton_Right);
    if (wheel != 0.0f) {
        io.AddMouseWheelEvent(0.0f, wheel);
    }

    ImGui::NewFrame();

    /* Dock host, as in s_draw_dockspace(). */
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("##DockHost", NULL,
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar
                 | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize
                 | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus
                 | ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground);
    ImGui::PopStyleVar(3);

    const ImGuiID dockspace_id = ImGui::GetID("RobotViewerDockSpace");
    if (ImGui::DockBuilderGetNode(dockspace_id) == NULL) {
        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

        ImGuiID controls_node = 0;
        ImGuiID view_node     = 0;
        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, s_control_panel_fraction,
                                    &controls_node, &view_node);
        if (s_dock_the_view) {
            ImGui::DockBuilderDockWindow("Robot 3D View", view_node);
        }
        if (s_dock_the_controls) {
            ImGui::DockBuilderDockWindow("Joint Controls", controls_node);
        }
        ImGui::DockBuilderFinish(dockspace_id);
    }
    ImGui::DockSpace(dockspace_id);
    ImGui::End();

    /* Viewport panel, as in s_draw_viewport() minus the rendering. */
    ImGui::SetNextWindowPos(s_float_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(s_float_size, ImGuiCond_FirstUseEver);
    const bool visible = ImGui::Begin("Robot 3D View", NULL,
                                      ImGuiWindowFlags_NoScrollbar
                                      | ImGuiWindowFlags_NoScrollWithMouse
                                      | ImGuiWindowFlags_NoBackground);
    s_content_origin = ImGui::GetCursorScreenPos();
    const ImVec2 size = ImGui::GetContentRegionAvail();

    s_hovered = false;
    s_active  = false;
    if (visible && size.x >= 1.0f && size.y >= 1.0f) {
        ImGui::InvisibleButton("##viewport", size,
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        s_hovered = ImGui::IsItemHovered();
        s_active  = ImGui::IsItemActive();
    }
    s_panel_pos = ImGui::GetWindowPos();
    ImGui::End();

    ImGui::SetNextWindowPos(s_controls_float_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(s_controls_float_size, ImGuiCond_FirstUseEver);
    ImGui::Begin("Joint Controls", NULL, 0);
    s_controls_content_origin = ImGui::GetCursorScreenPos();
    ImGui::TextUnformatted("stub");
    s_controls_pos = ImGui::GetWindowPos();
    ImGui::End();

    ImGui::Render();
}

/** @brief Feed the frame's pointer state to the camera exactly as main.cpp does. */
static void s_feed_camera(rbt_camera_t *camera, float wheel)
{
    const ImGuiIO &io = ImGui::GetIO();
    const rbt_camera_input_t input = {
        io.MousePos.x,
        io.MousePos.y,
        s_hovered ? wheel : 0.0f,
        s_active && ImGui::IsMouseDown(ImGuiMouseButton_Left),
        s_active && ImGui::IsMouseDown(ImGuiMouseButton_Right),
    };
    rbt_camera_input(camera, &input);
}

/**
 * @brief Start a context configured like the application's.
 *
 * The docking choices must be settled before the first frame: the default
 * layout is built once, on the frame that finds no dock node yet.
 */
static void s_begin_context(bool dock_the_view, bool dock_the_controls = true)
{
    s_dock_the_view     = dock_the_view;
    s_dock_the_controls = dock_the_controls;

    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize  = s_display_size;
    io.DeltaTime    = 1.0f / 60.0f;
    io.IniFilename  = NULL;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    unsigned char *pixels = NULL;
    int tex_w = 0;
    int tex_h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &tex_w, &tex_h);
    io.Fonts->SetTexID((ImTextureID)1);

    /* Settle: the panel needs a frame to take its position before anything
     * can be aimed at its title bar. */
    for (int i = 0; i < 4; i++) {
        s_frame(ImVec2(10.0f, 10.0f), -1, 0.0f);
    }
}

/** @brief Midpoint of the panel's title bar or tab strip, derived from the live layout. */
static ImVec2 s_title_bar_point(void)
{
    return ImVec2(s_panel_pos.x + 150.0f,
                  s_panel_pos.y + (s_content_origin.y - s_panel_pos.y) * 0.5f);
}

/** @brief A point well inside the 3D content region. */
static ImVec2 s_content_point(void)
{
    return ImVec2(s_content_origin.x + 100.0f, s_content_origin.y + 100.0f);
}

/**
 * @brief Press, drag to the right and release; report what moved.
 *
 * @param on_title_bar Grab the title bar rather than the content region.
 * @param panel_moved  Set when the panel ended up somewhere else.
 * @param camera_turned Set when yaw changed over the drag.
 */
static void s_drag(bool docked, bool on_title_bar, bool *panel_moved, bool *camera_turned)
{
    s_begin_context(docked);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    ImVec2 mouse = on_title_bar ? s_title_bar_point() : s_content_point();
    const ImVec2 panel_before = s_panel_pos;
    const float  yaw_before   = camera.yaw_deg;

    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        s_feed_camera(&camera, 0.0f);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f);
    s_feed_camera(&camera, 0.0f);

    *panel_moved   = fabsf(s_panel_pos.x - panel_before.x) > 1.0f;
    *camera_turned = fabsf(camera.yaw_deg - yaw_before) > s_yaw_epsilon;

    ImGui::DestroyContext();
}

/** @brief Dragging the title bar moves the panel and must leave the camera alone. */
static void s_case_title_bar_moves_panel(bool docked)
{
    bool panel_moved = false;
    bool camera_turned = false;
    s_drag(docked, true, &panel_moved, &camera_turned);

    printf("%s panel, drag the title bar:\n", docked ? "Docked" : "Floating");
    if (!docked) {
        s_check(panel_moved, "panel moves");
    }
    s_check(!camera_turned, "camera does not turn");
}

/** @brief Dragging inside the view orbits and must not move the panel. */
static void s_case_content_orbits(bool docked)
{
    bool panel_moved = false;
    bool camera_turned = false;
    s_drag(docked, false, &panel_moved, &camera_turned);

    printf("%s panel, drag inside the 3D view:\n", docked ? "Docked" : "Floating");
    s_check(!panel_moved, "panel stays put");
    s_check(camera_turned, "camera turns");
}

/** @brief The wheel zooms over the view and is ignored elsewhere. */
static void s_case_wheel_is_gated(void)
{
    s_begin_context(false);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    printf("Wheel gating:\n");

    const float distance_before = camera.distance;
    s_frame(ImVec2(10.0f, 10.0f), -1, 1.0f);
    s_feed_camera(&camera, 1.0f);
    s_check(camera.distance == distance_before, "wheel outside the view does not zoom");

    s_frame(s_content_point(), -1, 1.0f);
    s_feed_camera(&camera, 1.0f);
    s_check(camera.distance < distance_before, "wheel over the view zooms in");

    ImGui::DestroyContext();
}

/**
 * @brief Releasing the button ends the orbit, even off the panel.
 *
 * The camera used to keep its own "a drag started over the view" flag and
 * orbit for as long as the button looked held, so a release it missed left it
 * turning with every later pointer movement.
 */
static void s_case_release_ends_orbit(void)
{
    s_begin_context(false);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    printf("Release outside the panel:\n");

    ImVec2 mouse = s_content_point();
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        s_feed_camera(&camera, 0.0f);
        mouse.x += s_drag_step;
    }
    const float yaw_during = camera.yaw_deg;
    s_check(fabsf(yaw_during - 45.0f) > s_yaw_epsilon, "the drag turned the camera");

    /* Let go far away from the panel, then keep moving. */
    mouse = ImVec2(20.0f, 700.0f);
    s_frame(mouse, -1, 0.0f);
    s_feed_camera(&camera, 0.0f);

    const float yaw_after_release = camera.yaw_deg;
    for (int step = 0; step < s_drag_frames; step++) {
        mouse.x += s_drag_step;
        s_frame(mouse, -1, 0.0f);
        s_feed_camera(&camera, 0.0f);
    }
    s_check(fabsf(camera.yaw_deg - yaw_after_release) < s_yaw_epsilon,
            "moving after the release turns nothing");

    ImGui::DestroyContext();
}

/**
 * @brief A panel with no item under the pointer still only moves by its title bar.
 *
 * This is what ConfigWindowsMoveFromTitleBarOnly buys. The 3D view is covered
 * by its own InvisibleButton, so the controls panel is what actually exercises
 * the setting: its body is mostly empty space.
 */
static void s_case_body_drag_does_not_move_a_panel(void)
{
    s_begin_context(true, false);

    printf("Floating controls panel:\n");

    /* Well below the one widget, so the pointer sits on bare panel body. */
    ImVec2 mouse(s_controls_content_origin.x + 40.0f, s_controls_content_origin.y + 200.0f);
    float start_x = s_controls_pos.x;
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f);
    s_check(fabsf(s_controls_pos.x - start_x) < 1.0f, "dragging the body does not move it");

    mouse = ImVec2(s_controls_pos.x + 100.0f,
                   s_controls_pos.y + (s_controls_content_origin.y - s_controls_pos.y) * 0.5f);
    start_x = s_controls_pos.x;
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f);
    s_check(fabsf(s_controls_pos.x - start_x) > 1.0f, "dragging the title bar moves it");

    ImGui::DestroyContext();
}

int main(void)
{
    IMGUI_CHECKVERSION();

    s_case_title_bar_moves_panel(false);
    s_case_content_orbits(false);
    s_case_title_bar_moves_panel(true);
    s_case_content_orbits(true);
    s_case_wheel_is_gated();
    s_case_release_ends_orbit();
    s_case_body_drag_does_not_move_a_panel();

    printf("\n%s\n", s_failures == 0 ? "ui_input: all checks passed"
                                     : "ui_input: FAILURES above");
    return s_failures == 0 ? 0 : 1;
}
