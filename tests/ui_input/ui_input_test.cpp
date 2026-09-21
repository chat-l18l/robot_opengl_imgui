/**
 * @file ui_input_test.cpp
 * @brief Headless check that pointer input reaches the camera only when it should.
 *
 * Why: the viewport panel and the camera both want the mouse, and every bug
 * covered here was found by hand, in the running application, by noticing the
 * arm turning when it had no business turning.
 *
 * What: drives the application's own rbt_ui_dockspace(), rbt_viewport_begin()
 * and rbt_viewport_camera_input() from src/ui_layout.cpp, feeding the real
 * rbt_camera_input() from src/camera.cpp. Dear ImGui resolves hover and active
 * state without a window or a GL context, so nothing here is a stand-in for
 * the code that ships.
 *
 * The only thing the test supplies itself is the dock arrangement, because it
 * needs panels both docked and floating, which the application never varies.
 *
 * Two timing details this mirrors on purpose:
 *
 * - The camera is fed inside the frame, before Render(), exactly where
 *   main.cpp does it. io.MouseWheel only holds a value for the duration of the
 *   frame, so reading it afterwards yields zero and silently tests nothing.
 * - ImGui trickles its input queue, so a pointer move and a wheel notch
 *   submitted in the same frame do not both land in that frame. Real hardware
 *   spreads them out anyway; s_wheel_at() does the same.
 */

#include "camera.h"
#include "ui_layout.h"

#include <imgui.h>
#include <imgui_internal.h>  /* FindWindowByName, to watch a panel move. */

#include <math.h>
#include <stdio.h>

/* Where the panels sit when they are not docked, in screen pixels. */
static const ImVec2 s_view_float_pos(300.0f, 200.0f);
static const ImVec2 s_view_float_size(600.0f, 400.0f);
static const ImVec2 s_controls_float_pos(950.0f, 100.0f);
static const ImVec2 s_controls_float_size(300.0f, 500.0f);

static const ImVec2 s_display_size(1280.0f, 800.0f);
static const float  s_controls_fraction = 0.24f;

/** Pointer travel per simulated frame, in pixels. */
static const float s_drag_step = 15.0f;
/** Frames of pointer travel per simulated drag. */
static const int s_drag_frames = 6;
/** Yaw change below this counts as "the camera did not turn". */
static const float s_yaw_epsilon = 0.01f;
/** Movement below this counts as "the panel stayed put". */
static const float s_move_epsilon = 1.0f;

static int s_failures = 0;

/** What the shared viewport panel reported during the last simulated frame. */
static rbt_viewport_t s_view;
/** Content origin of the stand-in control panel, for aiming at its title bar. */
static ImVec2 s_controls_content_origin;

static bool s_dock_the_view;
static bool s_dock_the_controls;

/** @brief Record one expectation; a failure is reported and counted, never fatal. */
static void s_check(bool ok, const char *what)
{
    printf("  %-58s %s\n", what, ok ? "pass" : "FAIL");
    if (!ok) {
        s_failures++;
    }
}

/**
 * @brief Arrangement for the case being run.
 *
 * The node is always created, even when neither panel is docked, so the dock
 * space stops asking for a layout after the first frame.
 */
static void s_test_layout(ImGuiID dockspace_id, const ImVec2 &size)
{
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, size);

    ImGuiID controls_node = 0;
    ImGuiID view_node     = 0;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, s_controls_fraction,
                                &controls_node, &view_node);

    if (s_dock_the_view) {
        ImGui::DockBuilderDockWindow(RBT_VIEWPORT_TITLE, view_node);
    }
    if (s_dock_the_controls) {
        ImGui::DockBuilderDockWindow(RBT_CONTROLS_TITLE, controls_node);
    }
    ImGui::DockBuilderFinish(dockspace_id);
}

/**
 * @brief Run one simulated frame.
 *
 * @param mouse  Pointer position in screen pixels.
 * @param button Mouse button to report as held, or -1 for none.
 * @param wheel  Scroll delta for this frame.
 * @param camera Fed from inside the frame when given, as the application does.
 */
static void s_frame(ImVec2 mouse, int button, float wheel, rbt_camera_t *camera = NULL)
{
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(mouse.x, mouse.y);
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, button == ImGuiMouseButton_Left);
    io.AddMouseButtonEvent(ImGuiMouseButton_Right, button == ImGuiMouseButton_Right);
    if (wheel != 0.0f) {
        io.AddMouseWheelEvent(0.0f, wheel);
    }

    ImGui::NewFrame();

    rbt_ui_dockspace(s_test_layout);

    ImGui::SetNextWindowPos(s_view_float_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(s_view_float_size, ImGuiCond_FirstUseEver);
    s_view = rbt_viewport_begin();
    rbt_viewport_end();

    /* Stand-in for the control panel: the widgets do not matter, only that
     * most of the body is bare, which is what a window move would grab. */
    ImGui::SetNextWindowPos(s_controls_float_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(s_controls_float_size, ImGuiCond_FirstUseEver);
    ImGui::Begin(RBT_CONTROLS_TITLE, NULL, 0);
    s_controls_content_origin = ImGui::GetCursorScreenPos();
    ImGui::TextUnformatted("stub");
    ImGui::End();

    if (camera != NULL) {
        const rbt_camera_input_t input = rbt_viewport_camera_input(&s_view);
        rbt_camera_input(camera, &input);
    }

    ImGui::Render();
}

/** @brief Current position of a panel's window, to watch it move. */
static ImVec2 s_window_pos(const char *title)
{
    const ImGuiWindow *window = ImGui::FindWindowByName(title);
    return window != NULL ? window->Pos : ImVec2(-1.0f, -1.0f);
}

/**
 * @brief Start a context configured like the application's.
 *
 * The docking choices must be settled before the first frame: the layout is
 * built once, on the frame that finds no dock node yet.
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
    rbt_ui_configure_io();

    unsigned char *pixels = NULL;
    int tex_w = 0;
    int tex_h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &tex_w, &tex_h);
    io.Fonts->SetTexID((ImTextureID)1);

    /* Settle: a panel needs a frame to take its position before anything can
     * be aimed at its title bar. */
    for (int i = 0; i < 4; i++) {
        s_frame(ImVec2(10.0f, 10.0f), -1, 0.0f);
    }
}

/** @brief Midpoint between a panel's top edge and its content, so: its title bar. */
static ImVec2 s_title_bar_point(ImVec2 window_pos, ImVec2 content_origin)
{
    return ImVec2(window_pos.x + 100.0f,
                  window_pos.y + (content_origin.y - window_pos.y) * 0.5f);
}

/** @brief A point well inside the 3D content region. */
static ImVec2 s_content_point(void)
{
    return ImVec2(s_view.pos.x + 100.0f, s_view.pos.y + 100.0f);
}

/**
 * @brief Press, drag to the right and release; report what moved.
 *
 * @param on_title_bar  Grab the panel's title bar rather than its content.
 * @param panel_moved   Set when the panel ended up somewhere else.
 * @param camera_turned Set when yaw changed over the drag.
 */
static void s_drag_viewport(bool docked, bool on_title_bar, bool *panel_moved, bool *camera_turned)
{
    s_begin_context(docked);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    const ImVec2 panel_before = s_window_pos(RBT_VIEWPORT_TITLE);
    const float  yaw_before   = camera.yaw_deg;

    ImVec2 mouse = on_title_bar ? s_title_bar_point(panel_before, s_view.pos) : s_content_point();

    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f, &camera);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f, &camera);

    *panel_moved   = fabsf(s_window_pos(RBT_VIEWPORT_TITLE).x - panel_before.x) > s_move_epsilon;
    *camera_turned = fabsf(camera.yaw_deg - yaw_before) > s_yaw_epsilon;

    ImGui::DestroyContext();
}

/** @brief Dragging the title bar moves the panel and must leave the camera alone. */
static void s_case_title_bar_moves_panel(bool docked)
{
    bool panel_moved = false;
    bool camera_turned = false;
    s_drag_viewport(docked, true, &panel_moved, &camera_turned);

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
    s_drag_viewport(docked, false, &panel_moved, &camera_turned);

    printf("%s panel, drag inside the 3D view:\n", docked ? "Docked" : "Floating");
    s_check(!panel_moved, "panel stays put");
    s_check(camera_turned, "camera turns");
}

/**
 * @brief Park the pointer somewhere, then send one wheel notch.
 *
 * The idle frames either side are what keep the move and the notch in separate
 * frames, so the notch is actually delivered where the pointer now is.
 */
static void s_wheel_at(rbt_camera_t *camera, ImVec2 mouse)
{
    for (int i = 0; i < 3; i++) {
        s_frame(mouse, -1, 0.0f, camera);
    }
    s_frame(mouse, -1, 1.0f, camera);
    for (int i = 0; i < 3; i++) {
        s_frame(mouse, -1, 0.0f, camera);
    }
}

/** @brief The wheel zooms over the view and is ignored elsewhere. */
static void s_case_wheel_is_gated(void)
{
    s_begin_context(false);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    printf("Wheel gating:\n");

    const float distance_before = camera.distance;
    s_wheel_at(&camera, ImVec2(10.0f, 10.0f));
    s_check(camera.distance == distance_before, "wheel outside the view does not zoom");

    s_wheel_at(&camera, s_content_point());
    s_check(camera.distance < distance_before, "wheel over the view zooms in");

    ImGui::DestroyContext();
}

/**
 * @brief Releasing the button ends the orbit, even off the panel.
 *
 * The camera used to keep its own "a drag started over the view" flag and
 * orbit for as long as the button looked held, so a release it missed would
 * have left it turning with every later pointer movement.
 */
static void s_case_release_ends_orbit(void)
{
    s_begin_context(false);

    rbt_camera_t camera;
    rbt_camera_reset(&camera);

    printf("Release outside the panel:\n");

    const float yaw_before = camera.yaw_deg;
    ImVec2 mouse = s_content_point();
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f, &camera);
        mouse.x += s_drag_step;
    }
    s_check(fabsf(camera.yaw_deg - yaw_before) > s_yaw_epsilon, "the drag turned the camera");

    /* Let go far away from the panel, then keep moving. */
    mouse = ImVec2(20.0f, 700.0f);
    s_frame(mouse, -1, 0.0f, &camera);

    const float yaw_after_release = camera.yaw_deg;
    for (int step = 0; step < s_drag_frames; step++) {
        mouse.x += s_drag_step;
        s_frame(mouse, -1, 0.0f, &camera);
    }
    s_check(fabsf(camera.yaw_deg - yaw_after_release) < s_yaw_epsilon,
            "moving after the release turns nothing");

    ImGui::DestroyContext();
}

/**
 * @brief A panel with no item under the pointer still only moves by its title bar.
 *
 * This is what ConfigWindowsMoveFromTitleBarOnly buys. The 3D view is covered
 * by the viewport's own InvisibleButton, so the control panel is what actually
 * exercises the setting: its body is mostly empty space.
 */
static void s_case_body_drag_does_not_move_a_panel(void)
{
    s_begin_context(true, false);

    printf("Floating controls panel:\n");

    /* Well below the one widget, so the pointer sits on bare panel body. */
    ImVec2 mouse(s_controls_content_origin.x + 40.0f, s_controls_content_origin.y + 200.0f);
    float start_x = s_window_pos(RBT_CONTROLS_TITLE).x;
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f);
    s_check(fabsf(s_window_pos(RBT_CONTROLS_TITLE).x - start_x) < s_move_epsilon,
            "dragging the body does not move it");

    mouse = s_title_bar_point(s_window_pos(RBT_CONTROLS_TITLE), s_controls_content_origin);
    start_x = s_window_pos(RBT_CONTROLS_TITLE).x;
    for (int step = 0; step < s_drag_frames; step++) {
        s_frame(mouse, ImGuiMouseButton_Left, 0.0f);
        mouse.x += s_drag_step;
    }
    s_frame(mouse, -1, 0.0f);
    s_check(fabsf(s_window_pos(RBT_CONTROLS_TITLE).x - start_x) > s_move_epsilon,
            "dragging the title bar moves it");

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
