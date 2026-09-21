/**
 * @file ui_layout.cpp
 * @brief Dock host, viewport panel and pointer routing.
 */

#include "ui_layout.h"

#include <imgui_internal.h>  /* DockBuilder, for the first-run layout. */

#include <assert.h>

/** Identifier of the root dock node, and the key its layout is stored under. */
static const char *s_dockspace_name = "RobotViewerDockSpace";

/** Fraction of the width the control panel takes in the first-run layout. */
static const float s_control_panel_fraction = 0.24f;

void rbt_ui_configure_io(void)
{
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
}

void rbt_ui_dockspace(rbt_ui_layout_fn layout)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();

    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking
                                 | ImGuiWindowFlags_NoTitleBar
                                 | ImGuiWindowFlags_NoCollapse
                                 | ImGuiWindowFlags_NoResize
                                 | ImGuiWindowFlags_NoMove
                                 | ImGuiWindowFlags_NoBringToFrontOnFocus
                                 | ImGuiWindowFlags_NoNavFocus
                                 | ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("##DockHost", NULL, flags);
    ImGui::PopStyleVar(3);

    const ImGuiID dockspace_id = ImGui::GetID(s_dockspace_name);
    if (layout != NULL && ImGui::DockBuilderGetNode(dockspace_id) == NULL) {
        layout(dockspace_id, viewport->WorkSize);
    }
    ImGui::DockSpace(dockspace_id);

    ImGui::End();
}

void rbt_ui_default_layout(ImGuiID dockspace_id, const ImVec2 &size)
{
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, size);

    ImGuiID controls_node = 0;
    ImGuiID view_node     = 0;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, s_control_panel_fraction,
                                &controls_node, &view_node);

    ImGui::DockBuilderDockWindow(RBT_VIEWPORT_TITLE, view_node);
    ImGui::DockBuilderDockWindow(RBT_CONTROLS_TITLE, controls_node);
    ImGui::DockBuilderFinish(dockspace_id);
}

rbt_viewport_t rbt_viewport_begin(void)
{
    rbt_viewport_t view = {ImVec2(0.0f, 0.0f), ImVec2(0.0f, 0.0f), false, false, false};

    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar
                                 | ImGuiWindowFlags_NoScrollWithMouse
                                 | ImGuiWindowFlags_NoBackground;

    /* Begin() reports false for a collapsed panel or an inactive dock tab. */
    const bool open = ImGui::Begin(RBT_VIEWPORT_TITLE, NULL, flags);

    view.pos     = ImGui::GetCursorScreenPos();
    view.size    = ImGui::GetContentRegionAvail();
    view.visible = open && view.size.x >= 1.0f && view.size.y >= 1.0f;

    if (view.visible) {
        ImGui::InvisibleButton("##viewport", view.size,
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        view.hovered = ImGui::IsItemHovered();
        view.active  = ImGui::IsItemActive();
    }
    return view;
}

void rbt_viewport_end(void)
{
    ImGui::End();
}

rbt_camera_input_t rbt_viewport_camera_input(const rbt_viewport_t *view)
{
    assert(view != NULL);

    const ImGuiIO &io = ImGui::GetIO();
    const rbt_camera_input_t input = {
        io.MousePos.x,
        io.MousePos.y,
        view->hovered ? io.MouseWheel : 0.0f,
        view->active && ImGui::IsMouseDown(ImGuiMouseButton_Left),
        view->active && ImGui::IsMouseDown(ImGuiMouseButton_Right),
    };
    return input;
}
