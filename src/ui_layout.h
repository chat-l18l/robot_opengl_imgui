/**
 * @file ui_layout.h
 * @brief The ImGui structure of the application window.
 *
 * Why this is not inside main.cpp: the viewport panel decides who owns the
 * pointer, the camera or the panel arrangement, and getting that split wrong
 * is only visible as the arm turning when it should not. ImGui resolves hover
 * and active state without a window or a GL context, so that decision can be
 * tested directly. Keeping the panel's Begin, its InvisibleButton and the
 * input it hands the camera here means tests/ui_input drives the same code the
 * application runs rather than a copy of it.
 *
 * Rendering deliberately stays out: this module makes no GL calls.
 */

#pragma once

#include "camera.h"

#include <imgui.h>

/** Panel titles. They double as the keys the dock layout is stored under. */
#define RBT_VIEWPORT_TITLE "Robot 3D View"
#define RBT_CONTROLS_TITLE "Joint Controls"

/** @brief What the viewport panel reported this frame. */
typedef struct {
    ImVec2 pos;      /**< Content-region top-left, in screen coordinates. */
    ImVec2 size;     /**< Content-region size, in screen coordinates. */
    bool   visible;  /**< Open, not collapsed, and large enough to draw into. */
    bool   hovered;  /**< The pointer is over the content region. */
    bool   active;   /**< The content region holds the pointer for a drag. */
} rbt_viewport_t;

/**
 * @brief Apply the interaction settings the panels depend on.
 *
 * Docking, because the panels live in a dock space, and title-bar-only moving,
 * because ImGui otherwise lets a panel be dragged from anywhere in its body,
 * which over the 3D view competes with orbiting.
 *
 * Pre: an ImGui context is current.
 */
void rbt_ui_configure_io(void);

/** @brief Arranges the panels the first time, when no saved layout exists. */
typedef void (*rbt_ui_layout_fn)(ImGuiID dockspace_id, const ImVec2 &size);

/**
 * @brief Draw the full-window dock host and its dock space.
 *
 * The host paints no background, which is what lets a panel show something
 * drawn outside ImGui underneath it.
 *
 * @param layout Called once, on the frame that finds no dock node yet. NULL
 *               leaves the panels floating.
 */
void rbt_ui_dockspace(rbt_ui_layout_fn layout);

/** @brief The application's arrangement: viewport on the left, controls right. */
void rbt_ui_default_layout(ImGuiID dockspace_id, const ImVec2 &size);

/**
 * @brief Begin the viewport panel and claim its content region for the pointer.
 *
 * Claiming matters: with no item under the pointer ImGui reads a press in the
 * panel body as a drag of the panel itself, which over a 3D view competes with
 * orbiting. An item also gives proper hit testing, so a panel on top takes the
 * pointer instead.
 *
 * Always pair with rbt_viewport_end(), including when the panel is not visible.
 */
rbt_viewport_t rbt_viewport_begin(void);

/** @brief Close the viewport panel. */
void rbt_viewport_end(void);

/**
 * @brief Translate this frame's pointer state into camera input.
 *
 * Drag ownership comes from ImGui's own answer, IsItemActive: true from the
 * press inside the view until the release, wherever the pointer wanders, and
 * cleared when the window loses focus. The camera keeps no opinion of its own
 * about who holds the mouse.
 */
rbt_camera_input_t rbt_viewport_camera_input(const rbt_viewport_t *view);
