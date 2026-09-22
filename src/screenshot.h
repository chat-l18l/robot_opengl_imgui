/**
 * @file screenshot.h
 * @brief Capture part of the framebuffer to a PNG, for the documentation images.
 *
 * Why this lives in the application rather than a screen-grabbing tool: the
 * images in the README should be reproducible. Regenerating them after the arm
 * or the shading changes is then a command, not an afternoon of cropping.
 */

#pragma once

#include "gl_core.h"

/**
 * @brief Read a rectangle of the current read framebuffer and write it as a PNG.
 *
 * Coordinates are framebuffer pixels with the origin at the bottom left, as GL
 * counts them. The rows come back bottom-up and are flipped on the way out.
 *
 * A failure to allocate, read or write is operational: it is logged and
 * reported, never asserted.
 *
 * @return true when the file was written.
 */
bool rbt_screenshot_capture(const char *path, GLint x, GLint y, GLsizei width, GLsizei height);
