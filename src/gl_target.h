/**
 * @file gl_target.h
 * @brief Offscreen framebuffer the 3D scene renders into.
 *
 * Why: the scene used to go straight into the default framebuffer under a
 * scissor rectangle, before ImGui submitted its draw data. That only holds as
 * long as nothing ImGui paints afterwards covers the rectangle, and it stops
 * holding the moment the panel floats over another one, or the dock space
 * fills its empty central node with ImGuiCol_DockingEmptyBg. Rendering into a
 * texture turns the scene into an ordinary widget that composites in draw
 * order, wherever the panel sits.
 */

#pragma once

#include "gl_core.h"

/**
 * @brief A colour texture plus the framebuffers needed to render into it.
 *
 * With multisampling the scene is drawn into multisampled renderbuffers and
 * resolved into @ref texture, because a multisampled attachment cannot be
 * sampled by an ordinary sampler2D. Without it the texture is drawn into
 * directly and no resolve happens.
 */
typedef struct {
    GLuint  msaa_fbo;     /**< Multisampled framebuffer, 0 when samples == 0. */
    GLuint  msaa_color;   /**< Multisampled colour renderbuffer. */
    GLuint  msaa_depth;   /**< Multisampled depth renderbuffer. */

    GLuint  resolve_fbo;  /**< Single-sample framebuffer owning the texture. */
    GLuint  texture;      /**< Colour texture handed to ImGui, 0 when unallocated. */
    GLuint  depth;        /**< Depth renderbuffer, used only when samples == 0. */

    GLsizei width;        /**< Allocated width in pixels. */
    GLsizei height;       /**< Allocated height in pixels. */
    GLint   samples;      /**< Effective sample count; 0 means no multisampling. */
} rbt_target_t;

/**
 * @brief Make the target match @p width by @p height at @p samples.
 *
 * Reallocates only when the request differs from what is allocated, so calling
 * this every frame costs nothing while the panel keeps its size. A request
 * that GL refuses is an operational failure: it is logged, the target is left
 * empty and false comes back.
 *
 * @param samples Requested sample count; 0 or 1 disables multisampling, and
 *                anything above GL_MAX_SAMPLES is clamped to it.
 * @return true when the target is complete and ready to render into.
 */
bool rbt_target_resize(rbt_target_t *target, GLsizei width, GLsizei height, GLint samples);

/** @brief Release every GL object the target owns. Safe on an empty target. */
void rbt_target_destroy(rbt_target_t *target);

/**
 * @brief Bind the target and set the viewport to cover it.
 * Pre: the last rbt_target_resize succeeded.
 */
void rbt_target_begin(const rbt_target_t *target);

/**
 * @brief Resolve multisampling if needed and rebind the default framebuffer.
 * Post: @ref rbt_target_t::texture holds the rendered image.
 */
void rbt_target_end(const rbt_target_t *target);
