/**
 * @file gl_target.cpp
 * @brief Offscreen render target allocation and resolve.
 */

#include "gl_target.h"

#include <assert.h>
#include <stdio.h>

/** Upper bound on either dimension, so an absurd panel size cannot ask GL for an absurd allocation. */
static const GLsizei s_max_dimension = 8192;

/** @brief Clamp a requested sample count to what this implementation supports. */
static GLint s_clamp_samples(GLint samples)
{
    if (samples <= 1) {
        return 0;
    }

    GLint max_samples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &max_samples);
    return samples < max_samples ? samples : max_samples;
}

/** @brief Describe a framebuffer status code for a log line. */
static const char *s_status_name(GLenum status)
{
    switch (status) {
    case GL_FRAMEBUFFER_COMPLETE:                      return "complete";
    case GL_FRAMEBUFFER_UNDEFINED:                     return "undefined";
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:         return "incomplete attachment";
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: return "missing attachment";
    case GL_FRAMEBUFFER_UNSUPPORTED:                   return "unsupported format";
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:        return "incomplete multisample";
    default:                                           return "unknown";
    }
}

/**
 * @brief Build the single-sample framebuffer and its texture.
 *
 * The depth renderbuffer is attached here only when the scene renders straight
 * into this framebuffer; with multisampling the depth buffer lives beside the
 * multisampled colour buffer instead.
 */
static bool s_create_resolve(rbt_target_t *target, bool needs_depth)
{
    glGenTextures(1, &target->texture);
    glBindTexture(GL_TEXTURE_2D, target->texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, target->width, target->height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, &target->resolve_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, target->resolve_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target->texture, 0);

    if (needs_depth) {
        glGenRenderbuffers(1, &target->depth);
        glBindRenderbuffer(GL_RENDERBUFFER, target->depth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, target->width, target->height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, target->depth);
    }

    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "target: resolve framebuffer is %s\n", s_status_name(status));
        return false;
    }
    return true;
}

/** @brief Build the multisampled framebuffer the scene is drawn into. */
static bool s_create_msaa(rbt_target_t *target)
{
    glGenFramebuffers(1, &target->msaa_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, target->msaa_fbo);

    glGenRenderbuffers(1, &target->msaa_color);
    glBindRenderbuffer(GL_RENDERBUFFER, target->msaa_color);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, target->samples, GL_RGB8,
                                     target->width, target->height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target->msaa_color);

    glGenRenderbuffers(1, &target->msaa_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, target->msaa_depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, target->samples, GL_DEPTH_COMPONENT24,
                                     target->width, target->height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, target->msaa_depth);

    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "target: multisampled framebuffer is %s\n", s_status_name(status));
        return false;
    }
    return true;
}

bool rbt_target_resize(rbt_target_t *target, GLsizei width, GLsizei height, GLint samples)
{
    assert(target != NULL);

    if (width < 1 || height < 1) {
        return false;
    }
    if (width > s_max_dimension) {
        width = s_max_dimension;
    }
    if (height > s_max_dimension) {
        height = s_max_dimension;
    }

    const GLint wanted_samples = s_clamp_samples(samples);
    if (target->texture != 0
        && target->width == width
        && target->height == height
        && target->samples == wanted_samples) {
        return true;
    }

    rbt_target_destroy(target);
    target->width   = width;
    target->height  = height;
    target->samples = wanted_samples;

    const bool ok = s_create_resolve(target, wanted_samples == 0)
                    && (wanted_samples == 0 || s_create_msaa(target));

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    if (!ok) {
        rbt_target_destroy(target);
        return false;
    }
    return true;
}

void rbt_target_destroy(rbt_target_t *target)
{
    assert(target != NULL);

    if (target->msaa_color != 0) {
        glDeleteRenderbuffers(1, &target->msaa_color);
    }
    if (target->msaa_depth != 0) {
        glDeleteRenderbuffers(1, &target->msaa_depth);
    }
    if (target->depth != 0) {
        glDeleteRenderbuffers(1, &target->depth);
    }
    if (target->msaa_fbo != 0) {
        glDeleteFramebuffers(1, &target->msaa_fbo);
    }
    if (target->resolve_fbo != 0) {
        glDeleteFramebuffers(1, &target->resolve_fbo);
    }
    if (target->texture != 0) {
        glDeleteTextures(1, &target->texture);
    }

    target->msaa_fbo    = 0;
    target->msaa_color  = 0;
    target->msaa_depth  = 0;
    target->resolve_fbo = 0;
    target->texture     = 0;
    target->depth       = 0;
    target->width       = 0;
    target->height      = 0;
    target->samples     = 0;
}

void rbt_target_begin(const rbt_target_t *target)
{
    assert(target != NULL);
    assert(target->texture != 0);

    glBindFramebuffer(GL_FRAMEBUFFER, target->samples > 0 ? target->msaa_fbo : target->resolve_fbo);
    glViewport(0, 0, target->width, target->height);
}

void rbt_target_end(const rbt_target_t *target)
{
    assert(target != NULL);
    assert(target->texture != 0);

    if (target->samples > 0) {
        /* A multisampled blit demands matching rectangles and GL_NEAREST. */
        glBindFramebuffer(GL_READ_FRAMEBUFFER, target->msaa_fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target->resolve_fbo);
        glBlitFramebuffer(0, 0, target->width, target->height,
                          0, 0, target->width, target->height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
