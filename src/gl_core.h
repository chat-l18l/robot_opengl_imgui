/**
 * @file gl_core.h
 * @brief OpenGL 3.3 core entry point, without a loader library.
 *
 * On Linux with mesa-common-dev installed, libGL.so exports every GL 3.3
 * core function as a linkable symbol, so defining GL_GLEXT_PROTOTYPES and
 * linking -lGL replaces glad/glew entirely.
 *
 * This header must be included before any other GL-using header.
 */

#pragma once

/* Ask glext.h for real prototypes instead of function-pointer typedefs. */
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#include <GL/gl.h>
#include <GL/glext.h>
