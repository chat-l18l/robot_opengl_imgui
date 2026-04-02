// ============================================================
// gl_core.h — OpenGL 3.3 Core definities (geen glad nodig)
//
// Op Linux (Ubuntu) met mesa-common-dev geïnstalleerd,
// exporteert libGL.so alle GL 3.3 core functies als
// gelinkte symbolen. We hoeven alleen de headers te
// includen en te linken met -lGL.
// ============================================================
#pragma once

// Zorg dat GL extension functies prototypes krijgen
// (i.p.v. alleen function pointer typedefs)
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#include <GL/gl.h>
#include <GL/glext.h>
