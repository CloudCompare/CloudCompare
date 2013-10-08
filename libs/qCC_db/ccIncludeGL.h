//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_INCLUDE_GL_HEADER
#define CC_INCLUDE_GL_HEADER

//Local
#include <ccPlatform.h>

//GLEW (if needed, must be included first)
#ifdef USE_GLEW
#include <GL/glew.h>
#endif

#include <QGLContext>
#include <QGLWidget>
#include <QGLFormat>

#ifdef CC_MAC_OS
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#ifndef GL_INVALID_TEXTURE_ID
#define GL_INVALID_TEXTURE_ID 0xffffffff
#endif

#endif //CC_INCLUDE_GL_HEADER
