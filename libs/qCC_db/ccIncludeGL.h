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

//CCLib
#include <CCPlatform.h>

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

//! Shortcuts to OpenGL commands independent on the input type
class ccGL
{
public:
	//type-less glVertex3Xv call (X=f,d)
	static inline void Vertex3v(const float* v) { glVertex3fv(v); }
	static inline void Vertex3v(const double* v) { glVertex3dv(v); }

	//type-less glVertex3X call (X=f,d)
	static inline void Vertex3(float x, float y, float z) { glVertex3f(x,y,z); }
	static inline void Vertex3(double x, double y, double z) { glVertex3d(x,y,z); }

	//type-less glScaleX call (X=f,d)
	static inline void Scale(float x, float y, float z) { glScalef(x,y,z); }
	static inline void Scale(double x, double y, double z) { glScaled(x,y,z); }

	//type-less glNormal3Xv call (X=f,d)
	static inline void Normal3v(const float* v) { glNormal3fv(v); }
	static inline void Normal3v(const double* v) { glNormal3dv(v); }

	//type-less glRotateX call (X=f,d)
	static inline void Rotate(float a, float x, float y, float z) { glRotatef(a,x,y,z); }
	static inline void Rotate(double a, double x, double y, double z) { glRotated(a,x,y,z); }

	//type-less glTranslateX call (X=f,d)
	static inline void Translate(float x, float y, float z) { glTranslatef(x,y,z); }
	static inline void Translate(double x, double y, double z) { glTranslated(x,y,z); }
};

#endif //CC_INCLUDE_GL_HEADER
