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

#include "ccGLUtils.h"

//Local
#include "ccLog.h"

//CCLib
#include <CCConst.h>

//system
#include <assert.h>

//*********** OPENGL TEXTURES ***********//

void _DisplayTexture2D(ccQOpenGLFunctions* glFunc, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
	assert(glFunc);
	
	glFunc->glPushAttrib(GL_ENABLE_BIT);
	glFunc->glEnable(GL_TEXTURE_2D);

	glFunc->glColor4ub(255, 255, 255, alpha);
	glFunc->glBegin(GL_QUADS);
	glFunc->glTexCoord2f(0.0, 1.0);
	glFunc->glVertex2i(x, y + h);
	glFunc->glTexCoord2f(0.0, 0.0);
	glFunc->glVertex2i(x, y);
	glFunc->glTexCoord2f(1.0, 0.0);
	glFunc->glVertex2i(x + w, y);
	glFunc->glTexCoord2f(1.0, 1.0);
	glFunc->glVertex2i(x + w, y + h);
	glFunc->glEnd();

	glFunc->glBindTexture(GL_TEXTURE_2D, 0);
	glFunc->glPopAttrib();
}

void ccGLUtils::DisplayTexture2DPosition(ccQGLContext* context, QPixmap pixmap, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
	if (!context)
	{
		assert(false);
		return;
	}

#ifndef USE_QtOpenGL_CLASSES
	context->bindTexture(pixmap, GL_TEXTURE_2D);
#else
	QOpenGLTexture texture(pixmap.toImage());
	texture.bind();
#endif

	ccQOpenGLFunctions* glFunc = context->versionFunctions<ccQOpenGLFunctions>();

	_DisplayTexture2D(glFunc, x, y, w, h, alpha);
}

void ccGLUtils::DisplayTexture2D(ccQGLContext* context, QPixmap pixmap, int w, int h, unsigned char alpha/*=255*/)
{
	DisplayTexture2DPosition(context, pixmap, -w / 2, -h / 2, w, h, alpha);
}

void ccGLUtils::DisplayTexture2DPosition(ccQGLContext* context, GLuint texID, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
	if (!context)
	{
		assert(false);
		return;
	}
	ccQOpenGLFunctions* glFunc = context->versionFunctions<ccQOpenGLFunctions>();
	glFunc->glBindTexture(GL_TEXTURE_2D, texID);

	_DisplayTexture2D(glFunc, x, y, w, h, alpha);

	glFunc->glBindTexture(GL_TEXTURE_2D, 0);
}

void ccGLUtils::DisplayTexture2D(ccQGLContext* context, GLuint texID, int w, int h, unsigned char alpha/*=255*/)
{
	DisplayTexture2DPosition(context, texID, -w / 2, -h / 2, w, h, alpha);
}

//*********** OPENGL MATRICES ***********//

ccGLMatrixd ccGLUtils::GenerateViewMat(CC_VIEW_ORIENTATION orientation)
{
	CCVector3d eye(0,0,0);
	CCVector3d center(0,0,0);
	CCVector3d top(0,0,0);

	//we look at (0,0,0) by default
	switch (orientation)
	{
	case CC_TOP_VIEW:
		eye.z =  1.0;
		top.y =  1.0;
		break;
	case CC_BOTTOM_VIEW:
		eye.z = -1.0;
		top.y =  1.0;
		break;
	case CC_FRONT_VIEW:
		eye.y = -1.0;
		top.z =  1.0;
		break;
	case CC_BACK_VIEW:
		eye.y =  1.0;
		top.z =  1.0;
		break;
	case CC_LEFT_VIEW:
		eye.x = -1.0;
		top.z =  1.0;
		break;
	case CC_RIGHT_VIEW:
		eye.x =  1.0;
		top.z =  1.0;
		break;
	case CC_ISO_VIEW_1:
		eye.x = -1.0;
		eye.y = -1.0;
		eye.z =  1.0;
		top.x =  1.0;
		top.y =  1.0;
		top.z =  1.0;
		break;
	case CC_ISO_VIEW_2:
		eye.x =  1.0;
		eye.y =  1.0;
		eye.z =  1.0;
		top.x = -1.0;
		top.y = -1.0;
		top.z =  1.0;
		break;
	}

	return ccGLMatrixd::FromViewDirAndUpDir(center-eye,top);
}

bool ccGLUtils::CatchGLError(GLenum err, const char* context)
{
	//see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
	switch(err)
	{
	case GL_NO_ERROR:
		return false;
		break;
	case GL_INVALID_ENUM:
		ccLog::Warning("[%s] OpenGL error: invalid enumerator",context);
		break;
	case GL_INVALID_VALUE:
		ccLog::Warning("[%s] OpenGL error: invalid value",context);
		break;
	case GL_INVALID_OPERATION:
		ccLog::Warning("[%s] OpenGL error: invalid operation",context);
		break;
	case GL_STACK_OVERFLOW:
		ccLog::Error("[%s] OpenGL error: stack overflow",context);
		break;
	case GL_STACK_UNDERFLOW:
		ccLog::Error("[%s] OpenGL error: stack underflow",context);
		break;
	case GL_OUT_OF_MEMORY:
		ccLog::Error("[%s] OpenGL error: out of memory",context);
		break;
	case GL_INVALID_FRAMEBUFFER_OPERATION:
		ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation",context);
		break;
	}

	return true;
}

