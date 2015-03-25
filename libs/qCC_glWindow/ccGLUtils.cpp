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

//*********** OPENGL TEXTURES ***********//

void ccGLUtils::DisplayTexture2DPosition(GLuint tex, int x, int y, int w, int h, uchar alpha/*=255*/)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);

	glColor4ub(255, 255, 255, alpha);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0,1.0);
	glVertex2i(x, y+h);
	glTexCoord2f(0.0,0.0);
	glVertex2i(x, y);
	glTexCoord2f(1.0,0.0);
	glVertex2i(x+w, y);
	glTexCoord2f(1.0,1.0);
	glVertex2i(x+w, y+h);
	glEnd();

	glBindTexture(GL_TEXTURE_2D,0);
	glDisable(GL_TEXTURE_2D);
}

void ccGLUtils::DisplayTexture2D(GLuint tex, int w, int h, uchar alpha/*=255*/)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);

	float halfW = static_cast<float>(w)/2;
	float halfH = static_cast<float>(h)/2;

	glColor4ub(255, 255, 255, alpha);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0);
	glVertex2f(-halfW, -halfH);
	glTexCoord2f(1, 0);
	glVertex2f( halfW, -halfH);
	glTexCoord2f(1, 1);
	glVertex2f( halfW,  halfH);
	glTexCoord2f(0, 1);
	glVertex2f(-halfW,  halfH);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
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

	return ccGLMatrixd::LookAt(eye,center,top);
}

bool ccGLUtils::CatchGLError(const char* context)
{
	GLenum err = glGetError();

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

