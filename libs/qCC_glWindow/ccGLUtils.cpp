//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccGLUtils.h"

#include <QOpenGLTexture>

//*********** OPENGL TEXTURES ***********//

void ccGLUtils::DisplayTexture2DPosition(QImage image, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
	QOpenGLTexture texture(image);

	DisplayTexture2DPosition(texture.textureId(), x, y, w, h, alpha);
}

void ccGLUtils::DisplayTexture2DPosition(GLuint texID, int x, int y, int w, int h, unsigned char alpha/*=255*/)
{
	QOpenGLContext* context = QOpenGLContext::currentContext();
	if (!context)
	{
		assert(false);
		return;
	}
	QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
	if (glFunc)
	{
		glFunc->glBindTexture(GL_TEXTURE_2D, texID);

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

		glFunc->glPopAttrib();

		glFunc->glBindTexture(GL_TEXTURE_2D, 0);
	}
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
