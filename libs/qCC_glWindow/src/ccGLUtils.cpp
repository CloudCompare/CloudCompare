// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccGLUtils.h"

#include <QOpenGLTexture>
#include <QOpenGLVersionFunctionsFactory>

//*********** OPENGL TEXTURES ***********//

void ccGLUtils::DisplayTexture2DPosition(QImage image, int x, int y, int w, int h, unsigned char alpha /*=255*/, TextureArea area /*=Full*/)
{
	QOpenGLTexture texture(image);

	DisplayTexture2DPosition(texture.textureId(), x, y, w, h, alpha, area);
}

void ccGLUtils::DisplayTexture2DPosition(GLuint texID, int x, int y, int w, int h, unsigned char alpha /*=255*/, TextureArea area /*=Full*/)
{
	QOpenGLContext* context = QOpenGLContext::currentContext();
	if (!context)
	{
		assert(false);
		return;
	}
	auto* glFunc = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(context);
	if (glFunc)
	{
		glFunc->glBindTexture(GL_TEXTURE_2D, texID);

		glFunc->glPushAttrib(GL_ENABLE_BIT);
		glFunc->glEnable(GL_TEXTURE_2D);

		glFunc->glColor4ub(255, 255, 255, alpha);
		glFunc->glBegin(GL_QUADS);
		switch (area)
		{
		case Full:
		default:
			glFunc->glTexCoord2f(0.0, 1.0);
			glFunc->glVertex2i(x, y + h);
			glFunc->glTexCoord2f(0.0, 0.0);
			glFunc->glVertex2i(x, y);
			glFunc->glTexCoord2f(1.0, 0.0);
			glFunc->glVertex2i(x + w, y);
			glFunc->glTexCoord2f(1.0, 1.0);
			glFunc->glVertex2i(x + w, y + h);
			break;

		case HalfLeft:
			glFunc->glTexCoord2f(0.0, 1.0);
			glFunc->glVertex2i(x, y + h);
			glFunc->glTexCoord2f(0.0, 0.0);
			glFunc->glVertex2i(x, y);
			glFunc->glTexCoord2f(0.5, 0.0);
			glFunc->glVertex2i(x + w, y);
			glFunc->glTexCoord2f(0.5, 1.0);
			glFunc->glVertex2i(x + w, y + h);
			break;

		case HalfRight:
			glFunc->glTexCoord2f(0.5, 1.0);
			glFunc->glVertex2i(x, y + h);
			glFunc->glTexCoord2f(0.5, 0.0);
			glFunc->glVertex2i(x, y);
			glFunc->glTexCoord2f(1.0, 0.0);
			glFunc->glVertex2i(x + w, y);
			glFunc->glTexCoord2f(1.0, 1.0);
			glFunc->glVertex2i(x + w, y + h);
			break;
		}
		glFunc->glEnd();

		glFunc->glPopAttrib();

		glFunc->glBindTexture(GL_TEXTURE_2D, 0);
	}
}

//*********** OPENGL MATRICES ***********//

ccGLMatrixd ccGLUtils::GenerateViewMat(CC_VIEW_ORIENTATION orientation,
                                       const CCVector3d&   vertDir /*=CCVector3d(0, 0, 1)*/,
                                       double*             _vertAngle_rad /*=nullptr*/,
                                       double*             _orthoAngle_rad /*=nullptr*/)
{
	double vertAngle_rad  = 0.0;
	double orthoAngle_rad = 0.0;

	switch (orientation)
	{
	case CC_TOP_VIEW:
		vertAngle_rad  = 0.0;
		orthoAngle_rad = M_PI_2;
		break;
	case CC_BOTTOM_VIEW:
		vertAngle_rad  = -M_PI;
		orthoAngle_rad = -M_PI_2;
		break;
	case CC_FRONT_VIEW:
		vertAngle_rad  = 0.0;
		orthoAngle_rad = 0.0;
		break;
	case CC_BACK_VIEW:
		vertAngle_rad  = -M_PI;
		orthoAngle_rad = 0.0;
		break;
	case CC_LEFT_VIEW:
		vertAngle_rad  = M_PI_2;
		orthoAngle_rad = 0.0;
		break;
	case CC_RIGHT_VIEW:
		vertAngle_rad  = -M_PI_2;
		orthoAngle_rad = 0.0;
		break;
	case CC_ISO_VIEW_1:
		vertAngle_rad  = M_PI / 4;
		orthoAngle_rad = M_PI / 4;
		break;
	case CC_ISO_VIEW_2:
		vertAngle_rad  = -M_PI / 4;
		orthoAngle_rad = -M_PI / 4;
		break;
	default:
		ccLog::Warning("Internal error: unhandled view mode");
		if (_vertAngle_rad)
		{
			*_vertAngle_rad = std::numeric_limits<double>::quiet_NaN();
		}
		if (_orthoAngle_rad)
		{
			*_orthoAngle_rad = std::numeric_limits<double>::quiet_NaN();
		}
		return {};
	}

	if (_vertAngle_rad)
	{
		*_vertAngle_rad = vertAngle_rad;
	}
	if (_orthoAngle_rad)
	{
		*_orthoAngle_rad = orthoAngle_rad;
	}

	return GenerateViewMat(vertDir, vertAngle_rad, orthoAngle_rad);
}

ccGLMatrixd ccGLUtils::GenerateViewMat(const CCVector3d& vertDir,
                                       double            vertAngle_rad,
                                       double            orthoAngle_rad)
{
	ccGLMatrixd vertRot;
	vertRot.initFromParameters(vertAngle_rad, vertDir, CCVector3d(0, 0, 0));
	ccGLMatrixd horizRot;
	horizRot.initFromParameters(orthoAngle_rad, CCVector3d(1, 0, 0), CCVector3d(0, 0, 0));

	CCVector3d orthoDir = vertDir.orthogonal(); // returns +Y if vertDir = +Z

	ccGLMatrixd viewMat = horizRot * ccGLMatrixd::FromViewDirAndUpDir(orthoDir, vertDir) * vertRot;

	return viewMat;
}
