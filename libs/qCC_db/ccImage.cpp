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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2219                                                              $
//$LastChangedDate:: 2012-07-20 18:03:24 +0200 (ven., 20 juil. 2012)       $
//**************************************************************************
//

#include "ccImage.h"

#include "ccIncludeGL.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QImageReader>
#include <QFileInfo>

//System
#include <assert.h>

ccImage::ccImage()
	: ccHObject("Not loaded")
	, m_width(0)
	, m_height(0)
	, m_aspectRatio(1.0f)
	, m_texU(1.0f)
	, m_texV(1.0f)
	, m_texAlpha(1.0f)
	, m_textureID(0)
	, m_boundWin(0)
#ifdef INCLUDE_IMAGE_FILENAME
	, m_completeFileName("")
#endif
{
	setVisible(true);
    lockVisibility(false);
	setEnabled(false);
}

ccImage::ccImage(const QImage& image, const QString& name)
	: ccHObject(name)
	, m_width(image.width())
	, m_height(image.height())
	, m_texU(1.0f)
	, m_texV(1.0f)
	, m_texAlpha(1.0f)
	, m_textureID(0)
	, m_boundWin(0)
	, m_image(image)
#ifdef INCLUDE_IMAGE_FILENAME
	, m_completeFileName("")
#endif
{
	setVisible(true);
    lockVisibility(false);
	setEnabled(true);
}

bool ccImage::load(const QString& filename, QString& error)
{
	QImageReader reader(filename);
	//m_image = QImage(filename);
	QImage image = reader.read();
	if (image.isNull())
	{
		error = reader.errorString();
		return false;
	}

	setImage(image);

	setName(QFileInfo(filename).fileName());
	setEnabled(true);

#ifdef INCLUDE_IMAGE_FILENAME
	setCompleteFileName(filename);
#endif

	return true;
}

void ccImage::setImage(const QImage& image)
{
	//previous image?
	if (!m_image.isNull())
		unbindTexture();

	m_image = image;
	m_width = m_image.width();
	m_height = m_image.height();
	setAspectRatio(m_height>0 ? (float)m_width/(float)m_height : 1.0f);

	//default behavior (this will be updated later, depending
	//on the OpenGL version of the bound QGLWidget)
	m_texU = 1.0;
	m_texV = 1.0;
}

bool ccImage::unbindTexture()
{
    if (!m_boundWin || !m_textureID)
        return false;

	m_boundWin->releaseTexture(m_textureID);

	m_textureID=0;
    m_boundWin=0;

    return true;
}

bool ccImage::bindToGlTexture(ccGenericGLDisplay* win, bool pow2Texture/*=false*/)
{
	assert(win);

	if (m_image.isNull())
        return false;

    if (!m_textureID || m_boundWin!=win)
    {
        if (m_textureID && m_boundWin!=win)
            unbindTexture();

        m_boundWin = win;
		m_textureID = m_boundWin->getTexture(m_image);

		//OpenGL version < 2.0 require texture with 2^n width & height
		if (!win->supportOpenGLVersion(QGLFormat::OpenGL_Version_2_0)
			&& glewIsSupported("GL_ARB_texture_non_power_of_two")==0)
		{
			// update nearest smaller power of 2 (for textures with old OpenGL versions)
			unsigned paddedWidth = (m_width > 0 ? 1 << (unsigned)floor(log((double)m_width)/log(2.0)) : 0);
			unsigned paddedHeight = (m_height > 0 ? 1 << (unsigned)floor(log((double)m_height)/log(2.0)) : 0);
			m_texU = float(m_width)/float(paddedWidth);
			m_texV = float(m_height)/float(paddedHeight);
		}
		else
		{
			m_texU = 1.0;
			m_texV = 1.0;
		}
    }

	if (m_textureID != GL_INVALID_TEXTURE_ID)
	{
		glBindTexture( GL_TEXTURE_2D, m_textureID );
		return true;
	}

	return false;
}

void ccImage::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_image.isNull())
        return;

    if (MACRO_Draw2D(context))
    {
        if (MACRO_Foreground(context))
        {
            glPushAttrib(GL_COLOR_BUFFER_BIT);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            glEnable(GL_TEXTURE_2D);

            if (bindToGlTexture(context._win))
			{
				//we make the texture fit inside viewport
				int realWidth = (int)((float)m_height * m_aspectRatio); //take aspect ratio into account!
				GLfloat cw = GLfloat(context.glW)/GLfloat(realWidth);
				GLfloat ch = GLfloat(context.glH)/GLfloat(m_height);
				GLfloat zoomFactor = (cw > ch ? ch : cw)*0.5f;
				GLfloat dX = GLfloat(realWidth)*zoomFactor;
				GLfloat dY = GLfloat(m_height)*zoomFactor;

				glColor4f(1, 1, 1, m_texAlpha);
				glBegin(GL_QUADS);
				glTexCoord2f(0,m_texV);		glVertex2f(-dX, -dY);
				glTexCoord2f(m_texU,m_texV);	glVertex2f(dX, -dY);
				glTexCoord2f(m_texU,0);		glVertex2f(dX, dY);
				glTexCoord2f(0,0);		glVertex2f(-dX, dY);
				glEnd();
			}

            glDisable(GL_TEXTURE_2D);

            glPopAttrib();
        }
    }
}

void ccImage::setAlpha(float value)
{
    if (value<=0)
        m_texAlpha=0;
    else if (value>1.0f)
        m_texAlpha=1.0f;
    else
        m_texAlpha = value;
}

#ifdef INCLUDE_IMAGE_FILENAME
void ccImage::setCompleteFileName(const char* name)
{
	strcpy(completeFileName,name);
}
#endif
