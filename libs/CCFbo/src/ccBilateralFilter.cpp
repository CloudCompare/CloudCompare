//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccBilateralFilter.h"

//system
#include <algorithm>
#include <assert.h>
#include <cmath>

//! Max kernel size
/** Can't add this as a static const definition (in the header file)
as this causes a 'double definition' link error on Windows - if
the definition is done in the cpp file - or a 'missing definition'
link error on MacOS - if the definition is not done in the cpp file ;)
**/
#define KERNEL_MAX_HALF_SIZE 7 // (KERNEL_MAX_HALF_SIZE+1) * (KERNEL_MAX_HALF_SIZE+1) = 64

ccBilateralFilter::ccBilateralFilter()
	: ccGlFilter("Bilateral smooth")
	, m_width(0)
	, m_height(0)
	, m_halfSpatialSize(0)
	, m_spatialSigma(0)
	, m_depthSigma(0)
	, m_dampingPixelDist(64, 0)
	, m_useCurrentViewport(false)
	, m_glFuncIsValid(false)
{
	setParams(2, 2.0f, 0.4f);
}

ccGlFilter* ccBilateralFilter::clone() const
{
	ccBilateralFilter* filter = new ccBilateralFilter();
	//copy parameters
	filter->setParams(m_halfSpatialSize, m_spatialSigma, m_depthSigma);
	filter->m_useCurrentViewport = m_useCurrentViewport;

	return filter;
}

void ccBilateralFilter::useExistingViewport(bool state)
{
	m_useCurrentViewport = state;
}

void ccBilateralFilter::reset()
{
	m_shader.removeAllShaders();
	m_fbo.reset();
	m_width = m_height = 0;
}

bool ccBilateralFilter::init(unsigned width, unsigned height, QString shadersPath, QString& error)
{
	if (width == 0 || height == 0)
	{
		error = "[Bilateral] Null texture size";
		return false;
	}

	if (!m_glFuncIsValid)
	{
		if (!m_glFunc.initializeOpenGLFunctions())
		{
			return false;
		}
		m_glFuncIsValid = true;
	}

	setValid(false);

	if (!m_fbo.init(static_cast<unsigned>(width), static_cast<unsigned>(height)))
	{
		error = "[Bilateral] Can't initialize FBO";
		reset();
		return false;
	}

	if (!m_fbo.start())
	{
		return false;
	}
	
	if (!m_fbo.initColor(GL_RGB, GL_RGB, GL_FLOAT))
	{
		return false;
	}
	m_fbo.stop();

	if (m_shader.shaders().isEmpty())
	{
		if (!m_shader.fromFile(shadersPath, "Bilateral/bilateral", error))
		{
			error = "[Bilateral] Can't load bilateral shaders";
			reset();
			return false;
		}
	}

	m_width = width;
	m_height = height;

	setValid(true);

	return true;
}

void ccBilateralFilter::setParams(unsigned halfSpatialSize, float spatialSigma, float depthSigma)
{
	m_halfSpatialSize	= std::min<unsigned>(halfSpatialSize,KERNEL_MAX_HALF_SIZE);
	m_spatialSigma		= spatialSigma;
	m_depthSigma		= depthSigma;

	updateDampingTable();
}

void ccBilateralFilter::shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters)
{
	if (!isValid())
	{
		return;
	}

	if (!m_fbo.isValid() || !m_shader.isLinked())
	{
		return;
	}

	if (!m_useCurrentViewport)
	{
		//we must use corner-based screen coordinates
		m_glFunc.glMatrixMode(GL_PROJECTION);
		m_glFunc.glPushMatrix();
		m_glFunc.glLoadIdentity();
		m_glFunc.glOrtho(0.0, static_cast<GLdouble>(m_width), 0.0, static_cast<GLdouble>(m_height), 0.0, 1.0);
		m_glFunc.glMatrixMode(GL_MODELVIEW);
		m_glFunc.glPushMatrix();
		m_glFunc.glLoadIdentity();
		assert(m_glFunc.glGetError() == GL_NO_ERROR);
	}

	//	HORIZONTAL
	m_fbo.start();

	m_shader.bind();
	m_shader.setUniformValue("s2_I", 0);	// image to blur
	m_shader.setUniformValue("s2_D", 1);	// image to modulate filter
	m_shader.setUniformValue("SX", static_cast<float>(m_width));
	m_shader.setUniformValue("SY", static_cast<float>(m_height));
	m_shader.setUniformValue("NHalf", m_halfSpatialSize);
	m_shader.setUniformValueArray("DistCoefs", m_dampingPixelDist.data(), 64, 1);
	m_shader.setUniformValue("SigmaDepth", m_depthSigma);

	//Texture 1 --> 2D
	m_glFunc.glActiveTexture(GL_TEXTURE1);
	m_glFunc.glBindTexture(GL_TEXTURE_2D, texDepth);

	//Texture 0 --> 2D
	m_glFunc.glActiveTexture(GL_TEXTURE0);
	m_glFunc.glBindTexture(GL_TEXTURE_2D, texColor);

	m_glFunc.glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	m_glFunc.glBegin(GL_QUADS);
	m_glFunc.glTexCoord2f(0.0f, 0.0f);
	m_glFunc.glVertex2i(0, 0);
	m_glFunc.glTexCoord2f(1.0f, 0.0f);
	m_glFunc.glVertex2i(m_width, 0);
	m_glFunc.glTexCoord2f(1.0f, 1.0f);
	m_glFunc.glVertex2i(m_width, m_height);
	m_glFunc.glTexCoord2f(0.0f, 1.0f);
	m_glFunc.glVertex2i(0, m_height);
	m_glFunc.glEnd();

	m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

	//Texture 0 --> 2D
	//m_glFunc.glActiveTexture(GL_TEXTURE0);
	//m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

	//Texture 1 --> 2D
	m_glFunc.glActiveTexture(GL_TEXTURE1);
	m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

	m_shader.release();
	m_fbo.stop();

	//restore GL_TEXTURE_0 by default
	m_glFunc.glActiveTexture(GL_TEXTURE0);

	if (!m_useCurrentViewport)
	{
		m_glFunc.glMatrixMode(GL_PROJECTION);
		m_glFunc.glPopMatrix();
		m_glFunc.glMatrixMode(GL_MODELVIEW);
		m_glFunc.glPopMatrix();
	}
	assert(m_glFunc.glGetError() == GL_NO_ERROR);
}

void ccBilateralFilter::updateDampingTable()
{
	assert(m_halfSpatialSize <= KERNEL_MAX_HALF_SIZE);

	//constant quotient
	float q = m_halfSpatialSize * m_spatialSigma;
	q = 2 * (q*q);

	for (unsigned c = 0; c <= m_halfSpatialSize; c++)
	{
		for (unsigned d = 0; d <= m_halfSpatialSize; d++)
		{
			//pixel distance based damping
			m_dampingPixelDist[c*(m_halfSpatialSize + 1) + d] = std::exp((c*c + d*d) / (-q));
		}
	}
}
