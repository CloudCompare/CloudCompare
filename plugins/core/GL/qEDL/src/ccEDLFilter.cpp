//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qEDL                        #
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

#include "ccEDLFilter.h"

//ccFBO
#include <ccFrameBufferObject.h>
#include <ccBilateralFilter.h>
#include <ccShader.h>
//qCC_gl
#include <ccGLUtils.h>

//Qt
#include <QOpenGLContext>

//system
#include <assert.h>
#include <cmath>

//For MSVC
#ifndef M_PI
#define M_PI 3.141592653589793238462643
#endif

ccEDLFilter::ccEDLFilter()
	: ccGlFilter("EyeDome Lighting (disable normals and increase points size for a better result!)")
	, m_screenWidth(0)
	, m_screenHeight(0)
	, m_EDLShader(nullptr)
	, m_fboMix(nullptr)
	, m_mixShader(nullptr)
	, m_expScale(100.0f)
	, m_glFuncIsValid(false)
{
	for (unsigned i = 0; i < FBO_COUNT; ++i)
	{
		m_fbos[i] = nullptr;
	}

	//smoothing filter for full resolution
	m_bilateralFilters[0].enabled  = false;
	m_bilateralFilters[0].halfSize = 1;
	m_bilateralFilters[0].sigma    = 1.0f;
	m_bilateralFilters[0].sigmaZ   = 0.2f;

	//smoothing filter for half resolution
	m_bilateralFilters[1].enabled  = true;
	m_bilateralFilters[1].halfSize = 2;
	m_bilateralFilters[1].sigma    = 2.0f;
	m_bilateralFilters[1].sigmaZ   = 0.4f;

	//smoothing filter for quarter resolution
	m_bilateralFilters[2].enabled  = true;
	m_bilateralFilters[2].halfSize = 2;
	m_bilateralFilters[2].sigma    = 2.0f;
	m_bilateralFilters[2].sigmaZ   = 0.4f;

	setLightDir(static_cast<float>(M_PI / 2.0), static_cast<float>(M_PI / 2.0));

	memset(m_neighbours, 0, sizeof(float) * 8 * 2);
	for (unsigned c = 0; c < 8; c++)
	{
		m_neighbours[2 * c]     = static_cast<float>(std::cos(c * M_PI / 4.0));
		m_neighbours[2 * c + 1] = static_cast<float>(std::sin(c * M_PI / 4.0));
	}
}

ccEDLFilter::~ccEDLFilter()
{
	reset();
}

ccGlFilter* ccEDLFilter::clone() const
{
	ccEDLFilter* filter = new ccEDLFilter();

	//copy parameters (only those that can be changed by the user!)
	filter->setStrength(m_expScale);
	filter->m_lightDir[0] = m_lightDir[0];
	filter->m_lightDir[1] = m_lightDir[1];
	filter->m_lightDir[2] = m_lightDir[2];

	return filter;
}

void ccEDLFilter::reset()
{
	for (unsigned i = 0; i < FBO_COUNT; ++i)
	{
		if (m_fbos[i])
		{
			delete m_fbos[i];
			m_fbos[i] = nullptr;
		}

		if (m_bilateralFilters[i].filter)
		{
			delete m_bilateralFilters[i].filter;
			m_bilateralFilters[i].filter = nullptr;
		}
	}

	if (m_fboMix)
		delete m_fboMix;
	m_fboMix = nullptr;

	if (m_EDLShader)
		delete m_EDLShader;
	m_EDLShader = nullptr;

	if (m_mixShader)
		delete m_mixShader;
	m_mixShader = nullptr;

	m_screenWidth = m_screenHeight = 0;
}

bool ccEDLFilter::init(unsigned width, unsigned height, QString shadersPath, QString& error)
{
	return init(width, height, GL_RGBA, GL_LINEAR, shadersPath, error);
}

bool ccEDLFilter::init(unsigned width, unsigned height, GLenum internalFormat, GLenum minMagFilter, QString shadersPath, QString& error)
{
	if (width == 0 || height == 0)
	{
		error = "Invalid texture size";
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

	for (unsigned i = 0; i < FBO_COUNT; ++i)
	{
		unsigned scale = (1 << i);
		unsigned w = width / scale;
		unsigned h = height / scale;

		ccFrameBufferObject* &fbo = m_fbos[i];
		if (!fbo)
		{
			fbo = new ccFrameBufferObject();
		}
		if (	!fbo->init(w, h)
			||	!fbo->initColor(internalFormat, GL_RGBA, GL_FLOAT, minMagFilter))
		{
			error = QString("[EDL Filter] FBO 1:%1 initialization failed!").arg(scale);
			reset();
			return false;
		}

		if (m_bilateralFilters[i].enabled)
		{
			if (!m_bilateralFilters[i].filter)
			{
				m_bilateralFilters[i].filter = new ccBilateralFilter();
			}
			if (m_bilateralFilters[i].filter->init(w, h, shadersPath, error))
			{
				m_bilateralFilters[i].filter->useExistingViewport(true);
			}
			else
			{
				delete m_bilateralFilters[i].filter;
				m_bilateralFilters[i].filter = nullptr;
				m_bilateralFilters[i].enabled = false;
			}
		}
		else if (m_bilateralFilters[i].filter)
		{
			delete m_bilateralFilters[i].filter;
			m_bilateralFilters[i].filter = nullptr;
		}
	}

	if (!m_fboMix)
	{
		m_fboMix = new ccFrameBufferObject();
	}
	if (!m_fboMix->init(width, height))
	{
		error = "[EDL Filter] FBO 'mix' initialization failed!";
		reset();
		return false;
	}
	m_fboMix->initColor(internalFormat, GL_RGBA, GL_FLOAT);

	if (!m_EDLShader)
	{
		m_EDLShader = new ccShader();
		if (!m_EDLShader->fromFile(shadersPath, "EDL/edl_shade", error))
		{
			reset();
			return false;
		}
	}

	if (!m_mixShader)
	{
		m_mixShader = new ccShader();
		if (!m_mixShader->fromFile(shadersPath,"EDL/edl_mix",error))
		{
			reset();
			return false;
		}
	}

	m_screenWidth = width;
	m_screenHeight = height;

	setValid(true);

	return true;
}

void ccEDLFilter::shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters)
{
	if (!isValid())
	{
		return;
	}

	if (m_screenWidth < 4 || m_screenHeight < 4)
	{
		//ccLog::Warning("[ccEDLFilter::shade] Screen is too small!");
		return;
	}

	//perspective mode
	int perspectiveMode = parameters.perspectiveMode ? 1 : 0;
	//light-balancing based on the current zoom (for ortho. mode only)
	float lightMod = perspectiveMode ? 3.0f : static_cast<float>(std::sqrt(2.0 * std::max(parameters.zoom, 0.7))); //1.41 ~ sqrt(2)

	//we must use corner-based screen coordinates
	m_glFunc.glMatrixMode(GL_PROJECTION);
	m_glFunc.glPushMatrix();
	m_glFunc.glLoadIdentity();
	m_glFunc.glOrtho(0.0, (GLdouble)m_screenWidth, 0.0, (GLdouble)m_screenHeight, 0.0, 1.0/*parameters.zNear,parameters.zFar*/);
	m_glFunc.glMatrixMode(GL_MODELVIEW);
	m_glFunc.glPushMatrix();
	m_glFunc.glLoadIdentity();

	assert(m_glFunc.glGetError() == GL_NO_ERROR);

	for (unsigned i = 0; i < FBO_COUNT; ++i)
	{
		ccFrameBufferObject* fbo = m_fbos[i];
		unsigned scale = (1 << i); //1, 2, 4

		fbo->start();

		m_EDLShader->bind();

		m_EDLShader->setUniformValue("s1_color", 1);
		m_EDLShader->setUniformValue("s2_depth", 0);
		m_EDLShader->setUniformValue("Sx", static_cast<float>(m_screenWidth));
		m_EDLShader->setUniformValue("Sy", static_cast<float>(m_screenHeight));
		m_EDLShader->setUniformValue("Zoom", lightMod);
		m_EDLShader->setUniformValue("PerspectiveMode", perspectiveMode);
		m_EDLShader->setUniformValue("Pix_scale", static_cast<float>(scale));
		m_EDLShader->setUniformValue("Exp_scale", m_expScale);
		m_EDLShader->setUniformValue("Zm", static_cast<float>(parameters.zNear));
		m_EDLShader->setUniformValue("ZM", static_cast<float>(parameters.zFar));
		m_EDLShader->setUniformValueArray("Light_dir", reinterpret_cast<const GLfloat*>(m_lightDir), 1, 3);
		m_EDLShader->setUniformValueArray("Neigh_pos_2D", reinterpret_cast<const GLfloat*>(m_neighbours), 8, 2);

		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, texColor);

		m_glFunc.glActiveTexture(GL_TEXTURE0);
		ccGLUtils::DisplayTexture2DPosition(texDepth, 0, 0, m_screenWidth / scale, m_screenHeight / scale);

		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

		m_EDLShader->release();
		fbo->stop();

		assert(m_glFunc.glGetError() == GL_NO_ERROR);

		//smooth the result
		const BilateralFilterDesc& bl = m_bilateralFilters[i];
		if (bl.filter)
		{
			bl.filter->setParams(bl.halfSize, bl.sigma, bl.sigmaZ);
			bl.filter->shade(texDepth, fbo->getColorTexture(), parameters);
			assert(m_glFunc.glGetError() == GL_NO_ERROR);
		}
	}

	//***	COMPOSITING		***/
	if (m_fboMix)
	{
		m_fboMix->start();

		m_mixShader->bind();
		m_mixShader->setUniformValue("s2_I1", 0);
		m_mixShader->setUniformValue("s2_I2", 1);
		m_mixShader->setUniformValue("s2_I4", 2);
		m_mixShader->setUniformValue("s2_D", 3);
		m_mixShader->setUniformValue("A0", 1.0f);
		m_mixShader->setUniformValue("A1", 0.5f);
		m_mixShader->setUniformValue("A2", 0.25f);
		m_mixShader->setUniformValue("absorb", 1);

		GLuint texCol0 = m_bilateralFilters[0].filter ? m_bilateralFilters[0].filter->getTexture() : m_fbos[0]->getColorTexture();
		GLuint texCol1 = m_bilateralFilters[1].filter ? m_bilateralFilters[1].filter->getTexture() : m_fbos[1]->getColorTexture();
		GLuint texCol2 = m_bilateralFilters[2].filter ? m_bilateralFilters[2].filter->getTexture() : m_fbos[2]->getColorTexture();

		m_glFunc.glActiveTexture(GL_TEXTURE3);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, texDepth);

		m_glFunc.glActiveTexture(GL_TEXTURE2);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, texCol2);

		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, texCol1);

		m_glFunc.glActiveTexture(GL_TEXTURE0);
		ccGLUtils::DisplayTexture2DPosition(texCol0, 0, 0, m_screenWidth, m_screenHeight);

		//m_glFunc.glBindTexture(GL_TEXTURE_2D,0);
		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);
		m_glFunc.glActiveTexture(GL_TEXTURE2);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);
		m_glFunc.glActiveTexture(GL_TEXTURE3);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

		m_mixShader->release();
		m_fboMix->stop();

		assert(m_glFunc.glGetError() == GL_NO_ERROR);
	}

	//restore GL_TEXTURE_0 by default
	m_glFunc.glActiveTexture(GL_TEXTURE0);
	
	assert(m_glFunc.glGetError() == GL_NO_ERROR);

	m_glFunc.glMatrixMode(GL_PROJECTION);
	m_glFunc.glPopMatrix();
	m_glFunc.glMatrixMode(GL_MODELVIEW);
	m_glFunc.glPopMatrix();
	assert(m_glFunc.glGetError() == GL_NO_ERROR);
}

GLuint ccEDLFilter::getTexture()
{
	return (m_fboMix ? m_fboMix->getColorTexture() : 0);
}

void ccEDLFilter::setLightDir(float theta_rad, float phi_rad)
{
	m_lightDir[0] = std::sin(phi_rad)*std::cos(theta_rad);
	m_lightDir[1] = std::cos(phi_rad);
	m_lightDir[2] = std::sin(phi_rad)*std::sin(theta_rad);
}
