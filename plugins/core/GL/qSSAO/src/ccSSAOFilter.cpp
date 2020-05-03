//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qSSAO                       #
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

#include "ccSSAOFilter.h"

//CC_FBO
#include <ccFrameBufferObject.h>
#include <ccBilateralFilter.h>
#include <ccShader.h>
//qCC_gl
#include <ccGLUtils.h>

//RandomKit
#include <randomkit.h>

//system
#include <stdlib.h>
#include <math.h>
#include <vector>

#ifndef GL_RGBA32F
#define GL_RGBA32F 0x8814
#endif

#ifndef GL_RGB16F
#define GL_RGB16F 0x881B
#endif

ccSSAOFilter::ccSSAOFilter()
	: ccGlFilter("Screen Space Ambient Occlusion")
	, m_w(0)
	, m_h(0)
	, m_fbo(nullptr)
	, m_shader(nullptr)
	, m_texReflect(0)
	, m_glFuncIsValid(0)
{
	setParameters(/*N=*/32,/*Kz=*/500.0f,/*R=*/0.05f,/*F=*/50.0f);

	m_bilateralFilterEnabled = false;
	m_bilateralFilter        = nullptr;
	m_bilateralGHalfSize     = 2;
	m_bilateralGSigma        = 0.5f;
	m_bilateralGSigmaZ       = 0.4f;

	memset(m_ssao_neighbours, 0, sizeof(float) * 3 * MAX_N);
	sampleSphere();
}

ccSSAOFilter::~ccSSAOFilter()
{
	reset();
}

ccGlFilter* ccSSAOFilter::clone() const
{
	ccSSAOFilter* filter = new ccSSAOFilter();

	//copy parameters
	filter->setParameters(m_N, m_Kz, m_R, m_F);

	return filter;
}

void ccSSAOFilter::reset()
{
	if (m_glFuncIsValid && m_glFunc.glIsTexture(m_texReflect))
	{
		m_glFunc.glDeleteTextures(1, &m_texReflect);
	}
	m_texReflect = 0;

	if (m_fbo)
	{
		delete m_fbo;
		m_fbo = nullptr;
	}

	if (m_shader)
	{
		delete m_shader;
		m_shader = nullptr;
	}

	if (m_bilateralFilter)
	{
		delete m_bilateralFilter;
		m_bilateralFilter = nullptr;
	}
}

bool ccSSAOFilter::init(unsigned width, unsigned height, QString shadersPath, QString& error)
{
	return init(width, height, true, true, shadersPath, error);
}

bool ccSSAOFilter::init(unsigned width,
						unsigned height,
						bool enableBilateralFilter,
						bool useReflectTexture,
						QString shadersPath,
						QString& error )
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

	//in case of reinit
	if (!m_fbo)
	{
		m_fbo = new ccFrameBufferObject();
	}

	if (	!m_fbo->init(width, height)
		||	!m_fbo->initColor(GL_RGBA32F, GL_RGBA, GL_FLOAT, GL_LINEAR) )
	{
		error = "[SSAO] FrameBufferObject initialization failed!";
		reset();
		return false;
	}

	if (!m_shader)
	{
		m_shader = new ccShader();
		if (!m_shader->fromFile(shadersPath, "SSAO/ssao", error))
		{
			error = "[SSAO] Can't load SSAO shaders";
			reset();
			return false;
		}
	}

	m_bilateralFilterEnabled = enableBilateralFilter;
	if (m_bilateralFilterEnabled)
	{
		if (!m_bilateralFilter)
			m_bilateralFilter = new ccBilateralFilter();
		if (!m_bilateralFilter->init(width, height, shadersPath, error))
		{
			delete m_bilateralFilter;
			m_bilateralFilter = nullptr;
			m_bilateralFilterEnabled = false;
		}
		else
		{
			m_bilateralFilter->useExistingViewport(true);
		}
	}
	else if (m_bilateralFilter)
	{
		delete m_bilateralFilter;
		m_bilateralFilter = nullptr;
	}

	m_w = width;
	m_h = height;

	if (useReflectTexture)
	{
		initReflectTexture();
	}
	else
	{
		//remove the existing texture
		if (m_glFuncIsValid && m_glFunc.glIsTexture(m_texReflect))
		{
			m_glFunc.glDeleteTextures(1, &m_texReflect);
		}
		m_texReflect = 0;
	}

	setValid(true);

	return true;
}

void ccSSAOFilter::sampleSphere()
{
	// Initialize the sobol QRNG
	rk_sobol_state s;
	if (rk_sobol_init(3, &s, nullptr, rk_sobol_Ldirections, nullptr) != RK_SOBOL_OK)
	{
		return;
	}
	rk_sobol_randomshift(&s, nullptr);

	//	draw in sphere
	float* ssao_neighbours = m_ssao_neighbours;

	for (unsigned n_in_sphere = 0; n_in_sphere < MAX_N; )
	{
		double x[5];
		rk_sobol_double(&s, x);
		
		double px = x[0] * 2 - 1.0;
		double py = x[1] * 2 - 1.0;
		double pz = x[2] * 2 - 1.0;
		
		if ( px*px + py*py + pz*pz <= 1.0 )
		{
			*ssao_neighbours++ = static_cast<float>(px);
			*ssao_neighbours++ = static_cast<float>(py);
			*ssao_neighbours++ = static_cast<float>(pz);

			++n_in_sphere;
		}
	}

	rk_sobol_free(&s);
}

void ccSSAOFilter::shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters)
{
	if (!isValid())
	{
		return;
	}
	assert(m_fbo);

	//we must use corner-based screen coordinates
	m_glFunc.glMatrixMode(GL_PROJECTION);
	m_glFunc.glPushMatrix();
	m_glFunc.glLoadIdentity();
	m_glFunc.glOrtho(0.0, static_cast<GLdouble>(m_w), 0.0, static_cast<GLdouble>(m_h), 0.0, 1.0);
	m_glFunc.glMatrixMode(GL_MODELVIEW);
	m_glFunc.glPushMatrix();
	m_glFunc.glLoadIdentity();
	assert(m_glFunc.glGetError() == GL_NO_ERROR);

	bool hasReflectTexture = m_glFunc.glIsTexture(m_texReflect);

	m_fbo->start();

	m_shader->bind();
	m_shader->setUniformValue("s2_Z",0);
	m_shader->setUniformValue("s2_R",1);
	m_shader->setUniformValue("s2_C",2);
	m_shader->setUniformValue("R",   m_R);
	m_shader->setUniformValue("F",   m_F);
	m_shader->setUniformValue("Kz",  m_Kz);
	//m_shader->setUniformValue("N", N);
	m_shader->setUniformValue("B_REF", hasReflectTexture ? 1 : 0);
	m_shader->setUniformValueArray("P", m_ssao_neighbours, MAX_N, 3);

	m_glFunc.glActiveTexture(GL_TEXTURE2);
	m_glFunc.glBindTexture(GL_TEXTURE_2D, texColor);

	GLuint texReflect = 0;
	if (hasReflectTexture)
	{
		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, m_texReflect);
	}

	m_glFunc.glActiveTexture(GL_TEXTURE0);
	ccGLUtils::DisplayTexture2DPosition(texDepth, 0, 0, m_w, m_h);

	if (hasReflectTexture)
	{
		m_glFunc.glActiveTexture(GL_TEXTURE1);
		m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);
	}

	m_glFunc.glActiveTexture(GL_TEXTURE2);
	m_glFunc.glBindTexture(GL_TEXTURE_2D, 0);

	m_shader->release();
	m_fbo->stop();

	if (m_bilateralFilter)
	{
		m_bilateralFilter->setParams(m_bilateralGHalfSize, m_bilateralGSigma, m_bilateralGSigmaZ);
		m_bilateralFilter->shade(texDepth, m_fbo->getColorTexture(), parameters);
		assert(m_glFunc.glGetError() == GL_NO_ERROR);
	}

	//restore GL_TEXTURE_0 by default
	m_glFunc.glActiveTexture(GL_TEXTURE0);

	m_glFunc.glMatrixMode(GL_PROJECTION);
	m_glFunc.glPopMatrix();
	m_glFunc.glMatrixMode(GL_MODELVIEW);
	m_glFunc.glPopMatrix();
	assert(m_glFunc.glGetError() == GL_NO_ERROR);
}

GLuint ccSSAOFilter::getTexture()
{
	if (m_bilateralFilter)
	{
		return m_bilateralFilter->getTexture();
	}

	return (m_fbo ? m_fbo->getColorTexture() : 0);
}

void ccSSAOFilter::setParameters(int _N, float _Kz, float _R, float _F)
{
	m_N  = _N;
	m_Kz = _Kz;
	m_R  = _R;
	m_F  = _F;
}

inline double frand()
{
	return static_cast<double>(rand()) / RAND_MAX;
}

void randomPointInSphere(double& vx, double& vy, double& vz)
{
	do
	{
		vx = frand();
		vy = frand();
		vz = frand();
	}
	while (vx*vx + vy*vy + vz*vz > 1.0);
}

void ccSSAOFilter::initReflectTexture()
{
	/***	INIT TEXTURE OF RELFECT VECTORS		***/
	/**		Fully random texture	**/
	int texSize = m_w*m_h;
	std::vector<float> reflectTexture;
	try
	{
		reflectTexture.resize(3*texSize, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return;
	}

	for (int i=0; i<texSize; i++)
	{
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		randomPointInSphere(x, y, z);

		double norm = x*x + y*y + z*z;
		norm = (norm > 1.0e-8 ? 1.0 / sqrt(norm) : 0.0);

		reflectTexture[i * 3    ] = static_cast<float>((1.0 + x*norm) / 2);
		reflectTexture[i * 3 + 1] = static_cast<float>((1.0 + y*norm) / 2);
		reflectTexture[i * 3 + 2] = static_cast<float>((1.0 + z*norm) / 2);
	}

	assert(m_glFuncIsValid);

	m_glFunc.glPushAttrib(GL_ENABLE_BIT);
	m_glFunc.glEnable(GL_TEXTURE_2D);

	m_glFunc.glGenTextures  (1, &m_texReflect);
	m_glFunc.glBindTexture  (GL_TEXTURE_2D, m_texReflect);
	m_glFunc.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	m_glFunc.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	m_glFunc.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	m_glFunc.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	m_glFunc.glTexImage2D   (GL_TEXTURE_2D, 0, GL_RGB16F, m_w, m_h, 0, GL_RGB, GL_FLOAT, &reflectTexture[0]);
	m_glFunc.glBindTexture  (GL_TEXTURE_2D, 0);

	m_glFunc.glPopAttrib();
	assert(m_glFunc.glGetError() == GL_NO_ERROR);


	// According to Wikipedia, noise is made of 4*4 repeated tiles	to have only high frequency
}
