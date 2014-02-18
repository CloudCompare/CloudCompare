//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccBilateralFilter.h"

//Local
#include "ccFrameBufferObject.h"
#include "ccShader.h"
#include "ccFBOUtils.h"

//system
#include <math.h>
#include <assert.h>
#include <algorithm>

ccBilateralFilter::ccBilateralFilter()
	: ccGlFilter("Bilateral smooth")
	, m_width(0)
	, m_height(0)
	, m_fbo(0)
	, m_shader(0)
	, m_useCurrentViewport(false)
{
	memset(m_dampingPixelDist, 0, KERNEL_MAX_HALF_SIZE*KERNEL_MAX_HALF_SIZE); //will be updated right away by 'setParameters'

	setParams(2,2.0f,0.4f);
}

ccBilateralFilter::~ccBilateralFilter()
{
    reset();
}

void ccBilateralFilter::useExistingViewport(bool state)
{
    m_useCurrentViewport = state;
}

void ccBilateralFilter::reset()
{
    if (m_fbo)
        delete m_fbo;
    m_fbo = 0;

    if (m_shader)
        delete m_shader;
    m_shader = 0;

    m_width = m_height = 0;
}

bool ccBilateralFilter::init(int width, int height, const char* shadersPath)
{
    if (!m_fbo)
        m_fbo = new ccFrameBufferObject();
    if (!m_fbo->init(width,height))
    {
        //ccLog::Warning("[Bilateral Filter] Can't initialize FBO!");
        reset();
        return false;
    }

    m_fbo->start();
    m_fbo->initTexture(0,GL_RGB/*GL_RGB32F*/,GL_RGB,GL_FLOAT);
    m_fbo->stop();

    if (!m_shader)
        m_shader = new ccShader();
    if (!m_shader->fromFile(shadersPath, "bilateral"))
    {
        //ccLog::Warning("[Bilateral Filter] Can't load shader!");
        reset();
        return false;
    }

    m_width = width;
    m_height = height;

    return true;
}

void ccBilateralFilter::setParams(unsigned spatialSize, float spatialSigma, float depthSigma)
{
    m_halfSpatialSize	= std::min<unsigned>(spatialSize,KERNEL_MAX_HALF_SIZE);
    m_spatialSigma		= spatialSigma;
    m_depthSigma		= depthSigma;

	updateDampingTable();
}

void ccBilateralFilter::shade(GLuint texDepth, GLuint texColor, float zoom)
{
    if (!m_fbo || !m_shader)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (!m_useCurrentViewport)
    {
        //we must use corner-based screen coordinates
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0.0,(GLdouble)m_width,0.0,(GLdouble)m_height,0.0,1.0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
    }

    //	HORIZONTAL
    m_fbo->start();
    m_fbo->setDrawBuffers1();

    m_shader->start();
    m_shader->setUniform1i("s2_I",0);	// image to blur
    m_shader->setUniform1i("s2_D",1);	// image to modulate filter
    m_shader->setUniform1f("SX",static_cast<float>(m_width));
    m_shader->setUniform1f("SY",static_cast<float>(m_height));
    m_shader->setUniform1i("NHalf",m_halfSpatialSize);
	m_shader->setTabUniform1fv("DistCoefs",64,m_dampingPixelDist);
    m_shader->setUniform1f("SigmaDepth",m_depthSigma);

    //Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D,texDepth);

    //Texture 0 --> 2D
    glActiveTexture(GL_TEXTURE0);
    //TEX_2D_ON;
    //glBindTexture(GL_TEXTURE_2D,texColor);

    ccFBOUtils::DisplayTexture2DCorner(texColor,m_width,m_height);

    //Texture 0 --> 2D
    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D,0);
    //TEX_2D_OFF;

    //Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D,0);
    TEX_2D_OFF;

    m_shader->stop();
    m_fbo->stop();

    if (!m_useCurrentViewport)
    {
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }

    glPopAttrib();
}

GLuint ccBilateralFilter::getTexture()
{
    return (m_fbo ? m_fbo->getColorTexture(0) : 0);
}

void ccBilateralFilter::updateDampingTable()
{
	assert(m_halfSpatialSize <= KERNEL_MAX_HALF_SIZE);

	//constant quotient
	float q = static_cast<float>(m_halfSpatialSize) * m_spatialSigma;
	q = 2.0f * (q*q);

	for (unsigned c=0; c<=m_halfSpatialSize; c++)
	{
        for (unsigned d=0; d<=m_halfSpatialSize; d++)
        {
            //pixel distance based damping
            m_dampingPixelDist[c*(m_halfSpatialSize+1)+d] = exp(-static_cast<float>(c*c+d*d)/q);
		}
	}
}
