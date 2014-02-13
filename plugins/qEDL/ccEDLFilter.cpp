//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qEDL                        #
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

#include "ccEDLFilter.h"

//ccFBO
#include <ccFrameBufferObject.h>
#include <ccBilateralFilter.h>
#include <ccShader.h>
#include <ccFBOUtils.h>

//system
#include <math.h>

//For MSVC
#ifndef M_PI
#define M_PI 3.141592653589793238462643
#endif

ccEDLFilter::ccEDLFilter()
	: ccGlFilter("EyeDome Lighting (disable normals and increase points size for a better result!)")
	, m_screenWidth(0)
	, m_screenHeight(0)
	, fbo_edl0(0)
	, fbo_edl1(0)
	, fbo_edl2(0)
	, shader_edl(0)
	, fbo_mix(0)
	, shader_mix(0)
    , d0(1.0)
    , d1(2.0)
    , d2(2.0)
    , nneighbours(1)
    , F(5.0)
    , power(1.0)
{
    //smoothing filter for full resolution
	m_bilateralFilter0.enabled	= false;
	m_bilateralFilter0.size		= 3;
	m_bilateralFilter0.sigma	= 1.0f;
	m_bilateralFilter0.sigmaZ	= 0.2f;

    //smoothing filter for half resolution
	m_bilateralFilter1.enabled	= true;
	m_bilateralFilter1.size		= 5;
	m_bilateralFilter1.sigma	= 2.0f;
	m_bilateralFilter1.sigmaZ	= 0.4f;

    //smoothing filter for quarter resolution
	m_bilateralFilter2.enabled	= true;
	m_bilateralFilter2.size		= 5;
	m_bilateralFilter2.sigma	= 2.0f;
	m_bilateralFilter2.sigmaZ	= 0.4f;

    //mixing coefficients
    mix0						=	1.0f;
    mix1						=	0.5f;
    mix2						=	0.25f;
    absorb						=	true;

    //focus
    b_depth_focus				=	false;
    depth_focus					=	0.5f;
    b_screen_focus				=	false;
    screen_focus_x				=	0.5f;
    screen_focus_y				=	0.5f;
    screen_focus_sigma			=	0.1f;

    setLightDir((float)(M_PI*0.5),(float)(M_PI*0.5));
	
	memset(neighbours, 0, sizeof(float)*8*4);
	for (int c=0;c<8;c++)
	{
		neighbours[4*c]		= (float)cos(static_cast<double>(c)*M_PI/4.0);
		neighbours[4*c+1]	= (float)sin(static_cast<double>(c)*M_PI/4.0);
	}
}

ccEDLFilter::~ccEDLFilter()
{
    reset();
}

void ccEDLFilter::reset()
{
    if (fbo_edl0)
        delete fbo_edl0;
    fbo_edl0=0;

    if (fbo_edl1)
        delete fbo_edl1;
    fbo_edl1=0;

    if (fbo_edl2)
        delete fbo_edl2;
    fbo_edl2=0;

    if (fbo_mix)
        delete fbo_mix;
    fbo_mix=0;

    if (shader_edl)
        delete shader_edl;
    shader_edl=0;

    if (shader_mix)
        delete shader_mix;
    shader_mix=0;

    if (m_bilateralFilter0.filter)
        delete m_bilateralFilter0.filter;
    m_bilateralFilter0.filter=0;

    if (m_bilateralFilter1.filter)
        delete m_bilateralFilter1.filter;
    m_bilateralFilter1.filter=0;

    if (m_bilateralFilter2.filter)
        delete m_bilateralFilter2.filter;
    m_bilateralFilter2.filter=0;

    m_screenWidth = m_screenHeight = 0;
}

bool ccEDLFilter::init(int width, int height,const char* shadersPath)
{
    return init(width, height, GL_RGBA, GL_LINEAR, shadersPath);
}

bool ccEDLFilter::init(int width, int height, GLenum internalFormat, GLenum minMagFilter, const char* shadersPath)
{
    if (!fbo_edl0)
        fbo_edl0 = new ccFrameBufferObject();
    if (!fbo_edl0->init(width,height))
    {
        //ccLog::Warning("[EDL Filter] FBO 1:1 initialization failed!");
        reset();
        return false;
    }

    if (!fbo_edl1)
        fbo_edl1 = new ccFrameBufferObject();
    if (!fbo_edl1->init(width/2,height/2))
    {
        //ccLog::Warning("[EDL Filter] FBO 1:2 initialization failed!");
        reset();
        return false;
    }

    if (!fbo_edl2)
        fbo_edl2 = new ccFrameBufferObject();
    if (!fbo_edl2->init(width/4,height/4))
    {
        //ccLog::Warning("[EDL Filter] FBO 1:4 initialization failed!");
        reset();
        return false;
    }

    fbo_edl0->initTexture(0,internalFormat,GL_RGBA,GL_FLOAT,minMagFilter);
    fbo_edl1->initTexture(0,internalFormat,GL_RGBA,GL_FLOAT,minMagFilter);
    fbo_edl2->initTexture(0,internalFormat,GL_RGBA,GL_FLOAT,minMagFilter);

    if (!fbo_mix)
        fbo_mix = new ccFrameBufferObject();
    if (!fbo_mix->init(width,height))
    {
        //ccLog::Warning("[EDL Filter] FBO 'mix' initialization failed!");
        reset();
        return false;
    }
    fbo_mix->initTexture(0,internalFormat,GL_RGBA,GL_FLOAT);

    if (!shader_edl)
    {
        shader_edl = new ccShader();
        if (!shader_edl->fromFile(shadersPath,"EDL/edl_shade"))
        {
            //ccLog::Warning("[EDL Filter] can't find 'edl_shade' shader!");
            reset();
            return false;
        }
    }

    if (!shader_mix)
    {
        shader_mix = new ccShader();
        if (!shader_mix->fromFile(shadersPath,"EDL/edl_mix"))
        {
            //ccLog::Warning("[EDL Filter] can't find 'edl_mix' shader!");
            reset();
            return false;
        }
    }

	if (m_bilateralFilter0.enabled)
    {
        if (!m_bilateralFilter0.filter)
            m_bilateralFilter0.filter = new ccBilateralFilter();
        if (!m_bilateralFilter0.filter->init(width,height,shadersPath))
        {
            delete m_bilateralFilter0.filter;
            m_bilateralFilter0.filter = 0;
            m_bilateralFilter0.enabled = false;
        }
        else
        {
            m_bilateralFilter0.filter->useExistingViewport(true);
        }
    }
    else if (m_bilateralFilter0.filter)
    {
        delete m_bilateralFilter0.filter;
        m_bilateralFilter0.filter = 0;
    }

    if (m_bilateralFilter1.enabled)
    {
        if (!m_bilateralFilter1.filter)
            m_bilateralFilter1.filter = new ccBilateralFilter();
        if (!m_bilateralFilter1.filter->init(width/2,height/2,shadersPath))
        {
            delete m_bilateralFilter1.filter;
            m_bilateralFilter1.filter = 0;
            m_bilateralFilter1.enabled = false;
        }
        else
        {
            m_bilateralFilter1.filter->useExistingViewport(true);
        }
    }
    else if (m_bilateralFilter1.filter)
    {
        delete m_bilateralFilter1.filter;
        m_bilateralFilter1.filter = 0;
    }

    if (m_bilateralFilter2.enabled)
    {
        if (!m_bilateralFilter2.filter)
            m_bilateralFilter2.filter = new ccBilateralFilter();
        if (!m_bilateralFilter2.filter->init(width/4,height/4,shadersPath))
        {
            delete m_bilateralFilter2.filter;
            m_bilateralFilter2.filter = 0;
            m_bilateralFilter2.enabled = false;
        }
        else
        {
            m_bilateralFilter2.filter->useExistingViewport(true);
        }
    }
    else if (m_bilateralFilter2.filter)
    {
        delete m_bilateralFilter2.filter;
        m_bilateralFilter2.filter = 0;
    }

    m_screenWidth = width;
    m_screenHeight = height;

    return true;
}

void ccEDLFilter::shade(GLuint texDepth, GLuint texColor, float zoom)
{
	shade(texDepth, texColor, 0.0f, 1.0f, sqrt(2.0f*(zoom>0.7f ? zoom : 0.7f))); //1.41 ~ sqrt(2)
}

void ccEDLFilter::shade(GLuint texDepth, GLuint texColor, float z_min, float z_max, float zoom)
{
    if (!fbo_edl0)
    {
        //ccLog::Warning("[ccEDLFilter::shade] Internal error: structures not initialized!");
        return;
    }

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    //
    /***	FULL SIZE	***/
    //

    //we must use corner-based screen coordinates
    glMatrixMode(GL_PROJECTION);
	glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0,(GLdouble)m_screenWidth,0.0,(GLdouble)m_screenHeight,(GLdouble)z_min,(GLdouble)z_max);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
    glLoadIdentity();

    fbo_edl0->start();
    fbo_edl0->setDrawBuffers1();

    shader_edl->start();
    shader_edl->setUniform1i("s1_color",1);
    shader_edl->setUniform1i("s2_depth",0);

    shader_edl->setUniform1f("sx",zoom/float(m_screenWidth));
    shader_edl->setUniform1f("sy",zoom/float(m_screenHeight));
    shader_edl->setUniform1f("d",d0);
    shader_edl->setUniform1f("P",power);
    shader_edl->setUniform1f("F_scale",F);
    shader_edl->setUniform1i("Nnb",nneighbours);
    shader_edl->setUniform1f("Zm",z_min);
    shader_edl->setUniform1f("ZM",z_max);
    shader_edl->setUniform3fv("L",light_dir);
    shader_edl->setTabUniform4fv("N",8,neighbours);

    shader_edl->setUniform1i("ATMOSPHERIC_ON",b_depth_focus?1:0);
    shader_edl->setUniform1f("Z_FOCUS",depth_focus);

    shader_edl->setUniform1i("SCREEN_FOCUS_ON",b_screen_focus?1:0);
    shader_edl->setUniform1f("SCREEN_FOCUS_X",screen_focus_x);
    shader_edl->setUniform1f("SCREEN_FOCUS_Y",screen_focus_y);
    shader_edl->setUniform1f("SCREEN_FOCUS_SIG",screen_focus_sigma);

    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D, texColor);

    glActiveTexture(GL_TEXTURE0);
    ccFBOUtils::DisplayTexture2DCorner(texDepth,m_screenWidth,m_screenHeight);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    TEX_2D_OFF;

    shader_edl->stop();
    fbo_edl0->stop();
    //
    /***	FULL SIZE	***/

    /***	HALF SIZE	***/
    //

    fbo_edl1->start();
    fbo_edl1->setDrawBuffers1();

    shader_edl->start();
    shader_edl->setUniform1i("s1_color",1);
    shader_edl->setUniform1i("s2_depth",0);

    shader_edl->setUniform1f("sx",zoom/float(m_screenWidth>>1));
    shader_edl->setUniform1f("sy",zoom/float(m_screenHeight>>1));
    shader_edl->setUniform1f("d",d1);
    shader_edl->setUniform1f("P",power);
    shader_edl->setUniform1f("F_scale",F);
    shader_edl->setUniform1i("Nnb",nneighbours);
    shader_edl->setUniform1f("Zm",z_min);
    shader_edl->setUniform1f("ZM",z_max);
    shader_edl->setUniform3fv("L",light_dir);
    shader_edl->setTabUniform4fv("N",8,neighbours);
    //*/

    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D, texColor);

    glActiveTexture(GL_TEXTURE0);
    ccFBOUtils::DisplayTexture2DCorner(texDepth,m_screenWidth/2,m_screenHeight/2);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    TEX_2D_OFF;

    shader_edl->stop();
    fbo_edl1->stop();
    //
    /***	HALF SIZE	***/

    /***	QUARTER SIZE	***/
    //
    fbo_edl2->start();
    fbo_edl2->setDrawBuffers1();

    shader_edl->start();
    shader_edl->setUniform1i("s1_color",1);
    shader_edl->setUniform1i("s2_depth",0);

    shader_edl->setUniform1f("sx",zoom/float(m_screenWidth>>2));
    shader_edl->setUniform1f("sy",zoom/float(m_screenHeight>>2));
    shader_edl->setUniform1f("d",d2);
    shader_edl->setUniform1f("P",power);
    shader_edl->setUniform1f("F_scale",F);
    shader_edl->setUniform1i("Nnb",nneighbours);
    shader_edl->setUniform1f("Zm",z_min);
    shader_edl->setUniform1f("ZM",z_max);
    shader_edl->setTabUniform4fv("N",8,neighbours);
    //*/

    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D, texColor);

    glActiveTexture(GL_TEXTURE0);
    ccFBOUtils::DisplayTexture2DCorner(texDepth,m_screenWidth/4,m_screenHeight/4);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    TEX_2D_OFF;

    shader_edl->stop();
    fbo_edl2->stop();
    //
    /***	QUARTER SIZE	***/

    /***	SMOOTH RESULTS	***/
    if (m_bilateralFilter0.filter)
    {
		m_bilateralFilter0.filter->setParameters(m_bilateralFilter0.size,m_bilateralFilter0.sigma,m_bilateralFilter0.sigmaZ);
        m_bilateralFilter0.filter->shade(texDepth,fbo_edl0->getColorTexture(0),zoom);
    }
    if (m_bilateralFilter1.filter)
    {
        m_bilateralFilter1.filter->setParameters(m_bilateralFilter1.size,m_bilateralFilter1.sigma,m_bilateralFilter1.sigmaZ);
        m_bilateralFilter1.filter->shade(texDepth,fbo_edl1->getColorTexture(0),zoom);
    }
    if (m_bilateralFilter2.filter)
    {
        m_bilateralFilter2.filter->setParameters(m_bilateralFilter2.size,m_bilateralFilter2.sigma,m_bilateralFilter2.sigmaZ);
        m_bilateralFilter2.filter->shade(texDepth,fbo_edl2->getColorTexture(0),zoom);
    }
    /***	SMOOTH RESULTS	***/

    //***	COMPOSITING		***/
    //
    fbo_mix->start();
    fbo_mix->setDrawBuffers1();

    shader_mix->start();
    shader_mix->setUniform1i("s2_I1",0);
    shader_mix->setUniform1i("s2_I2",1);
    shader_mix->setUniform1i("s2_I4",2);
    shader_mix->setUniform1i("s2_D",3);
    shader_mix->setUniform1f("A0",mix0);
    shader_mix->setUniform1f("A1",mix1);
    shader_mix->setUniform1f("A2",mix2);
    shader_mix->setUniform1i("absorb",absorb?1:0);

    glActiveTexture(GL_TEXTURE3);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D,texDepth);

    glActiveTexture(GL_TEXTURE2);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D, m_bilateralFilter2.filter ? m_bilateralFilter2.filter->getTexture() : fbo_edl2->getColorTexture(0));

    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D, m_bilateralFilter1.filter ? m_bilateralFilter1.filter->getTexture() : fbo_edl1->getColorTexture(0));

    glActiveTexture(GL_TEXTURE0);
    //TEX_2D_ON;

    ccFBOUtils::DisplayTexture2DCorner(m_bilateralFilter0.filter ? m_bilateralFilter0.filter->getTexture() : fbo_edl0->getColorTexture(0),m_screenWidth,m_screenHeight);

    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D,0);
    //TEX_2D_OFF;
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D,0);
    TEX_2D_OFF;
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D,0);
    TEX_2D_OFF;
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D,0);
    TEX_2D_OFF;

    shader_mix->stop();
    fbo_mix->stop();

    glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

    glPopAttrib();
}

GLuint ccEDLFilter::getTexture()
{
    return getTexture(0);
}

GLuint ccEDLFilter::getTexture(int index)
{
    switch (index)
    {
    case 0:
        return (fbo_mix ? fbo_mix->getColorTexture(0) : 0);
    case 1:
        return (fbo_edl0 ? fbo_edl0->getColorTexture(0) : 0);
    case 2:
        return (fbo_edl1 ? fbo_edl1->getColorTexture(0) : 0);
    case 3:
        return (fbo_edl2 ? fbo_edl2->getColorTexture(0) : 0);
    case 4:
        return (m_bilateralFilter0.filter ? m_bilateralFilter0.filter->getTexture() : 0);
    case 5:
        return (m_bilateralFilter1.filter ? m_bilateralFilter1.filter->getTexture() : 0);
    case 6:
        return (m_bilateralFilter2.filter ? m_bilateralFilter2.filter->getTexture() : 0);
    }

    //ccLog::Warning("[ccEDLFilter::getTexture] Internal error: bad argument");
    return 0;
}

void ccEDLFilter::setLightDir(float theta, float phi)
{
    light_dir[0]	=	sin(phi)*cos(theta);
    light_dir[1]	=	cos(phi);
    light_dir[2]	=	sin(phi)*sin(theta);
}

void ccEDLFilter::setMousePos(int x, int y)
{
    screen_focus_x	=	float(x)/float(m_screenWidth);
    screen_focus_y	=	float(y)/float(m_screenHeight);
}
