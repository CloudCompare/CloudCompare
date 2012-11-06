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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//

#include "ccBilateralFilter.h"
#include "ccFrameBufferObject.h"
#include "ccShader.h"
#include "ccFBOUtils.h"

#include <math.h>
#include <algorithm>

ccBilateralFilter::ccBilateralFilter() : ccGlFilter("Bilateral smooth")
{
    w =	0;
    h =	0;
    shader = 0;
    fbo = 0;

	for (unsigned i=0;i<225;++i)
		dampingPixelDist[i]=1.0;

	setParameters(5,2.0f,0.4f);

    useExistingViewport(false);
}

ccBilateralFilter::~ccBilateralFilter()
{
    reset();
}

void ccBilateralFilter::useExistingViewport(bool state)
{
    useCurrentViewport = state;
}

void ccBilateralFilter::reset()
{
    if (fbo)
        delete fbo;
    fbo=0;

    if (shader)
        delete shader;
    shader=0;

    w=h=0;
}

bool ccBilateralFilter::init(int width, int height, const char* shadersPath)
{
    if (!fbo)
        fbo	= new ccFrameBufferObject();
    if (!fbo->init(width,height))
    {
        //ccConsole::Warning("[Bilateral Filter] Can't initialize FBO!");
        reset();
        return false;
    }

    fbo->start();
    fbo->initTexture(0,GL_RGB/*GL_RGB32F*/,GL_RGB,GL_FLOAT);
    fbo->stop();

    if (!shader)
        shader = new ccShader();
    if (!shader->fromFile(shadersPath, "bilateral"))
    {
        //ccConsole::Warning("[Bilateral Filter] Can't load shader!");
        reset();
        return false;
    }

    w =	width;
    h =	height;

    return true;
}

void ccBilateralFilter::setSizeSigmaSpatial(int size, float sigma)
{
	filter_spatial_size		=	std::min(size,15);
    filter_spatial_sigma	=	sigma;
	updateDampingTable();
}

void ccBilateralFilter::setSigmaDepth(float sigma)
{
    filter_depth_sigma	=	sigma;
	//updateDampingTable();
}

void ccBilateralFilter::setParameters(int spatial_size, float spatial_sigma, float depth_sigma)
{
    filter_spatial_size		=	std::min(spatial_size,15);
    filter_spatial_sigma	=	spatial_sigma;
    filter_depth_sigma		=	depth_sigma;
	updateDampingTable();
}

void ccBilateralFilter::shade(GLuint texDepth, GLuint texColor, float zoom)
{
    if (!fbo)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (!useCurrentViewport)
    {
        //we must use corner-based screen coordinates
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0.0,(GLdouble)w,0.0,(GLdouble)h,0.0,1.0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
    }

    //	HORIZONTAL
    fbo->start();
    fbo->setDrawBuffers1();

    shader->start();
    shader->setUniform1i("s2_I",0);	//	image to blur
    shader->setUniform1i("s2_D",1); //	image to modulate filter
    shader->setUniform1f("SX",1.0f/(float)w);
    shader->setUniform1f("SY",1.0f/(float)h);
    shader->setUniform1i("N",filter_spatial_size);
    shader->setUniform1f("sigma",filter_spatial_sigma);
    shader->setUniform1i("use_gauss_p",1);
	shader->setTabUniform1fv("gauss_p",225,dampingPixelDist);
    shader->setUniform1f("sigmaz",filter_depth_sigma);
    //*/

    //Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    TEX_2D_ON;
    glBindTexture(GL_TEXTURE_2D,texDepth);

    //Texture 0 --> 2D
    glActiveTexture(GL_TEXTURE0);
    //TEX_2D_ON;
    //glBindTexture(GL_TEXTURE_2D,texColor);

    ccFBOUtils::DisplayTexture2DCorner(texColor,w,h);

    //Texture 0 --> 2D
    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D,0);
    //TEX_2D_OFF;

    //Texture 1 --> 2D
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D,0);
    TEX_2D_OFF;

    shader->stop();
    fbo->stop();

    if (!useCurrentViewport)
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
    return (fbo ? fbo->getColorTexture(0) : 0);
}

void ccBilateralFilter::updateDampingTable()
{
	int N = filter_spatial_size;
	int hN = (N>>1);		// filter half width (N should be odd)

	for(int c=-hN;c<hN+1;c++)
	{
        for(int d=-hN;d<hN+1;d++)
        {
            //pixel distance based damping
            float dist = float(c*c+d*d)/(filter_spatial_sigma*(float)(hN*hN));
            dampingPixelDist[(c+hN)*N+(d+hN)] =	exp(-dist*dist/2.0f);
		}
	}
}

