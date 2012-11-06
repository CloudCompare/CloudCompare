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
//$Rev:: 1870                                                              $
//$LastChangedDate:: 2011-07-06 00:45:48 +0200 (mer., 06 juil. 2011)       $
//**************************************************************************
//

/***************************************************************/
//
//		FILTERBILATERAL
//
//		Cross Bilateral Filter - from Tomasi98/Durand02/Eisemann04
//
//		created by Christian Boucheny (INRIA/LPPA/EDF R&D) on 12/21/2007
//
//      freely updated by Daniel Girardeau-Montaut (EDF R&D) on 03/31/2009
//
/*****************************************************************/

#ifndef	CC_BILATERAL_FILTER_HEADER
#define	CC_BILATERAL_FILTER_HEADER

#include "ccGlFilter.h"

class ccShader;
class ccFrameBufferObject;

#define	BILATERAL_FILTER_SIZE	16

class ccBilateralFilter : public ccGlFilter
{
public:

	ccBilateralFilter();
    virtual ~ccBilateralFilter();

    void reset();

    //inherited from ccGlFilter
	virtual bool init(int width,int height,const char* shadersPath);
	virtual void shade(GLuint texDepth, GLuint texColor, float zoom = 1.0);
	virtual GLuint getTexture();

	void setSizeSigmaSpatial(int size, float sigma);
	void setSigmaDepth(float sigma);
	void setParameters(int spatial_size, float spatial_sigma, float depth_sigma);

	void useExistingViewport(bool state);

protected:

	void updateDampingTable();

	int w;
	int	h;

	ccFrameBufferObject*	fbo;
	ccShader*		        shader;

	int		filter_spatial_size;
	float	filter_spatial_sigma;
	float	filter_depth_sigma;

	float dampingPixelDist[225];

	bool    useCurrentViewport;
};

#endif
