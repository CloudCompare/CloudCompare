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

class ccBilateralFilter : public ccGlFilter
{
public:

    //! Default constructor
	ccBilateralFilter();
    //! Destructor
    virtual ~ccBilateralFilter();

    void reset();

    //inherited from ccGlFilter
	virtual bool init(int width,int height,const char* shadersPath);
	virtual void shade(GLuint texDepth, GLuint texColor, float zoom = 1.0);
	virtual GLuint getTexture();

	//! Max kernel size
	static const unsigned KERNEL_MAX_SIZE = 15;

	void setParameters(int spatialSize, float spatialSigma, float depthSigma);

	void useExistingViewport(bool state);

protected:

	void updateDampingTable();

	int m_width;
	int	m_height;

	ccFrameBufferObject* m_fbo;
	ccShader* m_shader;

	int m_filterSpatialSize;
	float m_filterSpatialSigma;
	float m_filterDepthSigma;

	float m_dampingPixelDist[KERNEL_MAX_SIZE*KERNEL_MAX_SIZE];

	bool m_useCurrentViewport;
};

#endif
