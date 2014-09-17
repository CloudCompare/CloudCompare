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

/***************************************************************/
//
//		FILTER_EDL
//
//		EyeDome Lighting
//		(with bilateral filtering instead of gaussian filtering)
//
//		Output:
//			shaded image
//
//		creation:		April 23 2008
//						Christian Boucheny (EDF R&D / INRIA)
//		modification:	August 18 2008
//						Christian Boucheny (EDF R&D / INRIA)
//		modification:	April 2009
//						Daniel Girardeau-Montaut (creation of CC plugin)
//		modification:	February 17 2014
//						Daniel Girardeau-Montaut (simplification)
//
/*****************************************************************/
#ifndef	CC_EDL_FILTER_HEADER
#define	CC_EDL_FILTER_HEADER

//ccFBO
#include <ccGlFilter.h>
#include <ccBilateralFilter.h>

class ccShader;
class ccFrameBufferObject;

//!	EyeDome Lighting
class ccEDLFilter : public ccGlFilter
{
public:

	//! Default constructor
	ccEDLFilter();
	//! Default destructor
	virtual ~ccEDLFilter();

	//inherited from ccGlFilter
	virtual ccGlFilter* clone() const;
	virtual bool init(int width, int height, QString shadersPath, QString& error);
	virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters);
	virtual GLuint getTexture();

	//! Resets filter
	void reset();

	//! Inits filter
	bool init(	int width,
				int height,
				GLenum internalFormat,
				GLenum minMagFilter,
				QString shadersPath,
				QString& error);

	//! Returns given texture index
	GLuint getTexture(int index);

	//! Sets light direction
	void setLightDir(float theta_rad, float phi_rad);

	//! Sets strength
	/** \param value strength value (default: 100)
	**/
	void setStrength(float value) { exp_scale = value; }

private:

	int	m_screenWidth;
	int	m_screenHeight;

	ccFrameBufferObject*	fbo_edl0;
	ccFrameBufferObject*	fbo_edl1;
	ccFrameBufferObject*	fbo_edl2;
	ccShader*				shader_edl;

	ccFrameBufferObject*	fbo_mix;
	ccShader*				shader_mix;

	float	neighbours[8*2];
	float	exp_scale;

	//! Bilateral filter and its parameters
	struct BilateralFilter
	{
		ccBilateralFilter* filter;
		unsigned halfSize;
		float sigma;
		float sigmaZ;
		bool enabled;

		BilateralFilter()
			: filter(0)
			, halfSize(0)
			, sigma(0.0f)
			, sigmaZ(0.0f)
			, enabled(false)
		{
		}

		~BilateralFilter()
		{
			if (filter)
				delete filter;
		}
	};

	//	Bilateral filtering
	BilateralFilter m_bilateralFilter0;
	BilateralFilter m_bilateralFilter1;
	BilateralFilter m_bilateralFilter2;

	// Light direction
	float	light_dir[3];
};
//
//	EyeDome Lighting
//
/////////////////////////////////////////////




#endif
