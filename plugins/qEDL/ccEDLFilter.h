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

//Qt
#include <QOpenGLFunctions_2_1>

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
	virtual ccGlFilter* clone() const override;
	virtual bool init(unsigned width, unsigned height, QString shadersPath, QString& error) override;
	virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters) override;
	virtual GLuint getTexture() override;

	//! Resets filter
	void reset();

	//! Inits filter
	bool init(	unsigned width,
				unsigned height,
				GLenum internalFormat,
				GLenum minMagFilter,
				QString shadersPath,
				QString& error);

	//! Sets light direction
	void setLightDir(float theta_rad, float phi_rad);

	//! Sets strength
	/** \param value strength value (default: 100)
	**/
	inline void setStrength(float value) { m_expScale = value; }

private:

	unsigned m_screenWidth;
	unsigned m_screenHeight;

	//! Number of FBOs
	static const unsigned FBO_COUNT = 3;

	ccFrameBufferObject*	m_fbos[FBO_COUNT];
	ccShader*				m_EDLShader;

	ccFrameBufferObject*	m_fboMix;
	ccShader*				m_mixShader;

	float	m_neighbours[8*2];
	float	m_expScale;

	//! Bilateral filter descriptor
	struct BilateralFilterDesc
	{
		ccBilateralFilter* filter;
		unsigned halfSize;
		float sigma;
		float sigmaZ;
		bool enabled;

		BilateralFilterDesc()
			: filter(0)
			, halfSize(0)
			, sigma(0)
			, sigmaZ(0)
			, enabled(false)
		{
		}

		~BilateralFilterDesc()
		{
			if (filter)
				delete filter;
		}
	};

	//	Bilateral filters (one per FBO at most)
	BilateralFilterDesc m_bilateralFilters[FBO_COUNT];

	// Light direction
	float m_lightDir[3];

	//! Associated OpenGL functions set
	QOpenGLFunctions_2_1 m_glFunc;
	//! Associated OpenGL functions set validity
	bool m_glFuncIsValid;
};

#endif
