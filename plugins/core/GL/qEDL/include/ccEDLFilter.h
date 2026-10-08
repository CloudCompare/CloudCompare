#pragma once

// ##########################################################################
// #                                                                        #
// #                       CLOUDCOMPARE PLUGIN: qEDL                        #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

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

// ccFbo
#include <ccBilateralFilter.h>
#include <ccGlFilter.h>

// Qt
#include <QOpenGLFunctions_2_1>

class ccShader;
class ccFrameBufferObject;

//!	EyeDome Lighting
class ccEDLFilter : public ccGlFilter
{
  public:
	//! Default constructor
	ccEDLFilter();

	// inherited from ccGlFilter
	ccGlFilter* clone() const override;
	bool        init(unsigned width, unsigned height, const QString& shadersPath, QString& error, bool silent) override;
	void        shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters) override;
	GLuint      getTexture() override;

	//! Resets filter
	void reset();

	//! Inits filter
	bool init(unsigned       width,
	          unsigned       height,
	          GLenum         internalFormat,
	          GLenum         minMagFilter,
	          const QString& shadersPath,
	          QString&       error);

	//! Sets light direction
	void setLightDir(float theta_rad, float phi_rad);

	//! Sets strength
	/** \param value strength value (default: 100)
	 **/
	inline void setStrength(float value)
	{
		m_expScale = value;
	}

  private:
	unsigned m_screenWidth;
	unsigned m_screenHeight;

	//! Number of FBOs
	static constexpr unsigned FBO_COUNT = 3;

	std::array<std::unique_ptr<ccFrameBufferObject>, FBO_COUNT> m_fbos;
	std::unique_ptr<ccShader>                                   m_EDLShader;

	std::unique_ptr<ccFrameBufferObject> m_fboMix;
	std::unique_ptr<ccShader>            m_mixShader;

	std::array<float, 8 * 2> m_neighbours;
	float                    m_expScale;

	//! Bilateral filter descriptor
	struct BilateralFilterDesc
	{
		std::unique_ptr<ccBilateralFilter> filter;
		unsigned                           halfSize;
		float                              sigma;
		float                              sigmaZ;
		bool                               enabled;

		BilateralFilterDesc()
		    : filter(nullptr)
		    , halfSize(0)
		    , sigma(0)
		    , sigmaZ(0)
		    , enabled(false)
		{
		}

		~BilateralFilterDesc() = default;
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
