#pragma once

// ##########################################################################
// #                                                                        #
// #                       CLOUDCOMPARE PLUGIN: qSSAO                       #
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
//		FILTER_SSAO
//
//		Screen Space Ambient Occlusion
//		Adapted from Crytek and Inigo Quilez
//
//		Output:
//			shaded image
//
//		creation:		23 avril 2008
//						Christian Boucheny (EDF R&D / INRIA)
//		modification:	Avril 2009
//						Daniel Girardeau-Montaut
//
/*****************************************************************/

// CCFbo
#include <ccGlFilter.h>

// Qt
#include <QOpenGLFunctions_2_1>

class ccShader;
class ccBilateralFilter;
class ccFrameBufferObject;

class ccSSAOFilter : public ccGlFilter
{
  public:
	ccSSAOFilter();
	~ccSSAOFilter() override;

	void reset();

	// inherited from ccGlFilter
	ccGlFilter* clone() const override;
	bool        init(unsigned width, unsigned height, const QString& shadersPath, QString& error, bool silent) override;
	void        shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters) override;
	GLuint      getTexture() override;

	void setParameters(float Kz, float R, float F);

  protected:
	//! Maximum number of sampling directions
	static constexpr int MAX_N = 32; // see shader code

	void initReflectTexture();
	void sampleSphere();

	unsigned m_w;
	unsigned m_h;

	std::unique_ptr<ccFrameBufferObject> m_fbo;
	std::unique_ptr<ccShader>            m_shader;
	GLuint                               m_texReflect;

	float m_Kz; // attenuation with distance
	float m_R;  // radius in image of neighbour sphere
	float m_F;  // amplification

	//!	Full sphere sampling
	std::array<float, MAX_N * 3> m_ssaoNeighbours;

	//!	Random sampling seed
	unsigned m_randSeed;

	std::unique_ptr<ccBilateralFilter> m_bilateralFilter;
	bool                               m_bilateralFilterEnabled;
	unsigned                           m_bilateralGHalfSize;
	float                              m_bilateralGSigma;
	float                              m_bilateralGSigmaZ;

	//! Associated OpenGL functions set
	QOpenGLFunctions_2_1 m_glFunc;
	//! Associated OpenGL functions set validity
	bool m_glFuncIsValid;
};
