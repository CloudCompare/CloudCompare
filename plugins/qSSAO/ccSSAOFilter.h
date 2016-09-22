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

#ifndef	CC_FILTER_SSAO_H
#define	CC_FILTER_SSAO_H

//CC_FBO
#include <ccGlFilter.h>

//Qt
#include <QOpenGLFunctions_2_1>

//system
#include <vector>

class ccShader;
class ccBilateralFilter;
class ccFrameBufferObject;

class ccSSAOFilter : public ccGlFilter
{
public:

	ccSSAOFilter();
	virtual ~ccSSAOFilter();

	void reset();

	//inherited from ccGlFilter
	virtual ccGlFilter* clone() const override;
	virtual bool init(unsigned width, unsigned height, QString shadersPath, QString& error) override;
	virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters) override;
	virtual GLuint getTexture() override;

	bool init(	unsigned width,
				unsigned height,
				bool enableBilateralFilter,
				bool useReflectTexture,
				QString shadersPath,
				QString& error);

	void setParameters(int N, float Kz, float R, float F);

protected:

	void initReflectTexture();
	void sampleSphere();

	unsigned m_w;
	unsigned m_h;

	ccFrameBufferObject* m_fbo;
	ccShader* m_shader;
	GLuint m_texReflect;

	int   m_N;								// nb of neighbours
	float m_Kz;								// attenuation with distance
	float m_R;								// radius in image of neighbour sphere
	float m_F;								// amplification

	//! Maximum number of sampling directions
	static const int MAX_N = 256;

	//!	Full sphere sampling
	float m_ssao_neighbours[3 * MAX_N];

	ccBilateralFilter* m_bilateralFilter;
	bool               m_bilateralFilterEnabled;
	unsigned           m_bilateralGHalfSize;
	float              m_bilateralGSigma;
	float              m_bilateralGSigmaZ;

	//! Associated OpenGL functions set
	QOpenGLFunctions_2_1 m_glFunc;
	//! Associated OpenGL functions set validity
	bool m_glFuncIsValid;
};

#endif
