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

#include <ccGlFilter.h>

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
	virtual ccGlFilter* clone() const;
	virtual bool init(int width, int height, QString shadersPath, QString& error);
	virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters);
	virtual GLuint getTexture();

	bool init(	int width,
				int height,
				bool enableBilateralFilter,
				bool useReflectTexture,
				QString shadersPath,
				QString& error,
				GLenum textureMinMagFilter = GL_LINEAR);

	void setParameters(int N, float Kz, float R, float F);

protected:

	void initReflectTexture();
	void sampleSphere();

	GLuint m_texReflect;

	int m_w;
	int m_h;

	ccFrameBufferObject* m_fbo;
	ccShader* m_shader;

	int   m_N;								// nb of neighbours
	float m_Kz;								// attenuation with distance
	float m_R;								// radius in image of neighbour sphere
	float m_F;								// amplification

	//! Maximum number of sampling directions
	static const int MAX_N = 256;

	float m_ssao_neighbours[3*MAX_N];	//	full sphere sampling

	ccBilateralFilter* m_bilateralFilter;
	bool               m_bilateralFilterEnabled;
	unsigned           m_bilateralGHalfSize;
	float              m_bilateralGSigma;
	float              m_bilateralGSigmaZ;
};

#endif
