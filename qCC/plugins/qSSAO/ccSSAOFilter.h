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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

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
//		creation:	    23 avril 2008
//					    Christian Boucheny (EDF R&D / INRIA)
//		modification:	Avril 2009
//					    Daniel Girardeau-Montaut
//
/*****************************************************************/

///**************************/
////
////	HOW TO USE SSAO FILTER
////
//#include "filter_ssao.h"
//ccSSAOFilter*	filter_ssao;
//int			ssao_N;		// nb of neighbours
//float		ssao_Kz;	// attenuation with distance
//float		ssao_R;		// radius in image of neighbour sphere
//float		ssao_F;		// amplification
//void	initSSAOFilter()
//{
//	filter_ssao	=	new ccSSAOFilter(wf,hf);
//	filter_ssao->sampleSphere();
//	//
//	ssao_N				=	32;
//	ssao_Kz				=	500.;
//	ssao_R				=	0.05;
//	ssao_F				=	20.;
//}
//void computeSSAO()
//{
//	filter_ssao->setParameters(ssao_N,ssao_Kz,ssao_R,ssao_F);
//	filter_ssao->shade(Fbo_d->getDepthTexture(),ssao_use_reflect*ssao_tex_reflect);
//
//	tex_ssao = filter_ssao->getTexture();
//}
///**************************/

#ifndef	CC_FILTER_SSAO_H
#define	CC_FILTER_SSAO_H

#include <ccGlFilter.h>

class ccShader;
class ccBilateralFilter;
class ccFrameBufferObject;

const unsigned SSAO_MAX_N = 256;

class ccSSAOFilter : public ccGlFilter
{
public:

	ccSSAOFilter();
	virtual ~ccSSAOFilter();

	void reset();

    //inherited from ccGlFilter
	virtual bool init(int width,int height,const char* shadersPath);
	virtual void shade(GLuint texDepth, GLuint texColor, float zoom = 1.0);
	virtual GLuint getTexture();

	bool init(int width,
                int height,
                bool enableBilateralFilter,
                bool useReflectTexture,
                const char* shadersPath,
                GLenum textureMinMagFilter = GL_LINEAR);

	void setParameters(int N, float Kz, float R, float F);

protected:

    void initReflectTexture();
    GLuint texReflect;

	int w;
	int	h;

	ccFrameBufferObject*    fbo;
	ccShader*		        shader;

	int			N;		// nb of neighbours
	float		Kz;		// attenuation with distance
	float		R;		// radius in image of neighbour sphere
	float		F;		// amplification

	void sampleSphere();
	float ssao_neighbours[3*SSAO_MAX_N];	//	full sphere sampling

	ccBilateralFilter*	bilateralFilter;
	bool				bilateralFilterEnabled;
	int					bilateralGSize;
	float				bilateralGSigma;
	float				bilateralGSigmaZ;
};

#endif
