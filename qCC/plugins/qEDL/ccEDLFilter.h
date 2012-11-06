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
//		FILTER_EDL
//
//		EyeDome Lighting
//		(with bilateral filtering instead of gaussian filtering)
//
//		Output:
//			shaded image
//
//		creation:		23 avril 2008
//					    Christian Boucheny (EDF R&D / INRIA)
//		modification:	18  aout 2008
//					    Christian Boucheny (EDF R&D / INRIA)
//		modification:	Avril 2009
//					    Daniel Girardeau-Montaut
//
/*****************************************************************/
#ifndef	CC_EDL_FILTER_HEADER
#define	CC_EDL_FILTER_HEADER

#include <ccGlFilter.h>

class ccShader;
class ccBilateralFilter;
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
	virtual bool init(int width,int height,const char* shadersPath);
	virtual void shade(GLuint texDepth, GLuint texColor, float zoom = 1.0);
	virtual GLuint getTexture();

    //! Resets filter
    void reset();

    //! Inits filter
    bool init(int width,
                int height,
                GLenum internalFormat,
                GLenum minMagFilter,
                const char* shadersPath);

    //! Real shading process
    void shade(GLuint texDepth, GLuint texColor, float z_min, float z_max, float zoom);

    //! Returns given texture index
    GLuint getTexture(int index);

    //! Sets light direction
	void setLightDir(float theta, float phi);
	//! Sets mouse position
	void setMousePos(int x, int y);

private:

	int	w;
	int	h;

	ccFrameBufferObject*	fbo_edl0;
	ccFrameBufferObject*	fbo_edl1;
	ccFrameBufferObject*	fbo_edl2;
	ccShader*		        shader_edl;

	ccFrameBufferObject*	fbo_mix;
	ccShader*		        shader_mix;

	//
	float	d0;
	float	d1;
	float	d2;
	int		nneighbours;
	float*	neighbours;
	float	F;
	float	power;

	float	mix0;
	float	mix1;
	float	mix2;
	bool	filter0_enabled;
	bool	filter1_enabled;
	bool	filter2_enabled;
	bool	absorb;

	//	Bilateral filtering
	ccBilateralFilter*	filter_bilateral_0;
	ccBilateralFilter*	filter_bilateral_1;
	ccBilateralFilter*	filter_bilateral_2;
	int		G_size_bilateral0;
	float	G_sigma_bilateral0;
	float	G_sigma_bilateral_z0;
	int		G_size_bilateral1;
	float	G_sigma_bilateral1;
	float	G_sigma_bilateral_z1;
	int		G_size_bilateral2;
	float	G_sigma_bilateral2;
	float	G_sigma_bilateral_z2;

	//	FOCUS EN PROFONDEUR
	float	depth_focus;
	bool	b_depth_focus;

	//	FOCUS EN ESPACE IMAGE
	float	screen_focus_x;
	float	screen_focus_y;
	float	screen_focus_sigma;
	bool	b_screen_focus;

	//
	float	light_dir[3];
};
//
//	EyeDome Lighting
//
/////////////////////////////////////////////




#endif
