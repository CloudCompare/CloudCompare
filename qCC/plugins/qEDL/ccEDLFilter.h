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

	int	m_screenWidth;
	int	m_screenHeight;

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
	float	neighbours[8*4];
	float	F;
	float	power;

	float	mix0;
	float	mix1;
	float	mix2;
	bool	absorb;

	//! Bilateral filter and ist parameters
	struct BilateralFilter
	{
		ccBilateralFilter* filter;
		int size;
		float sigma;
		float sigmaZ;
		bool enabled;

		BilateralFilter()
			: filter(0)
			, size(0)
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
