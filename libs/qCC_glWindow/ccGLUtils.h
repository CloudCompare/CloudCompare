//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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

#ifndef CC_GL_UTILS_HEADER
#define CC_GL_UTILS_HEADER

//qCC_db
#include <ccIncludeGL.h> //Always first!
#include <ccGLMatrix.h>

//! View orientation
enum CC_VIEW_ORIENTATION {	CC_TOP_VIEW,	/**< Top view (eye: +Z) **/
							CC_BOTTOM_VIEW,	/**< Bottom view **/
							CC_FRONT_VIEW,	/**< Front view **/
							CC_BACK_VIEW,	/**< Back view **/
							CC_LEFT_VIEW,	/**< Left view **/
							CC_RIGHT_VIEW,	/**< Right view **/
							CC_ISO_VIEW_1,	/**< Isometric view 1: front, right and top **/
							CC_ISO_VIEW_2,	/**< Isometric view 2: back, left and top **/
};

class ccGLUtils
{
public:

	/***************************************************
					OpenGL Textures
	***************************************************/

	static void DisplayTexture2DPosition(GLuint tex, int x, int y, int w, int h, uchar alpha = 255);
	static void DisplayTexture2D(GLuint tex, int w, int h, uchar alpha = 255);

	/***************************************************
					OpenGL Matrices
	***************************************************/

	//! Returns a 4x4 'OpenGL' matrix corresponding to a default 'view' orientation
	/** \param orientation view orientation
		\return corresponding GL matrix
	**/
	static ccGLMatrixd GenerateViewMat(CC_VIEW_ORIENTATION orientation);

	/***************************************************
					OpenGL Helpers
	***************************************************/

	//! Catches last GL error (if any)
	/** Displays an error message. In debug mode, pauses execution
		and then exits.
		\param context name of the method/object that try to catch the error
		\return true if an error occurred, false otherwise
	**/
	static bool CatchGLError(const char* context);

};

#endif //CC_GL_UTILS_HEADER
