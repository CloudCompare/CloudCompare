//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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

#ifndef CC_GL_UTILS_HEADER
#define CC_GL_UTILS_HEADER

//qCC_db
#include <ccIncludeGL.h> //Always first!

//Qt
#include <QImage>

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

	static void DisplayTexture2DPosition(QImage image, int x, int y, int w, int h, unsigned char alpha = 255);
	inline static void DisplayTexture2D(QImage image, int w, int h, unsigned char alpha = 255) { DisplayTexture2DPosition(image, -w / 2, -h / 2, w, h, alpha); }

	static void DisplayTexture2DPosition(GLuint texID, int x, int y, int w, int h, unsigned char alpha = 255);
	inline static void DisplayTexture2D(GLuint texID, int w, int h, unsigned char alpha = 255) { DisplayTexture2DPosition(texID, -w / 2, -h / 2, w, h, alpha); }

	/***************************************************
					OpenGL Matrices
	***************************************************/

	//! Returns a 4x4 'OpenGL' matrix corresponding to a default 'view' orientation
	/** \param orientation view orientation
		\return corresponding GL matrix
	**/
	static ccGLMatrixd GenerateViewMat(CC_VIEW_ORIENTATION orientation);

};

#endif //CC_GL_UTILS_HEADER
