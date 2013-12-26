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
enum CC_VIEW_ORIENTATION {  CC_TOP_VIEW,	/**< Top view (eye: +Z) **/
                            CC_BOTTOM_VIEW,	/**< Bottom view **/
                            CC_FRONT_VIEW,	/**< Front view **/
                            CC_BACK_VIEW,	/**< Back view **/
                            CC_LEFT_VIEW,	/**< Left view **/
                            CC_RIGHT_VIEW,	/**< Right view **/
                            CC_ISO_VIEW_1,	/**< Isometric view 1: front, right and top **/
                            CC_ISO_VIEW_2,	/**< Isometric view 2: back, left and top **/
};

#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGLUtils
#else
class ccGLUtils
#endif
{
public:

	/***************************************************
                    OpenGL Textures
	***************************************************/

    static void DisplayTexture2DPosition(GLuint tex, int x, int y, int w, int h, uchar alpha = 255);
    static void DisplayTexture2D(GLuint tex, int w, int h, uchar alpha = 255);
    static void DisplayTexture2DCorner(GLuint tex, int w, int h, uchar alpha = 255);

	//! Saves a given texture (texID) to an image file
    static bool SaveTextureToFile(const char* filename, GLuint texID, unsigned w, unsigned h);

	//! Loads texture from GPU mem to an RGBA array (i.e. 32 bits per pixel)
	static void SaveTextureToArray(unsigned char* data, GLuint texID, unsigned w, unsigned h);

	/***************************************************
                    OpenGL Matrices
	***************************************************/

	//! Generates the rotation matrix that transforms a vector in another given one
	/** WARNING: an OpenGL context must be active!
		\param sourceVec the original UNIT vector
		\param destVec the resulting UNIT vector
		\return 'OpenGL' style 4x4 matrix
	**/
	static ccGLMatrix GenerateGLRotationMatrixFromVectors(const CCVector3& sourceVec, const CCVector3& destVec);

	//! Generates the rotation matrix corresponding to an axis (vector) and an angle
	/** WARNING: an OpenGL context must be active!
		\param axis the rotation axis (as a vector)
		\param angle_deg the rotation angle (in degrees)
		\return 'OpenGL' style 4x4 matrix
	**/
	static ccGLMatrix GenerateGLRotationMatrixFromAxisAndAngle(const CCVector3& axis, PointCoordinateType angle_deg);

    //! Returns a 4x4 'OpenGL' matrix corresponding to a given vue orientation
    /** \param orientation view orientation
        \return corresponding GL matrix
    **/
    static ccGLMatrix GenerateViewMat(CC_VIEW_ORIENTATION orientation);

	/***************************************************
                    OpenGL Helpers
	***************************************************/

	//! Makes all active GL light sources neutral (i.e. 'gray')
	/** WARNING: an OpenGL context must be active!
	**/
	static void MakeLightsNeutral();

	//! Catches last GL error (if any)
	/** Displays an error message. In debug mode, pauses execution
		and then exits.
		\param context name of the method/object that try to catch the error
		\return true if an error occurred, false otherwise
	**/
	static bool CatchGLError(const char* context);


};

#endif //CC_GL_UTILS_HEADER
