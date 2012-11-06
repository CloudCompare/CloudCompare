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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_GL_UTILS
#define CC_GL_UTILS

//qCC_db
#include <ccIncludeGL.h>
#include <ccGLMatrix.h>

//! View orientation
enum CC_VIEW_ORIENTATION {  CC_TOP_VIEW,
                            CC_BOTTOM_VIEW,
                            CC_FRONT_VIEW,
                            CC_BACK_VIEW,
                            CC_LEFT_VIEW,
                            CC_RIGHT_VIEW,
};

class ccGLUtils
{
public:

	/***************************************************
                    OpenGL Textures
	***************************************************/

    static void DisplayTexture2DPosition(GLuint tex, int x, int y, int w, int h, unsigned char alpha=255);
    static void DisplayTexture2D(GLuint tex, int w, int h, unsigned char alpha=255);
    static void DisplayTexture2DCorner(GLuint tex, int w, int h, unsigned char alpha=255);

    static bool SaveTextureToFile(const char* filename, GLuint texID, unsigned w, unsigned h);

	//! Loads texture from GPU mem to an RGBA array (i.e. 32 bits per pixel)
	static void SaveTextureToArray(unsigned char* data, GLuint texID, unsigned w, unsigned h);

	/***************************************************
                    OpenGL Matrices
	***************************************************/

	//! Generates the rotation matrix that transforms a vector in another given one
	/** WARNING: an OpenGL context must be active!
		\param source the original UNIT vector
		\param dest the resulting UNIT vector
		\return 'OpenGL' style 4x4 matrix
	**/
	static ccGLMatrix GenerateGLRotationMatrixFromVectors(const float* sourceVec, const float* destVec);

	//! Generates the rotation matrix corresponding to an axis (vector) and an angle
	/** WARNING: an OpenGL context must be active!
		\param axis the rotation axis (as a vector)
		\param angle the rotation angle (in degrees)
		\return 'OpenGL' style 4x4 matrix
	**/
	static ccGLMatrix GenerateGLRotationMatrixFromAxisAndAngle(const float* axis, float angle_deg);

	//! Multiplies tow OpenGL style matrices
	/** This method uses OpenGL by default.
		\param A the first matrix
		\param B the second matrix
		\param dest the resulting matrix (A*B)
	**/
	static void MultGLMatrices(const float* A, const float* B, float* dest);

	//! Transposes an OpenGL style matrix
	/** \param A a matrix (4*4=16 values)
		\param dest the resulting matrix (tA)
	**/
	static void TransposeGLMatrix(const float* A, float* dest);

    //! Returns a 4x4 'OpenGL' matrix corresponding to a given vue orientation
    /** \param orientation view orientation
        \return corresponding GL matrix
    **/
    static ccGLMatrix GenerateViewMat(CC_VIEW_ORIENTATION orientation);


	/***************************************************
                    OpenGL Extensions
	***************************************************/

    //! Loads all available OpenGL extensions
    static bool InitGLEW();

    //! Checks for availability of a given OpenGL extension
    static bool CheckExtension(const char *extName);

    //! Shortcut: checks Shaders support
    static bool CheckShadersAvailability();

    //! Shortcut: checks FBO support
    static bool CheckFBOAvailability();

	//! Shortcut: checks VBO support
    static bool CheckVBOAvailability();

	//! Catches last GL error (if any)
	/** Displays an error message. In debug mode, pauses execution
		and then exits.
		\param context name of the method/object that try to catch the error
		\return true if an error occured, false otherwise
	**/
	static bool CatchGLError(const char* context);


};

#endif
