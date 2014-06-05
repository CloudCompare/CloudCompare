//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_FBO_UTILS
#define CC_FBO_UTILS

#include "ccGlew.h"

//! Misc. FBO management related methods
class ccFBOUtils
{
public:

	/***************************************************
					OpenGL Textures
	***************************************************/

	static void DisplayTexture2DCorner(GLuint tex, int w, int h);

	/***************************************************
					OpenGL Extensions
	***************************************************/

	//! Loads all available OpenGL extensions
	static bool InitGLEW();

	//! Checks for availability of a given OpenGL extension
	static bool CheckExtension(const char *extName);

	//! Shortcut: checks Shaders support
	static bool CheckShadersAvailability();

	//! Shortcut: checks OpenGL FBO support
	static bool CheckFBOAvailability();

	//! Shortcut: checks OpenGL VBO support
	static bool CheckVBOAvailability();

	//! Shortcut: checks OpenGL VA support
	static bool CheckVAAvailability();

};

#endif
