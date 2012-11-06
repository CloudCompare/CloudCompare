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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1685                                                              $
//$LastChangedDate:: 2010-10-20 23:13:01 +0200 (mer., 20 oct. 2010)        $
//**************************************************************************
//

#ifndef CC_FBO_UTILS
#define CC_FBO_UTILS

#include "ccGlew.h"

#define	TEX_1D_ON	glEnable(GL_TEXTURE_1D)
#define	TEX_1D_OFF	glDisable(GL_TEXTURE_1D)
#define	TEX_2D_ON	glEnable(GL_TEXTURE_2D)
#define	TEX_2D_OFF	glDisable(GL_TEXTURE_2D)
#define	TEX_3D_ON	glEnable(GL_TEXTURE_3D)
#define	TEX_3D_OFF	glDisable(GL_TEXTURE_3D)

class ccFBOUtils
{
public:

	/***************************************************
                    OpenGL Textures
	***************************************************/

    static void DisplayTexture2D(GLuint tex, int w, int h);
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

    //! Shortcut: checks FBO support
    static bool CheckFBOAvailability();
};

#endif
