#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

// Qt
#include <QOpenGLShaderProgram>
#include <QSharedPointer>

class ccScalarField;
class QOpenGLFunctions_2_1;

//! OpenGL shader program helper
class ccGLSL
{
  public:
	enum Attribute
	{
		ATTR_POS  = 0,
		ATTR_NOR  = 1,
		ATTR_COL  = 2,
		ATTR_SF   = 4,
		ATTR_VIS  = 8,
		ATTR_CLIP = 16,
		// For internal use only
		ATTR_LOG_SCALE  = 32,
		ATTR_SYM_SCALE  = 64,
		ATTR_HIDDEN_VAL = 128
	};

	//! Builds a shader program for displaying a point cloud
	/** Depending on the provided attributes, the shader program will use the following attribute names:
		- ATTR_POS  = "aPosition"
		- ATTR_NOR  = "aNormalIndex"
		- ATTR_COL  = "aColor"
		- ATTR_SF   = "aSFValue"
		- ATTR_VIS  = "aVisib"

	    And the following uniform names should be set before usage:
	    - uColorScaleTex, uTexWidth, uTexHeight, uMinVal, uMaxVal, uMinSat, uMaxSat, uSatRange, uOutOfRangeGreyScale (for scalar fields)
	    - uNormalLUT, uLUTWidth, uLUTHeight (for normals)
	    - uLight0Enabled, uLight1Enabled (for lighting - e.g. if ATTR_NOR is set)

		\param glFunc OpenGL functions
	    \param attributes bit field of ccGLSL::Attribute
	    \param sf optional scalar field (for color ramp)
	    \return shader program (nullptr if an error occurred)
	**/
	static QSharedPointer<QOpenGLShaderProgram> BuildDisplayProgram(QOpenGLFunctions_2_1* glFunc,
	                                                                int                   attributes,
	                                                                ccScalarField*        sf = nullptr);

	//! Releases OpenGL ressources (GLSL programs)
	static void ReleaseOpenGLRessources();
};
