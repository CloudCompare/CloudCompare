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
#include <QOpenGLTexture>
#include <QSharedPointer>

class ccScalarField;
class QOpenGLFunctions_2_1;

//! OpenGL shader program helper
class ccGLSL
{
  public:
	enum Attribute
	{
		ATTR_POS = 0,
		ATTR_NOR = 1,
		ATTR_COL = 2,
		ATTR_SF  = 4,
		ATTR_VIS = 8,
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

	    If normals are used (ATTR_NOR), a 2D texture containing a normal LUT (see ccGLSL::GetNormalLUTTexture) must be set as uniform
	    (see ccGLSL::SetLUTTextureUniforms) and must be bound to unit 0.

	    If a scalar field is used (ATTR_SF), a 2D texture containing the color ramp (see ccColorScale::getTexture) must be set as uniform
	    (see ccGLSL::SetSFTextureUniforms) and must be bound to unit 1.

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

	//! Returns a 2D texture containing a normal LUT (for OpenGL rendering)
	/** The texture will be created on the first call, and then stored in the shared texture database
	    \param glFunc OpenGL functions (OpenGL 2.1)
	    \return the texture
	**/
	static QSharedPointer<QOpenGLTexture> GetNormalLUTTexture(QOpenGLFunctions_2_1* glFunc);

	//! Sets the uniforms related to a LUT texture (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	    \param lutTex LUT texture
	**/
	static void SetLUTTextureUniforms(QOpenGLFunctions_2_1* glFunc,
	                                  QOpenGLShaderProgram* prog,
	                                  QOpenGLTexture*       lutTex);

	//! Sets the uniforms related to a scalar field texture (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	    \param sfTex LUT texture
	    \param sf scalar field
	**/
	static void SetSFTextureUniforms(QOpenGLFunctions_2_1* glFunc,
	                                 QOpenGLShaderProgram* prog,
	                                 QOpenGLTexture*       sfTex,
	                                 ccScalarField*        sf);

	//! Sets the uniforms related to lighting (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	**/
	static void SetLightUniforms(QOpenGLFunctions_2_1* glFunc,
	                             QOpenGLShaderProgram* prog);

  protected:
	//! Creates a 2D texture containing a normal LUT (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \return created texture
	**/
	static QSharedPointer<QOpenGLTexture> CreateNormalLUTTexture(QOpenGLFunctions_2_1* glFunc);
};
