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
	//! Attribute flags
	enum AttributeFlags
	{
		ATTR_POS_FLAG = 0,
		ATTR_NOR_FLAG = 1,
		ATTR_COL_FLAG = 2,
		ATTR_SF_FLAG  = 4,
		ATTR_VIS_FLAG = 8,
		ATTR_TEX_FLAG = 16,
		// For internal use only
		ATTR_LOG_SCALE_FLAG  = 32,
		ATTR_SYM_SCALE_FLAG  = 64,
		ATTR_HIDDEN_VAL_FLAG = 128
	};

	//! Array indexes
	enum ArrayIndexes
	{
		ATTR_POS_ARRAY = 0,
		ATTR_NOR_ARRAY = 1,
		ATTR_COL_ARRAY = 2,
		ATTR_SF_ARRAY  = 3,
		ATTR_VIS_ARRAY = 4,
		ATTR_TEX_ARRAY = 5
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
	    (see ccGLSL::SetLUTTextureUniforms) and should be bound to unit 1 by default.

	    If a scalar field is used (ATTR_SF), a 2D texture containing the color ramp (see ccColorScale::getTexture) must be set as uniform
	    (see ccGLSL::SetSFTextureUniforms) and should be bound to unit 2 by default.

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

	//! Sets the uniforms related to texture (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	    \param textureUnit texture unit to which the texture is bound
	**/
	static void SetTextureUniforms(QOpenGLFunctions_2_1* glFunc,
	                               QOpenGLShaderProgram* prog,
	                               GLint                 textureUnit = 0);

	//! Sets the uniforms related to a LUT texture (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	    \param lutTex LUT texture
	    \param textureUnit texture unit to which the texture is bound
	**/
	static void SetLUTTextureUniforms(QOpenGLFunctions_2_1* glFunc,
	                                  QOpenGLShaderProgram* prog,
	                                  QOpenGLTexture*       lutTex,
	                                  GLint                 textureUnit = 1);

	//! Sets the uniforms related to a scalar field texture (for OpenGL rendering)
	/** \param glFunc OpenGL functions (OpenGL 2.1)
	    \param prog shader program
	    \param sfTex LUT texture
	    \param sf scalar field
	    \param textureUnit texture unit to which the texture is bound
	**/
	static void SetSFTextureUniforms(QOpenGLFunctions_2_1* glFunc,
	                                 QOpenGLShaderProgram* prog,
	                                 QOpenGLTexture*       sfTex,
	                                 ccScalarField*        sf,
	                                 GLint                 textureUnit = 2);

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
