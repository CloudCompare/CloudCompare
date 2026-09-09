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

#include "../include/ccGLSLHelper.h"

// Local
#include "../include/ccMaterialDB.h"
#include "../include/ccNormalVectors.h"
#include "../include/ccScalarField.h"

// Qt
#include <QOpenGLFunctions_2_1>

//! Map or already built shader programs
static QMap<int, QSharedPointer<QOpenGLShaderProgram>> s_programs;

void ccGLSL::ReleaseOpenGLRessources()
{
	if (!QOpenGLContext::currentContext())
	{
		ccLog::Warning("[ccGLSL::ReleaseOpenGLRessources] No valid OpenGL context");
		return;
	}

	s_programs.clear();
}

QSharedPointer<QOpenGLShaderProgram> ccGLSL::BuildDisplayProgram(QOpenGLFunctions_2_1* glFunc,
                                                                 int                   attributes,
                                                                 ccScalarField*        sf /*=nullptr*/)
{
	if (!glFunc)
	{
		assert(false);
		return nullptr;
	}

	// add secondary attributes
	if (attributes & ATTR_SF_FLAG)
	{
		if (!sf)
		{
			assert(false);
			return nullptr;
		}

		if (sf->logScale())
		{
			attributes |= ATTR_LOG_SCALE_FLAG;
		}
		else if (sf->symmetricalScale())
		{
			attributes |= ATTR_SYM_SCALE_FLAG;
		}
		if (!sf->areNaNValuesShownInGrey())
		{
			attributes |= ATTR_HIDDEN_VAL_FLAG;
		}
	}

	if (s_programs.contains(attributes))
	{
		return s_programs[attributes];
	}

	// Vertex programs
	static const char* VertexProgHeaderSrc =
	    "#version 120\n"
	    "attribute vec3 aPosition;\n"
	    "varying vec4 vColor;\n"
	    "varying vec4 vVertexPos;\n";

	static const char* VertexProgVisibilityAttributesSrc =
	    "attribute float aVisib;\n";

	static const char* VertexProgColorAttributesSrc =
	    "attribute vec4 aColor;\n";

	static const char* VertexProgTextureAttributesSrc =
	    "attribute vec2 aTexCoord;\n"
	    "varying vec2 vTexCoord;\n";

	static const char* VertexProgSFAttributesSrc =
	    "attribute float aSFValue;\n"
	    "uniform sampler2D uColorScaleTex;\n"
	    "uniform int uTexWidth;\n"
	    "uniform int uTexHeight;\n"
	    "uniform float uMinVal;\n"
	    "uniform float uMaxVal;\n"
	    "uniform float uMinSat;\n"
	    "uniform float uMaxSat;\n"
	    "uniform float uSatRange;\n"
	    "uniform float uOutOfRangeGreyScale;\n";

	static const char* VertexProgNormAttributesSrc =
	    "attribute float aNormalIndex;\n"
	    "varying vec3 vNormal;\n"
	    "uniform sampler2D uNormalLUT;\n"
	    "uniform int uLUTWidth;\n"
	    "uniform int uLUTHeight;\n";

	static const char* VertexProgFetchNormFuncSrc =
	    "vec3 fetchNormalFromLUT(float fi)\n"
	    "{\n"
	    "	float w  = float(uLUTWidth);\n"
	    "	float h  = float(uLUTHeight);\n"
	    "	float tx = mod(fi, w);\n"
	    "	float ty = floor(fi / w);\n"
	    "	vec2 uv = vec2((tx + 0.5) / w, (ty + 0.5) / h);\n"
	    "	vec3 enc = texture2D(uNormalLUT, uv).rgb;\n"
	    "	vec3 n = enc * 2.0 - 1.0;\n"
	    "	return normalize(n);\n"
	    "}\n";

	static const char* VertexProgFetchSFColorFuncSrc =
	    "vec4 fetchColorFromTex(float sfVal)\n"
	    "{\n"
	    "   float v = normalizeSFVal(sfVal);\n"
	    "   v = v * float(uTexWidth * uTexHeight - 1);\n"
	    "	float w  = float(uTexWidth);\n"
	    "	float h  = float(uTexHeight);\n"
	    "	float tx = mod(v, w);\n"
	    "	float ty = floor(v/ w);\n"
	    "	vec2 uv = vec2(tx / w, ty / h);\n"
	    "	vec4 color = texture2D(uColorScaleTex, uv);\n"
	    "	return color;\n"
	    "}\n";

	static const char* NormalizeNonSymmetricalValueFuncSrc =
	    "float normalizeSFVal(float sfVal)\n"
	    "{\n"
	    "   if (sfVal <= uMinSat)\n"
	    "      return 0.0;\n"
	    "   if (sfVal >= uMaxSat)\n"
	    "      return 1.0;\n"
	    "   return (sfVal - uMinSat) / uSatRange;\n"
	    "}\n";

	static const char* NormalizeSymmetricalValueFuncSrc =
	    "float normalizeSFVal(float sfVal)\n"
	    "{\n"
	    "   if (abs(sfVal) <= uMinSat)\n"
	    "      return 0.5;\n"
	    "   if (sfVal >= 0)\n"
	    "   {\n"
	    "      if (sfVal > uMaxSat)\n"
	    "         return 1.0;\n"
	    "      return (1.0 + (sfVal - uMinSat) / uSatRange) / 2.0;\n"
	    "   }\n"
	    "   else\n"
	    "   {\n"
	    "      if (sfVal < -uMaxSat)\n"
	    "         return 0.0;\n"
	    "      return (1.0 + (sfVal + uMinSat) / uSatRange) / 2.0;\n"
	    "   }\n"
	    "}\n";

	static const char* NormalizeValueLogScaleFuncSrc =
	    "float normalizeSFVal(float sfVal)\n"
	    "{\n"
	    " 	float dLog = log(max(abs(sfVal), 0.00001)) / log(10.0);\n"
	    "   if (dLog <= uMinSat)\n"
	    "      return 0.0;\n"
	    "   if (dLog >= uMaxSat)\n"
	    "      return 1.0;\n"
	    "   return (dLog - uMinSat) / uSatRange;\n"
	    "}\n";

	static const char* VertexProgMainStartSrc =
	    "void main()\n"
	    "{\n";

	static const char* VertexProgMainUseDefaultGLColorSrc =
	    "   vColor = gl_Color;\n";

	static const char* VertexProgMainTextureCoordSrc =
	    "   vTexCoord = aTexCoord;\n";

	static const char* VertexProgMainUseInputColorSrc =
	    "   vColor = aColor;\n";

	static const char* VertexProgMainFetchSFColorSrc =
	    "   if (aSFValue >= uMinVal && aSFValue <= uMaxVal)\n"
	    "   {\n"
	    "      vColor = fetchColorFromTex(aSFValue);\n"
	    "   }\n"
	    "   else\n"
	    "   {\n"
	    "      vColor = vec4(uOutOfRangeGreyScale, uOutOfRangeGreyScale, uOutOfRangeGreyScale, 1.0);\n"
	    "   }\n";

	static const char* VertexProgMainFetchSFColorHideNaNSrc =
	    "   if (aSFValue >= uMinVal && aSFValue <= uMaxVal)\n"
	    "   {\n"
	    "      vColor = fetchColorFromTex(aSFValue);\n"
	    "   }\n"
	    "   else\n"
	    "   {\n"
	    "      gl_Position = vec4(0, 0, 0, -1.0);\n" // impossible position, should be discarded by the GPU
	    "      return;\n"
	    "   }\n";

	static const char* VertexProgMainTestVisibilitySrc =
	    "   if (aVisib > 0.0)\n" // CCCoreLib::POINT_VISIBLE = 0
	    "   {\n"
	    "      gl_Position = vec4(0, 0, 0, -1.0);\n" // impossible position, should be discarded by the GPU
	    "      return;\n"
	    "   }\n";

	static const char* VertexProgMainFetchNormalSrc =
	    "   vNormal = gl_NormalMatrix * fetchNormalFromLUT(aNormalIndex);\n";

	static const char* VertexProgMainEndSrc =
	    "   gl_ClipVertex = gl_ModelViewMatrix * vec4(aPosition, 1.0);\n"
	    "   vVertexPos = gl_ClipVertex;\n"
	    "   gl_Position = gl_ModelViewProjectionMatrix * vec4(aPosition, 1.0);\n"
	    "}\n";

	// Fragment programs
	static const char* FragmentProgStartSrc =
	    "#version 120\n";

	static const char* FragmentProgColorAttributesSrc =
	    "varying vec4 vColor;\n";

	static const char* FragmentProgTextureAttributesSrc =
	    "varying vec2 vTexCoord;"
	    "uniform sampler2D uTexture;\n";

	static const char* FragmentProgNormalAttributesSrc =
	    "varying vec3 vNormal;\n"
	    "varying vec4 vVertexPos;\n"
	    "uniform int uLight0Enabled;\n"
	    "uniform int uLight1Enabled;\n"
	    "uniform int uLightModelTwoSide;\n";

	static const char* FragmentProgCalculateLightingSrc =
	    "vec4 calculateLighting(vec3 normVec, gl_LightSourceParameters lightSource, gl_MaterialParameters matParams)\n"
	    "{\n"
	    "    vec3 L = normalize(lightSource.position.xyz - vVertexPos.xyz);\n" // we are in Eye Coordinates, so EyePos is (0,0,0)
	    "    vec3 E = normalize(-vVertexPos.xyz);\n"
	    "    vec3 R = normalize(-reflect(L,normVec));\n"
	    "    vec4 Iamb = lightSource.ambient;\n"                                                                          // calculate Ambient Term:
	    "    vec4 Idiff = lightSource.diffuse * max(dot(normVec, L), 0.0);\n"                                             // calculate Diffuse Term
	    "    vec4 Ispec = lightSource.specular * pow(max(dot(R,E), 0.0), matParams.shininess);\n"                         // calculate Specular Term
	    "    vec4 normColor = (matParams.ambient * Iamb) + (matParams.diffuse * Idiff) + (matParams.specular * Ispec);\n" // write Total Color
	    "    return normColor;\n"
	    "}\n";

	static const char* FragmentProgMainStartSrc =
	    "void main()\n"
	    "{\n"
	    "   gl_FragColor = vColor;\n";

	static const char* FragmentProgMainTextureSrc =
	    "   gl_FragColor *= texture2D(uTexture, vTexCoord);\n";

	static const char* FragmentProgMainNormalSrc =
	    "    vec4 normColor = gl_FrontLightModelProduct.sceneColor;\n"
	    "    if (uLight0Enabled == 1)\n"
	    "    {\n"
	    "        normColor += calculateLighting(vNormal, gl_LightSource[0], gl_FrontMaterial);\n"
	    "    }\n"
	    "    if (uLight1Enabled == 1)\n"
	    "    {\n"
	    "        normColor += calculateLighting(vNormal, gl_LightSource[1], gl_FrontMaterial);\n"
	    "    }\n"
	    "    if (uLightModelTwoSide == 1)\n"
	    "    {\n"
	    "        normColor += gl_BackLightModelProduct.sceneColor;\n"
	    "        if (uLight0Enabled == 1)\n"
	    "        {\n"
	    "            normColor += calculateLighting(-vNormal, gl_LightSource[0], gl_BackMaterial);\n"
	    "        }\n"
	    "        if (uLight1Enabled == 1)\n"
	    "        {\n"
	    "            normColor += calculateLighting(-vNormal, gl_LightSource[1], gl_BackMaterial);\n"
	    "        }\n"
	    "    }\n"
	    "    normColor = clamp(normColor, 0.0, 1.0);\n"
	    "    gl_FragColor *= normColor;\n";

	static const char* FragmentProgMainEndSrc =
	    "}\n";

	QString vertexProgSrc = VertexProgHeaderSrc;
	{
		// add attributes
		if (attributes & ATTR_TEX_FLAG)
			vertexProgSrc += VertexProgTextureAttributesSrc;
		if (attributes & ATTR_SF_FLAG)
			vertexProgSrc += VertexProgSFAttributesSrc;
		else if (attributes & ATTR_COL_FLAG)
			vertexProgSrc += VertexProgColorAttributesSrc;
		if (attributes & ATTR_NOR_FLAG)
			vertexProgSrc += VertexProgNormAttributesSrc;
		if (attributes & ATTR_VIS_FLAG)
			vertexProgSrc += VertexProgVisibilityAttributesSrc;

		// add special functions
		if (attributes & ATTR_SF_FLAG)
		{
			if (attributes & ATTR_LOG_SCALE_FLAG)
			{
				vertexProgSrc += NormalizeValueLogScaleFuncSrc;
			}
			else
			{
				vertexProgSrc += ((attributes & ATTR_SYM_SCALE_FLAG) ? NormalizeSymmetricalValueFuncSrc : NormalizeNonSymmetricalValueFuncSrc);
			}
			vertexProgSrc += VertexProgFetchSFColorFuncSrc;
		}

		if (attributes & ATTR_NOR_FLAG)
		{
			vertexProgSrc += VertexProgFetchNormFuncSrc;
		}

		// main function
		{
			vertexProgSrc += VertexProgMainStartSrc;

			if (attributes & ATTR_VIS_FLAG)
			{
				vertexProgSrc += VertexProgMainTestVisibilitySrc;
			}

			if (attributes & ATTR_TEX_FLAG)
			{
				vertexProgSrc += VertexProgMainTextureCoordSrc;
			}

			if (attributes & ATTR_SF_FLAG)
			{
				vertexProgSrc += ((attributes & ATTR_HIDDEN_VAL_FLAG) ? VertexProgMainFetchSFColorHideNaNSrc : VertexProgMainFetchSFColorSrc);
			}
			else if (attributes & ATTR_COL_FLAG)
			{
				vertexProgSrc += VertexProgMainUseInputColorSrc;
			}
			else
			{
				vertexProgSrc += VertexProgMainUseDefaultGLColorSrc;
			}

			// normal transfer (if any)
			if (attributes & ATTR_NOR_FLAG)
			{
				vertexProgSrc += VertexProgMainFetchNormalSrc;
			}

			vertexProgSrc += VertexProgMainEndSrc;
		}
	}

	QString fragmentProgSrc = FragmentProgStartSrc;
	{
		fragmentProgSrc += FragmentProgColorAttributesSrc;

		// add attributes
		if (attributes & ATTR_TEX_FLAG)
		{
			fragmentProgSrc += FragmentProgTextureAttributesSrc;
		}

		if (attributes & ATTR_NOR_FLAG)
		{
			fragmentProgSrc += FragmentProgNormalAttributesSrc;
		}

		// add special functions
		if (attributes & ATTR_NOR_FLAG)
		{
			fragmentProgSrc += FragmentProgCalculateLightingSrc;
		}

		// main function
		{
			// display colors
			fragmentProgSrc += FragmentProgMainStartSrc;

			// modulate with texture
			if (attributes & ATTR_TEX_FLAG)
			{
				fragmentProgSrc += FragmentProgMainTextureSrc;
			}

			// modulate with normals
			if (attributes & ATTR_NOR_FLAG)
			{
				fragmentProgSrc += FragmentProgMainNormalSrc;
			}

			fragmentProgSrc += FragmentProgMainEndSrc;
		}
	}

	ccLog::PrintDebug("Vertex shader (attributes: " + QString::number(attributes) + "):\n" + vertexProgSrc);
	QOpenGLShader vertexShader(QOpenGLShader::Vertex);
	if (false == vertexShader.compileSourceCode(vertexProgSrc))
	{
		ccLog::Warning(QString("[BuildDisplayProgram] Vertex shader compilation failed: ") + vertexShader.log());
		return nullptr;
	}

	ccLog::PrintDebug("Fragment shader (attributes: " + QString::number(attributes) + "):\n" + fragmentProgSrc);
	QOpenGLShader fragmentShader(QOpenGLShader::Fragment);
	if (false == fragmentShader.compileSourceCode(fragmentProgSrc))
	{
		ccLog::Warning(QString("[BuildDisplayProgram] Fragment shader compilation failed: ") + fragmentShader.log());
		return nullptr;
	}
	QSharedPointer<QOpenGLShaderProgram> program(new QOpenGLShaderProgram);
	program->addShader(&vertexShader);
	program->addShader(&fragmentShader);

	// bind attribute locations before linking for stable locations
	program->bindAttributeLocation("aPosition", ATTR_POS_ARRAY);
	if (attributes & ATTR_VIS_FLAG)
	{
		program->bindAttributeLocation("aVisib", ATTR_VIS_ARRAY);
	}
	if (attributes & ATTR_SF_FLAG)
	{
		assert((attributes & ATTR_COL_FLAG) == 0);
		program->bindAttributeLocation("aSFValue", ATTR_SF_ARRAY);
	}
	else if (attributes & ATTR_COL_FLAG)
	{
		assert((attributes & ATTR_SF_FLAG) == 0);
		program->bindAttributeLocation("aColor", ATTR_COL_ARRAY);
	}
	if (attributes & ATTR_TEX_FLAG)
	{
		program->bindAttributeLocation("aTexCoord", ATTR_TEX_ARRAY);
	}
	if (attributes & ATTR_NOR_FLAG)
	{
		program->bindAttributeLocation("aNormalIndex", ATTR_NOR_ARRAY);
	}

	if (false == program->link())
	{
		ccLog::Warning(QString("[BuildDisplayProgram] Shader program linking failed: ") + program->log());
		return nullptr;
	}

	s_programs[attributes] = program;

	return program;
}

QSharedPointer<QOpenGLTexture> ccGLSL::CreateNormalLUTTexture(QOpenGLFunctions_2_1* glFunc)
{
	if (!glFunc)
	{
		assert(false);
		return nullptr;
	}

	const unsigned totalNormals = ccNormalCompressor::MAX_VALID_NORM_CODE + 1; // number of valid codes
	// Query max texture size
	GLint maxTexSize = 0;
	glFunc->glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
	if (maxTexSize <= 0)
	{
		return nullptr;
	}

	// choose width as large as possible (but <= maxTexSize) to minimize height
	int width  = std::min(static_cast<int>(totalNormals), maxTexSize);
	int height = static_cast<int>(std::ceil(static_cast<float>(totalNormals) / float(width)));

	const size_t texels = static_cast<size_t>(width) * static_cast<size_t>(height);

	// buffer RGB unsigned bytes
	std::vector<unsigned char> pixels;
	try
	{
		pixels.resize(texels * 3);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccMesh::CreateNormalLUTTexture] Not enough memory");
		return nullptr;
	}

	// fill: for each index, call ccNormalCompressor::Decompress
	for (unsigned i = 0; i < totalNormals; ++i)
	{
		CCVector3 n = ccNormalVectors::GetNormal(i);

		// map [-1,1] -> [0,255]
		// handle NULL_NORM_CODE: Decompress sets to 0, so it becomes 127 -> you can change if needed
		pixels[i * 3 + 0] = static_cast<unsigned char>(std::round(std::clamp((n.x * 0.5 + 0.5), 0.0, 1.0) * 255.0));
		pixels[i * 3 + 1] = static_cast<unsigned char>(std::round(std::clamp((n.y * 0.5 + 0.5), 0.0, 1.0) * 255.0));
		pixels[i * 3 + 2] = static_cast<unsigned char>(std::round(std::clamp((n.z * 0.5 + 0.5), 0.0, 1.0) * 255.0));
	}

	// generate GL texture
	QSharedPointer<QOpenGLTexture> tex(new QOpenGLTexture(QOpenGLTexture::Target2D));

	// configure texture
	tex->setFormat(QOpenGLTexture::RGB8_UNorm);
	tex->setSize(width, height);
	tex->allocateStorage();
	tex->setWrapMode(QOpenGLTexture::ClampToEdge);
	tex->setMinificationFilter(QOpenGLTexture::Nearest);
	tex->setMagnificationFilter(QOpenGLTexture::Nearest);

	// upload data
	tex->setData(QOpenGLTexture::RGB, QOpenGLTexture::UInt8, pixels.data());

	return tex;
}

QSharedPointer<QOpenGLTexture> ccGLSL::GetNormalLUTTexture(QOpenGLFunctions_2_1* glFunc)
{
	// If not done already, create the LUT texture and bind it to unit 0
	if (nullptr == ccMaterial::GetTextureDB())
	{
		assert(false);
		return nullptr;
	}

	QSharedPointer<QOpenGLTexture> lutTex = ccMaterial::GetTextureDB()->getOpenGLTexture("CompressedNormalsLUT");
	if (!lutTex.isNull())
	{
		return lutTex;
	}

	// try to create it if necessary
	lutTex = CreateNormalLUTTexture(glFunc);
	if (!lutTex.isNull())
	{
		ccMaterial::GetTextureDB()->addOpenGLTexture("CompressedNormalsLUT", lutTex);
	}

	return lutTex;
}

void ccGLSL::SetTextureUniforms(QOpenGLFunctions_2_1* glFunc,
                                QOpenGLShaderProgram* prog,
                                GLint                 textureUnit /*=0*/)
{
	if (!glFunc || !prog || textureUnit < 0)
	{
		assert(false);
		return;
	}

	glFunc->glUniform1i(prog->uniformLocation("uTexture"), textureUnit);
}

void ccGLSL::SetLUTTextureUniforms(QOpenGLFunctions_2_1* glFunc,
                                   QOpenGLShaderProgram* prog,
                                   QOpenGLTexture*       lutTex,
                                   GLint                 textureUnit /*=1*/)
{
	if (!glFunc || !prog || !lutTex || textureUnit < 0)
	{
		assert(false);
		return;
	}
	// set sampler uniform to unit 0
	glFunc->glUniform1i(prog->uniformLocation("uNormalLUT"), textureUnit);
	// set LUT dimensions
	glFunc->glUniform1i(prog->uniformLocation("uLUTWidth"), lutTex->width());
	glFunc->glUniform1i(prog->uniformLocation("uLUTHeight"), lutTex->height());
}

void ccGLSL::SetSFTextureUniforms(QOpenGLFunctions_2_1* glFunc,
                                  QOpenGLShaderProgram* prog,
                                  QOpenGLTexture*       sfTex,
                                  ccScalarField*        sf,
                                  GLint                 textureUnit /*=2*/)
{
	if (!glFunc || !prog || !sfTex || !sf || textureUnit < 0)
	{
		assert(false);
		return;
	}
	// set sampler uniform to unit 1
	glFunc->glUniform1i(prog->uniformLocation("uColorScaleTex"), textureUnit);
	// set texture dimensions
	glFunc->glUniform1i(prog->uniformLocation("uTexWidth"), sfTex->width());
	glFunc->glUniform1i(prog->uniformLocation("uTexHeight"), sfTex->height());

	auto   colorScale = sf->getColorScale();
	double offset     = sf->getOffset();

	float minVal              = static_cast<float>(sf->displayRange().start() - offset);
	float maxVal              = static_cast<float>(sf->displayRange().stop() - offset);
	float minSat              = static_cast<float>(sf->saturationRange().start() - offset);
	float maxSat              = static_cast<float>(sf->saturationRange().stop() - offset);
	float satRange            = static_cast<float>(sf->saturationRange().range());
	float outOfRangeGreyScale = ccColor::lightGreyRGB.r / 255.0f;

	glFunc->glUniform1f(prog->uniformLocation("uMinVal"), minVal);
	glFunc->glUniform1f(prog->uniformLocation("uMaxVal"), maxVal);
	glFunc->glUniform1f(prog->uniformLocation("uMinSat"), minSat);
	glFunc->glUniform1f(prog->uniformLocation("uMaxSat"), maxSat);
	glFunc->glUniform1f(prog->uniformLocation("uSatRange"), satRange);
	glFunc->glUniform1f(prog->uniformLocation("uOutOfRangeGreyScale"), outOfRangeGreyScale);
}

void ccGLSL::SetLightUniforms(QOpenGLFunctions_2_1* glFunc,
                              QOpenGLShaderProgram* prog)
{
	if (!glFunc || !prog)
	{
		assert(false);
		return;
	}

	glFunc->glUniform1i(prog->uniformLocation("uLight0Enabled"), glFunc->glIsEnabled(GL_LIGHT0) ? 1 : 0);
	glFunc->glUniform1i(prog->uniformLocation("uLight1Enabled"), glFunc->glIsEnabled(GL_LIGHT1) ? 1 : 0);
	GLboolean twoSide;
	glFunc->glGetBooleanv(GL_LIGHT_MODEL_TWO_SIDE, &twoSide);
	glFunc->glUniform1i(prog->uniformLocation("uLightModelTwoSide"), twoSide ? 1 : 0);
}
