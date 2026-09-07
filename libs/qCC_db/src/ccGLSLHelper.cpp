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
	if (attributes & ATTR_SF)
	{
		if (!sf)
		{
			assert(false);
			return nullptr;
		}

		if (sf->logScale())
		{
			attributes |= ATTR_LOG_SCALE;
		}
		else if (sf->symmetricalScale())
		{
			attributes |= ATTR_SYM_SCALE;
		}
		if (!sf->areNaNValuesShownInGrey())
		{
			attributes |= ATTR_HIDDEN_VAL;
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
	    "    vColor = gl_Color;\n";

	static const char* VertexProgMainUseInputColorSrc =
	    "    vColor = aColor;\n";

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
	static const char* ColorOnlyFragmentProgSrc =
	    "#version 120\n"
	    "varying vec4 vColor;\n"
	    "void main()\n"
	    "{\n"
	    "    gl_FragColor = vColor;\n"
	    "}\n";

	static const char* ColorAndNormalFragmentProgSrc =
	    "#version 120\n"
	    "varying vec4 vColor;\n"
	    "varying vec3 vNormal;\n"
	    "varying vec4 vVertexPos;\n"
	    "uniform int uLight0Enabled;\n"
	    "uniform int uLight1Enabled;\n"
	    "void main()\n"
	    "{\n"
	    "   gl_FragColor = gl_FrontLightModelProduct.sceneColor;\n"
	    "   if (uLight0Enabled == 1)\n"
	    "   {\n"
	    "       vec3 L = normalize(gl_LightSource[0].position.xyz - vVertexPos.xyz);\n" // we are in Eye Coordinates, so EyePos is (0,0,0)
	    "       vec3 E = normalize(-vVertexPos.xyz);\n"
	    "       vec3 R = normalize(-reflect(L,vNormal));\n"
	    "       vec4 Iamb = gl_FrontLightProduct[0].ambient;\n"                                                         // calculate Ambient Term:
	    "       vec4 Idiff = gl_FrontLightProduct[0].diffuse * max(dot(vNormal,L), 0.0);\n"                             // calculate Diffuse Term
	    "       vec4 Ispec = gl_FrontLightProduct[0].specular * pow(max(dot(R,E), 0.0), gl_FrontMaterial.shininess);\n" // calculate Specular Term
	    "       gl_FragColor += Iamb + Idiff + Ispec;\n"                                                                // write Total Color
	    "   }\n"
	    "   if (uLight1Enabled == 1)\n"
	    "   {\n"
	    "       vec3 L = normalize(gl_LightSource[1].position.xyz - vVertexPos.xyz);\n" // we are in Eye Coordinates, so EyePos is (0,0,0)
	    "       vec3 E = normalize(-vVertexPos.xyz);\n"
	    "       vec3 R = normalize(-reflect(L,vNormal));\n"
	    "       vec4 Iamb = gl_FrontLightProduct[1].ambient;\n"                                                         // calculate Ambient Term:
	    "       vec4 Idiff = gl_FrontLightProduct[1].diffuse * max(dot(vNormal,L), 0.0);\n"                             // calculate Diffuse Term
	    "       vec4 Ispec = gl_FrontLightProduct[1].specular * pow(max(dot(R,E), 0.0), gl_FrontMaterial.shininess);\n" // calculate Specular Term
	    "       gl_FragColor += Iamb + Idiff + Ispec;\n"                                                                // write Total Color
	    "   }\n"
	    "   gl_FragColor = vColor * gl_FragColor;\n"
	    "}\n";

	QString vertexProgSrc = VertexProgHeaderSrc;
	{
		// add attributes
		if (attributes & ATTR_SF)
			vertexProgSrc += VertexProgSFAttributesSrc;
		else if (attributes & ATTR_COL)
			vertexProgSrc += VertexProgColorAttributesSrc;
		if (attributes & ATTR_NOR)
			vertexProgSrc += VertexProgNormAttributesSrc;
		if (attributes & ATTR_VIS)
			vertexProgSrc += VertexProgVisibilityAttributesSrc;

		// add special functions
		if (attributes & ATTR_SF)
		{
			if (attributes & ATTR_LOG_SCALE)
			{
				vertexProgSrc += NormalizeValueLogScaleFuncSrc;
			}
			else
			{
				vertexProgSrc += ((attributes & ATTR_SYM_SCALE) ? NormalizeSymmetricalValueFuncSrc : NormalizeNonSymmetricalValueFuncSrc);
			}
			vertexProgSrc += VertexProgFetchSFColorFuncSrc;
		}

		if (attributes & ATTR_NOR)
		{
			vertexProgSrc += VertexProgFetchNormFuncSrc;
		}

		// main function
		{
			vertexProgSrc += VertexProgMainStartSrc;

			if (attributes & ATTR_VIS)
			{
				vertexProgSrc += VertexProgMainTestVisibilitySrc;
			}

			// color transfer
			if (attributes & ATTR_SF)
			{
				vertexProgSrc += ((attributes & ATTR_HIDDEN_VAL) ? VertexProgMainFetchSFColorHideNaNSrc : VertexProgMainFetchSFColorSrc);
			}
			else if (attributes & ATTR_COL)
			{
				vertexProgSrc += VertexProgMainUseInputColorSrc;
			}
			else
			{
				vertexProgSrc += VertexProgMainUseDefaultGLColorSrc;
			}

			// normal transfer (if any)
			if (attributes & ATTR_NOR)
			{
				vertexProgSrc += VertexProgMainFetchNormalSrc;
			}

			vertexProgSrc += VertexProgMainEndSrc;
		}
	}

	QString fragmentProgSrc = (attributes & ATTR_NOR ? ColorAndNormalFragmentProgSrc : ColorOnlyFragmentProgSrc);

	QOpenGLShader vertexShader(QOpenGLShader::Vertex);
	if (false == vertexShader.compileSourceCode(vertexProgSrc))
	{
		ccLog::Warning(QString("[BuildDisplayProgram] Vertex shader compilation failed: ") + vertexShader.log());
		return nullptr;
	}
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
	program->bindAttributeLocation("aPosition", ATTR_POS);
	if (attributes & ATTR_VIS)
	{
		program->bindAttributeLocation("aVisib", ATTR_VIS);
	}
	if (attributes & ATTR_SF)
	{
		assert((attributes & ATTR_COL) == 0);
		program->bindAttributeLocation("aSFValue", ATTR_SF);
	}
	else if (attributes & ATTR_COL)
	{
		assert((attributes & ATTR_SF) == 0);
		program->bindAttributeLocation("aColor", ATTR_COL);
	}
	if (attributes & ATTR_NOR)
	{
		program->bindAttributeLocation("aNormalIndex", ATTR_NOR);
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

void ccGLSL::SetLUTTextureUniforms(QOpenGLFunctions_2_1* glFunc,
                                   QOpenGLShaderProgram* prog,
                                   QOpenGLTexture*       lutTex)
{
	if (!glFunc || !prog || !lutTex)
	{
		assert(false);
		return;
	}
	// set sampler uniform to unit 0
	glFunc->glUniform1i(prog->uniformLocation("uNormalLUT"), 0);
	// set LUT dimensions
	glFunc->glUniform1i(prog->uniformLocation("uLUTWidth"), lutTex->width());
	glFunc->glUniform1i(prog->uniformLocation("uLUTHeight"), lutTex->height());
}

void ccGLSL::SetSFTextureUniforms(QOpenGLFunctions_2_1* glFunc,
                                  QOpenGLShaderProgram* prog,
                                  QOpenGLTexture*       sfTex,
                                  ccScalarField*        sf)
{
	if (!glFunc || !prog || !sfTex || !sf)
	{
		assert(false);
		return;
	}
	// set sampler uniform to unit 1
	glFunc->glUniform1i(prog->uniformLocation("uColorScaleTex"), 1);
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
}
