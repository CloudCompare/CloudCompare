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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "../include/ccGenericMesh.h"

// Local
#include "../include/ccChunk.h"
#include "../include/ccColorScalesManager.h"
#include "../include/ccGLSLHelper.h"
#include "../include/ccGenericGLDisplay.h"
#include "../include/ccGenericPointCloud.h"
#include "../include/ccHObjectCaster.h"
#include "../include/ccPointCloud.h"

// CCCoreLib
#include <GenericProgressCallback.h>
#include <GenericTriangle.h>
#include <MeshSamplingTools.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>

// Qt
#include <QOpenGLShader>
#include <QPainter>

// System
#include <cassert>
#include <memory>

#if defined(_OPENMP)
// OpenMP
#include <omp.h>
#endif

ccGenericMesh::ccGenericMesh(QString name /*=QString()*/, unsigned uniqueID /*=ccUniqueIDGenerator::InvalidUniqueID*/)
    : GenericIndexedMesh()
    , ccShiftedObject(name, uniqueID)
    , m_triNormsShown(false)
    , m_materialsShown(false)
    , m_showWired(false)
    , m_stippling(false)
    , m_forceSunLightOn(false)
    , m_hasUniqueMaterial{}
{
	setVisible(true);
	lockVisibility(false);
}

void ccGenericMesh::showNormals(bool state)
{
	showTriNorms(state);
	ccHObject::showNormals(state);
}

// stipple mask (for semi-transparent display of meshes)
static const GLubyte s_byte0               = 1 | 4 | 16 | 64;
static const GLubyte s_byte1               = 2 | 8 | 32 | 128;
static const GLubyte s_stippleMask[4 * 32] = {s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1};

void ccGenericMesh::EnableGLStippleMask(QOpenGLContext* context, bool state)
{
	// get the set of OpenGL functions (version 2.1)
	auto* glFunc = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(context);
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	if (state)
	{
		glFunc->glPolygonStipple(s_stippleMask);
		glFunc->glEnable(GL_POLYGON_STIPPLE);
	}
	else
	{
		glFunc->glDisable(GL_POLYGON_STIPPLE);
	}
}

void ccGenericMesh::handleColorRamp(CC_DRAW_CONTEXT& context)
{
	if (MACRO_Draw2D(context))
	{
		if (MACRO_Foreground(context) && !context.sfColorScaleToDisplay)
		{
			if (sfShown())
			{
				ccGenericPointCloud* vertices = getAssociatedCloud();
				if (!vertices || !vertices->isA(CC_TYPES::POINT_CLOUD))
					return;

				ccPointCloud* cloud = static_cast<ccPointCloud*>(vertices);

				// we just need to 'display' the current SF scale if the vertices cloud is hidden
				//(otherwise, it will be taken in charge by the cloud itself)
				if (!cloud->sfColorScaleShown() || (cloud->sfShown() && cloud->isEnabled() && cloud->isVisible()))
					return;

				// we must also check that the parent is not a mesh itself with the same vertices! (in
				// which case it will also take that in charge)
				ccHObject* parent = getParent();
				if (parent && parent->isKindOf(CC_TYPES::MESH) && (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == vertices))
					return;

				cloud->addColorRampInfo(context);
				// cloud->drawScale(context);
			}
		}
	}
}

CCVector3* ccGenericMesh::GetVertexBuffer()
{
	static CCVector3 s_xyzBuffer[ccChunk::SIZE * 3];
	return s_xyzBuffer;
}

CCVector3* ccGenericMesh::GetNormalsBuffer()
{
	static CCVector3 s_normBuffer[ccChunk::SIZE * 3];
	return s_normBuffer;
}

ColorCompType* ccGenericMesh::GetColorsBuffer()
{
	static ColorCompType s_rgbBuffer[ccChunk::SIZE * 3 * 4];
	return s_rgbBuffer;
}

float* ccGenericMesh::GetTexCoordsBuffer()
{
	static float s_texCoordsBuffer[ccChunk::SIZE * 3 * 2];
	return s_texCoordsBuffer;
}

unsigned* ccGenericMesh::GetWireVertexIndexes()
{
	static unsigned s_vertWireIndexes[ccChunk::SIZE * 6];
	static bool     s_vertIndexesInitialized = false;
	// on first call, we init the array
	if (!s_vertIndexesInitialized)
	{
		unsigned* _vertWireIndexes = s_vertWireIndexes;
		for (unsigned i = 0; i < ccChunk::SIZE * 3; ++i)
		{
			*_vertWireIndexes++ = i;
			*_vertWireIndexes++ = (((i + 1) % 3) == 0 ? i - 2 : i + 1);
		}
		s_vertIndexesInitialized = true;
	}

	return s_vertWireIndexes;
}

void ccGenericMesh::resetHasUniqueMaterial()
{
	m_hasUniqueMaterial.reset();
}

bool ccGenericMesh::hasUniqueMaterial()
{
	if (m_hasUniqueMaterial.has_value())
	{
		return m_hasUniqueMaterial.value();
	}

	if (size() == 0 || !hasMaterials())
	{
		return false;
	}

	int firstIndex = getTriangleMtlIndex(0);
	if (firstIndex < 0)
	{
		m_hasUniqueMaterial = false;
		return false;
	}

	for (unsigned i = 1; i < size(); ++i)
	{
		if (getTriangleMtlIndex(i) != firstIndex)
		{
			m_hasUniqueMaterial = false;
			return false;
		}
	}

	m_hasUniqueMaterial = true;
	return true;
}

// Global OpenGL resources
static QOpenGLBuffer s_vboVertex;
static QOpenGLBuffer s_vboNormals;
static QOpenGLBuffer s_vboColor;
static QOpenGLBuffer s_vboTexCoords;

void ccGenericMesh::ReleaseOpenGLRessources()
{
	if (!QOpenGLContext::currentContext())
	{
		ccLog::Warning("[ccGenericMesh::ReleaseOpenGLRessources] No valid OpenGL context");
		return;
	}

	ccGLSL::ReleaseOpenGLRessources();

	auto releaseVBO = [](QOpenGLBuffer& vbo)
	{
		if (vbo.isCreated())
		{
			vbo.destroy();
		}
	};

	releaseVBO(s_vboVertex);
	releaseVBO(s_vboNormals);
	releaseVBO(s_vboColor);
	releaseVBO(s_vboTexCoords);
}

void ccGenericMesh::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	handleColorRamp(context);

	// 3D pass only
	if (!MACRO_Draw3D(context))
	{
		return;
	}

	// get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (glFunc == nullptr)
	{
		assert(false);
		return;
	}

	// check that we have valid vertices
	ccGenericPointCloud* associatedCloud = getAssociatedCloud();
	if (!associatedCloud || !associatedCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return;
	}
	ccPointCloud* vertices = static_cast<ccPointCloud*>(associatedCloud);

	// check that we have some triangles to display
	size_t triNum = size();
	if (triNum == 0)
	{
		return;
	}

	// get default display parameters (we'll refine them later)
	glDrawParams glParams;
	getDrawingParameters(glParams);

	// L.O.D.
	bool     lodEnabled = (triNum > context.minLODTriangleCount && context.decimateMeshOnMove && MACRO_LODActivated(context));
	unsigned decimStep  = (lodEnabled ? static_cast<unsigned>(ceil(static_cast<double>(triNum * 3) / context.minLODTriangleCount)) : 1);

	// wireframe ? (not compatible with LOD)
	bool showWired = !lodEnabled && isShownAsWire();

	// vertices visibility
	const ccGenericPointCloud::VisibilityTableType& verticesVisibility  = vertices->getTheVisibilityArray();
	bool                                            visibilityFiltering = (verticesVisibility.size() >= vertices->size());

	// other dispaly parameters
	bool applyMaterials     = false;
	bool uniqueMaterial     = false;
	bool showOverridenColor = false;
	bool showTextures       = false;
	bool showTriNormals     = false;
	bool lightIsEnabled     = false;

	// in the case we need to display scalar field colors (this can also impact the entity picking mode)
	ccScalarField::Shared currentDisplayedScalarField;
	bool                  sfMayHaveHiddenValues = false;
	ccColorScale::Shared  colorScale;

	if (glParams.showSF)
	{
		currentDisplayedScalarField = vertices->getCurrentDisplayedScalarField();
		if (currentDisplayedScalarField)
		{
			sfMayHaveHiddenValues = currentDisplayedScalarField->mayHaveHiddenValues();
			colorScale            = currentDisplayedScalarField->getColorScale();

			// get default color ramp if vertices have no scale associated?!
			if (!colorScale)
			{
				assert(false);
				colorScale = ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::BGYR);
			}
		}
		else
		{
			glParams.showSF = false;
		}
	}

	// color-based entity picking
	ccColor::Rgb pickingColor;
	bool         entityPickingMode = MACRO_EntityPicking(context);
	if (entityPickingMode)
	{
		// not fast at all!
		if (MACRO_FastEntityPicking(context))
		{
			return;
		}

		pickingColor = context.entityPicking.registerEntity(this);

		// minimal display for picking mode!
		glParams.showNorms  = false;
		glParams.showColors = false;
		if (!sfMayHaveHiddenValues)
		{
			glParams.showSF = false; // we keep it only if point with 'NaN' SF values are hidden)
		}
		applyMaterials = false;
		showTextures   = false;
		lightIsEnabled = false;
	}
	else
	{
		// per-triangle or per-vertex normals?
		showTriNormals         = (hasTriNormals() && triNormsShown());
		bool showVertexNormals = (vertices->hasNormals() && m_normalsDisplayed);
		// fix 'showNorms'
		glParams.showNorms = showTriNormals || showVertexNormals;

		// materials & textures
		applyMaterials = (hasMaterials() && materialsShown());
		uniqueMaterial = (applyMaterials && hasUniqueMaterial());
		showTextures   = (hasTextures() && materialsShown() && !lodEnabled);

		// whether to enable light or not
		lightIsEnabled = m_forceSunLightOn || MACRO_LightIsEnabled(context);

		// if there's no light, no use showing normals, and vice versa
		if (!lightIsEnabled)
		{
			glParams.showNorms = false;
		}
		else if (!glParams.showNorms)
		{
			lightIsEnabled = false;
		}
	}

	glFunc->glPushAttrib(GL_LIGHTING_BIT | GL_TRANSFORM_BIT | GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT);

	// by default with a program, we don't want to mess with the material properties
	glFunc->glDisable(GL_COLOR_MATERIAL); // covered by GL_LIGHTING_BIT

	if (lightIsEnabled)
	{
		if (m_forceSunLightOn)
		{
			glFunc->glEnable(GL_LIGHT0); // covered by GL_LIGHTING_BIT
		}
		glFunc->glEnable(GL_LIGHTING); // covered by GL_LIGHTING_BIT
	}

	// glColor must be set whenever there's no scalar field nor RGBA colors to be displayed
	// - in the GLSL programs, the 'vColor' input for the vertex program comes either from glColor or from the vertex color array,
	// or is derived from SF values
	// - it is then optionally multiplied by the texture color (if any) and by the light & material properties (if any)
	ccColor::Rgb defaultColor = ccColor::whiteRGB;

	// materials or color?
	bool applyDefaultMaterial = false;
	auto materials            = getMaterialSet();
	bool skipDiffuse          = true; // if applyDefaultMaterial is true but lights are off, the default material diffuse color is set as the current glColor by default
	// |--------|-----------|----------|-----|-------|-----------------||--------------|----------|-------------|
	// |        |  unique   |          |     |       |                 ||              |defaultMat|             |
	// | light? | material? | texture? | SF? | RGBA? | entity picking? ||   glColor    |->applyGL | skipDiffuse |
	// |--------|-----------|----------|-----|-------|-----------------||--------------|----------|-------------|
	// |   0    |     0     |    0     |  0  |   0   |        1        || pickingColor |     0    |    ?        |
	// |   0    |     0     |    0     |  1  |   0   |        1        || pickingColor |     0    |    ?        |
	if (entityPickingMode)
	{
		assert(!lightIsEnabled);
		defaultColor = pickingColor;
	}
	// |   0    |     0     |    0     |  1  |   0   |        0        ||      ?       |     ?    |    ?        |
	// |   0    |     1     |    0     |  1  |   0   |        0        ||      ?       |     ?    |    ?        |
	// |   0    |     1     |    1     |  1  |   0   |        0        ||      ?       |     0    |    ?        |
	// |   1    |     0     |    0     |  1  |   0   |       N/A       ||      ?       |     1    |    ?        |
	// |   1    |     1     |    0     |  1  |   0   |       N/A       ||      ?       |     ?    |    ?        |
	// |   1    |     1     |    1     |  1  |   0   |       N/A       ||      ?       |     ?    |    ?        |
	else if (glParams.showSF)
	{
		applyDefaultMaterial = (lightIsEnabled && !uniqueMaterial);
		if (lightIsEnabled)
		{
			// we must get rid of lights 'color' if a scalar field is displayed!
			ccMaterial::MakeLightsNeutral(context.qGLContext); // covered by GL_LIGHTING_BIT
		}
	}
	// |   0    |     0     |    0     |  0  |   1   |        0        ||      ?       |     ?    |    ?        |
	// |   0    |     1     |    0     |  0  |   1   |        0        ||      ?       |     ?    |    ?        |
	// |   0    |     1     |    1     |  0  |   1   |        0        ||      ?       |     0    |    ?        |
	// |   1    |     0     |    0     |  0  |   1   |       N/A       ||      ?       |     1    |    ?        |
	// |   1    |     1     |    0     |  0  |   1   |       N/A       ||      ?       |     ?    |    ?        |
	// |   1    |     1     |    1     |  0  |   1   |       N/A       ||      ?       |     ?    |    ?        |
	else if (glParams.showColors)
	{
		if (isColorOverridden())
		{
			glParams.showColors = false;
			showOverridenColor  = true;
			defaultColor        = m_tempColor;
		}
		applyDefaultMaterial = (lightIsEnabled && !uniqueMaterial);
	}
	// |   0    |     0     |    0     |  0  |   0   |        0        ||     N/A      |     1    |    false    | // with no light, defaultMat->applyGL will call glColor with the default material diffuse color
	// |   0    |     1     |    0     |  0  |   0   |        0        ||      ?       |     ?    |    ?        |
	// |   0    |     1     |    1     |  0  |   0   |        0        ||   whiteRGB   |     0    |    ?        |
	// |   1    |     0     |    0     |  0  |   0   |       N/A       ||   whiteRGB   |     1    |    false    |
	// |   1    |     1     |    0     |  0  |   0   |       N/A       ||   whiteRGB   |     ?    |    ?        |
	// |   1    |     1     |    1     |  0  |   0   |       N/A       ||   whiteRGB   |     ?    |    ?        |
	else
	{
		applyDefaultMaterial = !uniqueMaterial;
		if (applyDefaultMaterial)
		{
			skipDiffuse = false;
		}
	}

	// apply default color and material (if necessary)
	ccGL::Color(glFunc, defaultColor);
	if (applyDefaultMaterial && context.defaultMat)
	{
		context.defaultMat->applyGL(context.qGLContext, lightIsEnabled, skipDiffuse);
	}

	if (!entityPickingMode)
	{
		glFunc->glEnable(GL_BLEND); // covered by GL_COLOR_BUFFER_BIT
	}

	// in the case we need normals (i.e. lighting)
	auto             normalsIndexesTable = (glParams.showNorms ? vertices->normals() : nullptr);
	ccNormalVectors* compressedNormals   = (glParams.showNorms ? ccNormalVectors::GetUniqueInstance() : nullptr);

	// stipple mask
	bool stippling = (m_stippling && !entityPickingMode);
	if (stippling)
	{
		EnableGLStippleMask(context.qGLContext, true);
	}

	// normal acceleration texture (for fast normals display)
	static bool                    s_normalLUTTextureFailed = false;
	QSharedPointer<QOpenGLTexture> lutTex;

	static bool s_globalVBOCreationFailed = false;

	bool fallBackDisplay = (visibilityFiltering
	                        || ((applyMaterials || showTextures) && !uniqueMaterial)
	                        || (glParams.showSF && sfMayHaveHiddenValues)
	                        || (glParams.showNorms && s_normalLUTTextureFailed))
	                       || s_globalVBOCreationFailed
	                       || MACRO_NoShader(context);

	QSharedPointer<QOpenGLShaderProgram> prog;
	if (!fallBackDisplay)
	{
		// get the right GLSL program
		int attributes = ccGLSL::ATTR_POS_FLAG;
		if (glParams.showNorms)
		{
			attributes |= ccGLSL::ATTR_NOR_FLAG;
		}
		if (glParams.showColors || glParams.showSF)
		{
			attributes |= ccGLSL::ATTR_COL_FLAG;
		}
		if (showTextures)
		{
			assert(uniqueMaterial);
			attributes |= ccGLSL::ATTR_TEX_FLAG;
		}

		prog = ccGLSL::BuildDisplayProgram(glFunc, attributes);

		if (prog)
		{
			if (glParams.showNorms)
			{
				// create or retrieve the LUT texture
				lutTex = ccGLSL::GetNormalLUTTexture(glFunc);
				if (lutTex.isNull())
				{
					ccLog::Warning("Failed to create normals LUT texture! Cannot render fast normals.");
					s_normalLUTTextureFailed = true;
					prog.clear();
				}
			}

			// static VBO handles reused between calls
			auto createVBOIfNeeded = [&](QOpenGLBuffer& vbo, int sizeBytes)
			{
				if (prog && !vbo.isCreated())
				{
					if (vbo.create())
					{
						vbo.setUsagePattern(QOpenGLBuffer::StreamDraw);
						vbo.bind();
						vbo.allocate(sizeBytes);
						vbo.release();
					}
					else
					{
						s_globalVBOCreationFailed = true;
						prog.clear();
					}
				}
			};
			createVBOIfNeeded(s_vboVertex, static_cast<int>(ccChunk::SIZE * 3 * 3 * sizeof(PointCoordinateType)));
			if (attributes & ccGLSL::ATTR_NOR_FLAG)
			{
				createVBOIfNeeded(s_vboNormals, static_cast<int>(ccChunk::SIZE * 3 * sizeof(float)));
			}
			if (attributes & ccGLSL::ATTR_COL_FLAG)
			{
				createVBOIfNeeded(s_vboColor, static_cast<int>(ccChunk::SIZE * 4 * 3 * sizeof(unsigned char)));
			}
			if (attributes & ccGLSL::ATTR_TEX_FLAG)
			{
				createVBOIfNeeded(s_vboTexCoords, static_cast<int>(ccChunk::SIZE * 2 * 3 * sizeof(float)));
			}
		}
	}

	if (prog)
	{
		assert(!entityPickingMode || !glParams.showSF);
		assert(prog.isNull() == false);

		auto*  verticesBuffer = GetVertexBuffer();
		float* normalIndexes  = reinterpret_cast<float*>(GetNormalsBuffer());
		auto*  rgbColors      = GetColorsBuffer();
		auto*  texCoords      = GetTexCoordsBuffer();

		prog->bind();

		if (applyMaterials || showTextures)
		{
			assert(uniqueMaterial && materials && getTriangleMtlIndex(0) >= 0 && static_cast<size_t>(getTriangleMtlIndex(0)) < materials->size());
			auto material = materials->at(getTriangleMtlIndex(0));

			material->applyGL(context.qGLContext, lightIsEnabled, showTextures || isColorOverridden());

			if (showTextures)
			{
				GLuint texID = material->getTextureID();
				glFunc->glEnable(GL_TEXTURE_2D);
				glFunc->glActiveTexture(GL_TEXTURE0);
				glFunc->glBindTexture(GL_TEXTURE_2D, texID);

				ccGLSL::SetTextureUniforms(glFunc, prog.data(), 0);
			}
		}

		if (lutTex)
		{
			// bind texture to unit 1
			glFunc->glActiveTexture(GL_TEXTURE1);
			glFunc->glBindTexture(GL_TEXTURE_2D, lutTex->textureId());

			ccGLSL::SetLightUniforms(glFunc, prog.data());
			ccGLSL::SetLUTTextureUniforms(glFunc, prog.data(), lutTex.data(), 1);
		}

		// we can scan and process each chunk separately in an optimized way
		size_t chunkCount = ccChunk::Count(size());
		for (size_t k = 0; k < chunkCount; ++k)
		{
			const size_t chunkSize  = ccChunk::Size(k, size());
			size_t       chunkStart = ccChunk::StartPos(k);

			// vertices
			size_t vertexCount = 0;
			{
				CCVector3* _vertices = verticesBuffer;
				for (size_t n = 0; n < chunkSize; n += decimStep)
				{
					unsigned                          triangleIndex = static_cast<unsigned>(chunkStart + n);
					const CCCoreLib::VerticesIndexes* ti            = getTriangleVertIndexes(triangleIndex);
					assert(ti->i1 < vertices->size());
					assert(ti->i2 < vertices->size());
					assert(ti->i3 < vertices->size());
					*_vertices++ = *vertices->getPoint(ti->i1);
					*_vertices++ = *vertices->getPoint(ti->i2);
					*_vertices++ = *vertices->getPoint(ti->i3);
					vertexCount += 3;
				}
			}

			// texture coordinates
			size_t texCoordCount = 0;
			if (showTextures)
			{
				assert(uniqueMaterial && hasPerTriangleTexCoordIndexes() && getTexCoordinatesTable());

				const TexCoords2D* Tx1 = nullptr;
				const TexCoords2D* Tx2 = nullptr;
				const TexCoords2D* Tx3 = nullptr;

				float* _texCoords = texCoords;
				for (size_t n = 0; n < chunkSize; n += decimStep)
				{
					unsigned triangleIndex = static_cast<unsigned>(chunkStart + n);
					getTriangleTexCoordinates(triangleIndex, Tx1, Tx2, Tx3);
					if (Tx1 && Tx2 && Tx3)
					{
						*_texCoords++ = Tx1->tx;
						*_texCoords++ = Tx1->ty;
						*_texCoords++ = Tx2->tx;
						*_texCoords++ = Tx2->ty;
						*_texCoords++ = Tx3->tx;
						*_texCoords++ = Tx3->ty;
					}
					else
					{
						assert(false);
						*_texCoords++ = 0.0;
						*_texCoords++ = 0.0;
						*_texCoords++ = 0.0;
						*_texCoords++ = 0.0;
						*_texCoords++ = 0.0;
						*_texCoords++ = 0.0;
					}
					texCoordCount += 3;
				}
			}

			// scalar field
			size_t rgbColorCount = 0;
			if (glParams.showSF)
			{
				ccColor::Rgb* _rgbColors = reinterpret_cast<ccColor::Rgb*>(rgbColors);
				assert(colorScale);

				for (size_t n = 0; n < chunkSize; n += decimStep)
				{
					unsigned                          triangleIndex = static_cast<unsigned>(chunkStart + n);
					const CCCoreLib::VerticesIndexes* ti            = getTriangleVertIndexes(triangleIndex);
					assert(ti->i1 < currentDisplayedScalarField->size());
					assert(ti->i2 < currentDisplayedScalarField->size());
					assert(ti->i3 < currentDisplayedScalarField->size());
					*_rgbColors++ = *currentDisplayedScalarField->getValueColor(ti->i1);
					*_rgbColors++ = *currentDisplayedScalarField->getValueColor(ti->i2);
					*_rgbColors++ = *currentDisplayedScalarField->getValueColor(ti->i3);
					rgbColorCount += 3;
				}
			}
			else if (glParams.showColors) // colors
			{
				ccColor::Rgba* _rgbaColors     = reinterpret_cast<ccColor::Rgba*>(rgbColors);
				const auto     rgbaColorsTable = vertices->rgbaColors();
				for (size_t n = 0; n < chunkSize; n += decimStep)
				{
					unsigned                          triangleIndex = static_cast<unsigned>(chunkStart + n);
					const CCCoreLib::VerticesIndexes* ti            = getTriangleVertIndexes(triangleIndex);
					assert(ti->i1 < rgbaColorsTable->size());
					assert(ti->i2 < rgbaColorsTable->size());
					assert(ti->i3 < rgbaColorsTable->size());
					*(_rgbaColors)++ = rgbaColorsTable->at(ti->i1);
					*(_rgbaColors)++ = rgbaColorsTable->at(ti->i2);
					*(_rgbaColors)++ = rgbaColorsTable->at(ti->i3);
					rgbColorCount += 3;
				}
			}

			// normals (indexes)
			size_t normalCount = 0;
			if (glParams.showNorms)
			{
				float* _normalIndexes = normalIndexes;
				if (showTriNormals)
				{
					assert(hasTriNormals());
					CompressedNormType nc1 = 0;
					CompressedNormType nc2 = 0;
					CompressedNormType nc3 = 0;

					for (size_t n = 0; n < chunkSize; n += decimStep)
					{
						unsigned triangleIndex = static_cast<unsigned>(chunkStart + n);
						getTriangleCompressedNormals(triangleIndex, nc1, nc2, nc3);
						*_normalIndexes++ = static_cast<float>(nc1);
						*_normalIndexes++ = static_cast<float>(nc2);
						*_normalIndexes++ = static_cast<float>(nc3);
						normalCount += 3;
					}
				}
				else
				{
					for (size_t n = 0; n < chunkSize; n += decimStep)
					{
						unsigned                          triangleIndex = static_cast<unsigned>(chunkStart + n);
						const CCCoreLib::VerticesIndexes* ti            = getTriangleVertIndexes(triangleIndex);
						assert(ti->i1 < normalsIndexesTable->size());
						assert(ti->i2 < normalsIndexesTable->size());
						assert(ti->i3 < normalsIndexesTable->size());
						*_normalIndexes++ = static_cast<float>(normalsIndexesTable->at(ti->i1));
						*_normalIndexes++ = static_cast<float>(normalsIndexesTable->at(ti->i2));
						*_normalIndexes++ = static_cast<float>(normalsIndexesTable->at(ti->i3));

						normalCount += 3;
					}
				}
			}

			// Upload to VBOs and draw with shader

			// Vertexes are 3D floats or doubles (depending on the cloud's precision)
			{
				s_vboVertex.bind();
				s_vboVertex.write(0, verticesBuffer, static_cast<int>(vertexCount * 3 * sizeof(PointCoordinateType)));
				glFunc->glEnableVertexAttribArray(ccGLSL::ATTR_POS_ARRAY);
				glFunc->glVertexAttribPointer(ccGLSL::ATTR_POS_ARRAY, 3, sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE, GL_FALSE, 0, nullptr);
				s_vboVertex.release();
			}

			// Normal (indexes) is a single float per vertex (the index of the normal in the LUT texture)
			if (glParams.showNorms)
			{
				s_vboNormals.bind();
				s_vboNormals.write(0, normalIndexes, static_cast<int>(normalCount * sizeof(float)));
				glFunc->glEnableVertexAttribArray(ccGLSL::ATTR_NOR_ARRAY);
				glFunc->glVertexAttribPointer(ccGLSL::ATTR_NOR_ARRAY, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
				s_vboNormals.release();
			}

			// Texture coordinates
			if (showTextures)
			{
				// texture coordinates are 2D floats
				s_vboTexCoords.bind();
				s_vboTexCoords.write(0, texCoords, static_cast<int>(texCoordCount * 2 * sizeof(float)));
				glFunc->glEnableVertexAttribArray(ccGLSL::ATTR_TEX_ARRAY);
				glFunc->glVertexAttribPointer(ccGLSL::ATTR_TEX_ARRAY, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
				s_vboTexCoords.release();
			}

			// Colors
			if (glParams.showSF)
			{
				// colors are RGB unsigned bytes (3 components)
				s_vboColor.bind();
				s_vboColor.write(0, rgbColors, static_cast<int>(rgbColorCount * 3 * sizeof(unsigned char)));
				glFunc->glEnableVertexAttribArray(ccGLSL::ATTR_COL_ARRAY);
				// we upload 3-component unsigned bytes; align to vec4 in shader by setting alpha = 1.0 via glVertexAttrib4f if needed
				glFunc->glVertexAttribPointer(ccGLSL::ATTR_COL_ARRAY, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
				// ensure alpha = 1.0 for all vertices
				// Note: can't set alpha per-vertex when only 3 components provided; shader expects vec4 but attribute with 3 components will get implicit 1.0 as 4th component
				s_vboColor.release();
			}
			else if (glParams.showColors)
			{
				// colors are RGBA unsigned bytes
				s_vboColor.bind();
				s_vboColor.write(0, rgbColors, static_cast<int>(rgbColorCount * 4 * sizeof(unsigned char)));
				glFunc->glEnableVertexAttribArray(ccGLSL::ATTR_COL_ARRAY);
				glFunc->glVertexAttribPointer(ccGLSL::ATTR_COL_ARRAY, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
				s_vboColor.release();
			}

			// draw
			if (!showWired)
			{
				glFunc->glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES, 0, static_cast<GLint>(vertexCount));
			}
			else
			{
				glFunc->glDrawElements(GL_LINES, (static_cast<int>(chunkSize) / decimStep) * 6, GL_UNSIGNED_INT, GetWireVertexIndexes());
			}

			// cleanup per-chunk state
			glFunc->glDisableVertexAttribArray(ccGLSL::ATTR_POS_ARRAY);
			if (glParams.showNorms)
			{
				glFunc->glDisableVertexAttribArray(ccGLSL::ATTR_NOR_ARRAY);
			}
			if (showTextures)
			{
				glFunc->glDisableVertexAttribArray(ccGLSL::ATTR_TEX_ARRAY);
			}
			if (glParams.showSF || glParams.showColors)
			{
				glFunc->glDisableVertexAttribArray(ccGLSL::ATTR_COL_ARRAY);
			}

			ccGLDrawContext::CatchGLErrors(glFunc->glGetError(), "ccMesh::shader.program.end");
		}

		if (lutTex)
		{
			glFunc->glActiveTexture(GL_TEXTURE1);
			glFunc->glBindTexture(GL_TEXTURE_2D, 0);
		}
		if (showTextures)
		{
			glFunc->glDisable(GL_TEXTURE_2D);
			glFunc->glActiveTexture(GL_TEXTURE0);
			glFunc->glBindTexture(GL_TEXTURE_2D, 0);
		}
		prog->release();
	}
	else
	{
		// current vertex color (RGB)
		const ccColor::Rgb* rgb1 = nullptr;
		const ccColor::Rgb* rgb2 = nullptr;
		const ccColor::Rgb* rgb3 = nullptr;
		// current vertex color (RGBA)
		const ccColor::Rgba* rgba1 = nullptr;
		const ccColor::Rgba* rgba2 = nullptr;
		const ccColor::Rgba* rgba3 = nullptr;
		// current vertex normal
		const CCVector3* N1 = nullptr;
		const CCVector3* N2 = nullptr;
		const CCVector3* N3 = nullptr;
		// current vertex texture coordinates
		const TexCoords2D* Tx1 = nullptr;
		const TexCoords2D* Tx2 = nullptr;
		const TexCoords2D* Tx3 = nullptr;

		int    lasMtlIndex  = -1;
		GLuint currentTexID = 0;

		if (glParams.showSF || glParams.showColors || showOverridenColor)
		{
			glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE); // use color defined with glColor() as diffuse front and back material
			glFunc->glEnable(GL_COLOR_MATERIAL);                    // covered by GL_LIGHTING_BIT
		}
		if (glParams.showNorms)
		{
			glFunc->glEnable(GL_RESCALE_NORMAL); // not covered by glPushAttrib
		}
		const auto rgbaColorsTable = vertices->rgbaColors();

		GLenum triangleDisplayType = (lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP
		                                                                 : GL_TRIANGLES);
		glFunc->glBegin(triangleDisplayType);

		// loop on all triangles
		for (size_t n = 0; n < triNum; ++n)
		{
			// LOD: shall we display this triangle?
			if (n % decimStep)
			{
				continue;
			}

			// current triangle vertices
			const CCCoreLib::VerticesIndexes* tsi = getTriangleVertIndexes(static_cast<unsigned>(n));
			assert(tsi);

			if (visibilityFiltering)
			{
				// we skip the triangle if at least one vertex is hidden
				if ((verticesVisibility[tsi->i1] != CCCoreLib::POINT_VISIBLE)
				    || (verticesVisibility[tsi->i2] != CCCoreLib::POINT_VISIBLE)
				    || (verticesVisibility[tsi->i3] != CCCoreLib::POINT_VISIBLE))
				{
					continue;
				}
			}

			if (glParams.showSF)
			{
				assert(colorScale);
				rgb1 = currentDisplayedScalarField->getValueColor(tsi->i1);
				if (!rgb1)
					continue;
				rgb2 = currentDisplayedScalarField->getValueColor(tsi->i2);
				if (!rgb2)
					continue;
				rgb3 = currentDisplayedScalarField->getValueColor(tsi->i3);
				if (!rgb3)
					continue;

				if (entityPickingMode)
				{
					// in picking mode, we don't want to apply the colors, just filter the invisible triangles
					rgb1 = nullptr;
					rgb2 = nullptr;
					rgb3 = nullptr;
				}
			}
			else if (glParams.showColors)
			{
				rgba1 = &rgbaColorsTable->at(tsi->i1);
				rgba2 = &rgbaColorsTable->at(tsi->i2);
				rgba3 = &rgbaColorsTable->at(tsi->i3);
			}

			if (glParams.showNorms)
			{
				if (showTriNormals)
				{
					assert(hasTriNormals());
					getTriangleNormals(static_cast<unsigned>(n), N1, N2, N3);
				}
				else
				{
					N1 = &compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i1));
					N2 = &compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i2));
					N3 = &compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i3));
				}
			}

			if (applyMaterials || showTextures)
			{
				assert(materials);
				int newMatlIndex = this->getTriangleMtlIndex(static_cast<unsigned>(n));

				// do we need to change material?
				if (lasMtlIndex != newMatlIndex)
				{
					assert(newMatlIndex < static_cast<int>(materials->size()));
					glFunc->glEnd();
					if (showTextures)
					{
						if (newMatlIndex >= 0) // valid material index
						{
							GLuint newTexID = materials->at(newMatlIndex)->getTextureID();
							if (newTexID != currentTexID)
							{
								// the texture ID changes
								if (0 != newTexID)
								{
									// new and valid texture ID --> we bind it
									currentTexID = newTexID;
									glFunc->glEnable(GL_TEXTURE_2D); // it seems some driver now won't manage the case where no texture is bound and still try
									                                 // to display an (invalid) texture. So we have to enable texture mode only when necessary.
									glFunc->glBindTexture(GL_TEXTURE_2D, currentTexID);
								}
								else if (0 != currentTexID)
								{
									// the previous texture ID was valid --> we unbind it
									currentTexID = 0;
									glFunc->glBindTexture(GL_TEXTURE_2D, 0);
									glFunc->glDisable(GL_TEXTURE_2D); // it seems some driver now won't manage the case where no texture is bound and still
									                                  // try to display an (invalid) texture. So we disable the whole texture mode.
								}
							}
						}
						else if (0 != currentTexID)
						{
							currentTexID = 0;
							glFunc->glBindTexture(GL_TEXTURE_2D, 0);
							glFunc->glDisable(GL_TEXTURE_2D); // it seems some driver now won't manage the case where no texture is bound and still
							                                  // try to display an (invalid) texture. So we disable the whole texture mode.
						}
					}

					// if we don't have any current material, we apply the default one
					if (newMatlIndex >= 0)
						materials->at(newMatlIndex)->applyGL(context.qGLContext, lightIsEnabled, false);
					else
						context.defaultMat->applyGL(context.qGLContext, lightIsEnabled, false);

					glFunc->glBegin(triangleDisplayType);
					lasMtlIndex = newMatlIndex;
				}

				if (showTextures)
				{
					assert(hasPerTriangleTexCoordIndexes() && getTexCoordinatesTable());
					getTriangleTexCoordinates(static_cast<unsigned>(n), Tx1, Tx2, Tx3);
				}
			}

			if (showWired)
			{
				glFunc->glEnd();
				glFunc->glBegin(triangleDisplayType);
			}

			// vertex 1
			if (N1)
				ccGL::Normal3v(glFunc, N1->u);
			if (rgb1)
				ccGL::Color(glFunc, *rgb1);
			else if (rgba1)
				ccGL::Color(glFunc, *rgba1);
			if (Tx1)
				glFunc->glTexCoord2fv(Tx1->t);
			ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i1)->u);

			// vertex 2
			if (N2)
				ccGL::Normal3v(glFunc, N2->u);
			if (rgb2)
				ccGL::Color(glFunc, *rgb2);
			else if (rgba2)
				ccGL::Color(glFunc, *rgba2);
			if (Tx2)
				glFunc->glTexCoord2fv(Tx2->t);
			ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i2)->u);

			// vertex 3
			if (N3)
				ccGL::Normal3v(glFunc, N3->u);
			if (rgb3)
				ccGL::Color(glFunc, *rgb3);
			else if (rgba3)
				ccGL::Color(glFunc, *rgba3);
			if (Tx3)
				glFunc->glTexCoord2fv(Tx3->t);
			ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i3)->u);
		}

		glFunc->glEnd();

		if (showTextures)
		{
			if (0 != currentTexID)
			{
				currentTexID = 0;
				glFunc->glBindTexture(GL_TEXTURE_2D, 0);
				glFunc->glDisable(GL_TEXTURE_2D); // it seems some driver now won't manage the case where no texture is bound and still
				                                  // try to display an (invalid) texture. So we disable the whole texture mode.
			}
		}
	}

	if (stippling)
	{
		EnableGLStippleMask(context.qGLContext, false);
	}

	glFunc->glPopAttrib(); // GL_LIGHTING_BIT | GL_TRANSFORM_BIT | GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT
}

bool ccGenericMesh::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 29)
	{
		assert(false);
		return false;
	}

	if (!ccHObject::toFile_MeOnly(out, dataVersion))
	{
		return false;
	}

	//'show wired' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_showWired), sizeof(bool)) < 0)
		return WriteError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_triNormsShown), sizeof(bool)) < 0)
		return WriteError();

	//'materials shown' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_materialsShown), sizeof(bool)) < 0)
		return WriteError();

	//'polygon stippling' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_stippling), sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool ccGenericMesh::fromFile_MeOnly(QFile& in, LoadingContext& context)
{
	if (!ccHObject::fromFile_MeOnly(in, context))
	{
		return false;
	}

	//'show wired' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_showWired), sizeof(bool)) < 0)
	{
		return ReadError();
	}

	//'per-triangle normals shown' state (dataVersion>=29))
	if (context.dataVersion >= 29)
	{
		if (in.read(reinterpret_cast<char*>(&m_triNormsShown), sizeof(bool)) < 0)
		{
			return ReadError();
		}

		//'materials shown' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_materialsShown), sizeof(bool)) < 0)
		{
			return ReadError();
		}

		//'polygon stippling' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_stippling), sizeof(bool)) < 0)
		{
			return ReadError();
		}
	}

	return true;
}

short ccGenericMesh::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(29), ccHObject::minimumFileVersion_MeOnly());
}

ccPointCloud* ccGenericMesh::samplePoints(bool                                densityBased,
                                          double                              samplingParameter,
                                          bool                                withNormals,
                                          bool                                withRGB,
                                          bool                                withTexture,
                                          CCCoreLib::GenericProgressCallback* pDlg /*=nullptr*/)
{
	if (samplingParameter <= 0)
	{
		assert(false);
		return nullptr;
	}

	bool withFeatures = (withNormals || withRGB || withTexture);

	std::unique_ptr<std::vector<unsigned>> triIndices;
	if (withFeatures)
	{
		triIndices = std::make_unique<std::vector<unsigned>>();
	}

	CCCoreLib::PointCloud* sampledCloud = nullptr;
	if (densityBased)
	{
		sampledCloud = CCCoreLib::MeshSamplingTools::samplePointsOnMesh(this, samplingParameter, pDlg, triIndices.get());
	}
	else
	{
		sampledCloud = CCCoreLib::MeshSamplingTools::samplePointsOnMesh(this, static_cast<unsigned>(samplingParameter), pDlg, triIndices.get());
	}

	// convert to real point cloud
	ccPointCloud* cloud = nullptr;

	if (sampledCloud)
	{
		if (sampledCloud->size() == 0)
		{
			ccLog::Warning("[ccGenericMesh::samplePoints] No point was generated (sampling density is too low?)");
		}
		else
		{
			cloud = ccPointCloud::From(sampledCloud);
			if (!cloud)
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
			}
		}

		delete sampledCloud;
		sampledCloud = nullptr;
	}
	else
	{
		ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
	}

	if (!cloud)
	{
		return nullptr;
	}

	if (withFeatures && triIndices && triIndices->size() >= cloud->size())
	{
		// generate normals
		if (withNormals && hasNormals())
		{
			if (cloud->reserveTheNormsTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					CCVector3 N(0, 0, 1);
					interpolateNormals(triIndex, *P, N);
					cloud->addNorm(N);
				}

				cloud->showNormals(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate normals (not enough memory?)");
			}
		}

		// generate colors
		if (withTexture && hasMaterials())
		{
			if (cloud->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					ccColor::Rgba color;
					getColorFromMaterial(triIndex, *P, color, withRGB);
					cloud->addColor(color);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to export texture colors (not enough memory?)");
			}
		}
		else if (withRGB && hasColors())
		{
			if (cloud->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					ccColor::Rgb C;
					interpolateColors(triIndex, *P, C);
					cloud->addColor(C);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate colors (not enough memory?)");
			}
		}
	}

	// we rename the resulting cloud
	cloud->setName(getName() + QString(".sampled"));
	cloud->setDisplay(getDisplay());
	cloud->prepareDisplayForRefresh();

	// import parameters from the source mesh
	cloud->copyGlobalShiftAndScale(*this);
	cloud->setGLTransformationHistory(getGLTransformationHistory());

	return cloud;
}

void ccGenericMesh::importParametersFrom(const ccGenericMesh& mesh)
{
	// original shift & scale
	copyGlobalShiftAndScale(mesh);

	// stippling
	enableStippling(mesh.stipplingEnabled());
	// wired style
	showWired(mesh.isShownAsWire());

	// keep the transformation history!
	setGLTransformationHistory(mesh.getGLTransformationHistory());
	// and meta-data
	setMetaData(mesh.metaData());
}

void ccGenericMesh::computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const
{
	CCCoreLib::GenericTriangle* tri = const_cast<ccGenericMesh*>(this)->_getTriangle(triIndex);
	const CCVector3*            A   = tri->_getA();
	const CCVector3*            B   = tri->_getB();
	const CCVector3*            C   = tri->_getC();

	// barycentric interpolation weights
	weights.x = ((P - *B).cross(*C - *B)).normd() /*/2*/;
	weights.y = ((P - *C).cross(*A - *C)).normd() /*/2*/;
	weights.z = ((P - *A).cross(*B - *A)).normd() /*/2*/;

	// normalize weights
	double sum = weights.x + weights.y + weights.z;
	weights /= sum;
}

bool ccGenericMesh::trianglePicking(unsigned                    triIndex,
                                    const CCVector2d&           clickPos,
                                    const ccGLMatrix&           trans,
                                    bool                        noGLTrans,
                                    const ccGenericPointCloud&  vertices,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    CCVector3d&                 point,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	assert(triIndex < size());

	CCVector3 A3D;
	CCVector3 B3D;
	CCVector3 C3D;
	getTriangleVertices(triIndex, A3D, B3D, C3D);

	CCVector3d A2D;
	CCVector3d B2D;
	CCVector3d C2D;
	bool       insideA = false;
	bool       insideB = false;
	bool       insideC = false;

	if (noGLTrans)
	{
		camera.project(A3D, A2D, &insideA);
		camera.project(B3D, B2D, &insideB);
		camera.project(C3D, C2D, &insideC);
	}
	else
	{
		CCVector3 A3Dp = trans * A3D;
		CCVector3 B3Dp = trans * B3D;
		CCVector3 C3Dp = trans * C3D;
		camera.project(A3Dp, A2D, &insideA);
		camera.project(B3Dp, B2D, &insideB);
		camera.project(C3Dp, C2D, &insideC);
	}

	// if none of the vertices fall inside the frustum, the triangle is (probably) not visible...
	if (!insideA && !insideB && !insideC)
	{
		// If there's one huge triangle or the user zoom in a lot, it's not true!
		// So we only use this if there are a lot of triangles
		if (size() > 10000)
		{
			return false;
		}
	}

	// barycentric coordinates
	GLdouble detT = (B2D.y - C2D.y) * (A2D.x - C2D.x) + (C2D.x - B2D.x) * (A2D.y - C2D.y);
	if (CCCoreLib::LessThanEpsilon(std::abs(detT)))
	{
		return false;
	}
	GLdouble l1 = ((B2D.y - C2D.y) * (clickPos.x - C2D.x) + (C2D.x - B2D.x) * (clickPos.y - C2D.y)) / detT;
	GLdouble l2 = ((C2D.y - A2D.y) * (clickPos.x - C2D.x) + (A2D.x - C2D.x) * (clickPos.y - C2D.y)) / detT;
	GLdouble l3 = 1.0 - l1 - l2;

	double l1l2 = l1 + l2;
	if (l1l2 > 1.0)
	{
		// we fall outside of the triangle!
		return false;
	}

	// does the point falls inside the triangle?
	if (edgeOnly)
	{
		double espilon = 15 / ((C2D - A2D).norm() + (B2D - A2D).norm() + (C2D - B2D).norm());
		bool   onEdge  = false;

		if (l1 > -espilon && l1 <= 1.0 && l2 > -espilon && l2 <= 1.0)
		{
			if (l1 < espilon)
			{
				l1     = 0;
				onEdge = true;
			}
			if (l2 < espilon)
			{
				l2     = 0;
				onEdge = true;
			}
			if (l3 < espilon)
			{
				l3     = 0;
				onEdge = true;
			}

			if (!onEdge)
			{
				// not on an edge
				return false;
			}

			// normalize
			double sum = l1 + l2 + l3;
			l1 /= sum;
			l2 /= sum;
			l3 /= sum;
		}
		else
		{
			return false;
		}
	}
	else if (l1 < 0 || l1 > 1.0 || l2 < 0.0 || l2 > 1.0)
	{
		return false;
	}

	// now deduce the 3D position
	point = CCVector3d(l1 * A3D.x + l2 * B3D.x + l3 * C3D.x,
	                   l1 * A3D.y + l2 * B3D.y + l3 * C3D.y,
	                   l1 * A3D.z + l2 * B3D.z + l3 * C3D.z);

	if (barycentricCoords)
	{
		*barycentricCoords = CCVector3d(l1, l2, l3);
	}

	return true;
}

bool ccGenericMesh::trianglePicking(const CCVector2d&           clickPos,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    int&                        nearestTriIndex,
                                    double&                     nearestSquareDist,
                                    CCVector3d&                 nearestPoint,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	ccGLMatrix trans;
	bool       noGLTrans = !getAbsoluteGLTransformation(trans);

	// back project the clicked point in 3D
	CCVector3d clickPosd(clickPos.x, clickPos.y, 0);
	CCVector3d X(0, 0, 0);
	if (!camera.unproject(clickPosd, X))
	{
		return false;
	}

	nearestTriIndex   = -1;
	nearestSquareDist = -1.0;
	nearestPoint      = CCVector3d(0, 0, 0);
	if (barycentricCoords)
	{
		*barycentricCoords = CCVector3d(0, 0, 0);
	}

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

#if defined(_OPENMP) && !defined(_DEBUG) && !defined(TEST_PICKING)
#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
	for (int i = 0; i < static_cast<int>(size()); ++i)
	{
		CCVector3d P;
		CCVector3d BC;
		if (!trianglePicking(i,
		                     clickPos,
		                     trans,
		                     noGLTrans,
		                     *vertices,
		                     camera,
		                     edgeOnly,
		                     P,
		                     barycentricCoords ? &BC : nullptr))
		{
			continue;
		}

		double squareDist = (X - P).norm2d();
		if (nearestTriIndex < 0 || squareDist < nearestSquareDist)
		{
			nearestSquareDist = squareDist;
			nearestTriIndex   = i;
			nearestPoint      = P;
			if (barycentricCoords)
			{
				*barycentricCoords = BC;
			}
		}
	}

	return (nearestTriIndex >= 0);
}

bool ccGenericMesh::trianglePicking(unsigned                    triIndex,
                                    const CCVector2d&           clickPos,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    CCVector3d&                 point,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		return false;
	}

	ccGLMatrix trans;
	bool       noGLTrans = !getAbsoluteGLTransformation(trans);

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

	return trianglePicking(triIndex,
	                       clickPos,
	                       trans,
	                       noGLTrans,
	                       *vertices,
	                       camera,
	                       edgeOnly,
	                       point,
	                       barycentricCoords);
}

bool ccGenericMesh::computePointPosition(unsigned triIndex, const CCVector2d& uv, CCVector3& P, bool warningIfOutside /*=true*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		ccLog::Warning("Index out of range");
		return true;
	}

	CCVector3 A;
	CCVector3 B;
	CCVector3 C;
	getTriangleVertices(triIndex, A, B, C);

	double z = 1.0 - uv.x - uv.y;
	if (warningIfOutside && ((z < -1.0e-6) || (z > 1.0 + 1.0e-6)))
	{
		ccLog::Warning("Point falls outside of the triangle");
	}

	P = CCVector3(static_cast<PointCoordinateType>(uv.x * A.x + uv.y * B.x + z * C.x),
	              static_cast<PointCoordinateType>(uv.x * A.y + uv.y * B.y + z * C.y),
	              static_cast<PointCoordinateType>(uv.x * A.z + uv.y * B.z + z * C.z));

	return true;
}

void ccGenericMesh::setGlobalShift(const CCVector3d& shift)
{
	if (getAssociatedCloud())
	{
		// auto transfer the global shift info to the vertices
		getAssociatedCloud()->setGlobalShift(shift);
	}
	else
	{
		// we normally don't want to store this information at
		// the mesh level as it won't be saved.
		assert(false);
		ccShiftedObject::setGlobalShift(shift);
	}
}

void ccGenericMesh::setGlobalScale(double scale)
{
	if (getAssociatedCloud())
	{
		// auto transfer the global scale info to the vertices
		getAssociatedCloud()->setGlobalScale(scale);
	}
	else
	{
		// we normally don't want to store this information at
		// the mesh level as it won't be saved.
		assert(false);
		ccShiftedObject::setGlobalScale(scale);
	}
}

const CCVector3d& ccGenericMesh::getGlobalShift() const
{
	return (getAssociatedCloud() ? getAssociatedCloud()->getGlobalShift() : ccShiftedObject::getGlobalShift());
}

double ccGenericMesh::getGlobalScale() const
{
	return (getAssociatedCloud() ? getAssociatedCloud()->getGlobalScale() : ccShiftedObject::getGlobalScale());
}

bool ccGenericMesh::IsCloudVerticesOfMesh(ccGenericPointCloud* cloud, ccGenericMesh** mesh /*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a mesh
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::MESH) && static_cast<ccGenericMesh*>(parent)->getAssociatedCloud() == cloud)
		{
			if (mesh)
			{
				*mesh = static_cast<ccGenericMesh*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::MESH) && static_cast<ccGenericMesh*>(child)->getAssociatedCloud() == cloud)
		{
			if (mesh)
			{
				*mesh = static_cast<ccGenericMesh*>(child);
			}
			return true;
		}
	}

	return false;
}
