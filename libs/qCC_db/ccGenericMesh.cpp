//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Always first
#include "ccIncludeGL.h"

#include "ccGenericMesh.h"

//local
#include "ccChunk.h"
#include "ccColorScalesManager.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccHObjectCaster.h"
#include "ccMaterialSet.h"
#include "ccNormalVectors.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"

//CCLib
#include <GenericProgressCallback.h>
#include <GenericTriangle.h>
#include <MeshSamplingTools.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>

//system
#include <cassert>

ccGenericMesh::ccGenericMesh(QString name/*=QString()*/)
	: GenericIndexedMesh()
	, ccHObject(name)
	, m_triNormsShown(false)
	, m_materialsShown(false)
	, m_showWired(false)
	, m_showFaces(true)
	, m_stippling(false)
{
	setVisible(true);
	lockVisibility(false);
}

void ccGenericMesh::showNormals(bool state)
{
	showTriNorms(state);
	ccHObject::showNormals(state);
}

//stipple mask (for semi-transparent display of meshes)
static const GLubyte s_byte0 = 1 | 4 | 16 | 64;
static const GLubyte s_byte1 = 2 | 8 | 32 | 128;
static const GLubyte s_stippleMask[4*32] = {s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1,
	s_byte0,s_byte0,s_byte0,s_byte0,
	s_byte1,s_byte1,s_byte1,s_byte1};

void ccGenericMesh::EnableGLStippleMask(const QOpenGLContext* context, bool state)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context->versionFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

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

//Vertex buffer
CCVector3* ccGenericMesh::GetVertexBuffer()
{
	static CCVector3 s_xyzBuffer[ccChunk::SIZE * 3];
	return s_xyzBuffer;
}

//Normals buffer
CCVector3* ccGenericMesh::GetNormalsBuffer()
{
	static CCVector3 s_normBuffer[ccChunk::SIZE * 3];
	return s_normBuffer;
}

//Colors buffer
ccColor::Rgb* ccGenericMesh::GetColorsBuffer()
{
	static ccColor::Rgb s_rgbBuffer[ccChunk::SIZE * 3];
	return s_rgbBuffer;
}

CCVector2 * ccGenericMesh::GetTextureBuffer()
{
	static CCVector2 s_texBuffer[ccChunk::SIZE * 3];
	return s_texBuffer;
}

//Vertex indexes buffer (for wired display)
static unsigned s_vertWireIndexes[ccChunk::SIZE * 6];
static bool s_vertIndexesInitialized = false;
unsigned* ccGenericMesh::GetWireVertexIndexes()
{
	//on first call, we init the array
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

				//we just need to 'display' the current SF scale if the vertices cloud is hidden
				//(otherwise, it will be taken in charge by the cloud itself)
				if (!cloud->sfColorScaleShown() || (cloud->sfShown() && cloud->isEnabled() && cloud->isVisible()))
					return;

				//we must also check that the parent is not a mesh itself with the same vertices! (in
				//which case it will also take that in charge)
				ccHObject* parent = getParent();
				if (parent && parent->isKindOf(CC_TYPES::MESH) && (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == vertices))
					return;

				cloud->addColorRampInfo(context);
				//cloud->drawScale(context);
			}
		}
	}
}

void ccGenericMesh::notifyGeometryUpdate()
{
	ccHObject::notifyGeometryUpdate();

	releaseVBOs();
}

void ccGenericMesh::removeFromDisplay(const ccGenericGLDisplay * win)
{
	if (win == m_currentDisplay)
	{
		releaseVBOs();
	}

	//call parent's method
	ccHObject::removeFromDisplay(win);
}

void ccGenericMesh::setDisplay(ccGenericGLDisplay * win)
{
	if (m_currentDisplay && win != m_currentDisplay)
	{
		//be sure to release the VBOs before switching to another (or no) display!
		releaseVBOs();
	}

	ccHObject::setDisplay(win);
}

//the GL type depends on the PointCoordinateType 'size' (float or double)
static GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;

void ccGenericMesh::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
		return;

	handleColorRamp(context);

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//3D pass
	if (MACRO_Draw3D(context))
	{
		//any triangle?
		unsigned triNum = size();
		if (triNum == 0)
			return;

		//L.O.D.
		bool lodEnabled = (triNum > context.minLODTriangleCount && context.decimateMeshOnMove && MACRO_LODActivated(context));
		unsigned decimStep = (lodEnabled ? static_cast<unsigned>(ceil(static_cast<double>(triNum*3) / context.minLODTriangleCount)) : 1);
		unsigned displayedTriNum = triNum / decimStep;

		//display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		glParams.showNorms &= bool(MACRO_LightIsEnabled(context));

		//vertices visibility
		const ccGenericPointCloud::VisibilityTableType& verticesVisibility = vertices->getTheVisibilityArray();
		bool visFiltering = (verticesVisibility.size() == vertices->size());

		//wireframe ? (not compatible with LOD)
		bool showWired = isShownAsWire() && !lodEnabled;
		bool showFaces = isShownAsFace();

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());
		//fix 'showNorms'
		glParams.showNorms = showTriNormals || (vertices->hasNormals() && normalsShown());

		//materials & textures
		bool applyMaterials = (hasMaterials() && materialsShown());
		bool showTextures = (hasTextures() && materialsShown() && !lodEnabled);

		

		//GL name pushing
		bool pushName = MACRO_DrawEntityNames(context);
		if (pushName)
		{
			//not fast at all!
			if (MACRO_DrawFastNamesOnly(context))
				return;
			glFunc->glPushName(getUniqueIDForDisplay());
			//minimal display for picking mode!
			glParams.showNorms = false;
			glParams.showColors = false;
			//glParams.showSF --> we keep it only if SF 'NaN' values are hidden
			showTriNormals = false;
			applyMaterials = false;
			showTextures = false;
		}

		//in the case we need to display scalar field colors
		ccScalarField* currentDisplayedScalarField = nullptr;
		bool greyForNanScalarValues = true;
		//unsigned colorRampSteps = 0;
		ccColorScale::Shared colorScale(nullptr);

		if (glParams.showSF)
		{
			assert(vertices->isA(CC_TYPES::POINT_CLOUD));
			ccPointCloud* cloud = static_cast<ccPointCloud*>(vertices);

			greyForNanScalarValues = (cloud->getCurrentDisplayedScalarField() && cloud->getCurrentDisplayedScalarField()->areNaNValuesShownInGrey());
			if (greyForNanScalarValues && pushName)
			{
				//in picking mode, no need to take SF into account if we don't hide any points!
				glParams.showSF = false;
			}
			else
			{
				currentDisplayedScalarField = cloud->getCurrentDisplayedScalarField();
				colorScale = currentDisplayedScalarField->getColorScale();
				//colorRampSteps = currentDisplayedScalarField->getColorRampSteps();

				assert(colorScale);
				//get default color ramp if cloud has no scale associated?!
				if (!colorScale)
					colorScale = ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::BGYR);
			}
		}

		//materials or color?
		bool colorMaterial = false;
		if (glParams.showSF || glParams.showColors)
		{
			applyMaterials = false;
			colorMaterial = true;
			glFunc->glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glFunc->glEnable(GL_COLOR_MATERIAL);
		}

		//in the case we need to display vertex colors
		ColorsTableType* rgbColorsTable = nullptr;
		if (glParams.showColors)
		{
			if (isColorOverriden())
			{
				ccGL::Color3v(glFunc, m_tempColor.rgb);
				glParams.showColors = false;
			}
			else
			{
				assert(vertices->isA(CC_TYPES::POINT_CLOUD));
				rgbColorsTable = static_cast<ccPointCloud*>(vertices)->rgbColors();
			}
		}
		else
		{
			glFunc->glColor3fv(context.defaultMat->getDiffuseFront().rgba);
		}

		if (glParams.showNorms)
		{
			glFunc->glEnable(GL_RESCALE_NORMAL);
			glFunc->glEnable(GL_LIGHTING);
			context.defaultMat->applyGL(context.qGLContext, true, colorMaterial);
		}

		//in the case we need normals (i.e. lighting)
		NormsIndexesTableType* normalsIndexesTable = nullptr;
		ccNormalVectors* compressedNormals = nullptr;
		if (glParams.showNorms)
		{
			assert(vertices->isA(CC_TYPES::POINT_CLOUD));
			normalsIndexesTable = static_cast<ccPointCloud*>(vertices)->normals();
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		glParams.showTexture = applyMaterials || showTextures;

		//materials
		const ccMaterialSet* materials = getMaterialSet();

		//stipple mask
		if (stipplingEnabled())
		{
			EnableGLStippleMask(context.qGLContext, true);
		}

		if (!visFiltering /*&& !(applyMaterials || showTextures)*/ && (!glParams.showSF || greyForNanScalarValues))
		{
			bool useVBOs = false;
			if (!lodEnabled && context.useVBOs)
				useVBOs = updateVBOs(context, glParams);

			glFunc->glEnableClientState(GL_VERTEX_ARRAY);

			if (glParams.showNorms) 
				glFunc->glEnableClientState(GL_NORMAL_ARRAY);
			if (glParams.showSF || glParams.showColors)
				glFunc->glEnableClientState(GL_COLOR_ARRAY);
			if (glParams.showTexture) {
				glFunc->glEnableClientState(GL_TEXTURE_COORD_ARRAY);
				glFunc->glPushAttrib(GL_ENABLE_BIT);
				glFunc->glEnable(GL_TEXTURE_2D);
			}

			//we can scan and process each chunk separately in an optimized way
			//we mimic the way ccMesh behaves by using virtual chunks!
			size_t chunkCount = ccChunk::Count(displayedTriNum);
			size_t chunkStart = 0;
			for (size_t k = 0; k < chunkCount; ++k, chunkStart += ccChunk::SIZE)
			{
				//virtual chunk size
				const size_t chunkSize = ccChunk::Size(k, displayedTriNum);

				//vertices
				glChunkVertexPointer(context, k, decimStep, useVBOs);

				//scalar field
				if (glParams.showSF)
					glChunkSFPointer(context, k, decimStep, useVBOs);
				//colors
				else if (glParams.showColors)
					glChunkColorPointer(context, k, decimStep, useVBOs);
				
				if (glParams.showNorms)
					glChunkNormalPointer(context, k, decimStep, useVBOs);

				if (showTextures) {
					glChunkTexturePointer(context, k, decimStep, useVBOs);
				}

				if (showFaces || glParams.showTexture) {
					if (glParams.showTexture){
						int firstTriangleOffset = 0;
						GLuint oldTexID = 0;
						glFunc->glBindTexture(GL_TEXTURE_2D, 0);
						// draw by material
						for (auto _in_tri : m_vboManager.vbos[k]->matIndexTrisNum) {
							int mat_index = _in_tri.first;
							unsigned int tri_num = _in_tri.second;
							
							if (showTextures) {
								GLuint newTexID = (mat_index >= 0 && mat_index < materials->size()) ? materials->at(mat_index)->getTextureID() : 0;
								if (newTexID != oldTexID) {
									glFunc->glBindTexture(GL_TEXTURE_2D, newTexID);
									oldTexID = newTexID;
								}
 							}

							if (mat_index >= 0)
								(*materials)[mat_index]->applyGL(context.qGLContext, glParams.showNorms, false);
							else
								context.defaultMat->applyGL(context.qGLContext, glParams.showNorms, false);

							if (showFaces) {
								glFunc->glDrawArrays(GL_TRIANGLES, firstTriangleOffset, tri_num * 3);
								firstTriangleOffset += tri_num * 3;
							}
						}
					}
					else 
						glFunc->glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES, 0, (static_cast<int>(chunkSize) / decimStep) * 3);
				}
				
				if (showWired) {
					glFunc->glDrawElements(GL_LINES, (static_cast<int>(chunkSize) / decimStep) * 6, GL_UNSIGNED_INT, GetWireVertexIndexes());
				}
			}

			//disable arrays
			glFunc->glDisableClientState(GL_VERTEX_ARRAY);
			if (glParams.showNorms)
				glFunc->glDisableClientState(GL_NORMAL_ARRAY);
			if (glParams.showSF || glParams.showColors)
				glFunc->glDisableClientState(GL_COLOR_ARRAY);
			if (glParams.showTexture) {
				glFunc->glDisableClientState(GL_TEXTURE_COORD_ARRAY);
				glFunc->glPopAttrib();
				glFunc->glBindTexture(GL_TEXTURE_2D, 0);
				glFunc->glDisable(GL_TEXTURE_2D);
			}
		}
		else
		{
			//current vertex color
			const ccColor::Rgb *col1 = nullptr, *col2 = nullptr, *col3 = nullptr;
			//current vertex normal
			const PointCoordinateType *N1 = nullptr, *N2 = nullptr, *N3 = nullptr;
			//current vertex texture coordinates
			TexCoords2D *Tx1 = nullptr, *Tx2 = nullptr, *Tx3 = nullptr;

			//loop on all triangles
			int lasMtlIndex = -1;

			if (showTextures)
			{
				glFunc->glPushAttrib(GL_ENABLE_BIT);
				glFunc->glEnable(GL_TEXTURE_2D);
			}

			GLenum triangleDisplayType = lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP : GL_TRIANGLES;
			glFunc->glBegin(triangleDisplayType);

			//per-triangle normals
			const NormsIndexesTableType* triNormals = getTriNormsTable();

			GLuint currentTexID = 0;

			for (unsigned n = 0; n < triNum; ++n)
			{
				//current triangle vertices
				const CCLib::VerticesIndexes* tsi = getTriangleVertIndexes(n);

				//LOD: shall we display this triangle?
				if (n % decimStep)
					continue;

				if (visFiltering)
				{
					//we skip the triangle if at least one vertex is hidden
					if ((verticesVisibility[tsi->i1] != POINT_VISIBLE) ||
						(verticesVisibility[tsi->i2] != POINT_VISIBLE) ||
						(verticesVisibility[tsi->i3] != POINT_VISIBLE))
						continue;
				}

				if (glParams.showSF)
				{
					assert(colorScale);
					col1 = currentDisplayedScalarField->getValueColor(tsi->i1);
					if (!col1)
						continue;
					col2 = currentDisplayedScalarField->getValueColor(tsi->i2);
					if (!col2)
						continue;
					col3 = currentDisplayedScalarField->getValueColor(tsi->i3);
					if (!col3)
						continue;
				}
				else if (glParams.showColors)
				{
					col1 = &rgbColorsTable->at(tsi->i1);
					col2 = &rgbColorsTable->at(tsi->i2);
					col3 = &rgbColorsTable->at(tsi->i3);
				}

				if (glParams.showNorms)
				{
					if (showTriNormals)
					{
						assert(triNormals);
						int n1, n2, n3;
						getTriangleNormalIndexes(n, n1, n2, n3);
						N1 = (n1 >= 0 ?                 ccNormalVectors::GetNormal(triNormals->at(n1)).u : nullptr);
						N2 = (n1 == n2 ? N1 : n1 >= 0 ? ccNormalVectors::GetNormal(triNormals->at(n2)).u : nullptr);
						N3 = (n1 == n3 ? N1 : n3 >= 0 ? ccNormalVectors::GetNormal(triNormals->at(n3)).u : nullptr);

					}
					else
					{
						N1 = compressedNormals->getNormal(normalsIndexesTable->at(tsi->i1)).u;
						N2 = compressedNormals->getNormal(normalsIndexesTable->at(tsi->i2)).u;
						N3 = compressedNormals->getNormal(normalsIndexesTable->at(tsi->i3)).u;
					}
				}

				if (applyMaterials || showTextures)
				{
					assert(materials);
					int newMatlIndex = this->getTriangleMtlIndex(n);

					//do we need to change material?
					if (lasMtlIndex != newMatlIndex)
					{
						assert(newMatlIndex < static_cast<int>(materials->size()));
						glFunc->glEnd();
						if (showTextures)
						{
							if (currentTexID)
							{
								glFunc->glBindTexture(GL_TEXTURE_2D, 0);
								currentTexID = 0;
							}

							if (newMatlIndex >= 0)
							{
								currentTexID = materials->at(newMatlIndex)->getTextureID();
								if (currentTexID)
								{
									glFunc->glBindTexture(GL_TEXTURE_2D, currentTexID);
								}
							}
						}

						//if we don't have any current material, we apply default one
						if (newMatlIndex >= 0)
							(*materials)[newMatlIndex]->applyGL(context.qGLContext, glParams.showNorms,false);
						else
							context.defaultMat->applyGL(context.qGLContext, glParams.showNorms, false);
						glFunc->glBegin(triangleDisplayType);
						lasMtlIndex = newMatlIndex;
					}

					if (showTextures)
					{
						getTriangleTexCoordinates(n, Tx1, Tx2, Tx3);
					}
				}

				if (showWired)
				{
					glFunc->glEnd();
					glFunc->glBegin(triangleDisplayType);
				}

				//vertex 1
				if (N1)
					ccGL::Normal3v(glFunc, N1);
				if (col1)
					glFunc->glColor3ubv(col1->rgb);
				if (Tx1)
					glFunc->glTexCoord2fv(Tx1->t);
				ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i1)->u);

				//vertex 2
				if (N2)
					ccGL::Normal3v(glFunc, N2);
				if (col2)
					glFunc->glColor3ubv(col2->rgb);
				if (Tx2)
					glFunc->glTexCoord2fv(Tx2->t);
				ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i2)->u);

				//vertex 3
				if (N3)
					ccGL::Normal3v(glFunc, N3);
				if (col3)
					glFunc->glColor3ubv(col3->rgb);
				if (Tx3)
					glFunc->glTexCoord2fv(Tx3->t);
				ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i3)->u);
			}

			glFunc->glEnd();

			if (showTextures)
			{
				if (currentTexID)
				{
					glFunc->glBindTexture(GL_TEXTURE_2D, 0);
					currentTexID = 0;
				}
				glFunc->glPopAttrib();
			}
		}

		if (stipplingEnabled())
		{
			EnableGLStippleMask(context.qGLContext, false);
		}

		if (colorMaterial)
		{
			glFunc->glDisable(GL_COLOR_MATERIAL);
		}

		if (glParams.showNorms)
		{
			glFunc->glDisable(GL_LIGHTING);
			glFunc->glDisable(GL_RESCALE_NORMAL);
		}

		if (pushName)
		{
			glFunc->glPopName();
		}
	}
}

bool ccGenericMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

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

bool ccGenericMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//'show wired' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_showWired), sizeof(bool)) < 0)
		return ReadError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (dataVersion >= 29)
	{
		if (in.read(reinterpret_cast<char*>(&m_triNormsShown), sizeof(bool)) < 0)
			return ReadError();

		//'materials shown' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_materialsShown), sizeof(bool)) < 0)
			return ReadError();

		//'polygon stippling' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_stippling), sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

ccPointCloud* ccGenericMesh::samplePoints(	bool densityBased,
											double samplingParameter,
											bool withNormals,
											bool withRGB,
											bool withTexture,
											CCLib::GenericProgressCallback* pDlg/*=nullptr*/)
{
	if (samplingParameter <= 0)
	{
		assert(false);
		return nullptr;
	}

	bool withFeatures = (withNormals || withRGB || withTexture);

	QScopedPointer< std::vector<unsigned> > triIndices;
	if (withFeatures)
	{
		triIndices.reset(new std::vector<unsigned>);
	}

	CCLib::PointCloud* sampledCloud = nullptr;
	if (densityBased)
	{
		sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(this, samplingParameter, pDlg, triIndices.data());
	}
	else
	{
		sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(this, static_cast<unsigned>(samplingParameter), pDlg, triIndices.data());
	}

	//convert to real point cloud
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
		//generate normals
		if (withNormals && hasNormals())
		{
			if (cloud->reserveTheNormsTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->at(i);
					const CCVector3* P = cloud->getPoint(i);

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

		//generate colors
		if (withTexture && hasMaterials())
		{
			if (cloud->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->at(i);
					const CCVector3* P = cloud->getPoint(i);

					ccColor::Rgb C;
					getColorFromMaterial(triIndex, *P, C, withRGB);
					cloud->addRGBColor(C);
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
					unsigned triIndex = triIndices->at(i);
					const CCVector3* P = cloud->getPoint(i);

					ccColor::Rgb C;
					interpolateColors(triIndex, *P, C);
					cloud->addRGBColor(C);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate colors (not enough memory?)");
			}
		}
	}

	//we rename the resulting cloud
	cloud->setName(getName() + QString(".sampled"));
	cloud->setDisplay(getDisplay());
	cloud->prepareDisplayForRefresh();

	//import parameters from both the source vertices and the source mesh
	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (vertices)
	{
		cloud->setGlobalShift(vertices->getGlobalShift());
		cloud->setGlobalScale(vertices->getGlobalScale());
	}
	cloud->setGLTransformationHistory(getGLTransformationHistory());

	return cloud;
}

void ccGenericMesh::importParametersFrom(const ccGenericMesh* mesh)
{
	if (!mesh)
	{
		assert(false);
		return;
	}

	//original shift & scale
	//setGlobalShift(mesh->getGlobalShift());
	//setGlobalScale(mesh->getGlobalScale());

	//stippling
	enableStippling(mesh->stipplingEnabled());
	//wired style
	showWired(mesh->isShownAsWire());
	
	//keep the transformation history!
	setGLTransformationHistory(mesh->getGLTransformationHistory());
	//and meta-data
	setMetaData(mesh->metaData());

	releaseVBOs();
}

void ccGenericMesh::computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const
{
	CCLib::GenericTriangle* tri = const_cast<ccGenericMesh*>(this)->_getTriangle(triIndex);
	const CCVector3 *A = tri->_getA();
	const CCVector3 *B = tri->_getB();
	const CCVector3 *C = tri->_getC();

	//barcyentric intepolation weights
	weights.x = ((P-*B).cross(*C-*B)).normd()/*/2*/;
	weights.y = ((P-*C).cross(*A-*C)).normd()/*/2*/;
	weights.z = ((P-*A).cross(*B-*A)).normd()/*/2*/;

	//normalize weights
	double sum = weights.x + weights.y + weights.z;
	weights /= sum;
}

bool ccGenericMesh::trianglePicking(unsigned triIndex,
									const CCVector2d& clickPos,
									const ccGLMatrix& trans,
									bool noGLTrans,
									const ccGenericPointCloud& vertices,
									const ccGLCameraParameters& camera,
									CCVector3d& point,
									CCVector3d* barycentricCoords/*=nullptr*/) const
{
	assert(triIndex < size());

	CCVector3 A3D, B3D, C3D;
	getTriangleVertices(triIndex, A3D, B3D, C3D);

	CCVector3d A2D, B2D, C2D;
	if (noGLTrans)
	{
		// if none of its points fall into the frustrum the triangle is not visible...
		//DGM: we need to project ALL the points in case at least one is visible
		bool insideA = camera.project(A3D, A2D, true);
		bool insideB = camera.project(B3D, B2D, true);
		bool insideC = camera.project(C3D, C2D, true);
		if (!insideA && !insideB && !insideC)
		{
			return false;
		}
	}
	else
	{
		CCVector3 A3Dp = trans * A3D;
		CCVector3 B3Dp = trans * B3D;
		CCVector3 C3Dp = trans * C3D;
		// if none of its points fall into the frustrum the triangle is not visible...
		//DGM: we need to project ALL the points in case at least one is visible
		bool insideA = camera.project(A3Dp, A2D, true);
		bool insideB = camera.project(B3Dp, B2D, true);
		bool insideC = camera.project(C3Dp, C2D, true);
		if (!insideA && !insideB && !insideC)
		{
			return false;
		}
	}

	//barycentric coordinates
	GLdouble detT =  (B2D.y - C2D.y) *      (A2D.x - C2D.x) + (C2D.x - B2D.x) *      (A2D.y - C2D.y);
	GLdouble l1   = ((B2D.y - C2D.y) * (clickPos.x - C2D.x) + (C2D.x - B2D.x) * (clickPos.y - C2D.y)) / detT;
	GLdouble l2   = ((C2D.y - A2D.y) * (clickPos.x - C2D.x) + (A2D.x - C2D.x) * (clickPos.y - C2D.y)) / detT;

	//does the point falls inside the triangle?
	if (l1 >= 0 && l1 <= 1.0 && l2 >= 0.0 && l2 <= 1.0)
	{
		double l1l2 = l1 + l2;
		assert(l1l2 >= -1.0e-12);
		if (l1l2 > 1.0)
		{
			//we fall outside of the triangle!
			return false;
		}

		GLdouble l3 = 1.0 - l1 - l2;
		assert(l3 >= -1.0e-12);

		//now deduce the 3D position
		point = CCVector3d(	l1 * A3D.x + l2 * B3D.x + l3 * C3D.x,
							l1 * A3D.y + l2 * B3D.y + l3 * C3D.y,
							l1 * A3D.z + l2 * B3D.z + l3 * C3D.z);

		if (barycentricCoords)
		{
			*barycentricCoords = CCVector3d(l1, l2, l3);
		}

		return true;
	}
	else
	{
		return false;
	}
}

bool ccGenericMesh::trianglePicking(const CCVector2d& clickPos,
									const ccGLCameraParameters& camera,
									int& nearestTriIndex,
									double& nearestSquareDist,
									CCVector3d& nearestPoint,
									CCVector3d* barycentricCoords/*=nullptr*/) const
{
	ccGLMatrix trans;
	bool noGLTrans = !getAbsoluteGLTransformation(trans);

	//back project the clicked point in 3D
	CCVector3d clickPosd(clickPos.x, clickPos.y, 0);
	CCVector3d X(0, 0, 0);
	if (!camera.unproject(clickPosd, X))
	{
		return false;
	}

	nearestTriIndex = -1;
	nearestSquareDist = -1.0;
	nearestPoint = CCVector3d(0, 0, 0);
	if (barycentricCoords)
		*barycentricCoords = CCVector3d(0, 0, 0);

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

#if defined(_OPENMP) && !defined(_DEBUG)
	#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(size()); ++i)
	{
		CCVector3d P;
		CCVector3d BC;
		if (!trianglePicking(i, clickPos, trans, noGLTrans, *vertices, camera, P, barycentricCoords ? &BC : nullptr))
			continue;

		double squareDist = (X - P).norm2d();
		if (nearestTriIndex < 0 || squareDist < nearestSquareDist)
		{
			nearestSquareDist = squareDist;
			nearestTriIndex = static_cast<int>(i);
			nearestPoint = P;
			if (barycentricCoords)
				*barycentricCoords = BC;
		}
	}

	return (nearestTriIndex >= 0);
}

bool ccGenericMesh::trianglePicking(unsigned triIndex,
									const CCVector2d& clickPos,
									const ccGLCameraParameters& camera,
									CCVector3d& point,
									CCVector3d* barycentricCoords/*=nullptr*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		return false;
	}

	ccGLMatrix trans;
	bool noGLTrans = !getAbsoluteGLTransformation(trans);

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

	return trianglePicking(triIndex, clickPos, trans, noGLTrans, *vertices, camera, point, barycentricCoords);
}

bool ccGenericMesh::computePointPosition(unsigned triIndex, const CCVector2d& uv, CCVector3& P, bool warningIfOutside/*=true*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		ccLog::Warning("Index out of range");
		return true;
	}
	
	CCVector3 A, B, C;
	getTriangleVertices(triIndex, A, B, C);

	double z = 1.0 - uv.x - uv.y;
	if (warningIfOutside && ((z < -1.0e-6) || (z > 1.0 + 1.0e-6)))
	{
		ccLog::Warning("Point falls outside of the triangle");
	}
	
	P = CCVector3(	static_cast<PointCoordinateType>(uv.x * A.x + uv.y * B.x + z * C.x),
					static_cast<PointCoordinateType>(uv.x * A.y + uv.y * B.y + z * C.y),
					static_cast<PointCoordinateType>(uv.x * A.z + uv.y * B.z + z * C.z));

	return true;
}

static bool CatchGLErrors(GLenum err, const char* context)
{
	//catch GL errors
	{
		//see http://www.opengl.org/sdk/docs/man/xhtml/glGetError.xml
		switch (err)
		{
		case GL_NO_ERROR:
			return false;
		case GL_INVALID_ENUM:
			ccLog::Warning("[%s] OpenGL error: invalid enumerator", context);
			break;
		case GL_INVALID_VALUE:
			ccLog::Warning("[%s] OpenGL error: invalid value", context);
			break;
		case GL_INVALID_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid operation", context);
			break;
		case GL_STACK_OVERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack overflow", context);
			break;
		case GL_STACK_UNDERFLOW:
			ccLog::Warning("[%s] OpenGL error: stack underflow", context);
			break;
		case GL_OUT_OF_MEMORY:
			ccLog::Warning("[%s] OpenGL error: out of memory", context);
			break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:
			ccLog::Warning("[%s] OpenGL error: invalid framebuffer operation", context);
			break;
		}
	}

	return true;
}

//#define DONT_LOAD_NORMALS_IN_VBOS

bool ccGenericMesh::updateVBOs(const CC_DRAW_CONTEXT & context, const glDrawParams & glParams)
{
	if (isColorOverriden())
	{
		//nothing to do (we don't display true colors, SF or normals!)
		return false;
	}

	if (m_vboManager.state == vboSet::FAILED)
	{
		//ccLog::Warning(QString("[ccPointCloud::updateVBOs] VBOs are in a 'failed' state... we won't try to update them! (cloud '%1')").arg(getName()));
		return false;
	}

	if (!m_currentDisplay)
	{
		ccLog::Warning(QString("[ccPointCloud::updateVBOs] Need an associated GL context! (cloud '%1')").arg(getName()));
		//assert(false);
		return false;
	}

	bool pushName = MACRO_DrawEntityNames(context);
	ccGenericPointCloud* vertices = getAssociatedCloud(); if (!vertices) return false;
	
	ccScalarField* currentDisplayedScalarField = nullptr;
	bool greyForNanScalarValues = true;
	ccColorScale::Shared colorScale(nullptr);

	if (glParams.showSF) {
		ccPointCloud* cloud = static_cast<ccPointCloud*>(vertices); assert(cloud);
		
		currentDisplayedScalarField = cloud->getCurrentDisplayedScalarField();
		colorScale = currentDisplayedScalarField->getColorScale();
		//colorRampSteps = currentDisplayedScalarField->getColorRampSteps();

		assert(colorScale);
		//get default color ramp if cloud has no scale associated?!
		if (!colorScale)
			colorScale = ccColorScalesManager::GetUniqueInstance()->getDefaultScale(ccColorScalesManager::BGYR);
	}
	

	if (m_vboManager.state == vboSet::INITIALIZED)
	{
		//let's check if something has changed
		if (glParams.showColors && (!m_vboManager.hasColors || m_vboManager.colorIsSF))
		{
			m_vboManager.updateFlags |= vboSet::UPDATE_COLORS;
		}

		if (glParams.showSF
			&& (!m_vboManager.hasColors
				|| !m_vboManager.colorIsSF
				|| m_vboManager.sourceSF != currentDisplayedScalarField
				|| currentDisplayedScalarField->getModificationFlag() == true))
		{
			m_vboManager.updateFlags |= vboSet::UPDATE_COLORS;
		}

#ifndef DONT_LOAD_NORMALS_IN_VBOS
		if (glParams.showNorms && !m_vboManager.hasNormals)
		{
			m_vboManager.updateFlags |= vboSet::UPDATE_NORMALS;
		}
#endif

		if (glParams.showTexture && !m_vboManager.hasTexture) {
			m_vboManager.updateFlags |= vboSet::UPDATE_TEXTURE;
		}

		//nothing to do?
		if (m_vboManager.updateFlags == 0)
		{
			return true;
		}
	}
	else
	{
		m_vboManager.updateFlags = vboSet::UPDATE_ALL;
	}
	
	size_t chunksCount = ccChunk::Count(size());
	//allocate per-chunk descriptors if necessary
	if (m_vboManager.vbos.size() != chunksCount)
	{
		//properly remove the elements that are not needed anymore!
		for (size_t i = chunksCount; i < m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = nullptr;
			}
		}

		//resize the container
		try
		{
			m_vboManager.vbos.resize(chunksCount, nullptr);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[ccPointCloud::updateVBOs] Not enough memory! (cloud '%1')").arg(getName()));
			m_vboManager.state = vboSet::FAILED;
			return false;
		}
	}

	//init VBOs
	unsigned pointsInVBOs = 0;
	int totalSizeBytesBefore = m_vboManager.totalMemSizeBytes;
	m_vboManager.totalMemSizeBytes = 0;
	{
		//DGM: the context should be already active as this method should only be called from 'drawMeOnly'
		assert(!glParams.showSF || currentDisplayedScalarField);
		//assert(!glParams.showColors || m_rgbColors);	// TODO
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		//assert(!glParams.showNorms || (m_normals && m_normals->chunksCount() >= chunksCount));	// TODO
#endif

		m_vboManager.hasColors = glParams.showSF || glParams.showColors;
		m_vboManager.colorIsSF = glParams.showSF;
		m_vboManager.sourceSF = glParams.showSF ? currentDisplayedScalarField : nullptr;
#ifndef DONT_LOAD_NORMALS_IN_VBOS
		m_vboManager.hasNormals = glParams.showNorms;
#else
		m_vboManager.hasNormals = false;
#endif
		m_vboManager.hasTexture = glParams.showTexture;

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());

		size_t decimStep = 1;
		//process each chunk
		size_t chunkStart = 0;
		for (size_t i = 0; i < chunksCount; ++i, chunkStart+= ccChunk::SIZE)
		{
			int chunkSize = static_cast<int>(ccChunk::Size(i, size()));

			int chunkUpdateFlags = m_vboManager.updateFlags;
			bool reallocated = false;
			if (!m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i] = new VBO;
			}

			//allocate memory for current VBO
			int vboSizeBytes = m_vboManager.vbos[i]->init(chunkSize * 3, m_vboManager.hasColors, m_vboManager.hasNormals, &reallocated);	// XYLIU * 3 for triangle drawing

			QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
			if (glFunc)
			{
				CatchGLErrors(glFunc->glGetError(), "vbo.init");
			}

			if (vboSizeBytes > 0)
			{
				//ccLog::Print(QString("[VBO] VBO #%1 initialized (ID=%2)").arg(i).arg(m_vboManager.vbos[i]->bufferId()));

				if (reallocated)
				{
					//if the vbo is reallocated, then all its content has been cleared!
					chunkUpdateFlags = vboSet::UPDATE_ALL;
				}

				m_vboManager.vbos[i]->bind();

				//load points
				if (chunkUpdateFlags & vboSet::UPDATE_POINTS)
				{
					CCVector3* _vertices = GetVertexBuffer();
					for (size_t n = 0; n < chunkSize; n += decimStep)
					{
						const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
  						assert(ti->i1 < vertices->size());
  						assert(ti->i2 < vertices->size());
  						assert(ti->i3 < vertices->size());
						*_vertices++ = *vertices->getPoint(ti->i1);
						*_vertices++ = *vertices->getPoint(ti->i2);
						*_vertices++ = *vertices->getPoint(ti->i3);
					}
					m_vboManager.vbos[i]->write(0, GetVertexBuffer(), sizeof(CCVector3)*chunkSize * 3);
				}
				//load colors
				if (chunkUpdateFlags & vboSet::UPDATE_COLORS)
				{
					if (glParams.showSF) {
						//copy SF colors in static array
						{
							ccColor::Rgb* _sfColors = GetColorsBuffer();
							for (unsigned n = 0; n < chunkSize; n += decimStep)	{
								const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
								*_sfColors++ = *m_vboManager.sourceSF->getValueColor(ti->i1);
								*_sfColors++ = *m_vboManager.sourceSF->getValueColor(ti->i2);
								*_sfColors++ = *m_vboManager.sourceSF->getValueColor(ti->i3);
							}
						}
						//then send them in VRAM
						m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->rgbShift, GetColorsBuffer(), sizeof(ccColor::Rgb)*chunkSize * 3);
						//upadte 'modification' flag for current displayed SF
						m_vboManager.sourceSF->setModificationFlag(false);
					}
					else if (glParams.showColors) {
						ColorsTableType* rgbColorsTable = static_cast<ccPointCloud*>(vertices)->rgbColors();
						ccColor::Rgb* _rgbColors = GetColorsBuffer();
						for (unsigned n = 0; n < chunkSize; n += decimStep)
						{
							const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
							*_rgbColors++ = rgbColorsTable->at(ti->i1);
							*_rgbColors++ = rgbColorsTable->at(ti->i2);
							*_rgbColors++ = rgbColorsTable->at(ti->i3);
						}
						m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->rgbShift, GetColorsBuffer(), sizeof(ccColor::Rgb)*chunkSize * 3);
					}
				}

#ifndef DONT_LOAD_NORMALS_IN_VBOS
				//load normals
				if (glParams.showNorms && (chunkUpdateFlags & vboSet::UPDATE_NORMALS))
				{
					//we must decode the normals first!
					CCVector3* _normals = GetNormalsBuffer();
					if (showTriNormals)	{
						for (unsigned n = 0; n < chunkSize; n += decimStep)	{
							CCVector3 Na, Nb, Nc;
							getTriangleNormals(static_cast<unsigned>(chunkStart + n), Na, Nb, Nc);
							*_normals++ = Na;
							*_normals++ = Nb;
							*_normals++ = Nc;
						}
					}
					else {
						for (unsigned n = 0; n < chunkSize; n += decimStep)
						{
							const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
							*_normals++ = vertices->getPointNormal(ti->i1);
							*_normals++ = vertices->getPointNormal(ti->i2);
							*_normals++ = vertices->getPointNormal(ti->i3);
						}
					}
					m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->normalShift, GetNormalsBuffer(), sizeof(CCVector3)*chunkSize * 3);
				}
#endif

				if (glParams.showTexture && (chunkUpdateFlags & vboSet::UPDATE_TEXTURE)) {
					std::vector<std::pair<int, unsigned int>> & _matIndTrisNum = m_vboManager.vbos[i]->matIndexTrisNum;
					_matIndTrisNum.clear();
					//loop on all triangles
					int lasMtlIndex = -1;

					//materials
					const ccMaterialSet* materials = getMaterialSet();

					CCVector2* _texIndex = GetTextureBuffer();
					for (unsigned n = 0; n < chunkSize; n += decimStep)	{
						int newMatlIndex = this->getTriangleMtlIndex(chunkStart + n);
						//do we need to change material?
						if (lasMtlIndex != newMatlIndex) {
							if (newMatlIndex >= 0 && newMatlIndex < static_cast<int>(materials->size())) { //currentTexID = materials->at(newMatlIndex)->getTextureID();
								_matIndTrisNum.push_back(std::make_pair(newMatlIndex, 0));
							}
							lasMtlIndex = newMatlIndex;
						}
						if (!_matIndTrisNum.empty()) {
							_matIndTrisNum.back().second++;
						}

						TexCoords2D *Tx1 = nullptr, *Tx2 = nullptr, *Tx3 = nullptr;
						getTriangleTexCoordinates(static_cast<unsigned>(chunkStart + n), Tx1, Tx2, Tx3);
						*_texIndex++ = CCVector2(Tx1->t);
						*_texIndex++ = CCVector2(Tx2->t);
						*_texIndex++ = CCVector2(Tx3->t);
					}
					m_vboManager.vbos[i]->write(m_vboManager.vbos[i]->texShift, GetTextureBuffer(), sizeof(CCVector2)*chunkSize * 3);

					//! the triangle numbers per material

				}

				m_vboManager.vbos[i]->release();

				//if an error is detected
				QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
				assert(glFunc != nullptr);
				if (CatchGLErrors(glFunc->glGetError(), "ccMesh::updateVBOs"))
				{
					vboSizeBytes = -1;
				}
				else
				{
					m_vboManager.totalMemSizeBytes += vboSizeBytes;
					pointsInVBOs += chunkSize;
				}
			}

			if (vboSizeBytes < 0) //VBO initialization failed
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = nullptr;

				//we can stop here
				if (i == 0)
				{
					ccLog::Warning(QString("[ccMesh::updateVBOs] Failed to initialize VBOs (not enough memory?) (mesh '%1')").arg(getName()));
					m_vboManager.state = vboSet::FAILED;
					m_vboManager.vbos.resize(0);
					return false;
				}
				else
				{
					//shouldn't be better for the next VBOs!
					break;
				}
			}
		}
	}

	//Display vbo(s) status
	//{
	//	for (unsigned i=0; i<chunksCount; ++i)
	//		ccLog::Print(QString("[VBO] VBO #%1 status: %2 (ID=%3)")
	//			.arg(i)
	//			.arg(m_vboManager.vbos[i] && m_vboManager.vbos[i]->isCreated() ? "created" : "not created")
	//			.arg(m_vboManager.vbos[i] ? m_vboManager.vbos[i]->bufferId() : -1));
	//}

#ifdef _DEBUG
	if (m_vboManager.totalMemSizeBytes != totalSizeBytesBefore)
		ccLog::Print(QString("[VBO] VBO(s) (re)initialized for mesh '%1' (%2 Mb = %3% of triangles could be loaded)")
			.arg(getName())
			.arg(static_cast<double>(m_vboManager.totalMemSizeBytes) / (1 << 20), 0, 'f', 2)
			.arg(static_cast<double>(pointsInVBOs) / size() * 100.0, 0, 'f', 2));
#endif

	m_vboManager.state = vboSet::INITIALIZED;
	m_vboManager.updateFlags = 0;

	return true;
}

void ccGenericMesh::releaseVBOs()
{
	if (m_vboManager.state == vboSet::NEW)
		return;

	if (m_currentDisplay)
	{
		//'destroy' all vbos
		for (size_t i = 0; i < m_vboManager.vbos.size(); ++i)
		{
			if (m_vboManager.vbos[i])
			{
				m_vboManager.vbos[i]->destroy();
				delete m_vboManager.vbos[i];
				m_vboManager.vbos[i] = nullptr;
			}
		}
	}
	else
	{
		assert(m_vboManager.vbos.empty());
	}

	m_vboManager.vbos.resize(0);
	m_vboManager.hasColors = false;
	m_vboManager.hasNormals = false;
	m_vboManager.colorIsSF = false;
	m_vboManager.sourceSF = nullptr;
	m_vboManager.totalMemSizeBytes = 0;
	m_vboManager.state = vboSet::NEW;
}

// update for vbo
void ccGenericMesh::notifyNormalUpdate() { m_vboManager.updateFlags |= vboSet::UPDATE_NORMALS; }
void ccGenericMesh::notifyColorUpdate() { m_vboManager.updateFlags |= vboSet::UPDATE_COLORS; }
void ccGenericMesh::notifyTextureUpdate() { m_vboManager.updateFlags |= vboSet::UPDATE_TEXTURE; }

void ccGenericMesh::glChunkVertexPointer(const CC_DRAW_CONTEXT & context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	unsigned displayedTriNum = size() / decimStep;	if (displayedTriNum == 0) return;

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			glFunc->glVertexPointer(3, GL_COORD_TYPE, GLsizei(0), nullptr);
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkVertexPointer(context, chunkIndex, decimStep, false);
		}
	}
	else
	{
		//standard OpenGL copy
		glFunc->glVertexPointer(3, GL_COORD_TYPE, GLsizei(0)/*decimStep * 3 * sizeof(CCVector3)*/, GetVertexBuffer()/*ccChunk::Start(m_points, chunkIndex)*/);

		int chunkSize = static_cast<int>(ccChunk::Size(chunkIndex, displayedTriNum));
		const CCLib::VerticesIndexes* _vertIndexesChunkOrigin = getTriangleVertIndexes(ccChunk::StartPos(chunkIndex));
		const CCLib::VerticesIndexes* _vertIndexes = _vertIndexesChunkOrigin;
		CCVector3* _vertices = GetVertexBuffer();
		ccGenericPointCloud* vertices = getAssociatedCloud();
		for (size_t n = 0; n < chunkSize; n += decimStep, _vertIndexes += decimStep)
		{
			assert(_vertIndexes->i1 < vertices->size());
			assert(_vertIndexes->i2 < vertices->size());
			assert(_vertIndexes->i3 < vertices->size());
			*_vertices++ = *vertices->getPoint(_vertIndexes->i1);
			*_vertices++ = *vertices->getPoint(_vertIndexes->i2);
			*_vertices++ = *vertices->getPoint(_vertIndexes->i3);
		}
	}
}

void ccGenericMesh::glChunkColorPointer(const CC_DRAW_CONTEXT & context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	unsigned displayedTriNum = size() / decimStep;	if (displayedTriNum == 0) return;
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasColors
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, GLsizei(0), static_cast<const GLvoid*>(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkColorPointer(context, chunkIndex, decimStep, false);
		}
	}
	else 
	{
		//standard OpenGL copy
		size_t chunkSize = ccChunk::Size(chunkIndex, displayedTriNum);
		unsigned chunkStart = ccChunk::StartPos(chunkIndex);
		
		ccGenericPointCloud* vertices = getAssociatedCloud();	if (!vertices) return;
		ColorsTableType* rgbColorsTable = static_cast<ccPointCloud*>(vertices)->rgbColors(); if (!rgbColorsTable) return;

		ccColor::Rgb* _rgbColors = GetColorsBuffer();
		for (unsigned n = 0; n < chunkSize; n += decimStep)
		{
			const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
			*_rgbColors++ = rgbColorsTable->at(ti->i1);
			*_rgbColors++ = rgbColorsTable->at(ti->i2);
			*_rgbColors++ = rgbColorsTable->at(ti->i3);
		}

		glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, GLsizei(0), GetColorsBuffer());
	}
}

void ccGenericMesh::glChunkSFPointer(const CC_DRAW_CONTEXT & context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	unsigned displayedTriNum = size() / decimStep;	if (displayedTriNum == 0) return;
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasColors
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//assert(m_vboManager.colorIsSF && m_vboManager.sourceSF == currentDisplayedScalarField);
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int colorDataShift = m_vboManager.vbos[chunkIndex]->rgbShift;
			glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, GLsizei(0), static_cast<const GLvoid*>(start + colorDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkSFPointer(context, chunkIndex, decimStep, false);
		}
	}
	else //if (m_currentDisplayedScalarField)
	{
		size_t chunkSize = ccChunk::Size(chunkIndex, displayedTriNum);
		unsigned chunkStart = ccChunk::StartPos(chunkIndex);

		ccGenericPointCloud* vertices = getAssociatedCloud();
		ccScalarField* currentDisplayedScalarField = static_cast<ccPointCloud*>(vertices)->getCurrentDisplayedScalarField();
		if (!currentDisplayedScalarField) { return; }

		//we must convert the scalar values to RGB colors in a dedicated static array
		ccColor::Rgb* _sfColors = GetColorsBuffer();
		for (unsigned n = 0; n < chunkSize; n += decimStep)
		{
			const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
			*_sfColors++ = *currentDisplayedScalarField->getValueColor(ti->i1);
			*_sfColors++ = *currentDisplayedScalarField->getValueColor(ti->i2);
			*_sfColors++ = *currentDisplayedScalarField->getValueColor(ti->i3);
		}
		glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, GLsizei(0), GetColorsBuffer());
	}
}

void ccGenericMesh::glChunkNormalPointer(const CC_DRAW_CONTEXT & context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	//assert(m_normals);
	unsigned displayedTriNum = size() / decimStep;	if (displayedTriNum == 0) return;
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&&	m_vboManager.state == vboSet::INITIALIZED
		&&	m_vboManager.hasNormals
		&&	m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int normalDataShift = m_vboManager.vbos[chunkIndex]->normalShift;
			glFunc->glNormalPointer(GL_COORD_TYPE, GLsizei(0), static_cast<const GLvoid*>(start + normalDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkNormalPointer(context, chunkIndex, decimStep, false);
		}
	}
	else
	{
		//we must decode normals in a dedicated static array

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());
		size_t chunkSize = ccChunk::Size(chunkIndex, displayedTriNum);
		unsigned chunkStart = ccChunk::StartPos(chunkIndex);

		CCVector3* _normals = GetNormalsBuffer();
		if (showTriNormals) {
			for (unsigned n = 0; n < chunkSize; n += decimStep)
			{
				CCVector3 Na, Nb, Nc;
				getTriangleNormals(static_cast<unsigned>(chunkStart + n), Na, Nb, Nc);
				*_normals++ = Na;
				*_normals++ = Nb;
				*_normals++ = Nc;
			}
		}
		else {
			ccGenericPointCloud* vertices = getAssociatedCloud();
			for (unsigned n = 0; n < chunkSize; n += decimStep)
			{
				const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(static_cast<unsigned>(chunkStart + n));
				*_normals++ = vertices->getPointNormal(ti->i1);
				*_normals++ = vertices->getPointNormal(ti->i2);
				*_normals++ = vertices->getPointNormal(ti->i3);
			}
		}
		glFunc->glNormalPointer(GL_COORD_TYPE, GLsizei(0), GetNormalsBuffer());
	}
}

void ccGenericMesh::glChunkTexturePointer(const CC_DRAW_CONTEXT & context, size_t chunkIndex, unsigned decimStep, bool useVBOs)
{
	//assert(m_normals);
	unsigned displayedTriNum = size() / decimStep;	if (displayedTriNum == 0) return;
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (useVBOs
		&& m_vboManager.state == vboSet::INITIALIZED
		&& m_vboManager.hasTexture
		&& m_vboManager.vbos.size() > static_cast<size_t>(chunkIndex)
		&& m_vboManager.vbos[chunkIndex]
		&& m_vboManager.vbos[chunkIndex]->isCreated())
	{
		//we can use VBOs directly
		if (m_vboManager.vbos[chunkIndex]->bind())
		{
			const GLbyte* start = nullptr; //fake pointer used to prevent warnings on Linux
			int texDataShift = m_vboManager.vbos[chunkIndex]->texShift;
			glFunc->glTexCoordPointer(2, GL_COORD_TYPE, 0, static_cast<const GLvoid*>(start + texDataShift));
			m_vboManager.vbos[chunkIndex]->release();
		}
		else
		{
			ccLog::Warning("[VBO] Failed to bind VBO?! We'll deactivate them then...");
			m_vboManager.state = vboSet::FAILED;
			//recall the method
			glChunkTexturePointer(context, chunkIndex, decimStep, false);
		}
	}
	else {
		//we must decode normals in a dedicated static array
		size_t chunkSize = ccChunk::Size(chunkIndex, displayedTriNum);
		unsigned chunkStart = ccChunk::StartPos(chunkIndex);

		CCVector2* _texIndex = GetTextureBuffer();
		for (unsigned n = 0; n < chunkSize; n += decimStep)
		{
			TexCoords2D *Tx1 = nullptr, *Tx2 = nullptr, *Tx3 = nullptr;
			getTriangleTexCoordinates(static_cast<unsigned>(chunkStart + n), Tx1, Tx2, Tx3);
			*_texIndex++ = CCVector2(Tx1->t);
			*_texIndex++ = CCVector2(Tx2->t);
			*_texIndex++ = CCVector2(Tx3->t);
		}
		glFunc->glTexCoordPointer(2, GL_COORD_TYPE, GLsizei(0), GetTextureBuffer());
	}
}

