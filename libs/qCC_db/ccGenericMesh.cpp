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
#include "ccHObjectCaster.h"
#include "ccGenericPointCloud.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"
#include "ccScalarField.h"
#include "ccColorScalesManager.h"
#include "ccGenericGLDisplay.h"

//CCLib
#include <GenericTriangle.h>
#include <MeshSamplingTools.h>
#include <SimpleCloud.h>

//system
#include <assert.h>

ccGenericMesh::ccGenericMesh(QString name/*=QString()*/)
	: GenericIndexedMesh()
	, ccHObject(name)
	, m_triNormsShown(false)
	, m_materialsShown(false)
	, m_showWired(false)
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
PointCoordinateType* ccGenericMesh::GetVertexBuffer()
{
	static PointCoordinateType s_xyzBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
	return s_xyzBuffer;
}

//Normals buffer
PointCoordinateType* ccGenericMesh::GetNormalsBuffer()
{
	static PointCoordinateType s_normBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
	return s_normBuffer;
}

//Colors buffer
ColorCompType* ccGenericMesh::GetColorsBuffer()
{
	static ColorCompType s_rgbBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
	return s_rgbBuffer;
}

//Vertex indexes buffer (for wired display)
static unsigned s_vertWireIndexes[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*6];
static bool s_vertIndexesInitialized = false;
unsigned* ccGenericMesh::GetWireVertexIndexes()
{
	//on first call, we init the array
	if (!s_vertIndexesInitialized)
	{
		unsigned* _vertWireIndexes = s_vertWireIndexes;
		for (unsigned i=0; i<MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3; ++i)
		{
			*_vertWireIndexes++ = i;
			*_vertWireIndexes++ = ( ((i+1) % 3) == 0 ? i-2 : i+1);
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
		const ccGenericPointCloud::VisibilityTableType* verticesVisibility = vertices->getTheVisibilityArray();
		bool visFiltering = (verticesVisibility && verticesVisibility->isAllocated());

		//wireframe ? (not compatible with LOD)
		bool showWired = isShownAsWire() && !lodEnabled;

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());
		//fix 'showNorms'
		glParams.showNorms = showTriNormals || (vertices->hasNormals() && m_normalsDisplayed);

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
		ccScalarField* currentDisplayedScalarField = 0;
		bool greyForNanScalarValues = true;
		//unsigned colorRampSteps = 0;
		ccColorScale::Shared colorScale(0);

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
		ColorsTableType* rgbColorsTable = 0;
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
		NormsIndexesTableType* normalsIndexesTable = 0;
		ccNormalVectors* compressedNormals = 0;
		if (glParams.showNorms)
		{
			assert(vertices->isA(CC_TYPES::POINT_CLOUD));
			normalsIndexesTable = static_cast<ccPointCloud*>(vertices)->normals();
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		//stipple mask
		if (stipplingEnabled())
		{
			EnableGLStippleMask(context.qGLContext, true);
		}

		if (!visFiltering && !(applyMaterials || showTextures) && (!glParams.showSF || greyForNanScalarValues))
		{
			//the GL type depends on the PointCoordinateType 'size' (float or double)
			GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;

			glFunc->glEnableClientState(GL_VERTEX_ARRAY);
			glFunc->glVertexPointer(3, GL_COORD_TYPE, 0, GetVertexBuffer());

			if (glParams.showNorms)
			{
				glFunc->glEnableClientState(GL_NORMAL_ARRAY);
				glFunc->glNormalPointer(GL_COORD_TYPE, 0, GetNormalsBuffer());
			}
			if (glParams.showSF || glParams.showColors)
			{
				glFunc->glEnableClientState(GL_COLOR_ARRAY);
				glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, 0, GetColorsBuffer());
			}

			//we can scan and process each chunk separately in an optimized way
			//we mimic the way ccMesh beahves by using virtual chunks!
			unsigned chunks = static_cast<unsigned>(ceil(static_cast<double>(displayedTriNum) / MAX_NUMBER_OF_ELEMENTS_PER_CHUNK));
			unsigned chunkStart = 0;
			const ColorCompType* col = 0;
			for (unsigned k=0; k<chunks; ++k, chunkStart += MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				//virtual chunk size
				const unsigned chunkSize = k+1 < chunks ? MAX_NUMBER_OF_ELEMENTS_PER_CHUNK : (displayedTriNum % MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);

				//vertices
				PointCoordinateType* _vertices = GetVertexBuffer();
				for (unsigned n=0; n<chunkSize; n+=decimStep)
				{
					const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(chunkStart + n);
					memcpy(_vertices, vertices->getPoint(ti->i1)->u, sizeof(PointCoordinateType) * 3);
					_vertices += 3;
					memcpy(_vertices, vertices->getPoint(ti->i2)->u, sizeof(PointCoordinateType) * 3);
					_vertices += 3;
					memcpy(_vertices, vertices->getPoint(ti->i3)->u, sizeof(PointCoordinateType) * 3);
					_vertices += 3;
				}

				//scalar field
				if (glParams.showSF)
				{
					ColorCompType* _rgbColors = GetColorsBuffer();
					assert(colorScale);
					for (unsigned n=0; n<chunkSize; n+=decimStep)
					{
						const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(chunkStart + n);
						col = currentDisplayedScalarField->getValueColor(ti->i1);
						memcpy(_rgbColors, col, sizeof(ColorCompType) * 3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(ti->i2);
						memcpy(_rgbColors, col, sizeof(ColorCompType) * 3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(ti->i3);
						memcpy(_rgbColors, col, sizeof(ColorCompType) * 3);
						_rgbColors += 3;
					}
				}
				//colors
				else if (glParams.showColors)
				{
					ColorCompType* _rgbColors = GetColorsBuffer();

					for (unsigned n=0; n<chunkSize; n+=decimStep)
					{
						const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(chunkStart + n);
						memcpy(_rgbColors, rgbColorsTable->getValue(ti->i1), sizeof(ColorCompType) * 3);
						_rgbColors += 3;
						memcpy(_rgbColors, rgbColorsTable->getValue(ti->i2), sizeof(ColorCompType) * 3);
						_rgbColors += 3;
						memcpy(_rgbColors, rgbColorsTable->getValue(ti->i3), sizeof(ColorCompType) * 3);
						_rgbColors += 3;
					}
				}

				//normals
				if (glParams.showNorms)
				{
					PointCoordinateType* _normals = GetNormalsBuffer();
					if (showTriNormals)
					{
						for (unsigned n=0; n<chunkSize; n+=decimStep)
						{
							CCVector3 Na, Nb, Nc;
							getTriangleNormals(chunkStart + n, Na, Nb, Nc);
							memcpy(_normals, Na.u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
							memcpy(_normals, Nb.u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
							memcpy(_normals, Nc.u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
						}
					}
					else
					{
						for (unsigned n=0; n<chunkSize; n+=decimStep)
						{
							const CCLib::VerticesIndexes* ti = getTriangleVertIndexes(chunkStart + n);
							memcpy(_normals, vertices->getPointNormal(ti->i1).u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
							memcpy(_normals, vertices->getPointNormal(ti->i2).u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
							memcpy(_normals, vertices->getPointNormal(ti->i3).u, sizeof(PointCoordinateType) * 3);
							_normals += 3;
						}
					}
				}

				if (!showWired)
				{
					glFunc->glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES, 0, (chunkSize / decimStep) * 3);
				}
				else
				{
					glFunc->glDrawElements(GL_LINES, (chunkSize / decimStep) * 6, GL_UNSIGNED_INT, GetWireVertexIndexes());
				}
			}

			//disable arrays
			glFunc->glDisableClientState(GL_VERTEX_ARRAY);
			if (glParams.showNorms)
				glFunc->glDisableClientState(GL_NORMAL_ARRAY);
			if (glParams.showSF || glParams.showColors)
				glFunc->glDisableClientState(GL_COLOR_ARRAY);
		}
		else
		{
			//current vertex color
			const ColorCompType *col1 = 0, *col2 = 0, *col3 = 0;
			//current vertex normal
			const PointCoordinateType *N1 = 0, *N2 = 0, *N3 = 0;
			//current vertex texture coordinates
			float *Tx1 = 0, *Tx2 = 0, *Tx3 = 0;

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
			//materials
			const ccMaterialSet* materials = getMaterialSet();

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
					if ((verticesVisibility->getValue(tsi->i1) != POINT_VISIBLE) ||
						(verticesVisibility->getValue(tsi->i2) != POINT_VISIBLE) ||
						(verticesVisibility->getValue(tsi->i3) != POINT_VISIBLE))
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
					col1 = rgbColorsTable->getValue(tsi->i1);
					col2 = rgbColorsTable->getValue(tsi->i2);
					col3 = rgbColorsTable->getValue(tsi->i3);
				}

				if (glParams.showNorms)
				{
					if (showTriNormals)
					{
						assert(triNormals);
						int n1, n2, n3;
						getTriangleNormalIndexes(n, n1, n2, n3);
						N1 = (n1 >= 0 ? ccNormalVectors::GetNormal(triNormals->getValue(n1)).u : 0);
						N2 = (n1 == n2 ? N1 : n1 >= 0 ? ccNormalVectors::GetNormal(triNormals->getValue(n2)).u : 0);
						N3 = (n1 == n3 ? N1 : n3 >= 0 ? ccNormalVectors::GetNormal(triNormals->getValue(n3)).u : 0);

					}
					else
					{
						N1 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i1)).u;
						N2 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i2)).u;
						N3 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i3)).u;
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
					glFunc->glColor3ubv(col1);
				if (Tx1)
					glFunc->glTexCoord2fv(Tx1);
				ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i1)->u);

				//vertex 2
				if (N2)
					ccGL::Normal3v(glFunc, N2);
				if (col2)
					glFunc->glColor3ubv(col2);
				if (Tx2)
					glFunc->glTexCoord2fv(Tx2);
				ccGL::Vertex3v(glFunc, vertices->getPoint(tsi->i2)->u);

				//vertex 3
				if (N3)
					ccGL::Normal3v(glFunc, N3);
				if (col3)
					glFunc->glColor3ubv(col3);
				if (Tx3)
					glFunc->glTexCoord2fv(Tx3);
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
	if (out.write((const char*)&m_showWired,sizeof(bool)) < 0)
		return WriteError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (out.write((const char*)&m_triNormsShown,sizeof(bool)) < 0)
		return WriteError();

	//'materials shown' state (dataVersion>=29))
	if (out.write((const char*)&m_materialsShown,sizeof(bool)) < 0)
		return WriteError();

	//'polygon stippling' state (dataVersion>=29))
	if (out.write((const char*)&m_stippling,sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool ccGenericMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//'show wired' state (dataVersion>=20)
	if (in.read((char*)&m_showWired,sizeof(bool)) < 0)
		return ReadError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (dataVersion >= 29)
	{
		if (in.read((char*)&m_triNormsShown,sizeof(bool)) < 0)
			return ReadError();

		//'materials shown' state (dataVersion>=29))
		if (in.read((char*)&m_materialsShown,sizeof(bool)) < 0)
			return ReadError();

		//'polygon stippling' state (dataVersion>=29))
		if (in.read((char*)&m_stippling,sizeof(bool)) < 0)
			return ReadError();
	}

	return true;
}

ccPointCloud* ccGenericMesh::samplePoints(	bool densityBased,
											double samplingParameter,
											bool withNormals,
											bool withRGB,
											bool withTexture,
											CCLib::GenericProgressCallback* pDlg/*=0*/)
{
	bool withFeatures = (withNormals || withRGB || withTexture);

	GenericChunkedArray<1,unsigned>* triIndices = (withFeatures ? new GenericChunkedArray<1,unsigned> : 0);

	CCLib::SimpleCloud* sampledCloud = 0;
	if (densityBased)
	{
		sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(this,samplingParameter,pDlg,triIndices);
	}
	else
	{
		sampledCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(this,static_cast<unsigned>(samplingParameter),pDlg,triIndices);
	}

	//convert to real point cloud
	ccPointCloud* cloud = 0;

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
		sampledCloud = 0;
	}
	else
	{
		ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
	}

	if (!cloud)
	{
		if (triIndices)
			triIndices->release();
		return 0;
	}

	if (withFeatures && triIndices && triIndices->currentSize() >= cloud->size())
	{
		//generate normals
		if (withNormals && hasNormals())
		{
			if (cloud->reserveTheNormsTable())
			{
				for (unsigned i=0; i<cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->getValue(i);
					const CCVector3* P = cloud->getPoint(i);

					CCVector3 N(0,0,1);
					interpolateNormals(triIndex,*P,N);
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
				for (unsigned i=0; i<cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->getValue(i);
					const CCVector3* P = cloud->getPoint(i);

					ccColor::Rgb C;
					getColorFromMaterial(triIndex,*P,C,withRGB);
					cloud->addRGBColor(C.rgb);
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
				for (unsigned i=0; i<cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->getValue(i);
					const CCVector3* P = cloud->getPoint(i);

					ccColor::Rgb C;
					interpolateColors(triIndex,*P,C);
					cloud->addRGBColor(C.rgb);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate colors (not enough memory?)");
			}
		}
	}

	//release memory
	if (triIndices)
	{
		triIndices->release();
		triIndices = 0;
	}

	//we rename the resulting cloud
	cloud->setName(getName()+QString(".sampled"));
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

bool ccGenericMesh::trianglePicking(const CCVector2d& clickPos,
									const ccGLCameraParameters& camera,
									int& nearestTriIndex,
									double& nearestSquareDist,
									CCVector3d& nearestPoint)
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

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

#if defined(_OPENMP)
	#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(size()); ++i)
	{
		CCLib::VerticesIndexes* tsi = getTriangleVertIndexes(i);
		const CCVector3* A3D = vertices->getPoint(tsi->i1);
		const CCVector3* B3D = vertices->getPoint(tsi->i2);
		const CCVector3* C3D = vertices->getPoint(tsi->i3);

		CCVector3d A2D,B2D,C2D; 
		if (noGLTrans)
		{
			camera.project(*A3D, A2D);
			camera.project(*B3D, B2D);
			camera.project(*C3D, C2D);
		}
		else
		{
			CCVector3 A3Dp = *A3D;
			CCVector3 B3Dp = *B3D;
			CCVector3 C3Dp = *C3D;
			trans.apply(A3Dp);
			trans.apply(B3Dp);
			trans.apply(C3Dp);
			camera.project(A3Dp, A2D);
			camera.project(B3Dp, B2D);
			camera.project(C3Dp, C2D);
		}

		//barycentric coordinates
		GLdouble detT =  (B2D.y-C2D.y) *      (A2D.x-C2D.x) + (C2D.x-B2D.x) *      (A2D.y-C2D.y);
		GLdouble l1   = ((B2D.y-C2D.y) * (clickPos.x-C2D.x) + (C2D.x-B2D.x) * (clickPos.y-C2D.y)) / detT;
		GLdouble l2   = ((C2D.y-A2D.y) * (clickPos.x-C2D.x) + (A2D.x-C2D.x) * (clickPos.y-C2D.y)) / detT;

		//does the point falls inside the triangle?
		if (l1 >= 0 && l1 <= 1.0 && l2 >= 0.0 && l2 <= 1.0)
		{
			double l1l2 = l1+l2;
			assert(l1l2 >= 0);
			if (l1l2 > 1.0)
			{
				l1 /= l1l2;
				l2 /= l1l2;
			}

			GLdouble l3 = 1.0 - l1 - l2;
			assert(l3 >= -1.0e-12);

			//now deduce the 3D position
			CCVector3d P(	l1 * A3D->x + l2 * B3D->x + l3 * C3D->x,
							l1 * A3D->y + l2 * B3D->y + l3 * C3D->y,
							l1 * A3D->z + l2 * B3D->z + l3 * C3D->z);
			double squareDist = (X-P).norm2d();
			if (nearestTriIndex < 0 || squareDist < nearestSquareDist)
			{
				nearestSquareDist = squareDist;
				nearestTriIndex = static_cast<int>(i);
				nearestPoint = P;
			}
		}
	}

	return (nearestTriIndex >= 0);
}