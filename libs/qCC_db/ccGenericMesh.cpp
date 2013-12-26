//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
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

//CCLib
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

void ccGenericMesh::EnableGLStippleMask(bool state)
{
	if (state)
	{
		glPolygonStipple(s_stippleMask);
		glEnable(GL_POLYGON_STIPPLE);
	}
	else
	{
		glDisable(GL_POLYGON_STIPPLE);
	}
}

//Vertex buffer
PointCoordinateType s_xyzBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
PointCoordinateType* ccGenericMesh::GetVertexBuffer()
{
	return s_xyzBuffer;
}

//Normals buffer
PointCoordinateType s_normBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
PointCoordinateType* ccGenericMesh::GetNormalsBuffer()
{
	return s_normBuffer;
}

//Colors buffer
colorType s_rgbBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
colorType* ccGenericMesh::GetColorsBuffer()
{
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

unsigned ccGenericMesh::GET_MAX_LOD_FACES_NUMBER()
{
	return 2500000;
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
				if (!vertices || !vertices->isA(CC_POINT_CLOUD))
					return;

				ccPointCloud* cloud = static_cast<ccPointCloud*>(vertices);

				//we just need to 'display' the current SF scale if the vertices cloud is hidden
				//(otherwise, it will be taken in charge by the cloud itself)
				if (!cloud->sfColorScaleShown() || (cloud->sfShown() && cloud->isEnabled() && cloud->isVisible()))
					return;

				//we must also check that the parent is not a mesh itself with the same vertices! (in
				//which case it will also take that in charge)
				ccHObject* parent = getParent();
				if (parent && parent->isKindOf(CC_MESH) && (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == vertices))
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

	//3D pass
	if (MACRO_Draw3D(context))
	{
		//any triangle?
		unsigned triNum = size();
		if (triNum == 0)
			return;

		//L.O.D.
		bool lodEnabled = (triNum > GET_MAX_LOD_FACES_NUMBER() && context.decimateMeshOnMove && MACRO_LODActivated(context));
		unsigned decimStep = (lodEnabled ? (unsigned)ceil((float)triNum*3 / (float)GET_MAX_LOD_FACES_NUMBER()) : 1);
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
		//special case: triangle names pushing (for picking)
		bool pushTriangleNames = MACRO_DrawTriangleNames(context);
		pushName |= pushTriangleNames;

		if (pushName)
		{
			//not fast at all!
			if (MACRO_DrawFastNamesOnly(context))
				return;
			glPushName(getUniqueIDForDisplay());
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
		unsigned colorRampSteps = 0;
		ccColorScale::Shared colorScale(0);

		if (glParams.showSF)
		{
			assert(vertices->isA(CC_POINT_CLOUD));
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
				colorRampSteps = currentDisplayedScalarField->getColorRampSteps();

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
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
		}

		//in the case we need to display vertex colors
		ColorsTableType* rgbColorsTable = 0;
		if (glParams.showColors)
		{
			if (isColorOverriden())
			{
				glColor3ubv(m_tempColor);
				glParams.showColors = false;
			}
			else
			{
				assert(vertices->isA(CC_POINT_CLOUD));
				rgbColorsTable = static_cast<ccPointCloud*>(vertices)->rgbColors();
			}
		}
		else
		{
			glColor3fv(context.defaultMat.diffuseFront);
		}

		if (glParams.showNorms)
		{
			//DGM: Strangely, when Qt::renderPixmap is called, the OpenGL version can fall to 1.0!
			glEnable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
			glEnable(GL_LIGHTING);
			context.defaultMat.applyGL(true,colorMaterial);
		}

		//in the case we need normals (i.e. lighting)
		NormsIndexesTableType* normalsIndexesTable = 0;
		ccNormalVectors* compressedNormals = 0;
		if (glParams.showNorms)
		{
			assert(vertices->isA(CC_POINT_CLOUD));
			normalsIndexesTable = static_cast<ccPointCloud*>(vertices)->normals();
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		//stipple mask
		if (stipplingEnabled())
			EnableGLStippleMask(true);

		if (!pushTriangleNames && !visFiltering && !(applyMaterials || showTextures) && (!glParams.showSF || greyForNanScalarValues))
		{
			//the GL type depends on the PointCoordinateType 'size' (float or double)
			GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;
			
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3,GL_COORD_TYPE,0,GetVertexBuffer());

			if (glParams.showNorms)
			{
				glEnableClientState(GL_NORMAL_ARRAY);
				glNormalPointer(GL_COORD_TYPE,0,GetNormalsBuffer());
			}
			if (glParams.showSF || glParams.showColors)
			{
				glEnableClientState(GL_COLOR_ARRAY);
				glColorPointer(3,GL_UNSIGNED_BYTE,0,GetColorsBuffer());
			}

			//we can scan and process each chunk separately in an optimized way
			//we mimic the way ccMesh beahves by using virtual chunks!
			unsigned chunks = static_cast<unsigned>(ceil((double)displayedTriNum/(double)MAX_NUMBER_OF_ELEMENTS_PER_CHUNK));
			unsigned chunkStart = 0;
			const colorType* col = 0;
			for (unsigned k=0; k<chunks; ++k, chunkStart += MAX_NUMBER_OF_ELEMENTS_PER_CHUNK)
			{
				//virtual chunk size
				const unsigned chunkSize = k+1 < chunks ? MAX_NUMBER_OF_ELEMENTS_PER_CHUNK : (displayedTriNum % MAX_NUMBER_OF_ELEMENTS_PER_CHUNK);

				//vertices
				PointCoordinateType* _vertices = GetVertexBuffer();
				for (unsigned n=0; n<chunkSize; n+=decimStep)
				{
					const CCLib::TriangleSummitsIndexes* ti = getTriangleIndexes(chunkStart + n);
					memcpy(_vertices,vertices->getPoint(ti->i1)->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
					memcpy(_vertices,vertices->getPoint(ti->i2)->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
					memcpy(_vertices,vertices->getPoint(ti->i3)->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
				}

				//scalar field
				if (glParams.showSF)
				{
					colorType* _rgbColors = GetColorsBuffer();
					assert(colorScale);
					for (unsigned n=0; n<chunkSize; n+=decimStep)
					{
						const CCLib::TriangleSummitsIndexes* ti = getTriangleIndexes(chunkStart + n);
						col = currentDisplayedScalarField->getValueColor(ti->i1);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(ti->i2);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(ti->i3);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
					}
				}
				//colors
				else if (glParams.showColors)
				{
					colorType* _rgbColors = GetColorsBuffer();

					for (unsigned n=0; n<chunkSize; n+=decimStep)
					{
						const CCLib::TriangleSummitsIndexes* ti = getTriangleIndexes(chunkStart + n);
						memcpy(_rgbColors,rgbColorsTable->getValue(ti->i1),sizeof(colorType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(ti->i2),sizeof(colorType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(ti->i3),sizeof(colorType)*3);
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
							memcpy(_normals,Na.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,Nb.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,Nc.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
					}
					else
					{
						for (unsigned n=0; n<chunkSize; n+=decimStep)
						{
							const CCLib::TriangleSummitsIndexes* ti = getTriangleIndexes(chunkStart + n);
							memcpy(_normals,vertices->getPointNormal(ti->i1),sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,vertices->getPointNormal(ti->i2),sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,vertices->getPointNormal(ti->i3),sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
					}
				}

				if (!showWired)
				{
					glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES,0,(chunkSize/decimStep)*3);
				}
				else
				{
					glDrawElements(GL_LINES,(chunkSize/decimStep)*6,GL_UNSIGNED_INT,GetWireVertexIndexes());
				}
			}

			//disable arrays
			glDisableClientState(GL_VERTEX_ARRAY);
			if (glParams.showNorms)
				glDisableClientState(GL_NORMAL_ARRAY);
			if (glParams.showSF || glParams.showColors)
				glDisableClientState(GL_COLOR_ARRAY);
		}
		else
		{
			//current vertex color
			const colorType *col1=0,*col2=0,*col3=0;
			//current vertex normal
			const PointCoordinateType *N1=0,*N2=0,*N3=0;
			//current vertex texture coordinates
			float *Tx1=0,*Tx2=0,*Tx3=0;

			//loop on all triangles
			int lasMtlIndex = -1;

			if (showTextures)
			{
				//#define TEST_TEXTURED_BUNDLER_IMPORT
#ifdef TEST_TEXTURED_BUNDLER_IMPORT
				glPushAttrib(GL_COLOR_BUFFER_BIT);
				glEnable(GL_BLEND);
				glBlendFunc(context.sourceBlend, context.destBlend);
#endif

				glEnable(GL_TEXTURE_2D);
			}

			if (pushTriangleNames)
				glPushName(0);

			GLenum triangleDisplayType = lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP : GL_TRIANGLES;
			glBegin(triangleDisplayType);

			//per-triangle normals
			const NormsIndexesTableType* triNormals = getTriNormsTable();
			//materials
			const ccMaterialSet* materials = getMaterialSet();

			for (unsigned n=0; n<triNum; ++n)
			{
				//current triangle vertices
				const CCLib::TriangleSummitsIndexes* tsi = getTriangleIndexes(n);

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
						int n1,n2,n3;
						getTriangleNormalIndexes(n,n1,n2,n3);
						N1 = (n1>=0 ? ccNormalVectors::GetNormal(triNormals->getValue(n1)) : 0);
						N2 = (n1==n2 ? N1 : n1>=0 ? ccNormalVectors::GetNormal(triNormals->getValue(n2)) : 0);
						N3 = (n1==n3 ? N1 : n3>=0 ? ccNormalVectors::GetNormal(triNormals->getValue(n3)) : 0);

					}
					else
					{
						N1 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i1));
						N2 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i2));
						N3 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi->i3));
					}
				}

				if (applyMaterials || showTextures)
				{
					assert(materials);
					int newMatlIndex = this->getTriangleMtlIndex(n);

					//do we need to change material?
					if (lasMtlIndex != newMatlIndex)
					{
						assert(newMatlIndex<(int)materials->size());
						glEnd();
						if (showTextures)
						{
							GLuint texID = (newMatlIndex>=0 ? (*materials)[newMatlIndex].texID : 0);
							if (texID>0)
								assert(glIsTexture(texID));
							glBindTexture(GL_TEXTURE_2D, texID);
						}

						//if we don't have any current material, we apply default one
						(newMatlIndex>=0 ? (*materials)[newMatlIndex] : context.defaultMat).applyGL(glParams.showNorms,false);
						glBegin(triangleDisplayType);
						lasMtlIndex=newMatlIndex;
					}

					if (showTextures)
					{
						getTriangleTexCoordinates(n,Tx1,Tx2,Tx3);
					}
				}

				if (pushTriangleNames)
				{
					glEnd();
					glLoadName(n);
					glBegin(triangleDisplayType);
				}
				else if (showWired)
				{
					glEnd();
					glBegin(triangleDisplayType);
				}

				//vertex 1
				if (N1)
					ccGL::Normal3v(N1);
				if (col1)
					glColor3ubv(col1);
				if (Tx1)
					glTexCoord2fv(Tx1);
				ccGL::Vertex3v(vertices->getPoint(tsi->i1)->u);

				//vertex 2
				if (N2)
					ccGL::Normal3v(N2);
				if (col2)
					glColor3ubv(col2);
				if (Tx2)
					glTexCoord2fv(Tx2);
				ccGL::Vertex3v(vertices->getPoint(tsi->i2)->u);

				//vertex 3
				if (N3)
					ccGL::Normal3v(N3);
				if (col3)
					glColor3ubv(col3);
				if (Tx3)
					glTexCoord2fv(Tx3);
				ccGL::Vertex3v(vertices->getPoint(tsi->i3)->u);
			}

			glEnd();

			if (pushTriangleNames)
				glPopName();

			if (showTextures)
			{
#ifdef TEST_TEXTURED_BUNDLER_IMPORT
				glPopAttrib(); //GL_COLOR_BUFFER_BIT 
#endif
				glBindTexture(GL_TEXTURE_2D, 0);
				glDisable(GL_TEXTURE_2D);
			}
		}

		if (stipplingEnabled())
			EnableGLStippleMask(false);

		if (colorMaterial)
			glDisable(GL_COLOR_MATERIAL);

		if (glParams.showNorms)
		{
			glDisable(GL_LIGHTING);
			glDisable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
		}

		if (pushName)
			glPopName();
	}
}

bool ccGenericMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//'show wired' state (dataVersion>=20)
	if (out.write((const char*)&m_showWired,sizeof(bool))<0)
		return WriteError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (out.write((const char*)&m_triNormsShown,sizeof(bool))<0)
		return WriteError();

	//'materials shown' state (dataVersion>=29))
	if (out.write((const char*)&m_materialsShown,sizeof(bool))<0)
		return WriteError();

	//'polygon stippling' state (dataVersion>=29))
	if (out.write((const char*)&m_stippling,sizeof(bool))<0)
		return WriteError();

	return true;
}

bool ccGenericMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//'show wired' state (dataVersion>=20)
	if (in.read((char*)&m_showWired,sizeof(bool))<0)
		return ReadError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (dataVersion >= 29)
	{
		if (in.read((char*)&m_triNormsShown,sizeof(bool))<0)
			return ReadError();

		//'materials shown' state (dataVersion>=29))
		if (in.read((char*)&m_materialsShown,sizeof(bool))<0)
			return ReadError();

		//'polygon stippling' state (dataVersion>=29))
		if (in.read((char*)&m_stippling,sizeof(bool))<0)
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
		cloud = ccPointCloud::From(sampledCloud);
		delete sampledCloud;
		sampledCloud = 0;
	}

	if (!cloud)
	{
		if (triIndices)
			triIndices->release();

		ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
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

					CCVector3 N(0.0,0.0,1.0);
					interpolateNormals(triIndex,*P,N);
					cloud->addNorm(N.u);
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

					colorType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
					getColorFromMaterial(triIndex,*P,C,withRGB);
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
				for (unsigned i=0; i<cloud->size(); ++i)
				{
					unsigned triIndex = triIndices->getValue(i);
					const CCVector3* P = cloud->getPoint(i);

					colorType C[3]={MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
					interpolateColors(triIndex,*P,C);
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
	cloud->setName(getName()+QString(".sampled"));
	cloud->setDisplay(getDisplay());
	cloud->prepareDisplayForRefresh();
	
	//copy 'shift on load' information
	if (getAssociatedCloud())
	{
		const CCVector3d& shift = getAssociatedCloud()->getGlobalShift();
		cloud->setGlobalShift(shift);
		double scale = getAssociatedCloud()->getGlobalScale();
		cloud->setGlobalScale(scale);
	}

	return cloud;
}
