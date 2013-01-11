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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2247                                                              $
//$LastChangedDate:: 2012-10-04 23:34:01 +0200 (jeu., 04 oct. 2012)        $
//**************************************************************************
//

#include "ccIncludeGL.h"

#include "ccMesh.h"

//Local
#include "ccGenericPointCloud.h"
#include "ccNormalVectors.h"
#include "ccPointCloud.h"
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <ReferenceCloud.h>

//Qt
#include <QGLFormat>

//System
#include <assert.h>

ccMesh::ccMesh(ccGenericPointCloud* vertices)
	: ccGenericMesh(vertices,"Mesh")
	, m_triIndexes(0)
	, m_globalIterator(0)
	, m_triMtlIndexes(0)
	, m_materialsShown(false)
	, m_texCoordIndexes(0)
	, m_triNormalIndexes(0)
	, m_triNormsShown(false)
	, m_stippling(false)
{
	m_triIndexes = new triangleIndexesContainer();
	m_triIndexes->link();
}

ccMesh::ccMesh(CCLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices)
	: ccGenericMesh(giVertices, "Mesh")
	, m_triIndexes(0)
	, m_globalIterator(0)
	, m_triMtlIndexes(0)
	, m_materialsShown(false)
	, m_texCoordIndexes(0)
	, m_triNormalIndexes(0)
	, m_triNormsShown(false)
	, m_stippling(false)
{
	m_triIndexes = new triangleIndexesContainer();
	m_triIndexes->link();

	unsigned i,triNum = giMesh->size();
	if (!reserve(triNum))
		return;

	giMesh->placeIteratorAtBegining();
	for (i=0;i<triNum;++i)
	{
		const CCLib::TriangleSummitsIndexes* tsi = giMesh->getNextTriangleIndexes();
		addTriangle(tsi->i1,tsi->i2,tsi->i3);
	}

	if (!giVertices->hasNormals())
		computeNormals();
	showNormals(true);

	if (giVertices->hasColors())
		showColors(giVertices->colorsShown());

	if (giVertices->hasDisplayedScalarField())
		showSF(giVertices->sfShown());
}

ccMesh::~ccMesh()
{
	if (m_triIndexes)
		m_triIndexes->release();
	if (m_texCoordIndexes)
		m_texCoordIndexes->release();
	if (m_triMtlIndexes)
		m_triMtlIndexes->release();
	if (m_triNormalIndexes)
		m_triNormalIndexes->release();
}

ccGenericMesh* ccMesh::clone(ccGenericPointCloud* vertices/*=0*/,
							 ccMaterialSet* clonedMaterials/*=0*/,
							 NormsIndexesTableType* clonedNormsTable/*=0*/,
							 TextureCoordsContainer* cloneTexCoords/*=0*/)
{
	assert(m_associatedCloud);

	//vertices
	unsigned i,vertNum = m_associatedCloud->size();
	//triangles
	unsigned triNum = size();

	//temporary structure to check that vertices are really used (in case of vertices set sharing)
	unsigned* usedVerts = 0;
	CCLib::TriangleSummitsIndexes* tsi=0;

	ccGenericPointCloud* newVertices = vertices;

	//no input vertices set
	if (!newVertices)
	{
		//let's check the real vertex count
		usedVerts = new unsigned[vertNum];
		if (!usedVerts)
		{
			ccLog::Error("[ccMesh::clone] Not enough memory!");
			return 0;
		}
		memset(usedVerts,0,sizeof(unsigned)*vertNum);

		placeIteratorAtBegining();
		for (i=0;i<triNum;++i)
		{
			tsi = getNextTriangleIndexes();
			usedVerts[tsi->i1]=1;
			usedVerts[tsi->i2]=1;
			usedVerts[tsi->i3]=1;
		}

		//we check that all points in 'associatedCloud' are used by this mesh
		unsigned realVertCount=0;
		for (i=0;i<vertNum;++i)
			usedVerts[i]=(usedVerts[i]==1 ? realVertCount++ : vertNum);

		//the associated cloud is already the exact vertices set --> nothing to change
		if (realVertCount == vertNum)
		{
			newVertices = m_associatedCloud->clone();
		}
		else
		{
			//we create a temporary entity with used vertices only
			CCLib::ReferenceCloud* rc = new CCLib::ReferenceCloud(m_associatedCloud);
			if (rc->reserve(realVertCount))
			{
				for (i=0;i<vertNum;++i)
					if (usedVerts[i]!=vertNum)
						rc->addPointIndex(i);

				//and the associated vertices set
				newVertices = new ccPointCloud(rc,m_associatedCloud);
				if (newVertices->size() < rc->size())
				{
					//not enough memory!
					delete newVertices;
					newVertices=0;
				}
			}

			delete rc;
			rc=0;
		}
	}

	//failed to create a new vertices set!
	if (!newVertices)
	{
		if (usedVerts)
			delete[] usedVerts;
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return 0;
	}

	//mesh clone
	ccMesh* cloneMesh = new ccMesh(newVertices);
	if (!cloneMesh->reserve(triNum))
	{
		if (!vertices)
			delete newVertices;
		if (usedVerts)
			delete[] usedVerts;
		delete cloneMesh;
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return 0;
	}

	//let's create the new triangles
	if (usedVerts) //in case we have an equivalence table
	{
		placeIteratorAtBegining();
		for (i=0;i<triNum;++i)
		{
			tsi = getNextTriangleIndexes();
			cloneMesh->addTriangle(usedVerts[tsi->i1],usedVerts[tsi->i2],usedVerts[tsi->i3]);
		}

		delete[] usedVerts;
		usedVerts=0;
	}
	else
	{
		placeIteratorAtBegining();
		for (i=0;i<triNum;++i)
		{
			tsi = getNextTriangleIndexes();
			cloneMesh->addTriangle(tsi->i1,tsi->i2,tsi->i3);
		}
	}

	//triangle normals
	if (m_triNormals && m_triNormalIndexes)
	{
		//1st: try to allocate per-triangle normals indexes
		if (cloneMesh->reservePerTriangleNormalIndexes())
		{
			//2nd: clone the main array if not already done
			if (!clonedNormsTable)
			{
				clonedNormsTable = m_triNormals->clone(); //TODO: keep only what's necessary!
				if (clonedNormsTable)
					cloneMesh->addChild(clonedNormsTable);
				else
				{
					ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone per-triangle normals!");
					cloneMesh->removePerTriangleNormalIndexes(); //don't need this anymore!
				}
			}

			//if we have both the main array and per-triangle normals indexes, we can finish the job
			if (cloneMesh)
			{
				cloneMesh->setTriNormsTable(clonedNormsTable);
				assert(cloneMesh->m_triNormalIndexes);
				m_triNormalIndexes->copy(*cloneMesh->m_triNormalIndexes); //should be ok as array is already reserved!
			}
		}
		else
		{
			ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone per-triangle normal indexes!");
		}
	}

	//materials
	if (m_materials && m_triMtlIndexes)
	{
		//1st: try to allocate per-triangle materials indexes
		if (cloneMesh->reservePerTriangleMtlIndexes())
		{
			//2nd: clone the main array if not already done
			if (!clonedMaterials)
			{
				clonedMaterials = getMaterialSet()->clone(); //TODO: keep only what's necessary!
				if (clonedMaterials)
				{
					cloneMesh->addChild(clonedMaterials);
				}
				else
				{
					ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone materials set!");
					cloneMesh->removePerTriangleMtlIndexes(); //don't need this anymore!
				}
			}

			//if we have both the main array and per-triangle materials indexes, we can finish the job
			if (clonedMaterials)
			{
				cloneMesh->setMaterialSet(clonedMaterials);
				assert(cloneMesh->m_triMtlIndexes);
				m_triMtlIndexes->copy(*cloneMesh->m_triMtlIndexes); //should be ok as array is already reserved!
			}
		}
		else
		{
			ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone per-triangle materials!");
		}
	}

	//texture coordinates
	if (m_texCoords && m_texCoordIndexes)
	{
		//1st: try to allocate per-triangle texture info
		if (cloneMesh->reservePerTriangleTexCoordIndexes())
		{
			//2nd: clone the main array if not already done
			if (!cloneTexCoords)
			{
				cloneTexCoords = m_texCoords->clone(); //TODO: keep only what's necessary!
				if (cloneTexCoords)
					cloneMesh->addChild(cloneTexCoords);
				else
				{
					ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone texture coordinates!");
					cloneMesh->removePerTriangleTexCoordIndexes(); //don't need this anymore!
				}
			}

			//if we have both the main array and per-triangle texture info, we can finish the job
			if (cloneTexCoords)
			{
				cloneMesh->setTexCoordinatesTable(cloneTexCoords);
				assert(cloneMesh->m_texCoordIndexes);
				m_texCoordIndexes->copy(*cloneMesh->m_texCoordIndexes); //should be ok as array is already reserved!
			}
		}
		else
		{
			ccLog::Warning("[ccMesh::clone] Not enough memory: failed to clone per-triangle texture info!");
		}
	}

	if (!vertices)
	{
		if (hasNormals() && !m_triNormals)
			cloneMesh->computeNormals();
		newVertices->setEnabled(false);
		//we link the mesh structure with the new vertex set
		cloneMesh->addChild(newVertices);
		cloneMesh->setDisplay_recursive(getDisplay());
	}

	//stippling
	cloneMesh->m_stippling = m_stippling;

	cloneMesh->showNormals(normalsShown());
	cloneMesh->showColors(colorsShown());
	cloneMesh->showSF(sfShown());
	cloneMesh->showMaterials(materialsShown());
	cloneMesh->setName(getName()+QString(".clone"));
	cloneMesh->setVisible(isVisible());
	cloneMesh->setEnabled(isEnabled());

	return cloneMesh;
}

unsigned ccMesh::size() const
{
	return m_triIndexes->currentSize();
}

unsigned ccMesh::maxSize() const
{
	return m_triIndexes->capacity();
}

//std methods
void ccMesh::forEach(genericTriangleAction& anAction)
{
	m_triIndexes->placeIteratorAtBegining();
	for (unsigned i=0;i<m_triIndexes->currentSize();++i)
	{
		const unsigned* tri = m_triIndexes->getCurrentValue();
		m_currentTriangle.A = m_associatedCloud->getPoint(tri[0]);
		m_currentTriangle.B = m_associatedCloud->getPoint(tri[1]);
		m_currentTriangle.C = m_associatedCloud->getPoint(tri[2]);
		anAction(m_currentTriangle);
		m_triIndexes->forwardIterator();
	}
}

void ccMesh::placeIteratorAtBegining()
{
	m_globalIterator = 0;
}

CCLib::GenericTriangle* ccMesh::_getNextTriangle()
{
	if (m_globalIterator<m_triIndexes->currentSize())
		return _getTriangle(m_globalIterator++);

	return NULL;
}

CCLib::GenericTriangle* ccMesh::_getTriangle(unsigned triangleIndex) //temporary
{
	assert(triangleIndex<m_triIndexes->currentSize());

	const unsigned* tri = m_triIndexes->getValue(triangleIndex);
	m_currentTriangle.A = m_associatedCloud->getPoint(tri[0]);
	m_currentTriangle.B = m_associatedCloud->getPoint(tri[1]);
	m_currentTriangle.C = m_associatedCloud->getPoint(tri[2]);

	return &m_currentTriangle;
}

void ccMesh::getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(triangleIndex<m_triIndexes->currentSize());

	const unsigned* tri = m_triIndexes->getValue(triangleIndex);
	m_associatedCloud->getPoint(tri[0],A);
	m_associatedCloud->getPoint(tri[1],B);
	m_associatedCloud->getPoint(tri[2],C);
}

void ccMesh::refreshBB()
{
	if (!m_associatedCloud)
		return;

	if (!m_bBox.isValid() || getLastModificationTime() < m_associatedCloud->getLastModificationTime_recursive())
	{
		m_bBox.clear();

		unsigned i,count=m_triIndexes->currentSize();
		m_triIndexes->placeIteratorAtBegining();
		for (i=0;i<count;++i)
		{
			const unsigned* tri = m_triIndexes->getCurrentValue();
			assert(tri[0]<m_associatedCloud->size() && tri[1]<m_associatedCloud->size() && tri[2]<m_associatedCloud->size());
			m_bBox.add(*m_associatedCloud->getPoint(tri[0]));
			m_bBox.add(*m_associatedCloud->getPoint(tri[1]));
			m_bBox.add(*m_associatedCloud->getPoint(tri[2]));
			m_triIndexes->forwardIterator();
		}
	}
}

void ccMesh::getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[])
{
	refreshBB();

	memcpy(Mins,m_bBox.minCorner().u,3*sizeof(PointCoordinateType));
	memcpy(Maxs,m_bBox.maxCorner().u,3*sizeof(PointCoordinateType));
}

ccBBox ccMesh::getMyOwnBB()
{
	refreshBB();

	return m_bBox;
}

//specific methods
void ccMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	CCLib::TriangleSummitsIndexes t(i1,i2,i3);
	m_triIndexes->addElement(t.i);
}

bool ccMesh::reserve(unsigned n)
{
	if (m_triNormalIndexes)
		if (!m_triNormalIndexes->reserve(n))
			return false;

	if (m_triMtlIndexes)
		if (!m_triMtlIndexes->reserve(n))
			return false;

	if (m_texCoordIndexes)
		if (!m_texCoordIndexes->reserve(n))
			return false;

	return m_triIndexes->reserve(n);
}

bool ccMesh::resize(unsigned n)
{
	m_bBox.setValidity(false);
	updateModificationTime();

	if (m_triMtlIndexes)
	{
		if (!m_triMtlIndexes->resize(n,true,-1))
			return false;
	}

	if (m_texCoordIndexes)
	{
		int defaultValues[3]={-1,-1,-1};
		if (!m_texCoordIndexes->resize(n,true,defaultValues))
			return false;
	}

	if (m_triNormalIndexes)
	{
		int defaultValues[3]={-1,-1,-1};
		if (!m_triNormalIndexes->resize(n,true,defaultValues))
			return false;
	}

	return m_triIndexes->resize(n);
}

CCLib::TriangleSummitsIndexes* ccMesh::getTriangleIndexes(unsigned triangleIndex)
{
	return (CCLib::TriangleSummitsIndexes*)m_triIndexes->getValue(triangleIndex);
}

const CCLib::TriangleSummitsIndexes* ccMesh::getTriangleIndexes(unsigned triangleIndex) const
{
	return (CCLib::TriangleSummitsIndexes*)m_triIndexes->getValue(triangleIndex);
}

CCLib::TriangleSummitsIndexes* ccMesh::getNextTriangleIndexes()
{
	if (m_globalIterator<m_triIndexes->currentSize())
		return getTriangleIndexes(m_globalIterator++);

	return NULL;
}

#define GL_SET_NORM(vertexIndex) (glNormal3fv(compressedNormals->getNormal(normalsIndexesTable->getValue(vertexIndex))))

//Vertex indexes for OpenGL "arrays" drawing
static PointCoordinateType s_xyzBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
static PointCoordinateType s_normBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];
static colorType s_rgbBuffer[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3*3];

//static unsigned s_vertIndexes[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3];
static unsigned s_vertWireIndexes[MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*6];
static bool s_vertIndexesInitialized = false;
static PointCoordinateType s_blankNorm[3]={0.0,0.0,0.0};

//stipple mask (for semi-transparent display of meshes)
static const GLubyte s_byte0 = 1 | 4 | 16 | 64;
static const GLubyte s_byte1 = 2 | 8 | 32 | 128;
//static const GLubyte s_byte0 = 1 | 16;
//static const GLubyte s_byte1 = 0;
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

void ccMesh::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_associatedCloud)
		return;

	//first, call parent method
	ccGenericMesh::drawMeOnly(context);

	//3D pass
	if (MACRO_Draw3D(context))
	{
		//any triangle?
		unsigned n,triNum=m_triIndexes->currentSize();
		if (triNum==0)
			return;

		//L.O.D.
		bool lodEnabled = (triNum > MAX_LOD_FACES_NUMBER && context.decimateMeshOnMove && MACRO_LODActivated(context));
		int decimStep = (lodEnabled ? (int)ceil((float)triNum*3 / (float)MAX_LOD_FACES_NUMBER) : 1);

		//GL name push
		bool pushName = MACRO_DrawNames(context);
		if (pushName)
			glPushName(getUniqueID());

		//vertices visibility
		const ccGenericPointCloud::VisibilityTableType* visibilityArray = m_associatedCloud->getTheVisibilityArray();
		bool visFiltering = (visibilityArray ? visibilityArray->isAllocated() : false);

		//wireframe ? (not compatible with LOD)
		bool showWired = m_showWired && !lodEnabled;

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());

		//materials & textures
		bool applyMaterials = (hasMaterials() && materialsShown());
		bool showTextures = (hasTextures() && materialsShown() && !lodEnabled);

		//display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		glParams.showNorms &= bool(MACRO_LightIsEnabled(context));

		//materials or color?
		bool colorMaterial = false;
		if (glParams.showSF || glParams.showColors)
		{
			applyMaterials = false;
			colorMaterial = true;
			glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
		}

		//in the case we need to display scalar field colors
		ccScalarField* currentDisplayedScalarField = 0;
		ccColorTablesManager* colorTable = 0;
		bool greyForNanScalarValues = true;
		unsigned colorRampSteps = 0;
		CC_COLOR_RAMPS colorRamp = BGYR;

		if (glParams.showSF)
		{
			assert(m_associatedCloud->isA(CC_POINT_CLOUD));
			ccPointCloud* cloud = static_cast<ccPointCloud*>(m_associatedCloud);
			greyForNanScalarValues = cloud->areNanScalarValuesInGrey();
			currentDisplayedScalarField = cloud->getCurrentDisplayedScalarField();
			colorRamp = currentDisplayedScalarField->getColorRamp();
			colorRampSteps = currentDisplayedScalarField->getColorRampSteps();
			colorTable = ccColorTablesManager::GetUniqueInstance();
		}

		//in the case we need to display vertex colors
		ColorsTableType* rgbColorsTable = 0;
		if (glParams.showColors)
		{
			if (isColorOverriden())
			{
				glColor3ubv(tempColor);
				glParams.showColors = false;
			}
			else
			{
				assert(m_associatedCloud->isA(CC_POINT_CLOUD));
				rgbColorsTable = static_cast<ccPointCloud*>(m_associatedCloud)->rgbColors();
			}
		}
		else
		{
			glColor3fv(context.defaultMat.diffuseFront);
		}

		if (glParams.showNorms/* || showTriNormals*/)
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
			assert(m_associatedCloud->isA(CC_POINT_CLOUD));
			normalsIndexesTable = static_cast<ccPointCloud*>(m_associatedCloud)->normals();
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		//stipple mask
		if (m_stippling)
		{
			glPolygonStipple(s_stippleMask);
			glEnable(GL_POLYGON_STIPPLE);
		}

		if (!visFiltering && !(applyMaterials || showTextures) && (!glParams.showSF || greyForNanScalarValues))
		{
			#define OPTIM_MEM_CPY //use optimized mem. transfers
			#ifdef OPTIM_MEM_CPY
			const unsigned step = 3*(decimStep-1);
			#else
			const unsigned step = 3*decimStep;
			#endif

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3,GL_FLOAT,0,s_xyzBuffer);

			if (/*showTriNormals || */glParams.showNorms)
			{
				glEnableClientState(GL_NORMAL_ARRAY);
				glNormalPointer(GL_FLOAT,0,s_normBuffer);
			}
			if (glParams.showSF || glParams.showColors)
			{
				glEnableClientState(GL_COLOR_ARRAY);
				glColorPointer(3,GL_UNSIGNED_BYTE,0,s_rgbBuffer);
			}

			//we can scan and process each chunk separately in an optimized way
			unsigned k,chunks = m_triIndexes->chunksCount();
			const PointCoordinateType* P=0;
			const PointCoordinateType* N=0;
			const colorType* col=0;
			for (k=0;k<chunks;++k)
			{
				const unsigned chunkSize = m_triIndexes->chunkSize(k);

				//vertices
				const unsigned* _vertIndexes = m_triIndexes->chunkStartPtr(k);
				PointCoordinateType* _vertices = s_xyzBuffer;
			#ifdef OPTIM_MEM_CPY
				for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
				{
				    P = m_associatedCloud->getPoint(*_vertIndexes++)->u;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
				    P = m_associatedCloud->getPoint(*_vertIndexes++)->u;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
				    P = m_associatedCloud->getPoint(*_vertIndexes++)->u;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
					*(_vertices)++ = *(P)++;
				}
			#else
				for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
				{
					memcpy(_vertices,m_associatedCloud->getPoint(_vertIndexes[0])->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
					memcpy(_vertices,m_associatedCloud->getPoint(_vertIndexes[1])->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
					memcpy(_vertices,m_associatedCloud->getPoint(_vertIndexes[2])->u,sizeof(PointCoordinateType)*3);
					_vertices+=3;
				}
			#endif

				//scalar field
				if (glParams.showSF)
				{
					colorType* _rgbColors = s_rgbBuffer;
					_vertIndexes = m_triIndexes->chunkStartPtr(k);
			#ifdef OPTIM_MEM_CPY
					for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
					{
						DistanceType normalizedDist = currentDisplayedScalarField->getNormalizedValue(*_vertIndexes++);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;

						normalizedDist = currentDisplayedScalarField->getNormalizedValue(*_vertIndexes++);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;

						normalizedDist = currentDisplayedScalarField->getNormalizedValue(*_vertIndexes++);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
					}
			#else
					for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
					{
						DistanceType normalizedDist = currentDisplayedScalarField->getNormalizedValue(_vertIndexes[0]);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
						normalizedDist = currentDisplayedScalarField->getNormalizedValue(_vertIndexes[1]);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
						normalizedDist = currentDisplayedScalarField->getNormalizedValue(_vertIndexes[2]);
						col = (normalizedDist>=0.0 ? colorTable->getColor(normalizedDist,colorRampSteps,colorRamp) : ccColor::lightGrey);
						memcpy(_rgbColors,col,sizeof(colorType)*3);
						_rgbColors += 3;
					}
            #endif
				}
				//colors
				else if (glParams.showColors)
				{
					colorType* _rgbColors = s_rgbBuffer;
					_vertIndexes = m_triIndexes->chunkStartPtr(k);
			#ifdef OPTIM_MEM_CPY
					for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
					{
					    col = rgbColorsTable->getValue(*_vertIndexes++);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;

					    col = rgbColorsTable->getValue(*_vertIndexes++);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;

					    col = rgbColorsTable->getValue(*_vertIndexes++);
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
                        *(_rgbColors)++ = *(col)++;
					}
			#else
					for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
					{
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[0]),sizeof(colorType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[1]),sizeof(colorType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[2]),sizeof(colorType)*3);
						_rgbColors += 3;
					}
            #endif
				}

				//normals
				if (glParams.showNorms)
				{
					PointCoordinateType* _normals = s_normBuffer;
					if (showTriNormals)
					{
						assert(m_triNormalIndexes);
						int* _triNormalIndexes = m_triNormalIndexes->chunkStartPtr(k);
			#ifdef OPTIM_MEM_CPY
						for (n=0;n<chunkSize;n+=decimStep,_triNormalIndexes+=step)
						{
							assert(*_triNormalIndexes<(int)m_triNormals->currentSize());
							N = (*_triNormalIndexes>=0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;

							assert(*_triNormalIndexes<(int)m_triNormals->currentSize());
							N = (*_triNormalIndexes>=0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;

							assert(*_triNormalIndexes<(int)m_triNormals->currentSize());
							N = (*_triNormalIndexes>=0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
						}
            #else
						for (n=0;n<chunkSize;n+=decimStep,_triNormalIndexes+=step)
						{
							assert(_triNormalIndexes[0]<(int)m_triNormals->currentSize());
							N = (_triNormalIndexes[0]>=0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[0])) : s_blankNorm);
							memcpy(_normals,N,sizeof(PointCoordinateType)*3);
							_normals+=3;
							assert(_triNormalIndexes[1]<(int)m_triNormals->currentSize());
							N = (_triNormalIndexes[0]==_triNormalIndexes[1] ? N : _triNormalIndexes[1]>=0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[1])) : s_blankNorm);
							memcpy(_normals,N,sizeof(PointCoordinateType)*3);
							_normals+=3;
							assert(_triNormalIndexes[2]<(int)m_triNormals->currentSize());
							N = (_triNormalIndexes[0]==_triNormalIndexes[2] ? N : _triNormalIndexes[2]>=0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[2])) : s_blankNorm);
							memcpy(_normals,N,sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
			#endif
					}
					else
					{
						_vertIndexes = m_triIndexes->chunkStartPtr(k);
			#ifdef OPTIM_MEM_CPY
						for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
						{
						    N = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;

						    N = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;

						    N = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
							*(_normals)++ = *(N)++;
						}
			#else
						for (n=0;n<chunkSize;n+=decimStep,_vertIndexes+=step)
						{
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[0])),sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[1])),sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[2])),sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
			#endif
					}
				}

				if (!showWired)
				{
                    glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES,0,(chunkSize/decimStep)*3);
					//glDrawElements(lodEnabled ? GL_POINTS : GL_TRIANGLES,(chunkSize/decimStep)*3,GL_UNSIGNED_INT,s_vertIndexes);
				}
                else
                {
                    //on first display of a wired mesh, we need to init the corresponding vertex indexes array!
                    if (!s_vertIndexesInitialized)
                    {
                        unsigned* _vertWireIndexes = s_vertWireIndexes;
                        for (unsigned i=0;i<MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*3;++i)
                        {
                            //s_vertIndexes[i]=i;
                            *_vertWireIndexes++=i;
                            *_vertWireIndexes++=((i+1)%3 == 0 ? i-2 : i+1);
                        }
                        s_vertIndexesInitialized=true;
                    }
                    glDrawElements(GL_LINES,(chunkSize/decimStep)*6,GL_UNSIGNED_INT,s_vertWireIndexes);
                }
			}

			//disable arrays
			glDisableClientState(GL_VERTEX_ARRAY);
			if (/*showTriNormals || */glParams.showNorms)
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
			const float *Tx1=0,*Tx2=0,*Tx3=0;

			//loop on all triangles
			m_triIndexes->placeIteratorAtBegining();

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

			glBegin(lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP : GL_TRIANGLES);

			for (n=0;n<triNum;++n)
			{
				//current triangle vertices
				const CCLib::TriangleSummitsIndexes* tsi = (CCLib::TriangleSummitsIndexes*)m_triIndexes->getCurrentValue();
				m_triIndexes->forwardIterator();

				//LOD: shall we display this triangle?
				if (n % decimStep)
					continue;

				if (visFiltering)
				{
					if ((visibilityArray->getValue(tsi->i1) == 0) ||
						(visibilityArray->getValue(tsi->i2) == 0) ||
						(visibilityArray->getValue(tsi->i3) == 0))
						continue;
				}

				if (glParams.showSF)
				{
					DistanceType normalizedDist = currentDisplayedScalarField->getNormalizedValue(tsi->i1);
					if (normalizedDist>=0.0)
						col1 = colorTable->getColor(normalizedDist,colorRampSteps,colorRamp);
					else
					{
						if (greyForNanScalarValues)
							col1 = ccColor::lightGrey;
						else
							continue;
					}

					normalizedDist = currentDisplayedScalarField->getNormalizedValue(tsi->i2);
					if (normalizedDist>=0.0)
						col2 = colorTable->getColor(normalizedDist,colorRampSteps,colorRamp);
					else
					{
						if (greyForNanScalarValues)
							col2 = ccColor::lightGrey;
						else
							continue;
					}

					normalizedDist = currentDisplayedScalarField->getNormalizedValue(tsi->i3);
					if (normalizedDist>=0.0)
						col3 = colorTable->getColor(normalizedDist,colorRampSteps,colorRamp);
					else
					{
						if (greyForNanScalarValues)
							col3 = ccColor::lightGrey;
						else
							continue;
					}
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
						assert(m_triNormalIndexes);
						const int* idx = m_triNormalIndexes->getValue(n);
						assert(idx[0]<(int)m_triNormals->currentSize());
						assert(idx[1]<(int)m_triNormals->currentSize());
						assert(idx[2]<(int)m_triNormals->currentSize());
						N1 = (idx[0]>=0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[0])) : 0);
						N2 = (idx[0]==idx[1] ? N1 : idx[1]>=0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[1])) : 0);
						N3 = (idx[0]==idx[2] ? N1 : idx[2]>=0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[2])) : 0);

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
					assert(m_materials);
					int newMatlIndex = m_triMtlIndexes->getValue(n);

					//do we need to change material?
					if (lasMtlIndex != newMatlIndex)
					{
						assert(newMatlIndex<(int)m_materials->size());
						glEnd();
						if (showTextures)
						{
							GLuint texID = (newMatlIndex>=0 ? (*m_materials)[newMatlIndex].texID : 0);
							if (texID>0)
								assert(glIsTexture(texID));
							glBindTexture(GL_TEXTURE_2D, texID);
						}

						//if we don't have any current material, we apply default one
						(newMatlIndex>=0 ? (*m_materials)[newMatlIndex] : context.defaultMat).applyGL(glParams.showNorms/* || showTriNormals*/,false);
						glBegin(GL_TRIANGLES);
						lasMtlIndex=newMatlIndex;
					}

					if (showTextures)
					{
						assert(m_texCoords && m_texCoordIndexes);
						const int* txInd = m_texCoordIndexes->getValue(n);
						assert(txInd[0]<(int)m_texCoords->currentSize());
						assert(txInd[1]<(int)m_texCoords->currentSize());
						assert(txInd[2]<(int)m_texCoords->currentSize());
						Tx1 = (txInd[0]>=0 ? m_texCoords->getValue(txInd[0]) : 0);
						Tx2 = (txInd[1]>=0 ? m_texCoords->getValue(txInd[1]) : 0);
						Tx3 = (txInd[2]>=0 ? m_texCoords->getValue(txInd[2]) : 0);
					}
				}

				//vertex 1
				if (N1)
					glNormal3fv(N1);
				if (col1)
					glColor3ubv(col1);
				if (Tx1)
					glTexCoord2fv(Tx1);
				glVertex3fv(m_associatedCloud->getPoint(tsi->i1)->u);

				//vertex 2
				if (N2)
					glNormal3fv(N2);
				if (col2)
					glColor3ubv(col2);
				if (Tx2)
					glTexCoord2fv(Tx2);
				glVertex3fv(m_associatedCloud->getPoint(tsi->i2)->u);

				//vertex 3
				if (N3)
					glNormal3fv(N3);
				if (col3)
					glColor3ubv(col3);
				if (Tx3)
					glTexCoord2fv(Tx3);
				glVertex3fv(m_associatedCloud->getPoint(tsi->i3)->u);

				if (showWired)
				{
					glEnd();
					glBegin(GL_LINE_LOOP);
				}
			}
			glEnd();

			if (showTextures)
			{
#ifdef TEST_TEXTURED_BUNDLER_IMPORT
				glPopAttrib(); //GL_COLOR_BUFFER_BIT 
#endif
				glBindTexture(GL_TEXTURE_2D, 0);
				glDisable(GL_TEXTURE_2D);
			}
		}

		if (m_stippling)
			glDisable(GL_POLYGON_STIPPLE);

		if (colorMaterial)
			glDisable(GL_COLOR_MATERIAL);

		if (glParams.showNorms/* || showTriNormals*/)
		{
			glDisable(GL_LIGHTING);
			glDisable((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_1_2 ? GL_RESCALE_NORMAL : GL_NORMALIZE));
		}

		if (pushName)
			glPopName();
	}
}

ccGenericMesh* ccMesh::createNewMeshFromSelection(bool removeSelectedVertices, CCLib::ReferenceCloud* selection/*=NULL*/, ccGenericPointCloud* vertices/*=NULL*/)
{
	assert(m_associatedCloud);

	ccGenericPointCloud::VisibilityTableType* visibilityArray = m_associatedCloud->getTheVisibilityArray();
	if (!visibilityArray || !visibilityArray->isAllocated())
	{
		//ccConsole::Error(QString("[Mesh %1] Visibility table not instantiated!").arg(getName()));
		return NULL;
	}

	ccMesh* newTri = NULL;

	/*ccHObject* parent = getParent();
	if (parent!=NULL && parent->isKindOf(CC_MESH) && static_cast<ccGenericMesh*>(parent)->getAssociatedCloud()==m_associatedCloud)
	{
	newTri = new ccMesh(m_associatedCloud);

	unsigned i,maxTri=0,currentTri=0,triNum=m_triIndexes->currentSize();
	CCLib::TriangleSummitsIndexes* tsi = NULL;
	m_triIndexes->placeIteratorAtBegining();
	for (i=0;i<triNum;++i)
	{
	tsi = (CCLib::TriangleSummitsIndexes*)m_triIndexes->getCurrentValue();
	m_triIndexes->forwardIterator();

	if (bool(visibilityArray->getValue(tsi->i1)>0) &&
	bool(visibilityArray->getValue(tsi->i2)>0) &&
	bool(visibilityArray->getValue(tsi->i3)>0))
	{
	if (currentTri==maxTri)
	{
	maxTri+=1000;
	if (!newTri->reserve(maxTri))
	{
	delete newTri;
	//ccConsole::Error("Not enough memory");
	return NULL;
	}
	}

	newTri->addTriangle(tsi->i1,tsi->i2,tsi->i3);
	++currentTri;
	}
	}

	if (currentTri==0)
	{
	delete newTri;
	newTri = NULL;
	}
	else
	{
	newTri->resize(currentTri);
	newTri->showColors(colorsShown());
	newTri->showNormals(normalsShown());
	newTri->showSF(sfShown());
	}
	}
	else*/
	{
		ccGenericPointCloud* newVertices = NULL;

		if (vertices)
			newVertices = vertices;
		else
		{
			newVertices = m_associatedCloud->createNewCloudFromVisibilitySelection(false);
			if (!newVertices)
				return NULL;
		}
		assert(newVertices);

		CCLib::ReferenceCloud* rc = 0;
		if (selection)
		{
			assert(selection->getAssociatedCloud()==static_cast<CCLib::GenericIndexedCloud*>(m_associatedCloud));
			rc = selection;
		}
		else
		{
			//we create a temporary entity with the invisible vertices only
			rc = new CCLib::ReferenceCloud(m_associatedCloud);

			for (unsigned i=0;i<m_associatedCloud->size();++i)
				if (visibilityArray->getValue(i))
					rc->addPointIndex(i);
		}

		//we create a new mesh with
		CCLib::GenericIndexedMesh* result = CCLib::ManualSegmentationTools::segmentMesh(this,rc,true,NULL,newVertices);
		if (!selection)
		{
			delete rc;
			rc=0;
		}

		if (result)
		{
			newTri = new ccMesh(result,newVertices);
			if (!newTri)
			{
				if (!vertices)
					delete newVertices;
				newVertices = NULL;
				//ccConsole::Error("An error occured: not enough memory ?");
			}
			else
			{
				newTri->setName(getName()+QString(".part"));

				//shall we add any advanced features?
				bool addFeatures = false;
				if (m_triNormalIndexes)
					addFeatures |= newTri->reservePerTriangleNormalIndexes();
				if (m_triMtlIndexes)
					addFeatures |= newTri->reservePerTriangleMtlIndexes();
				if (m_texCoordIndexes)
					addFeatures |= newTri->reservePerTriangleTexCoordIndexes();

				if (addFeatures)
				{
					if (m_triNormals)
					{
						newTri->setTriNormsTable(m_triNormals);
						//DGM FIXME
						//newTri->addChild(m_triNormals,true);
					}
					if (m_materials)
					{
						newTri->setMaterialSet(m_materials);
						//DGM FIXME
						//newTri->addChild(m_materials,true);
					}
					if (m_texCoords)
					{
						newTri->setTexCoordinatesTable(m_texCoords);
						//DGM FIXME
						//newTri->addChild(m_texCoords,true);
					}

					unsigned triNum=m_triIndexes->currentSize();
					m_triIndexes->placeIteratorAtBegining();
					for (unsigned i=0;i<triNum;++i)
					{
						const CCLib::TriangleSummitsIndexes* tsi = (CCLib::TriangleSummitsIndexes*)m_triIndexes->getCurrentValue();
						m_triIndexes->forwardIterator();

						if (bool(visibilityArray->getValue(tsi->i1)>0) &&
							bool(visibilityArray->getValue(tsi->i2)>0) &&
							bool(visibilityArray->getValue(tsi->i3)>0))
						{
							if (m_triNormalIndexes)
							{
								const int* inds = m_triNormalIndexes->getValue(i);
								newTri->addTriangleNormalIndexes(inds[0],inds[1],inds[2]);
							}
							if (m_triMtlIndexes)
							{
								newTri->addTriangleMtlIndex(m_triMtlIndexes->getValue(i));
							}
							if (m_texCoordIndexes)
							{
								const int* inds = m_texCoordIndexes->getValue(i);
								newTri->addTriangleTexCoordIndexes(inds[0],inds[1],inds[2]);
							}
						}
					}
				}

				if (!vertices)
				{
					newTri->addChild(newVertices);
					newTri->setDisplay_recursive(getDisplay());
					newTri->showColors(colorsShown());
					newTri->showNormals(normalsShown());
					newTri->showMaterials(materialsShown());
					newTri->showSF(sfShown());
					newVertices->setEnabled(false);
				}
			}

			delete result;
			result=0;
		}
	}

	//shall we remove the selected vertices from this mesh ?
	if (removeSelectedVertices)
	{
		//we remove all visible points
		unsigned lastTri=0,triNum=m_triIndexes->currentSize();
		m_triIndexes->placeIteratorAtBegining();
		for (unsigned i=0;i<triNum;++i)
		{
			const CCLib::TriangleSummitsIndexes* tsi = (CCLib::TriangleSummitsIndexes*)m_triIndexes->getCurrentValue();
			m_triIndexes->forwardIterator();

			if (bool(visibilityArray->getValue(tsi->i1)==0) ||
				bool(visibilityArray->getValue(tsi->i2)==0) ||
				bool(visibilityArray->getValue(tsi->i3)==0))
			{
				if (i != lastTri)
				{
					m_triIndexes->setValue(lastTri, (unsigned*)tsi);

					if (m_triNormalIndexes)
						m_triNormalIndexes->setValue(lastTri, m_triNormalIndexes->getValue(i));
					if (m_triMtlIndexes)
						m_triMtlIndexes->setValue(lastTri, m_triMtlIndexes->getValue(i));
					if (m_texCoordIndexes)
						m_texCoordIndexes->setValue(lastTri, m_texCoordIndexes->getValue(i));
				}
				++lastTri;
			}
		}

		resize(lastTri);
		updateModificationTime();
	}

	return newTri;
}

void ccMesh::shiftTriangleIndexes(unsigned shift)
{
	m_triIndexes->placeIteratorAtBegining();
	unsigned *ti,i=0;
	for (;i<m_triIndexes->currentSize();++i)
	{
		ti = m_triIndexes->getCurrentValue();
		ti[0]+=shift;
		ti[1]+=shift;
		ti[2]+=shift;
		m_triIndexes->forwardIterator();
	}
}

/*********************************************************/
/**************    PER-TRIANGLE NORMALS    ***************/
/*********************************************************/

bool ccMesh::arePerTriangleNormalsEnabled() const
{
	return m_triNormalIndexes && m_triNormalIndexes->isAllocated();
}

void ccMesh::removePerTriangleNormalIndexes()
{
	if (m_triNormalIndexes)
		m_triNormalIndexes->release();
	m_triNormalIndexes=0;
}

void ccMesh::setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable/*=true*/)
{
	ccGenericMesh::setTriNormsTable(triNormsTable,autoReleaseOldTable);

	if (!m_triNormals)
		removePerTriangleNormalIndexes();
}

bool ccMesh::reservePerTriangleNormalIndexes()
{
	assert(!m_triNormalIndexes); //try to avoid doing this twice!
	if (!m_triNormalIndexes)
	{
		m_triNormalIndexes = new triangleNormalsIndexesSet();
		m_triNormalIndexes->link();
	}

	assert(m_triIndexes && m_triIndexes->isAllocated());

	return m_triNormalIndexes->reserve(m_triIndexes->capacity());
}

void ccMesh::addTriangleNormalIndexes(int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	int indexes[3]={i1,i2,i3};
	m_triNormalIndexes->addElement(indexes);
}

void ccMesh::setTriangleNormalIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->currentSize() > triangleIndex);
	int indexes[3]={i1,i2,i3};
	m_triNormalIndexes->setValue(triangleIndex,indexes);
}

bool ccMesh::hasTriNormals() const
{
	return m_triNormals && m_triNormals->isAllocated() && m_triNormalIndexes && (m_triNormalIndexes->currentSize() == m_triIndexes->currentSize());
}

/*********************************************************/
/************    PER-TRIANGLE TEX COORDS    **************/
/*********************************************************/

void ccMesh::setTexCoordinatesTable(TextureCoordsContainer* texCoordsTable, bool autoReleaseOldTable/*=true*/)
{
	ccGenericMesh::setTexCoordinatesTable(texCoordsTable,autoReleaseOldTable);

	if (!m_texCoords)
		removePerTriangleTexCoordIndexes(); //auto-remove per-triangle indexes
}

bool ccMesh::reservePerTriangleTexCoordIndexes()
{
	assert(!m_texCoordIndexes); //try to avoid doing this twice!
	if (!m_texCoordIndexes)
	{
		m_texCoordIndexes = new triangleTexCoordIndexesSet();
		m_texCoordIndexes->link();
	}

	assert(m_triIndexes && m_triIndexes->isAllocated());

	return m_texCoordIndexes->reserve(m_triIndexes->capacity());
}

void ccMesh::removePerTriangleTexCoordIndexes()
{
	triangleTexCoordIndexesSet* texCoordIndexes = m_texCoordIndexes;
	m_texCoordIndexes = 0;

	if (texCoordIndexes)
		texCoordIndexes->release();
}

void ccMesh::addTriangleTexCoordIndexes(int i1, int i2, int i3)
{
	assert(m_texCoordIndexes && m_texCoordIndexes->isAllocated());
	int indexes[3]={i1,i2,i3};
	m_texCoordIndexes->addElement(indexes);
}

void ccMesh::setTriangleTexCoordIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_texCoordIndexes && m_texCoordIndexes->currentSize() > triangleIndex);
	int indexes[3]={i1,i2,i3};
	m_texCoordIndexes->setValue(triangleIndex,indexes);
}

bool ccMesh::hasTextures() const
{
	return hasMaterials() && m_texCoords && m_texCoords->isAllocated() && m_texCoordIndexes && (m_texCoordIndexes->currentSize() == m_triIndexes->currentSize());
}

/*********************************************************/
/**************    PER-TRIANGLE MATERIALS    *************/
/*********************************************************/

bool ccMesh::hasMaterials() const
{
	return m_materials && !m_materials->empty() && m_triMtlIndexes && (m_triMtlIndexes->currentSize() == m_triIndexes->currentSize());
}

bool ccMesh::reservePerTriangleMtlIndexes()
{
	assert(!m_triMtlIndexes); //try to avoid doing this twice!
	if (!m_triMtlIndexes)
	{
		m_triMtlIndexes = new triangleMaterialIndexesSet();
		m_triMtlIndexes->link();
	}

	assert(m_triIndexes && m_triIndexes->isAllocated());

	return m_triMtlIndexes->reserve(m_triIndexes->capacity());
}

void ccMesh::removePerTriangleMtlIndexes()
{
	if (m_triMtlIndexes)
		m_triMtlIndexes->release();
	m_triMtlIndexes=0;
}

void ccMesh::addTriangleMtlIndex(int mtlIndex)
{
	assert(m_triMtlIndexes && m_triMtlIndexes->isAllocated());
	m_triMtlIndexes->addElement(mtlIndex);
}

void ccMesh::setTriangleMtlIndex(unsigned triangleIndex, int mtlIndex)
{
	assert(m_triMtlIndexes && m_triMtlIndexes->currentSize() > triangleIndex);
	m_triMtlIndexes->setValue(triangleIndex,mtlIndex);
}

void ccMesh::setDisplay(ccGenericGLDisplay* win)
{
	if (m_materials && !m_materials->empty())
	{
		const ccGenericGLDisplay* currentDisplay = m_materials->getAssociatedDisplay();
		//if the material set is not associated to any display --> we associate it with input display!
		if (currentDisplay==0)
			m_materials->associateTo(win);
		/*else //else if it is associated with a different display
		{
		//we clone the material set
		ccMaterialSet* newMtlSet = new ccMaterialSet();
		newMtlSet->link();
		m_materials->clone(*newMtlSet);
		//and we link the clone to the new display
		newMtlSet->associateTo(win);
		setMaterialSet(newMtlSet);
		newMtlSet->release();
		}
		//*/
	}

	ccDrawableObject::setDisplay(win);
}

bool ccMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericMesh::toFile_MeOnly(out))
		return false;

	//triangles indexes (dataVersion>=20)
	if (!m_triIndexes)
		return ccLog::Error("Internal error: mesh has no triangles array! (not enough memory?)");
	if (!ccSerializationHelper::GenericArrayToFile(*m_triIndexes,out))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = (m_triMtlIndexes && m_triMtlIndexes->isAllocated());
	if (out.write((const char*)&hasTriMtlIndexes,sizeof(bool))<0)
		return WriteError();
	if (hasTriMtlIndexes)
	{
		assert(m_triMtlIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_triMtlIndexes,out))
			return false;
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = (m_texCoordIndexes && m_texCoordIndexes->isAllocated());
	if (out.write((const char*)&hasTexCoordIndexes,sizeof(bool))<0)
		return WriteError();
	if (hasTexCoordIndexes)
	{
		assert(m_texCoordIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_texCoordIndexes,out))
			return false;
	}

	//'materials shown' state (dataVersion>=20))
	if (out.write((const char*)&m_materialsShown,sizeof(bool))<0)
		return WriteError();

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = (m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	if (out.write((const char*)&hasTriNormalIndexes,sizeof(bool))<0)
		return WriteError();
	if (hasTriNormalIndexes)
	{
		assert(m_triNormalIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_triNormalIndexes,out))
			return false;
	}

	//'per-triangle normals shown' state (dataVersion>=20))
	if (out.write((const char*)&m_triNormsShown,sizeof(bool))<0)
		return WriteError();

	//'polygon stippling' state (dataVersion>=20))
	if (out.write((const char*)&m_stippling,sizeof(bool))<0)
		return WriteError();

	return true;
}

bool ccMesh::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericMesh::fromFile_MeOnly(in, dataVersion))
		return false;

	//triangles indexes (dataVersion>=20)
	if (!m_triIndexes)
		return false;
	if (!ccSerializationHelper::GenericArrayFromFile(*m_triIndexes,in,dataVersion))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = false;
	if (in.read((char*)&hasTriMtlIndexes,sizeof(bool))<0)
		return ReadError();
	if (hasTriMtlIndexes)
	{
		if (!m_triMtlIndexes)
		{
			m_triMtlIndexes = new triangleMaterialIndexesSet();
			m_triMtlIndexes->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile(*m_triMtlIndexes,in,dataVersion))
		{
			m_triMtlIndexes->release();
			m_triMtlIndexes=0;
			return false;
		}
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = false;
	if (in.read((char*)&hasTexCoordIndexes,sizeof(bool))<0)
		return ReadError();
	if (hasTexCoordIndexes)
	{
		if (!m_texCoordIndexes)
		{
			m_texCoordIndexes = new triangleTexCoordIndexesSet();
			m_texCoordIndexes->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile(*m_texCoordIndexes,in,dataVersion))
		{
			m_texCoordIndexes->release();
			m_texCoordIndexes=0;
			return false;
		}
	}

	//'materials shown' state (dataVersion>=20))
	if (in.read((char*)&m_materialsShown,sizeof(bool))<0)
		return ReadError();

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = false;
	if (in.read((char*)&hasTriNormalIndexes,sizeof(bool))<0)
		return ReadError();
	if (hasTriNormalIndexes)
	{
		if (!m_triNormalIndexes)
		{
			m_triNormalIndexes = new triangleNormalsIndexesSet();
			m_triNormalIndexes->link();
		}
		assert(m_triNormalIndexes);
		if (!ccSerializationHelper::GenericArrayFromFile(*m_triNormalIndexes,in,dataVersion))
		{
			removePerTriangleNormalIndexes();
			return false;
		}
	}

	//'per-triangle normals shown' state (dataVersion>=20))
	if (in.read((char*)&m_triNormsShown,sizeof(bool))<0)
		return ReadError();

	//'polygon stippling' state (dataVersion>=20))
	if (in.read((char*)&m_stippling,sizeof(bool))<0)
		return ReadError();

	updateModificationTime();

	return true;
}

bool ccMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	assert(triIndex<size());

	if (!hasNormals())
		return false;

	//intepolation weights
	const unsigned* tri = m_triIndexes->getValue(triIndex);
	const CCVector3 *A = m_associatedCloud->getPointPersistentPtr(tri[0]);
	const CCVector3 *B = m_associatedCloud->getPointPersistentPtr(tri[1]);
	const CCVector3 *C = m_associatedCloud->getPointPersistentPtr(tri[2]);

	PointCoordinateType d1 = ((P-*B).cross(*C-*B)).norm()/*/2.0*/;
	PointCoordinateType d2 = ((P-*C).cross(*A-*C)).norm()/*/2.0*/;
	PointCoordinateType d3 = ((P-*A).cross(*B-*A)).norm()/*/2.0*/;

	CCVector3 N1,N2,N3;
	if (hasTriNormals()) //per-triangle normals
	{
		const int* normIndexes = m_triNormalIndexes->getValue(triIndex);
		assert(normIndexes);
		if (normIndexes[0]>=0)
			N1 = ccNormalVectors::GetNormal(m_triNormals->getValue(normIndexes[0]));
		else
			d1 = 0;
		if (normIndexes[1]>=0)
			N2 = ccNormalVectors::GetNormal(m_triNormals->getValue(normIndexes[1]));
		else
			d2 = 0;
		if (normIndexes[2]>=0)
			N3 = ccNormalVectors::GetNormal(m_triNormals->getValue(normIndexes[2]));
		else
			d3 = 0;
	}
	else //per-vertex normals
	{
		N1 = CCVector3(m_associatedCloud->getPointNormal(tri[0]));
		N2 = CCVector3(m_associatedCloud->getPointNormal(tri[1]));
		N3 = CCVector3(m_associatedCloud->getPointNormal(tri[2]));
	}

	N = N1*d1+N2*d2+N3*d3;

	N.normalize();

	return true;
}

bool ccMesh::interpolateColors(unsigned triIndex, const CCVector3& P, colorType rgb[])
{
	assert(triIndex<size());

	if (!hasColors())
		return false;

	//intepolation weights
	const unsigned* tri = m_triIndexes->getValue(triIndex);
	const CCVector3 *A = m_associatedCloud->getPointPersistentPtr(tri[0]);
	const CCVector3 *B = m_associatedCloud->getPointPersistentPtr(tri[1]);
	const CCVector3 *C = m_associatedCloud->getPointPersistentPtr(tri[2]);

	PointCoordinateType d1 = ((P-*B).cross(*C-*B)).norm()/*/2.0*/;
	PointCoordinateType d2 = ((P-*C).cross(*A-*C)).norm()/*/2.0*/;
	PointCoordinateType d3 = ((P-*A).cross(*B-*A)).norm()/*/2.0*/;
	//we must normalize weights
	PointCoordinateType dsum = d1+d2+d3;
	d1/=dsum;
	d2/=dsum;
	d3/=dsum;

	const colorType* C1 = m_associatedCloud->getPointColor(tri[0]);
	const colorType* C2 = m_associatedCloud->getPointColor(tri[0]);
	const colorType* C3 = m_associatedCloud->getPointColor(tri[0]);

	rgb[0] = (colorType)floor((float)C1[0]*d1+(float)C2[0]*d2+(float)C3[0]*d3);
	rgb[1] = (colorType)floor((float)C1[1]*d1+(float)C2[1]*d2+(float)C3[1]*d3);
	rgb[2] = (colorType)floor((float)C1[2]*d1+(float)C2[2]*d2+(float)C3[2]*d3);

	return true;
}

bool ccMesh::getColorFromTexture(unsigned triIndex, const CCVector3& P, colorType rgb[], bool interpolateColorIfNoTexture)
{
	assert(triIndex<size());

	int matIndex = -1;

	if (hasMaterials())
	{
		assert(m_materials);
		matIndex = m_triMtlIndexes->getValue(triIndex);
		assert(matIndex<(int)m_materials->size());
	}

	//do we need to change material?
	if (matIndex<0)
	{
		if (interpolateColorIfNoTexture)
			return interpolateColors(triIndex,P,rgb);
		return false;
	}

	const ccMaterial& material = (*m_materials)[matIndex];

	if (material.texture.isNull())
	{
		rgb[0] = (colorType)(material.diffuseFront[0]*MAX_COLOR_COMP);
		rgb[1] = (colorType)(material.diffuseFront[1]*MAX_COLOR_COMP);
		rgb[2] = (colorType)(material.diffuseFront[2]*MAX_COLOR_COMP);
		return true;
	}

	assert(m_texCoords && m_texCoordIndexes);
	const int* txInd = m_texCoordIndexes->getValue(triIndex);
	const float* Tx1 = (txInd[0]>=0 ? m_texCoords->getValue(txInd[0]) : 0);
	const float* Tx2 = (txInd[1]>=0 ? m_texCoords->getValue(txInd[1]) : 0);
	const float* Tx3 = (txInd[2]>=0 ? m_texCoords->getValue(txInd[2]) : 0);

	//intepolation weights
	const unsigned* tri = m_triIndexes->getValue(triIndex);
	const CCVector3 *A = m_associatedCloud->getPointPersistentPtr(tri[0]);
	const CCVector3 *B = m_associatedCloud->getPointPersistentPtr(tri[1]);
	const CCVector3 *C = m_associatedCloud->getPointPersistentPtr(tri[2]);

	PointCoordinateType d1 = ((P-*B).cross(*C-*B)).norm()/*/2.0*/;
	PointCoordinateType d2 = ((P-*C).cross(*A-*C)).norm()/*/2.0*/;
	PointCoordinateType d3 = ((P-*A).cross(*B-*A)).norm()/*/2.0*/;
	//we must normalize weights
	PointCoordinateType dsum = d1+d2+d3;
	d1/=dsum;
	d2/=dsum;
	d3/=dsum;

	float x = (Tx1 ? Tx1[0] : 0)*d1 + (Tx2 ? Tx2[0] : 0)*d2 + (Tx3 ? Tx3[0] : 0)*d3;
	float y = (Tx1 ? Tx1[1] : 0)*d1 + (Tx2 ? Tx2[1] : 0)*d2 + (Tx3 ? Tx3[1] : 0)*d3;

	if (x<0 || x>1.0f || y<0 || y>1.0f)
	{
		if (interpolateColorIfNoTexture)
			return interpolateColors(triIndex,P,rgb);
		return false;
	}

	QRgb pixel = material.texture.pixel(std::min((int)floor(x*float(material.texture.width())),material.texture.width()-1),
					std::min((int)floor(y*float(material.texture.height())),material.texture.height()-1));

	rgb[0] = qRed(pixel);
	rgb[1] = qGreen(pixel);
	rgb[2] = qBlue(pixel);

	return true;
}

static const unsigned s_defaultSubdivideGrowRate = 50;
static QMap<__int64,unsigned> s_alreadyCreatedVertices; //map to store already created edges middle points

static __int64 GenerateKey(unsigned edgeIndex1, unsigned edgeIndex2)
{
	if (edgeIndex1>edgeIndex2)
		std::swap(edgeIndex1,edgeIndex2);

	return ((((__int64)edgeIndex1)<<32) | (__int64)edgeIndex2);
}

bool ccMesh::pushSubdivide(PointCoordinateType maxArea, unsigned indexA, unsigned indexB, unsigned indexC)
{
	if (maxArea<=ZERO_TOLERANCE)
	{
		ccLog::Error("[ccMesh::pushSubdivide] Invalid input argument!");
		return false;
	}

	if (!getAssociatedCloud() || !getAssociatedCloud()->isA(CC_POINT_CLOUD))
	{
		ccLog::Error("[ccMesh::pushSubdivide] Vertices set must be a true point cloud!");
		return false;
	}
	ccPointCloud* vertices = static_cast<ccPointCloud*>(getAssociatedCloud());
	assert(vertices);
	const CCVector3* A(vertices->getPoint(indexA));
	const CCVector3* B(vertices->getPoint(indexB));
	const CCVector3* C(vertices->getPoint(indexC));

	//do we need to sudivide this triangle?
	PointCoordinateType area = ((*B-*A)*(*C-*A)).norm()/(PointCoordinateType)2.0;
	if (area > maxArea)
	{
		//we will add 3 new vertices, so we must be sure to have enough memory
		if (vertices->size()+2 >= vertices->capacity())
		{
			assert(s_defaultSubdivideGrowRate>2);
			if (!vertices->reserve(vertices->size()+s_defaultSubdivideGrowRate))
			{
				ccLog::Error("[ccMesh::pushSubdivide] Not enough memory!");
				return false;
			}
		}
		
		//add new vertices
		unsigned indexG1 = 0;
		{
			__int64 key = GenerateKey(indexA,indexB);
			QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG1 = vertices->size();
				CCVector3 G1 = (*A+*B)/(PointCoordinateType)2.0;
				vertices->addPoint(G1.u);
				//and add it to the map
				s_alreadyCreatedVertices.insert(key,indexG1);
			}
			else
			{
				indexG1 = it.value();
			}
		}
		unsigned indexG2 = 0;
		{
			__int64 key = GenerateKey(indexB,indexC);
			QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG2 = vertices->size();
				CCVector3 G2 = (*B+*C)/(PointCoordinateType)2.0;
				vertices->addPoint(G2.u);
				//and add it to the map
				s_alreadyCreatedVertices.insert(key,indexG2);
			}
			else
			{
				indexG2 = it.value();
			}
		}
		unsigned indexG3 = vertices->size();
		{
			__int64 key = GenerateKey(indexC,indexA);
			QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG3 = vertices->size();
				CCVector3 G3 = (*C+*A)/(PointCoordinateType)2.0;
				vertices->addPoint(G3.u);
				//and add it to the map
				s_alreadyCreatedVertices.insert(key,indexG3);
			}
			else
			{
				indexG3 = it.value();
			}
		}

		//add new triangles
		if (!pushSubdivide(maxArea, indexA, indexG1, indexG3))
			return false;
		if (!pushSubdivide(maxArea, indexB, indexG2, indexG1))
			return false;
		if (!pushSubdivide(maxArea, indexC, indexG3, indexG2))
			return false;
		if (!pushSubdivide(maxArea, indexG1, indexG2, indexG3))
			return false;
	}
	else
	{
		//we will add one triangle, so we must be sure to have enough memory
		if (size() == maxSize())
		{
			if (!reserve(size()+3*s_defaultSubdivideGrowRate))
			{
				ccLog::Error("[ccMesh::pushSubdivide] Not enough memory!");
				return false;
			}
		}

		//we keep this triangle as is
		addTriangle(indexA,indexB,indexC);
	}

	return true;
}

ccMesh* ccMesh::subdivide(float maxArea) const
{
	if (maxArea<=ZERO_TOLERANCE)
	{
		ccLog::Error("[ccMesh::subdivide] Invalid input argument!");
		return 0;
	}

	unsigned triCount = size();
	ccGenericPointCloud* vertices = getAssociatedCloud();
	unsigned vertCount = (vertices ? vertices->size() : 0);
	if (!vertices || vertCount*triCount==0)
	{
		ccLog::Error("[ccMesh::subdivide] Invalid mesh: no face or no vertex!");
		return 0;
	}

	ccPointCloud* resultVertices = 0;
	if (vertices->isA(CC_POINT_CLOUD))
		resultVertices = static_cast<ccPointCloud*>(vertices)->cloneThis();
	else
		resultVertices = new ccPointCloud(vertices);
	if (!resultVertices)
	{
		ccLog::Error("[ccMesh::subdivide] Not enough memory!");
		return 0;
	}

	ccMesh* resultMesh = new ccMesh(resultVertices);
	resultMesh->addChild(resultVertices);
	
	if (!resultMesh->reserve(triCount))
	{
		ccLog::Error("[ccMesh::subdivide] Not enough memory!");
		delete resultMesh;
		return 0;
	}

	s_alreadyCreatedVertices.clear();

	try
	{		
		for (unsigned i=0;i<triCount;++i)
		{
			const unsigned* tri = m_triIndexes->getValue(i);
			if (!resultMesh->pushSubdivide(maxArea,tri[0],tri[1],tri[2]))
			{
				ccLog::Error("[ccMesh::subdivide] Not enough memory!");
				delete resultMesh;
				return 0;
			}
		}
	}
	catch(...)
	{
		ccLog::Error("[ccMesh::subdivide] An error occured!");
		delete resultMesh;
		return 0;
	}

	//we must also 'fix' the triangles that share (at least) an edge with a subdivide triangle!
	try
	{
		unsigned newTriCount = resultMesh->size();
		for (unsigned i=0;i<newTriCount;++i)
		{
			unsigned* _face = resultMesh->m_triIndexes->getValue(i); //warning: array might change at each call to reallocate!
			unsigned indexA = _face[0];
			unsigned indexB = _face[1];
			unsigned indexC = _face[2];

			//test all edges
			int indexG1 = -1;
			{
				QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexA,indexB));
				if (it != s_alreadyCreatedVertices.end())
					indexG1 = (int)it.value();
			}
			int indexG2 = -1;
			{
				QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexB,indexC));
				if (it != s_alreadyCreatedVertices.end())
					indexG2 = (int)it.value();
			}
			int indexG3 = -1;
			{
				QMap<__int64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexC,indexA));
				if (it != s_alreadyCreatedVertices.end())
					indexG3 = (int)it.value();
			}

			//at least one edge is 'wrong'
			unsigned brokenEdges = (indexG1<0 ? 0:1)
								 + (indexG2<0 ? 0:1)
								 + (indexG3<0 ? 0:1);

			if (brokenEdges == 1)
			{
				int indexG = indexG1;
				unsigned char i1 = 2; //relative index facing the broken edge
				if (indexG2>=0)
				{
					indexG = indexG2;
					i1 = 0;
				}
				else if (indexG3>=0)
				{
					indexG = indexG3;
					i1 = 1;
				}
				assert(indexG>=0);
				assert(i1<3);

				unsigned indexes[3] = { indexA, indexB, indexC };

				//replace current triangle by one half
				_face[0] = indexes[i1];
				_face[1] = indexG;
				_face[2] = indexes[(i1+2)%3];
				//and add the other half (we can use pushSubdivide as the area should alredy be ok!)
				if (!resultMesh->pushSubdivide(maxArea,indexes[i1],indexes[(i1+1)%3],indexG))
				{
					ccLog::Error("[ccMesh::subdivide] Not enough memory!");
					delete resultMesh;
					return 0;
				}
			}
			else if (brokenEdges == 2)
			{
				if (indexG1<0) //broken edges: BC and CA
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexC;
					_face[1] = indexG3;
					_face[2] = indexG2;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(maxArea, indexA, indexG2, indexG3) ||
						!resultMesh->pushSubdivide(maxArea, indexA, indexB, indexG2))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return 0;
					}
				}
				else if (indexG2<0) //broken edges: AB and CA
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexA;
					_face[1] = indexG1;
					_face[2] = indexG3;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(maxArea, indexB, indexG3, indexG1) ||
						!resultMesh->pushSubdivide(maxArea, indexB, indexC, indexG3))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return 0;
					}
				}
				else /*if (indexG3<0)*/ //broken edges: AB and BC
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexB;
					_face[1] = indexG2;
					_face[2] = indexG1;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(maxArea, indexC, indexG1, indexG2) ||
						!resultMesh->pushSubdivide(maxArea, indexC, indexA, indexG1))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return 0;
					}
				}
			}
			else if (brokenEdges == 3) //works just as a standard subdivision in fact!
			{
				//replace current triangle by one quarter
				_face[0] = indexA;
				_face[1] = indexG1;
				_face[2] = indexG3;
				//and add the other 3 quarters (we can use pushSubdivide as the area should alredy be ok!)
				if (!resultMesh->pushSubdivide(maxArea, indexB, indexG2, indexG1) ||
					!resultMesh->pushSubdivide(maxArea, indexC, indexG3, indexG2) ||
					!resultMesh->pushSubdivide(maxArea, indexG1, indexG2, indexG3))
				{
					ccLog::Error("[ccMesh::subdivide] Not enough memory!");
					delete resultMesh;
					return 0;
				}
			}
		}
	}
	catch(...)
	{
		ccLog::Error("[ccMesh::subdivide] An error occured!");
		delete resultMesh;
		return 0;
	}

	s_alreadyCreatedVertices.clear();

	if (resultMesh->size() < resultMesh->maxSize())
		resultMesh->resize(resultMesh->size());
	if (resultVertices->size()<resultVertices->capacity())
		resultVertices->resize(resultVertices->size());

	//we import from the original mesh... what we can
	if (hasNormals())
	{
		resultMesh->computeNormals();
		resultMesh->showNormals(normalsShown());
	}
	resultMesh->setVisible(isVisible());

	return resultMesh;
}
