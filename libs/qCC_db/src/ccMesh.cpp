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

#include "ccMesh.h"

//Local
#include "ccGenericPointCloud.h"
#include "ccNormalVectors.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"
#include "ccSubMesh.h"
#include "ccScalarField.h"
#include "ccColorScalesManager.h"
#include "ccGenericGLDisplay.h"
#include "ccProgressDialog.h"
#include "ccChunk.h"

//CCCoreLib
#include <ManualSegmentationTools.h>
#include <PointProjectionTools.h>
#include <ReferenceCloud.h>
#include <Neighbourhood.h>
#include <Delaunay2dMesh.h>

//System
#include <string.h>
#include <assert.h>
#include <cmath> //for std::modf

static CCVector3 s_blankNorm(0, 0, 0);

ccMesh::ccMesh(ccGenericPointCloud* vertices, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccGenericMesh("Mesh", uniqueID)
	, m_associatedCloud(nullptr)
	, m_triNormals(nullptr)
	, m_texCoords(nullptr)
	, m_materials(nullptr)
	, m_triVertIndexes(nullptr)
	, m_globalIterator(0)
	, m_triMtlIndexes(nullptr)
	, m_texCoordIndexes(nullptr)
	, m_triNormalIndexes(nullptr)
{
	setAssociatedCloud(vertices);

	m_triVertIndexes = new triangleIndexesContainer();
	m_triVertIndexes->link();
}

ccMesh::ccMesh(CCCoreLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices)
	: ccGenericMesh("Mesh")
	, m_associatedCloud(nullptr)
	, m_triNormals(nullptr)
	, m_texCoords(nullptr)
	, m_materials(nullptr)
	, m_triVertIndexes(nullptr)
	, m_globalIterator(0)
	, m_triMtlIndexes(nullptr)
	, m_texCoordIndexes(nullptr)
	, m_triNormalIndexes(nullptr)
{
	setAssociatedCloud(giVertices);

	m_triVertIndexes = new triangleIndexesContainer();
	m_triVertIndexes->link();

	unsigned triNum = giMesh->size();
	if (!reserve(triNum))
		return;

	giMesh->placeIteratorAtBeginning();
	for (unsigned i = 0; i < triNum; ++i)
	{
		const CCCoreLib::VerticesIndexes* tsi = giMesh->getNextTriangleVertIndexes();
		addTriangle(tsi->i1, tsi->i2, tsi->i3);
	}

	//if (!giVertices->hasNormals())
	//	computeNormals();
	showNormals(giVertices->hasNormals());

	if (giVertices->hasColors())
		showColors(giVertices->colorsShown());

	if (giVertices->hasDisplayedScalarField())
		showSF(giVertices->sfShown());
}

ccMesh::~ccMesh()
{
	clearTriNormals();
	setMaterialSet(nullptr);
	setTexCoordinatesTable(nullptr);

	if (m_triVertIndexes)
		m_triVertIndexes->release();
	if (m_texCoordIndexes)
		m_texCoordIndexes->release();
	if (m_triMtlIndexes)
		m_triMtlIndexes->release();
	if (m_triNormalIndexes)
		m_triNormalIndexes->release();
}

void ccMesh::setAssociatedCloud(ccGenericPointCloud* cloud)
{
	m_associatedCloud = cloud;

	if (m_associatedCloud)
		m_associatedCloud->addDependency(this,DP_NOTIFY_OTHER_ON_DELETE | DP_NOTIFY_OTHER_ON_UPDATE);

	m_bBox.setValidity(false);
}

void ccMesh::onUpdateOf(ccHObject* obj)
{
	if (obj == m_associatedCloud)
	{
		m_bBox.setValidity(false);
		notifyGeometryUpdate(); //for sub-meshes
	}

	ccGenericMesh::onUpdateOf(obj);
}

void ccMesh::onDeletionOf(const ccHObject* obj)
{
	if (obj == m_associatedCloud)
		setAssociatedCloud(nullptr);

	ccGenericMesh::onDeletionOf(obj);
}

bool ccMesh::hasColors() const
{
	return (m_associatedCloud ? m_associatedCloud->hasColors() : false);
}

bool ccMesh::hasNormals() const
{
	return ((m_associatedCloud ? m_associatedCloud->hasNormals() : false) || hasTriNormals());
}

bool ccMesh::hasDisplayedScalarField() const
{
	return (m_associatedCloud ? m_associatedCloud->hasDisplayedScalarField() : false);
}

bool ccMesh::hasScalarFields() const
{
	return (m_associatedCloud ? m_associatedCloud->hasScalarFields() : false);
}

bool ccMesh::computeNormals(bool perVertex)
{
	return perVertex ? computePerVertexNormals() : computePerTriangleNormals();
}

bool ccMesh::computePerVertexNormals()
{
	if (!m_associatedCloud || !m_associatedCloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
	{
		ccLog::Warning("[ccMesh::computePerVertexNormals] Vertex set is not a standard cloud?!");
		return false;
	}

	unsigned triCount = size();
	if (triCount == 0)
	{
		ccLog::Warning("[ccMesh::computePerVertexNormals] Empty mesh!");
		return false;
	}
	unsigned vertCount = m_associatedCloud->size();
	if (vertCount < 3)
	{
		ccLog::Warning("[ccMesh::computePerVertexNormals] Not enough vertices! (<3)");
		return false;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_associatedCloud);

	//we instantiate a temporary structure to store each vertex normal (uncompressed)
	std::vector<CCVector3> theNorms;
	try
	{
		theNorms.resize(vertCount, s_blankNorm);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccMesh::computePerVertexNormals] Not enough memory!");
		return false;
	}

	//allocate compressed normals array on vertices cloud
	bool normalsWereAllocated = cloud->hasNormals();
	if (/*!normalsWereAllocated && */!cloud->resizeTheNormsTable()) //we call it whatever the case (just to be sure)
	{
		//warning message should have been already issued!
		return false;
	}

	//for each triangle
	placeIteratorAtBeginning();
	{
		for (unsigned i = 0; i < triCount; ++i)
		{
			CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();

			assert(tsi->i1 < vertCount && tsi->i2 < vertCount && tsi->i3 < vertCount);
			const CCVector3 *A = cloud->getPoint(tsi->i1);
			const CCVector3 *B = cloud->getPoint(tsi->i2);
			const CCVector3 *C = cloud->getPoint(tsi->i3);

			//compute face normal (right hand rule)
			CCVector3 N = (*B-*A).cross(*C-*A);
			//N.normalize(); //DGM: no normalization = weighting by surface!

			//we add this normal to all triangle vertices
			theNorms[tsi->i1] += N;
			theNorms[tsi->i2] += N;
			theNorms[tsi->i3] += N;
		}
	}

	//for each vertex
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			CCVector3& N = theNorms[i];
			//normalize the 'mean' normal
			N.normalize();
			cloud->setPointNormal(i, N);
		}
	}

	//apply it also to sub-meshes!
	showNormals_extended(true);

	if (!normalsWereAllocated)
		cloud->showNormals(true);

	return true;
}

bool ccMesh::computePerTriangleNormals()
{
	unsigned triCount = size();
	if (triCount == 0)
	{
		ccLog::Warning("[ccMesh::computePerTriangleNormals] Empty mesh!");
		return false;
	}

	//if some normal indexes already exists, we remove them (easier)
	if (m_triNormalIndexes)
		removePerTriangleNormalIndexes();
	setTriNormsTable(nullptr);

	NormsIndexesTableType* normIndexes = new NormsIndexesTableType();
	if (!normIndexes->reserveSafe(triCount))
	{
		normIndexes->release();
		ccLog::Warning("[ccMesh::computePerTriangleNormals] Not enough memory!");
		return false;
	}

	//for each triangle
	{
		for (unsigned i = 0; i < triCount; ++i)
		{
			const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(i);
			const CCVector3* A = m_associatedCloud->getPoint(tri.i1);
			const CCVector3* B = m_associatedCloud->getPoint(tri.i2);
			const CCVector3* C = m_associatedCloud->getPoint(tri.i3);

			//compute face normal (right hand rule)
			CCVector3 N = (*B-*A).cross(*C-*A);

			CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);
			normIndexes->emplace_back(nIndex);
		}
	}

	//set the per-triangle normal indexes
	{
		if (!reservePerTriangleNormalIndexes())
		{
			normIndexes->release();
			ccLog::Warning("[ccMesh::computePerTriangleNormals] Not enough memory!");
			return false;
		}

		setTriNormsTable(normIndexes);

		for (int i = 0; i < static_cast<int>(triCount); ++i)
			addTriangleNormalIndexes(i, i, i);
	}

	//apply it also to sub-meshes!
	showNormals_extended(true);

	return true;
}

bool ccMesh::normalsShown() const
{
	return (ccHObject::normalsShown() || triNormsShown());
}

bool ccMesh::processScalarField(MESH_SCALAR_FIELD_PROCESS process)
{
	if (!m_associatedCloud || !m_associatedCloud->isScalarFieldEnabled())
		return false;

	unsigned nPts = m_associatedCloud->size();

	//instantiate memory for per-vertex mean SF
	ScalarType* meanSF = new ScalarType[nPts];
	if (!meanSF)
	{
		//Not enough memory!
		return false;
	}

	//per-vertex counters
	unsigned *count = new unsigned[nPts];
	if (!count)
	{
		//Not enough memory!
		delete[] meanSF;
		return false;
	}

	//init arrays
	{
		for (unsigned i = 0; i < nPts; ++i)
		{
			meanSF[i] = m_associatedCloud->getPointScalarValue(i);
			count[i] = 1;
		}
	}

	//for each triangle
	unsigned nTri = size();
	{
		placeIteratorAtBeginning();
		for (unsigned i = 0; i < nTri; ++i)
		{
			const CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes(); //DGM: getNextTriangleVertIndexes is faster for mesh groups!

			//compute the sum of all connected vertices SF values
			meanSF[tsi->i1] += m_associatedCloud->getPointScalarValue(tsi->i2);
			meanSF[tsi->i2] += m_associatedCloud->getPointScalarValue(tsi->i3);
			meanSF[tsi->i3] += m_associatedCloud->getPointScalarValue(tsi->i1);

			//TODO DGM: we could weight this by the vertices distance?
			++count[tsi->i1];
			++count[tsi->i2];
			++count[tsi->i3];
		}
	}

	//normalize
	{
		for (unsigned i = 0; i < nPts; ++i)
			meanSF[i] /= count[i];
	}

	switch (process)
	{
	case SMOOTH_MESH_SF:
		{
			//Smooth = mean value
			for (unsigned i = 0; i < nPts; ++i)
				m_associatedCloud->setPointScalarValue(i, meanSF[i]);
		}
		break;
	case ENHANCE_MESH_SF:
		{
			//Enhance = old value + (old value - mean value)
			for (unsigned i = 0; i < nPts; ++i)
			{
				ScalarType v = 2 * m_associatedCloud->getPointScalarValue(i) - meanSF[i];
				m_associatedCloud->setPointScalarValue(i, v > 0 ? v : 0);
			}
		}
		break;
	}

	delete[] meanSF;
	delete[] count;

	return true;
}

void ccMesh::setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable/*=true*/)
{
	if (m_triNormals == triNormsTable)
		return;

	if (m_triNormals && autoReleaseOldTable)
	{
		int childIndex = getChildIndex(m_triNormals);
		m_triNormals->release();
		m_triNormals = nullptr;
		if (childIndex >= 0)
			removeChild(childIndex);
	}

	m_triNormals = triNormsTable;
	if (m_triNormals)
	{
		m_triNormals->link();
		int childIndex = getChildIndex(m_triNormals);
		if (childIndex < 0)
			addChild(m_triNormals);
	}
	else
	{
		removePerTriangleNormalIndexes(); //auto-remove per-triangle indexes (we don't need them anymore)
	}
}

void ccMesh::setMaterialSet(ccMaterialSet* materialSet, bool autoReleaseOldMaterialSet/*=true*/)
{
	if (m_materials == materialSet)
		return;

	if (m_materials && autoReleaseOldMaterialSet)
	{
		int childIndex = getChildIndex(m_materials);
		m_materials->release();
		m_materials = nullptr;
		if (childIndex >= 0)
			removeChild(childIndex);
	}

	m_materials = materialSet;
	if (m_materials)
	{
		m_materials->link();
		int childIndex = getChildIndex(m_materials);
		if (childIndex < 0)
			addChild(m_materials);
	}
	else
	{
		removePerTriangleMtlIndexes(); //auto-remove per-triangle indexes (we don't need them anymore)
	}

	//update display (for textures!)
	setDisplay(m_currentDisplay);
}

void ccMesh::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccGenericMesh::applyGLTransformation(trans);

	//we take care of per-triangle normals
	//(vertices and per-vertex normals should be taken care of by the recursive call)
	transformTriNormals(trans);
}

void ccMesh::transformTriNormals(const ccGLMatrix& trans)
{
    //we must take care of the triangle normals!
	if (m_triNormals && (!getParent() || !getParent()->isKindOf(CC_TYPES::MESH)))
    {
#if 0 //no use to use memory for this!
		size_t numTriNormals = m_triNormals->size();
		bool recoded = false;

        //if there are more triangle normals than the size of the compressed
		//normals array, we recompress the array instead of recompressing each normal
        if (numTriNormals > ccNormalVectors::GetNumberOfVectors())
        {
            NormsIndexesTableType* newNorms = new NormsIndexesTableType;
            if (newNorms->reserve(ccNormalVectors::GetNumberOfVectors()))
            {
				//decode
				{
					for (unsigned i=0; i<ccNormalVectors::GetNumberOfVectors(); i++)
					{
						CCVector3 new_n(ccNormalVectors::GetNormal(i));
						trans.applyRotation(new_n);
						CompressedNormType newNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
						newNorms->emplace_back(newNormIndex);
					}
				}

				//recode
                m_triNormals->placeIteratorAtBeginning();
				{
					for (unsigned i=0; i<numTriNormals; i++)
					{
						m_triNormals->setValue(i,newNorms->getValue(m_triNormals->getCurrentValue()));
						m_triNormals->forwardIterator();
					}
				}
                recoded = true;
            }
            newNorms->clear();
			newNorms->release();
			newNorms = 0;
        }

        //if there are less triangle normals than the compressed normals array size
        //(or if there is not enough memory to instantiate the temporary array),
		//we recompress each normal ...
        if (!recoded)
#endif
        {
			for (CompressedNormType& _theNormIndex : *m_triNormals)
            {
                CCVector3 new_n(ccNormalVectors::GetNormal(_theNormIndex));
                trans.applyRotation(new_n);
                _theNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
            }
        }
	}
}

bool ccMesh::laplacianSmooth(	unsigned nbIteration,
								PointCoordinateType factor,
								ccProgressDialog* progressCb/*=0*/)
{
	if (!m_associatedCloud)
		return false;

	//vertices
	unsigned vertCount = m_associatedCloud->size();
	//triangles
	unsigned faceCount = size();
	if (!vertCount || !faceCount)
		return false;

	std::vector<CCVector3> verticesDisplacement;
	try
	{
		verticesDisplacement.resize(vertCount);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//compute the number of edges to which belong each vertex
	std::vector<unsigned> edgesCount;
	try
	{
		edgesCount.resize(vertCount, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	placeIteratorAtBeginning();
	for(unsigned j=0; j<faceCount; j++)
	{
		const CCCoreLib::VerticesIndexes* tri = getNextTriangleVertIndexes();
		edgesCount[tri->i1] += 2;
		edgesCount[tri->i2] += 2;
		edgesCount[tri->i3] += 2;
	}

	//progress dialog
	CCCoreLib::NormalizedProgress nProgress(progressCb, nbIteration);
	if (progressCb)
	{
		progressCb->setMethodTitle(QObject::tr("Laplacian smooth"));
		progressCb->setInfo(QObject::tr("Iterations: %1\nVertices: %2\nFaces: %3").arg(nbIteration).arg(vertCount).arg(faceCount));
		progressCb->start();
	}

	//repeat Laplacian smoothing iterations
	for (unsigned iter = 0; iter < nbIteration; iter++)
	{
		std::fill(verticesDisplacement.begin(), verticesDisplacement.end(), CCVector3(0, 0, 0));

		//for each triangle
		placeIteratorAtBeginning();
		for (unsigned j = 0; j < faceCount; j++)
		{
			const CCCoreLib::VerticesIndexes* tri = getNextTriangleVertIndexes();

			const CCVector3* A = m_associatedCloud->getPoint(tri->i1);
			const CCVector3* B = m_associatedCloud->getPoint(tri->i2);
			const CCVector3* C = m_associatedCloud->getPoint(tri->i3);

			CCVector3 dAB = (*B-*A);
			CCVector3 dAC = (*C-*A);
			CCVector3 dBC = (*C-*B);

			verticesDisplacement[tri->i1] += dAB + dAC;
			verticesDisplacement[tri->i2] += dBC - dAB;
			verticesDisplacement[tri->i3] -= dAC + dBC;
		}

		if (!nProgress.oneStep())
		{
			//cancelled by user
			break;
		}

		//apply displacement
		for (unsigned i = 0; i < vertCount; i++)
		{
			if (edgesCount[i])
			{
				//this is a "persistent" pointer and we know what type of cloud is behind ;)
				CCVector3* P = const_cast<CCVector3*>(m_associatedCloud->getPointPersistentPtr(i));
				(*P) += verticesDisplacement[i] * (factor / edgesCount[i]);
			}
		}
	}

	m_associatedCloud->notifyGeometryUpdate();

	if (hasNormals())
		computeNormals(!hasTriNormals());

	return true;
}

ccMesh* ccMesh::cloneMesh(	ccGenericPointCloud* vertices/*=nullptr*/,
							ccMaterialSet* clonedMaterials/*=nullptr*/,
							NormsIndexesTableType* clonedNormsTable/*=nullptr*/,
							TextureCoordsContainer* cloneTexCoords/*=nullptr*/)
{
	assert(m_associatedCloud);

	//vertices
	unsigned vertNum = m_associatedCloud->size();
	//triangles
	unsigned triNum = size();

	//temporary structure to check that vertices are really used (in case of vertices set sharing)
	std::vector<unsigned> usedVerts;

	ccGenericPointCloud* newVertices = vertices;

	//no input vertices set
	if (!newVertices)
	{
		//let's check the real vertex count
		try
		{
			usedVerts.resize(vertNum,0);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("[ccMesh::clone] Not enough memory!");
			return nullptr;
		}

		//flag used vertices
		{
			placeIteratorAtBeginning();
			for (unsigned i = 0; i < triNum; ++i)
			{
				const CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
				usedVerts[tsi->i1] = 1;
				usedVerts[tsi->i2] = 1;
				usedVerts[tsi->i3] = 1;
			}
		}

		//we check that all points in 'associatedCloud' are used by this mesh
		unsigned realVertCount = 0;
		{
			for (unsigned i=0; i<vertNum; ++i)
				usedVerts[i] = (usedVerts[i] == 1 ? realVertCount++ : vertNum);
		}

		//the associated cloud is already the exact vertices set --> nothing to change
		if (realVertCount == vertNum)
		{
			newVertices = m_associatedCloud->clone(nullptr,true);
		}
		else
		{
			//we create a temporary entity with used vertices only
			CCCoreLib::ReferenceCloud rc(m_associatedCloud);
			if (rc.reserve(realVertCount))
			{
				for (unsigned i=0; i<vertNum; ++i)
				{
					if (usedVerts[i] != vertNum)
						rc.addPointIndex(i); //can't fail, see above
				}

				//and the associated vertices set
				assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
				newVertices = static_cast<ccPointCloud*>(m_associatedCloud)->partialClone(&rc);
				if (newVertices && newVertices->size() < rc.size())
				{
					//not enough memory!
					delete newVertices;
					newVertices = nullptr;
				}
			}
		}
	}

	//failed to create a new vertices set!
	if (!newVertices)
	{
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return nullptr;
	}

	//mesh clone
	ccMesh* cloneMesh = new ccMesh(newVertices);
	if (!cloneMesh->reserve(triNum))
	{
		if (!vertices)
			delete newVertices;
		delete cloneMesh;
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return nullptr;
	}

	//let's create the new triangles
	if (!usedVerts.empty()) //in case we have an equivalence table
	{
		placeIteratorAtBeginning();
		for (unsigned i = 0; i < triNum; ++i)
		{
			const CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
			cloneMesh->addTriangle(usedVerts[tsi->i1], usedVerts[tsi->i2], usedVerts[tsi->i3]);
		}
		usedVerts.resize(0);
	}
	else
	{
		placeIteratorAtBeginning();
		for (unsigned i = 0; i < triNum; ++i)
		{
			const CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
			cloneMesh->addTriangle(tsi->i1, tsi->i2, tsi->i3);
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
				if (!cloneTexCoords)
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
		if (hasNormals() && !cloneMesh->hasNormals())
			cloneMesh->computeNormals(!hasTriNormals());
		newVertices->setEnabled(false);
		//we link the mesh structure with the new vertex set
		cloneMesh->addChild(newVertices);
		cloneMesh->setDisplay_recursive(getDisplay());
	}

	cloneMesh->showNormals(normalsShown());
	cloneMesh->showColors(colorsShown());
	cloneMesh->showSF(sfShown());
	cloneMesh->showMaterials(materialsShown());
	cloneMesh->setName(getName()+QString(".clone"));
	cloneMesh->setVisible(isVisible());
	cloneMesh->setEnabled(isEnabled());
	cloneMesh->importParametersFrom(this);

	return cloneMesh;
}

ccMesh* ccMesh::TriangulateTwoPolylines(ccPolyline* p1, ccPolyline* p2, CCVector3* projectionDir/*=0*/)
{
	if (!p1 || p1->size() == 0 || !p2 || p2->size() == 0)
	{
		assert(false);
		return nullptr;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(p1->size() + p2->size()))
	{
		ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Not enough memory");
		delete vertices;
		return nullptr;
	}

	//merge the two sets of vertices
	{
		for (unsigned i = 0; i < p1->size(); ++i)
			vertices->addPoint(*p1->getPoint(i));
		for (unsigned j = 0; j < p2->size(); ++j)
			vertices->addPoint(*p2->getPoint(j));
	}
	assert(vertices->size() != 0);

	CCCoreLib::Neighbourhood N(vertices);

	//get plane coordinate system
	CCVector3 O = *N.getGravityCenter();
	CCVector3 X(1, 0, 0);
	CCVector3 Y(0, 1, 0);
	if (projectionDir)
	{
		//use the input projection dir.
		X = projectionDir->orthogonal();
		Y = projectionDir->cross(X);
	}
	else
	{
		//use the best fit plane (normal)
		if (!N.getLSPlane())
		{
			ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Failed to fit a plane through both polylines");
			delete vertices;
			return nullptr;
		}

		X = *N.getLSPlaneX();
		Y = *N.getLSPlaneY();
	}

	std::vector<CCVector2> points2D;
	std::vector<int> segments2D;
	try
	{
		points2D.reserve(p1->size() + p2->size());
		segments2D.reserve(p1->segmentCount() + p2->segmentCount());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Not enough memory");
		delete vertices;
		return nullptr;
	}

	//project the polylines on the best fitting plane
	{
		ccPolyline* polylines[2] = { p1, p2 };
		for (size_t i = 0; i < 2; ++i)
		{
			ccPolyline* poly = polylines[i];
			unsigned vertCount = poly->size();
			int vertIndex0 = static_cast<int>(points2D.size());
			bool closed = poly->isClosed();
			for (unsigned v = 0; v < vertCount; ++v)
			{
				const CCVector3* P = poly->getPoint(v);
				int vertIndex = static_cast<int>(points2D.size());
				
				CCVector3 OP = *P - O;
				CCVector2 P2D(OP.dot(X), OP.dot(Y));
				points2D.emplace_back(P2D);

				if (v + 1 < vertCount)
				{
					segments2D.emplace_back(vertIndex);
					segments2D.emplace_back(vertIndex + 1);
				}
				else if (closed)
				{
					segments2D.emplace_back(vertIndex);
					segments2D.emplace_back(vertIndex0);
				}
			}
		}
		assert(points2D.size() == p1->size() + p2->size());
		assert(segments2D.size() == (p1->segmentCount() + p2->segmentCount()) * 2);
	}

	CCCoreLib::Delaunay2dMesh* delaunayMesh = new CCCoreLib::Delaunay2dMesh;
	std::string errorStr;
	if (!delaunayMesh->buildMesh(points2D, segments2D, errorStr))
	{
		ccLog::Warning( QStringLiteral("Third party library error: %1").arg( QString::fromStdString( errorStr ) ) );
		delete delaunayMesh;
		delete vertices;
		return nullptr;
	}

	delaunayMesh->linkMeshWith(vertices, false);

	//remove the points outside of the 'concave' hull
	{
		//first compute the Convex hull
		std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> indexedPoints2D;
		try
		{
			indexedPoints2D.resize(points2D.size());
			for (size_t i = 0; i < points2D.size(); ++i)
			{
				indexedPoints2D[i] = points2D[i];
				indexedPoints2D[i].index = static_cast<unsigned>(i);
			}
			
			std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoints;
			if (CCCoreLib::PointProjectionTools::extractConvexHull2D(indexedPoints2D, hullPoints))
			{
				std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*>::iterator A = hullPoints.begin();
				for (; A != hullPoints.end(); ++A)
				{
					//current hull segment
					std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*>::iterator B = A;
					++B;
					if (B == hullPoints.end())
					{
						B = hullPoints.begin();
					}

					unsigned Aindex = (*A)->index;
					unsigned Bindex = (*B)->index;
					int Apoly = (Aindex < p1->size() ? 0 : 1);
					int Bpoly = (Bindex < p1->size() ? 0 : 1);
					//both vertices belong to the same polyline
					if (Apoly == Bpoly)
					{
						//if it creates an outer loop
						if (abs(static_cast<int>(Bindex) - static_cast<int>(Aindex)) > 1)
						{
							//create the corresponding contour
							unsigned iStart = std::min(Aindex, Bindex);
							unsigned iStop = std::max(Aindex, Bindex);
							std::vector<CCVector2> contour;
							contour.reserve(iStop - iStart + 1);
							for (unsigned j = iStart; j <= iStop; ++j)
							{
								contour.emplace_back(points2D[j]);
							}
							delaunayMesh->removeOuterTriangles(points2D, contour, /*remove inside = */false);
						}
					}
				}
			}
			else
			{
				ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Failed to compute the convex hull (can't clean the mesh borders)");
			}
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Not enough memory to clean the mesh borders");
		}

	}

	ccMesh* mesh = new ccMesh(delaunayMesh, vertices);
	if (mesh->size() != delaunayMesh->size())
	{
		//not enough memory (error will be issued later)
		delete mesh;
		mesh = nullptr;
	}

	//don't need this anymore
	delete delaunayMesh;
	delaunayMesh = nullptr;

	if (mesh)
	{
		mesh->addChild(vertices);
		mesh->setVisible(true);
		vertices->setEnabled(false);

		//global shift & scale (we copy it from the first polyline by default)
		mesh->copyGlobalShiftAndScale(*p1);
		//same thing for the display
		mesh->setDisplay(p1->getDisplay());
	}
	else
	{
		ccLog::Warning("[ccMesh::TriangulateTwoPolylines] Not enough memory");
		delete vertices;
		vertices = nullptr;
	}

	return mesh;
}

ccMesh* ccMesh::Triangulate(ccGenericPointCloud* cloud,
							CCCoreLib::TRIANGULATION_TYPES type,
							bool updateNormals/*=false*/,
							PointCoordinateType maxEdgeLength/*=0*/,
							unsigned char dim/*=2*/)
{
	if (!cloud || dim > 2)
	{
		ccLog::Warning("[ccMesh::Triangulate] Invalid input parameters!");
		return nullptr;
	}
	if (cloud->size() < 3)
	{
		ccLog::Warning("[ccMesh::Triangulate] Cloud has not enough points!");
		return nullptr;
	}
	
	//compute raw mesh
	std::string errorStr;
	CCCoreLib::GenericIndexedMesh* dummyMesh = CCCoreLib::PointProjectionTools::computeTriangulation(	cloud,
																								type,
																								maxEdgeLength,
																								dim,
																								errorStr);
	if (!dummyMesh)
	{
		ccLog::Warning( QStringLiteral("[ccMesh::Triangulate] Failed to construct Delaunay mesh (Triangle lib error: %1)")
						.arg( QString::fromStdString( errorStr ) ) );
		return nullptr;
	}

	//convert raw mesh to ccMesh
	ccMesh* mesh = new ccMesh(dummyMesh, cloud);

	//don't need this anymore
	delete dummyMesh;
	dummyMesh = nullptr;

	if (!mesh)
	{
		ccLog::Warning("[ccMesh::Triangulate] An error occurred while computing mesh! (not enough memory?)");
		return nullptr;
	}

	mesh->setName(cloud->getName()+QString(".mesh"));
	mesh->setDisplay(cloud->getDisplay());
	bool cloudHadNormals = cloud->hasNormals();
	//compute per-vertex normals if necessary
	if (!cloudHadNormals || updateNormals)
	{
		mesh->computeNormals(true);
	}
	mesh->showNormals(cloudHadNormals || !cloud->hasColors());
	mesh->copyGlobalShiftAndScale(*cloud);

	return mesh;
}

bool ccMesh::merge(const ccMesh* mesh, bool createSubMesh)
{
	if (!mesh)
	{
		assert(false);
		ccLog::Warning("[ccMesh::merge] Internal error: invalid input!");
		return false;
	}
	if (	!mesh->getAssociatedCloud()
		||	!mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD)
		||	!m_associatedCloud
		||	!m_associatedCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		ccLog::Warning("[ccMesh::merge] Requires meshes with standard vertices!");
		return false;
	}
	ccPointCloud* vertices = static_cast<ccPointCloud*>(mesh->getAssociatedCloud());

	//vertices count (before merge)
	const unsigned vertNumBefore = m_associatedCloud->size();
	//triangles count (before merge)
	const unsigned triNumBefore = size();

	bool success = false;

	for (int iteration = 0; iteration < 1; ++iteration) //fake loop for easy breaking/cleaning
	{
		//merge vertices
		unsigned vertIndexShift = 0;
		if (mesh->getAssociatedCloud() != m_associatedCloud)
		{
			unsigned vertAdded = mesh->getAssociatedCloud()->size();
			static_cast<ccPointCloud*>(m_associatedCloud)->append(vertices,m_associatedCloud->size(),true);

			//not enough memory?
			if (m_associatedCloud->size() < vertNumBefore + vertAdded)
			{
				ccLog::Warning("[ccMesh::merge] Not enough memory!");
				break;
			}
			vertIndexShift = vertNumBefore;
			if (vertNumBefore == 0)
			{
				//use the first merged cloud display properties
				m_associatedCloud->setVisible(vertices->isVisible());
				m_associatedCloud->setEnabled(vertices->isEnabled());
			}
		}
		showNormals(this->normalsShown() || mesh->normalsShown());
		showColors(this->colorsShown() || mesh->colorsShown());
		showSF(this->sfShown() || mesh->sfShown());

		//now for the triangles
		const unsigned triAdded = mesh->size();
		bool otherMeshHasMaterials = (mesh->m_materials && mesh->m_triMtlIndexes);
		bool otherMeshHasTexCoords = (mesh->m_texCoords && mesh->m_texCoordIndexes);
		bool otherMeshHasTriangleNormals = (mesh->m_triNormals && mesh->m_triNormalIndexes);
		{
			if (!reserve(triNumBefore + triAdded))
			{
				ccLog::Warning("[ccMesh::merge] Not enough memory!");
				break;
			}

			//we'll need those arrays later
			if (	(otherMeshHasMaterials && !m_triMtlIndexes && !reservePerTriangleMtlIndexes())
				||	(otherMeshHasTexCoords && !m_texCoordIndexes && !reservePerTriangleTexCoordIndexes())
				||	(otherMeshHasTriangleNormals && !m_triNormalIndexes && !reservePerTriangleNormalIndexes())
				)
			{
				ccLog::Warning("[ccMesh::merge] Not enough memory!");
				break;
			}

			for (unsigned i = 0; i < triAdded; ++i)
			{
				const CCCoreLib::VerticesIndexes* tsi = mesh->getTriangleVertIndexes(i);
				addTriangle(vertIndexShift + tsi->i1, vertIndexShift + tsi->i2, vertIndexShift + tsi->i3);
			}
		}

		//triangle normals
		bool hasTriangleNormals = m_triNormals && m_triNormalIndexes;
		if (hasTriangleNormals || otherMeshHasTriangleNormals)
		{
			//1st: does the other mesh has triangle normals
			if (otherMeshHasTriangleNormals)
			{
				size_t triIndexShift = 0;
				if (m_triNormals != mesh->m_triNormals)
				{
					//reserve mem for triangle normals
					if (!m_triNormals)
					{
						NormsIndexesTableType* normsTable = new NormsIndexesTableType();
						setTriNormsTable(normsTable);
					}
					assert(m_triNormals);
					size_t triNormalsCountBefore = m_triNormals->size();
					if (!m_triNormals->reserveSafe(triNormalsCountBefore + mesh->m_triNormals->size()))
					{
						ccLog::Warning("[ccMesh::merge] Not enough memory!");
						break;
					}
					//copy the values
					{
						for (unsigned i = 0; i < mesh->m_triNormals->size(); ++i)
							m_triNormals->emplace_back(mesh->m_triNormals->getValue(i));
					}
					triIndexShift = triNormalsCountBefore;
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triNormalIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i = 0; i < mesh->m_triNormalIndexes->size(); ++i)
					{
						const Tuple3i& indexes = mesh->m_triNormalIndexes->at(i);
						Tuple3i newIndexes(	indexes.u[0] < 0 ? -1 : indexes.u[0] + static_cast<int>(triIndexShift),
											indexes.u[1] < 0 ? -1 : indexes.u[1] + static_cast<int>(triIndexShift),
											indexes.u[2] < 0 ? -1 : indexes.u[2] + static_cast<int>(triIndexShift) );
						m_triNormalIndexes->emplace_back(newIndexes);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triNormalIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					Tuple3i defaultElement(-1, -1, -1);
					for (unsigned i = 0; i < mesh->size(); ++i)
						m_triNormalIndexes->emplace_back(defaultElement);
				}
			}
			showTriNorms(this->triNormsShown() || mesh->triNormsShown());
		}

		//materials
		bool hasMaterials = m_materials && m_triMtlIndexes;
		if (hasMaterials || otherMeshHasMaterials)
		{
			//1st: does the other mesh has materials?
			if (otherMeshHasMaterials)
			{
				std::vector<int> materialIndexMap;
				if (m_materials != mesh->m_materials)
				{
					//reserve mem for materials
					if (!m_materials)
					{
						ccMaterialSet* set = new ccMaterialSet("materials");
						setMaterialSet(set);
					}
					assert(m_materials);

					size_t otherMatSetSize = mesh->m_materials->size();
					try
					{
						materialIndexMap.resize(otherMatSetSize,-1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::merge] Not enough memory!");
						break;
					}
					//update map table
					for (size_t m = 0; m != otherMatSetSize; ++m)
					{
						materialIndexMap[m] = m_materials->addMaterial(mesh->m_materials->at(m));
					}
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triMtlIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i = 0; i < mesh->m_triMtlIndexes->size(); ++i)
					{
						int index = mesh->m_triMtlIndexes->getValue(i);
						assert(index < static_cast<int>(materialIndexMap.size()));
						int newIndex = (index < 0 ? -1 : materialIndexMap[index]);
						m_triMtlIndexes->emplace_back(newIndex);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triMtlIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					for (unsigned i = 0; i < mesh->size(); ++i)
						m_triMtlIndexes->emplace_back(-1);
				}
			}
		}
		showMaterials(this->materialsShown() || mesh->materialsShown());

		//texture coordinates
		bool hasTexCoords = m_texCoords && m_texCoordIndexes;
		if (hasTexCoords || otherMeshHasTexCoords)
		{
			//1st: does the other mesh has texture coordinates?
			if (otherMeshHasTexCoords)
			{
				size_t texCoordIndexShift = 0;
				if (m_texCoords != mesh->m_texCoords)
				{
					//reserve mem for triangle normals
					if (!m_texCoords)
					{
						TextureCoordsContainer* texCoordsTable = new TextureCoordsContainer;
						setTexCoordinatesTable(texCoordsTable);
					}
					assert(m_texCoords);
					size_t texCoordCountBefore = m_texCoords->size();
					if (!m_texCoords->reserveSafe(texCoordCountBefore + mesh->m_texCoords->size()))
					{
						ccLog::Warning("[ccMesh::merge] Not enough memory!");
						break;
					}
					//copy the values
					{
						static const TexCoords2D TxDef(-1.0f, -1.0f);
						for (unsigned i = 0; i < mesh->m_texCoords->size(); ++i)
						{
							const TexCoords2D& T = mesh->m_texCoords->at(i);
							m_texCoords->emplace_back(T);
						}
					}
					texCoordIndexShift = texCoordCountBefore;
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_texCoordIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i = 0; i < mesh->m_texCoordIndexes->size(); ++i)
					{
						const Tuple3i& indexes = mesh->m_texCoordIndexes->getValue(i);
						Tuple3i newIndexes(	indexes.u[0] < 0 ? -1 : indexes.u[0] + static_cast<int>(texCoordIndexShift),
											indexes.u[1] < 0 ? -1 : indexes.u[1] + static_cast<int>(texCoordIndexShift),
											indexes.u[2] < 0 ? -1 : indexes.u[2] + static_cast<int>(texCoordIndexShift) );
						m_texCoordIndexes->emplace_back(newIndexes);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_texCoordIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					Tuple3i defaultElement(-1, -1, -1);
					for (unsigned i = 0; i < mesh->m_texCoordIndexes->size(); ++i)
						m_texCoordIndexes->emplace_back(defaultElement);
				}
			}
		}

		//the end!
		showWired(this->isShownAsWire() || mesh->isShownAsWire());
		enableStippling(this->stipplingEnabled() || mesh->stipplingEnabled());
		success = true;
	}

	if (createSubMesh)
	{
		//triangles count (after merge)
		const unsigned triNumAfter = size();

		ccSubMesh* subMesh = new ccSubMesh(this);
		if (subMesh->reserve(triNumAfter - triNumBefore))
		{
			subMesh->addTriangleIndex(triNumBefore, triNumAfter);
			subMesh->setName(mesh->getName());
			subMesh->showMaterials(materialsShown());
			subMesh->showNormals(normalsShown());
			subMesh->showTriNorms(triNormsShown());
			subMesh->showColors(colorsShown());
			subMesh->showWired(isShownAsWire());
			subMesh->enableStippling(stipplingEnabled());
			subMesh->setEnabled(false);
			addChild(subMesh);
		}
		else
		{
			ccLog::Warning(QString("[Merge] Not enough memory to create the sub-mesh corresponding to mesh '%1'!").arg(mesh->getName()));
			delete subMesh;
			subMesh = nullptr;
		}
	}

	if (!success)
	{
		//revert to original state
		static_cast<ccPointCloud*>(m_associatedCloud)->resize(vertNumBefore);
		resize(triNumBefore);
	}

	return success;
}

unsigned ccMesh::size() const
{
	return static_cast<unsigned>(m_triVertIndexes->size());
}

unsigned ccMesh::capacity() const
{
	return static_cast<unsigned>(m_triVertIndexes->capacity());
}

void ccMesh::forEach(genericTriangleAction action)
{
	if (!m_associatedCloud)
		return;

	for (unsigned i = 0; i < m_triVertIndexes->size(); ++i)
	{
		const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->at(i);
		m_currentTriangle.A = m_associatedCloud->getPoint(tri.i1);
		m_currentTriangle.B = m_associatedCloud->getPoint(tri.i2);
		m_currentTriangle.C = m_associatedCloud->getPoint(tri.i3);
		action(m_currentTriangle);
	}
}

void ccMesh::placeIteratorAtBeginning()
{
	m_globalIterator = 0;
}

CCCoreLib::GenericTriangle* ccMesh::_getNextTriangle()
{
	if (m_globalIterator < m_triVertIndexes->size())
	{
		return _getTriangle(m_globalIterator++);
	}

	return nullptr;
}

CCCoreLib::GenericTriangle* ccMesh::_getTriangle(unsigned triangleIndex) //temporary
{
	assert(triangleIndex < m_triVertIndexes->size());

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triangleIndex);
	m_currentTriangle.A = m_associatedCloud->getPoint(tri.i1);
	m_currentTriangle.B = m_associatedCloud->getPoint(tri.i2);
	m_currentTriangle.C = m_associatedCloud->getPoint(tri.i3);

	return &m_currentTriangle;
}

void ccMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	assert(triangleIndex < m_triVertIndexes->size());

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triangleIndex);
	m_associatedCloud->getPoint(tri.i1, A);
	m_associatedCloud->getPoint(tri.i2, B);
	m_associatedCloud->getPoint(tri.i3, C);
}

void ccMesh::refreshBB()
{
	if (!m_associatedCloud || m_bBox.isValid())
		return;

	m_bBox.clear();

	size_t count = m_triVertIndexes->size();
	for (size_t i = 0; i < count; ++i)
	{
		const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->at(i);
		assert(tri.i1 < m_associatedCloud->size() && tri.i2 < m_associatedCloud->size() && tri.i3 < m_associatedCloud->size());
		m_bBox.add(*m_associatedCloud->getPoint(tri.i1));
		m_bBox.add(*m_associatedCloud->getPoint(tri.i2));
		m_bBox.add(*m_associatedCloud->getPoint(tri.i3));
	}

	notifyGeometryUpdate();
}

void ccMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	refreshBB();

	bbMin = m_bBox.minCorner();
	bbMax = m_bBox.maxCorner();
}

ccBBox ccMesh::getOwnBB(bool withGLFeatures/*=false*/)
{
	refreshBB();

	return m_bBox;
}

const ccGLMatrix& ccMesh::getGLTransformationHistory() const
{
	//DGM: it may happen that the vertices transformation history matrix is not the same as the mesh
	//(if applyGLTransformation is called directly on the vertices). Therefore we prefer the cloud's by default.
	return m_associatedCloud ? m_associatedCloud->getGLTransformationHistory() : m_glTransHistory;
}

//specific methods
void ccMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	m_triVertIndexes->emplace_back(CCCoreLib::VerticesIndexes(i1, i2, i3));
}

bool ccMesh::reserve(size_t n)
{
	if (m_triNormalIndexes)
		if (!m_triNormalIndexes->reserveSafe(n))
			return false;

	if (m_triMtlIndexes)
		if (!m_triMtlIndexes->reserveSafe(n))
			return false;

	if (m_texCoordIndexes)
		if (!m_texCoordIndexes->reserveSafe(n))
			return false;

	return m_triVertIndexes->reserveSafe(n);
}

bool ccMesh::resize(size_t n)
{
	m_bBox.setValidity(false);
	notifyGeometryUpdate();

	if (m_triMtlIndexes)
	{
		static const int s_defaultMtlIndex = -1;
		if (!m_triMtlIndexes->resizeSafe(n, true, &s_defaultMtlIndex))
			return false;
	}

	if (m_texCoordIndexes)
	{
		static const Tuple3i s_defaultTexCoords(-1, -1, -1);
		if (!m_texCoordIndexes->resizeSafe(n, true, &s_defaultTexCoords))
			return false;
	}

	if (m_triNormalIndexes)
	{
		static const Tuple3i s_defaultNormIndexes(-1, -1, -1);
		if (!m_triNormalIndexes->resizeSafe(n, true, &s_defaultNormIndexes))
			return false;
	}

	return m_triVertIndexes->resizeSafe(n);
}

void ccMesh::swapTriangles(unsigned index1, unsigned index2)
{
	assert(std::max(index1, index2) < size());

	m_triVertIndexes->swap(index1, index2);
	if (m_triMtlIndexes)
		m_triMtlIndexes->swap(index1, index2);
	if (m_texCoordIndexes)
		m_texCoordIndexes->swap(index1, index2);
	if (m_triNormalIndexes)
		m_triNormalIndexes->swap(index1, index2);
}

CCCoreLib::VerticesIndexes* ccMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	return &m_triVertIndexes->at(triangleIndex);
}

const CCCoreLib::VerticesIndexes* ccMesh::getTriangleVertIndexes(unsigned triangleIndex) const
{
	return &m_triVertIndexes->at(triangleIndex);
}

CCCoreLib::VerticesIndexes* ccMesh::getNextTriangleVertIndexes()
{
	if (m_globalIterator < m_triVertIndexes->size())
	{
		return getTriangleVertIndexes(m_globalIterator++);
	}

	return nullptr;
}

unsigned ccMesh::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
	{
		return m_parent->getParent()->getUniqueID();
	}
	else
	{
		return getUniqueID();
	}
}

void ccMesh::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_associatedCloud)
		return;

	handleColorRamp(context);

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
	{
		return;
	}

	//3D pass
	if (MACRO_Draw3D(context))
	{
		//any triangle?
		size_t triNum = m_triVertIndexes->size();
		if (triNum == 0)
		{
			return;
		}

		//L.O.D.
		bool lodEnabled = (triNum > context.minLODTriangleCount && context.decimateMeshOnMove && MACRO_LODActivated(context));
		unsigned decimStep = (lodEnabled ? static_cast<unsigned>(ceil(static_cast<double>(triNum * 3) / context.minLODTriangleCount)) : 1);

		//display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		//no normals shading without light!
		if (!MACRO_LightIsEnabled(context))
		{
			glParams.showNorms = false;
		}

		//vertices visibility
		const ccGenericPointCloud::VisibilityTableType& verticesVisibility = m_associatedCloud->getTheVisibilityArray();
		bool visFiltering = (verticesVisibility.size() >= m_associatedCloud->size());

		//wireframe ? (not compatible with LOD)
		bool showWired = isShownAsWire() && !lodEnabled;

		//per-triangle normals?
		bool showTriNormals = (hasTriNormals() && triNormsShown());
		//fix 'showNorms'
        glParams.showNorms = showTriNormals || (m_associatedCloud->hasNormals() && m_normalsDisplayed);

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
			assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
			ccPointCloud* cloud = static_cast<ccPointCloud*>(m_associatedCloud);

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

		glFunc->glPushAttrib(GL_LIGHTING_BIT | GL_TRANSFORM_BIT | GL_ENABLE_BIT);

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
		RGBAColorsTableType* rgbaColorsTable = nullptr;
		if (glParams.showColors)
		{
			if (isColorOverridden())
			{
				ccGL::Color4v(glFunc, m_tempColor.rgba);
				glParams.showColors = false;
			}
			else
			{
				assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
				rgbaColorsTable = static_cast<ccPointCloud*>(m_associatedCloud)->rgbaColors();
			}
		}
		else
		{
			ccGL::Color4v(glFunc, context.defaultMat->getDiffuseFront().rgba);
		}

		if (glParams.showNorms)
		{
			glFunc->glEnable(GL_RESCALE_NORMAL);
			glFunc->glEnable(GL_LIGHTING);
			context.defaultMat->applyGL(context.qGLContext, true, colorMaterial);
		}

		glFunc->glEnable(GL_BLEND);

		//in the case we need normals (i.e. lighting)
		NormsIndexesTableType* normalsIndexesTable = nullptr;
		ccNormalVectors* compressedNormals = nullptr;
		if (glParams.showNorms)
		{
			assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
			normalsIndexesTable = static_cast<ccPointCloud*>(m_associatedCloud)->normals();
			compressedNormals = ccNormalVectors::GetUniqueInstance();
		}

		//stipple mask
		if (m_stippling)
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
			if (glParams.showSF)
			{
				glFunc->glEnableClientState(GL_COLOR_ARRAY);
				glFunc->glColorPointer(3, GL_UNSIGNED_BYTE, 0, GetColorsBuffer());
			}
			else if (glParams.showColors)
			{
				glFunc->glEnableClientState(GL_COLOR_ARRAY);
				glFunc->glColorPointer(4, GL_UNSIGNED_BYTE, 0, GetColorsBuffer());
			}

			//we can scan and process each chunk separately in an optimized way
			size_t chunkCount = ccChunk::Count(m_triVertIndexes->size());
			for (size_t k = 0; k < chunkCount; ++k)
			{
				const size_t chunkSize = ccChunk::Size(k, m_triVertIndexes->size());
				const CCCoreLib::VerticesIndexes* _vertIndexesChunkOrigin = ccChunk::Start(*m_triVertIndexes, k);

				//vertices
				{
					const CCCoreLib::VerticesIndexes* _vertIndexes = _vertIndexesChunkOrigin;
					CCVector3* _vertices = GetVertexBuffer();
					for (size_t n = 0; n < chunkSize; n += decimStep, _vertIndexes += decimStep)
					{
						assert(_vertIndexes->i1 < m_associatedCloud->size());
						assert(_vertIndexes->i2 < m_associatedCloud->size());
						assert(_vertIndexes->i3 < m_associatedCloud->size());
						*_vertices++ = *m_associatedCloud->getPoint(_vertIndexes->i1);
						*_vertices++ = *m_associatedCloud->getPoint(_vertIndexes->i2);
						*_vertices++ = *m_associatedCloud->getPoint(_vertIndexes->i3);
					}
				}

				//scalar field
				if (glParams.showSF)
				{
					const CCCoreLib::VerticesIndexes* _vertIndexes = _vertIndexesChunkOrigin;
					ccColor::Rgb* _rgbColors = reinterpret_cast<ccColor::Rgb*>(GetColorsBuffer());
					assert(colorScale);

					for (size_t n = 0; n < chunkSize; n += decimStep, _vertIndexes += decimStep)
					{
						assert(_vertIndexes->i1 < currentDisplayedScalarField->size());
						assert(_vertIndexes->i2 < currentDisplayedScalarField->size());
						assert(_vertIndexes->i3 < currentDisplayedScalarField->size());
						*_rgbColors++ = *currentDisplayedScalarField->getValueColor(_vertIndexes->i1);
						*_rgbColors++ = *currentDisplayedScalarField->getValueColor(_vertIndexes->i2);
						*_rgbColors++ = *currentDisplayedScalarField->getValueColor(_vertIndexes->i3);
					}
				}
				//colors
				else if (glParams.showColors)
				{
					const CCCoreLib::VerticesIndexes* _vertIndexes = _vertIndexesChunkOrigin;
					ccColor::Rgba* _rgbaColors = reinterpret_cast<ccColor::Rgba*>(GetColorsBuffer());
					for (size_t n = 0; n < chunkSize; n += decimStep, _vertIndexes += decimStep)
					{
						assert(_vertIndexes->i1 < rgbaColorsTable->size());
						assert(_vertIndexes->i2 < rgbaColorsTable->size());
						assert(_vertIndexes->i3 < rgbaColorsTable->size());
						*(_rgbaColors)++ = rgbaColorsTable->at(_vertIndexes->i1);
						*(_rgbaColors)++ = rgbaColorsTable->at(_vertIndexes->i2);
						*(_rgbaColors)++ = rgbaColorsTable->at(_vertIndexes->i3);
					}
				}

				//normals
				if (glParams.showNorms)
				{
					CCVector3* _normals = GetNormalsBuffer();
					if (showTriNormals)
					{
						assert(m_triNormalIndexes);
						const Tuple3i* _triNormalIndexes = ccChunk::Start(*m_triNormalIndexes, k);
						for (size_t n = 0; n < chunkSize; n += decimStep, _triNormalIndexes += decimStep)
						{
							assert(_triNormalIndexes->u[0] < static_cast<int>(m_triNormals->size()));
							assert(_triNormalIndexes->u[1] < static_cast<int>(m_triNormals->size()));
							assert(_triNormalIndexes->u[2] < static_cast<int>(m_triNormals->size()));

							*_normals++ = (_triNormalIndexes->u[0] >= 0 ? compressedNormals->getNormal(m_triNormals->at(_triNormalIndexes->u[0])) : s_blankNorm);
							*_normals++ = (_triNormalIndexes->u[1] >= 0 ? compressedNormals->getNormal(m_triNormals->at(_triNormalIndexes->u[1])) : s_blankNorm);
							*_normals++ = (_triNormalIndexes->u[2] >= 0 ? compressedNormals->getNormal(m_triNormals->at(_triNormalIndexes->u[2])) : s_blankNorm);
						}
					}
					else
					{
						const CCCoreLib::VerticesIndexes* _vertIndexes = _vertIndexesChunkOrigin;
						for (size_t n = 0; n < chunkSize; n += decimStep, _vertIndexes += decimStep)
						{
							assert(_vertIndexes->i1 < normalsIndexesTable->size());
							assert(_vertIndexes->i2 < normalsIndexesTable->size());
							assert(_vertIndexes->i3 < normalsIndexesTable->size());
							*_normals++ = compressedNormals->getNormal(normalsIndexesTable->at(_vertIndexes->i1));
							*_normals++ = compressedNormals->getNormal(normalsIndexesTable->at(_vertIndexes->i2));
							*_normals++ = compressedNormals->getNormal(normalsIndexesTable->at(_vertIndexes->i3));
						}
					}
				}

				if (!showWired)
				{
					glFunc->glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES, 0, (static_cast<int>(chunkSize) / decimStep) * 3);
				}
				else
				{
					glFunc->glDrawElements(GL_LINES, (static_cast<int>(chunkSize) / decimStep) * 6, GL_UNSIGNED_INT, GetWireVertexIndexes());
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
			//current vertex color (RGB)
			const ccColor::Rgb* rgb1 = nullptr;
			const ccColor::Rgb* rgb2 = nullptr;
			const ccColor::Rgb* rgb3 = nullptr;
			//current vertex color (RGBA)
			const ccColor::Rgba* rgba1 = nullptr;
			const ccColor::Rgba* rgba2 = nullptr;
			const ccColor::Rgba* rgba3 = nullptr;
			//current vertex normal
			const PointCoordinateType* N1 = nullptr;
			const PointCoordinateType* N2 = nullptr;
			const PointCoordinateType* N3 = nullptr;
			//current vertex texture coordinates
			const TexCoords2D* Tx1 = nullptr;
			const TexCoords2D* Tx2 = nullptr;
			const TexCoords2D* Tx3 = nullptr;

			if (showTextures)
			{
				glFunc->glEnable(GL_TEXTURE_2D);
			}

			GLenum triangleDisplayType = lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP : GL_TRIANGLES;
			glFunc->glBegin(triangleDisplayType);

			GLuint currentTexID = 0;
			int lasMtlIndex = -1;

			//loop on all triangles
			for (size_t n = 0; n < triNum; ++n)
			{
				//LOD: shall we display this triangle?
				if (n % decimStep)
				{
					continue;
				}

				//current triangle vertices
				const CCCoreLib::VerticesIndexes& tsi = m_triVertIndexes->at(n);

				if (visFiltering)
				{
					//we skip the triangle if at least one vertex is hidden
					if ((verticesVisibility[tsi.i1] != CCCoreLib::POINT_VISIBLE) ||
						(verticesVisibility[tsi.i2] != CCCoreLib::POINT_VISIBLE) ||
						(verticesVisibility[tsi.i3] != CCCoreLib::POINT_VISIBLE))
						continue;
				}

				if (glParams.showSF)
				{
					assert(colorScale);
					rgb1 = currentDisplayedScalarField->getValueColor(tsi.i1);
					if (!rgb1)
						continue;
					rgb2 = currentDisplayedScalarField->getValueColor(tsi.i2);
					if (!rgb2)
						continue;
					rgb3 = currentDisplayedScalarField->getValueColor(tsi.i3);
					if (!rgb3)
						continue;
				}
				else if (glParams.showColors)
				{
					rgba1 = &rgbaColorsTable->at(tsi.i1);
					rgba2 = &rgbaColorsTable->at(tsi.i2);
					rgba3 = &rgbaColorsTable->at(tsi.i3);
				}

				if (glParams.showNorms)
				{
					if (showTriNormals)
					{
						assert(m_triNormalIndexes);
						const Tuple3i& idx = m_triNormalIndexes->at(n);
						assert(idx.u[0] < static_cast<int>(m_triNormals->size()));
						assert(idx.u[1] < static_cast<int>(m_triNormals->size()));
						assert(idx.u[2] < static_cast<int>(m_triNormals->size()));
						N1 = (idx.u[0] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx.u[0])).u : nullptr);
						N2 = (idx.u[0] == idx.u[1] ? N1 : idx.u[1] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx.u[1])).u : nullptr);
						N3 = (idx.u[0] == idx.u[2] ? N1 : idx.u[2] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx.u[2])).u : nullptr);
					}
					else
					{
						N1 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi.i1)).u;
						N2 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi.i2)).u;
						N3 = compressedNormals->getNormal(normalsIndexesTable->getValue(tsi.i3)).u;
					}
				}

				if (applyMaterials || showTextures)
				{
					assert(m_materials);
					int newMatlIndex = m_triMtlIndexes->getValue(n);

					//do we need to change material?
					if (lasMtlIndex != newMatlIndex)
					{
						assert(newMatlIndex < static_cast<int>(m_materials->size()));
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
								currentTexID = m_materials->at(newMatlIndex)->getTextureID();
								if (currentTexID)
								{
									glFunc->glBindTexture(GL_TEXTURE_2D, currentTexID);
								}
							}
						}

						//if we don't have any current material, we apply default one
						if (newMatlIndex >= 0)
							(*m_materials)[newMatlIndex]->applyGL(context.qGLContext, glParams.showNorms, false);
						else
							context.defaultMat->applyGL(context.qGLContext, glParams.showNorms, false);
						
						glFunc->glBegin(triangleDisplayType);
						lasMtlIndex = newMatlIndex;
					}

					if (showTextures)
					{
						assert(m_texCoords && m_texCoordIndexes);
						const Tuple3i& txInd = m_texCoordIndexes->getValue(n);
						assert(txInd.u[0] < static_cast<int>(m_texCoords->size()));
						assert(txInd.u[1] < static_cast<int>(m_texCoords->size()));
						assert(txInd.u[2] < static_cast<int>(m_texCoords->size()));
						Tx1 = (txInd.u[0] >= 0 ? &m_texCoords->getValue(txInd.u[0]) : nullptr);
						Tx2 = (txInd.u[1] >= 0 ? &m_texCoords->getValue(txInd.u[1]) : nullptr);
						Tx3 = (txInd.u[2] >= 0 ? &m_texCoords->getValue(txInd.u[2]) : nullptr);
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
				if (rgb1)
					glFunc->glColor3ubv(rgb1->rgb);
				else if (rgba1)
					glFunc->glColor4ubv(rgba1->rgba);
				if (Tx1)
					glFunc->glTexCoord2fv(Tx1->t);
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi.i1)->u);

				//vertex 2
				if (N2)
					ccGL::Normal3v(glFunc, N2);
				if (rgb2)
					glFunc->glColor3ubv(rgb2->rgb);
				else if (rgba2)
					glFunc->glColor4ubv(rgba2->rgba);
				if (Tx2)
					glFunc->glTexCoord2fv(Tx2->t);
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi.i2)->u);

				//vertex 3
				if (N3)
					ccGL::Normal3v(glFunc, N3);
				if (rgb3)
					glFunc->glColor3ubv(rgb3->rgb);
				else if (rgba3)
					glFunc->glColor4ubv(rgba3->rgba);
				if (Tx3)
					glFunc->glTexCoord2fv(Tx3->t);
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi.i3)->u);
			}

			glFunc->glEnd();

			if (showTextures)
			{
				if (currentTexID)
				{
					glFunc->glBindTexture(GL_TEXTURE_2D, 0);
					currentTexID = 0;
				}
			}
		}

		if (m_stippling)
		{
			EnableGLStippleMask(context.qGLContext, false);
		}

		glFunc->glPopAttrib(); // GL_LIGHTING_BIT | GL_TRANSFORM_BIT | GL_ENABLE_BIT

		if (pushName)
		{
			glFunc->glPopName();
		}
	}
}

ccMesh* ccMesh::createNewMeshFromSelection(bool removeSelectedFaces)
{
	if (!m_associatedCloud)
	{
		return nullptr;
	}

	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = m_associatedCloud->getTheVisibilityArray();
	if (verticesVisibility.size() < m_associatedCloud->size())
	{
		ccLog::Error(QString("[Mesh %1] Internal error: vertex visibility table not instantiated!").arg(getName()));
		return nullptr;
	}

	//create vertices for the new mesh
	ccGenericPointCloud* newVertices = m_associatedCloud->createNewCloudFromVisibilitySelection(false, nullptr, true);
	if (!newVertices)
	{
		ccLog::Error(QString("[Mesh %1] Failed to create segmented mesh vertices! (not enough memory)").arg(getName()));
		return nullptr;
	}
	else if (newVertices->size() == 0)
	{
		ccLog::Error(QString("[Mesh %1] Failed to create segmented mesh vertices! (no visible point in selection)").arg(getName()));
		delete newVertices;
		return nullptr;
	}
	assert(newVertices);

	//create a 'reference' cloud if none was provided
	QSharedPointer<CCCoreLib::ReferenceCloud> rc;
	{
		//we create a temporary entity with the visible vertices only
		rc.reset(new CCCoreLib::ReferenceCloud(m_associatedCloud));

		for (unsigned i = 0; i < m_associatedCloud->size(); ++i)
			if (verticesVisibility[i] == CCCoreLib::POINT_VISIBLE)
				if (!rc->addPointIndex(i))
				{
					ccLog::Error("Not enough memory!");
					return nullptr;
				}
	}

	//nothing to do
	if (rc->size() == 0 || (removeSelectedFaces && rc->size() == m_associatedCloud->size()))
	{
		return nullptr;
	}

	//we create a new mesh with the current selection
	CCCoreLib::GenericIndexedMesh* result = CCCoreLib::ManualSegmentationTools::segmentMesh(this, rc.data(), true, nullptr, newVertices);

	//don't use this anymore
	rc.clear();

	ccMesh* newMesh = nullptr;
	if (result)
	{
		newMesh = new ccMesh(result, newVertices);
		if (!newMesh)
		{
			delete newVertices;
			newVertices = nullptr;
			ccLog::Error("An error occurred: not enough memory?");
		}
		else
		{
			newMesh->setName(getName() + QString(".part"));

			//shall we add any advanced features?
			bool addFeatures = false;
			if (m_triNormals && m_triNormalIndexes)
				addFeatures |= newMesh->reservePerTriangleNormalIndexes();
			if (m_materials && m_triMtlIndexes)
				addFeatures |= newMesh->reservePerTriangleMtlIndexes();
			if (m_texCoords && m_texCoordIndexes)
				addFeatures |= newMesh->reservePerTriangleTexCoordIndexes();

			if (addFeatures)
			{
				//temporary structure for normal indexes mapping
				std::vector<int> newNormIndexes;
				NormsIndexesTableType* newTriNormals = nullptr;
				if (m_triNormals && m_triNormalIndexes)
				{
					assert(m_triNormalIndexes->size() == m_triVertIndexes->size());
					//create new 'minimal' subset
					newTriNormals = new NormsIndexesTableType();
					newTriNormals->link();
					try
					{
						newNormIndexes.resize(m_triNormals->size(), -1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new normals subset! (not enough memory)");
						newMesh->removePerTriangleNormalIndexes();
						newTriNormals->release();
						newTriNormals = nullptr;
					}
				}

				//temporary structure for texture indexes mapping
				std::vector<int> newTexIndexes;
				TextureCoordsContainer* newTriTexIndexes = nullptr;
				if (m_texCoords && m_texCoordIndexes)
				{
					assert(m_texCoordIndexes->size() == m_triVertIndexes->size());
					//create new 'minimal' subset
					newTriTexIndexes = new TextureCoordsContainer();
					newTriTexIndexes->link();
					try
					{
						newTexIndexes.resize(m_texCoords->size(), -1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new texture indexes subset! (not enough memory)");
						newMesh->removePerTriangleTexCoordIndexes();
						newTriTexIndexes->release();
						newTriTexIndexes = nullptr;
					}
				}

				//temporary structure for material indexes mapping
				std::vector<int> newMatIndexes;
				ccMaterialSet* newMaterials = nullptr;
				if (m_materials && m_triMtlIndexes)
				{
					assert(m_triMtlIndexes->size() == m_triVertIndexes->size());
					//create new 'minimal' subset
					newMaterials = new ccMaterialSet(m_materials->getName() + QString(".subset"));
					newMaterials->link();
					try
					{
						newMatIndexes.resize(m_materials->size(), -1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new material subset! (not enough memory)");
						newMesh->removePerTriangleMtlIndexes();
						newMaterials->release();
						newMaterials = nullptr;
						if (newTriTexIndexes) //we can release texture coordinates as well (as they depend on materials!)
						{
							newMesh->removePerTriangleTexCoordIndexes();
							newTriTexIndexes->release();
							newTriTexIndexes = nullptr;
							newTexIndexes.resize(0);
						}
					}
				}

				size_t triNum = m_triVertIndexes->size();
				for (size_t i = 0; i < triNum; ++i)
				{
					const CCCoreLib::VerticesIndexes& tsi = m_triVertIndexes->at(i);

					//all vertices must be visible
					if (verticesVisibility[tsi.i1] == CCCoreLib::POINT_VISIBLE &&
						verticesVisibility[tsi.i2] == CCCoreLib::POINT_VISIBLE &&
						verticesVisibility[tsi.i3] == CCCoreLib::POINT_VISIBLE)
					{
						//import per-triangle normals?
						if (newTriNormals)
						{
							assert(m_triNormalIndexes);

							//current triangle (compressed) normal indexes
							const Tuple3i& triNormIndexes = m_triNormalIndexes->getValue(i);

							//for each triangle of this mesh, try to determine if its normals are already in use
							//(otherwise add them to the new container and increase its index)
							for (unsigned j = 0; j < 3; ++j)
							{
								if (triNormIndexes.u[j] >= 0 && newNormIndexes[triNormIndexes.u[j]] < 0)
								{
									if (newTriNormals->size() == newTriNormals->capacity() 
										&& !newTriNormals->reserveSafe(newTriNormals->size() + 4096)) //auto expand
									{
										ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new normals subset! (not enough memory)");
										newMesh->removePerTriangleNormalIndexes();
										newTriNormals->release();
										newTriNormals = nullptr;
										break;
									}

									//import old normal to new subset (create new index)
									newNormIndexes[triNormIndexes.u[j]] = static_cast<int>(newTriNormals->size()); //new element index = new size - 1 = old size!
									newTriNormals->emplace_back(m_triNormals->getValue(triNormIndexes.u[j]));
								}
							}

							if (newTriNormals) //structure still exists?
							{
								newMesh->addTriangleNormalIndexes(	triNormIndexes.u[0] < 0 ? -1 : newNormIndexes[triNormIndexes.u[0]],
																	triNormIndexes.u[1] < 0 ? -1 : newNormIndexes[triNormIndexes.u[1]],
																	triNormIndexes.u[2] < 0 ? -1 : newNormIndexes[triNormIndexes.u[2]]);
							}
						}

						//import texture coordinates?
						if (newTriTexIndexes)
						{
							assert(m_texCoordIndexes);

							//current triangle texture coordinates indexes
							const Tuple3i& triTexIndexes = m_texCoordIndexes->getValue(i);

							//for each triangle of this mesh, try to determine if its textures coordinates are already in use
							//(otherwise add them to the new container and increase its index)
							for (unsigned j = 0; j < 3; ++j)
							{
								if (triTexIndexes.u[j] >= 0 && newTexIndexes[triTexIndexes.u[j]] < 0)
								{
									if (newTriTexIndexes->size() == newTriTexIndexes->capacity() 
										&& !newTriTexIndexes->reserveSafe(newTriTexIndexes->size() + 4096)) //auto expand
									{
										ccLog::Error("Failed to create new texture coordinates subset! (not enough memory)");
										newMesh->removePerTriangleTexCoordIndexes();
										newTriTexIndexes->release();
										newTriTexIndexes = nullptr;
										break;
									}
									//import old texture coordinate to new subset (create new index)
									newTexIndexes[triTexIndexes.u[j]] = static_cast<int>(newTriTexIndexes->size()); //new element index = new size - 1 = old size!
									newTriTexIndexes->emplace_back(m_texCoords->getValue(triTexIndexes.u[j]));
								}
							}

							if (newTriTexIndexes) //structure still exists?
							{
								newMesh->addTriangleTexCoordIndexes(triTexIndexes.u[0] < 0 ? -1 : newTexIndexes[triTexIndexes.u[0]],
																	triTexIndexes.u[1] < 0 ? -1 : newTexIndexes[triTexIndexes.u[1]],
																	triTexIndexes.u[2] < 0 ? -1 : newTexIndexes[triTexIndexes.u[2]]);
							}
						}

						//import materials?
						if (newMaterials)
						{
							assert(m_triMtlIndexes);

							//current triangle material index
							const int triMatIndex = m_triMtlIndexes->getValue(i);

							//for each triangle of this mesh, try to determine if its material is already in use
							//(otherwise add it to the new container and increase its index)
							if (triMatIndex >= 0 && newMatIndexes[triMatIndex] < 0)
							{
								//import old material to new subset (create new index)
								newMatIndexes[triMatIndex] = static_cast<int>(newMaterials->size()); //new element index = new size - 1 = old size!
								try
								{
									newMaterials->emplace_back(m_materials->at(triMatIndex));
								}
								catch (const std::bad_alloc&)
								{
									ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new materials subset! (not enough memory)");
									newMesh->removePerTriangleMtlIndexes();
									newMaterials->release();
									newMaterials = nullptr;
								}
							}

							if (newMaterials) //structure still exists?
							{
								newMesh->addTriangleMtlIndex(triMatIndex < 0 ? -1 : newMatIndexes[triMatIndex]);
							}
						}

					}
				}

				if (newTriNormals)
				{
					newTriNormals->resize(newTriNormals->size()); //smaller so it should always be ok!
					newMesh->setTriNormsTable(newTriNormals);
					newTriNormals->release();
					newTriNormals = nullptr;
				}

				if (newTriTexIndexes)
				{
					newMesh->setTexCoordinatesTable(newTriTexIndexes);
					newTriTexIndexes->release();
					newTriTexIndexes = nullptr;
				}

				if (newMaterials)
				{
					newMesh->setMaterialSet(newMaterials);
					newMaterials->release();
					newMaterials = nullptr;
				}
			}

			newMesh->addChild(newVertices);
			newMesh->setDisplay_recursive(getDisplay());
			newMesh->showColors(colorsShown());
			newMesh->showNormals(normalsShown());
			newMesh->showMaterials(materialsShown());
			newMesh->showSF(sfShown());
			newMesh->importParametersFrom(this);

			newVertices->setEnabled(false);
		}

		delete result;
		result = nullptr;
	}

	size_t triNum = m_triVertIndexes->size();

	//we must modify eventual sub-meshes!
	ccHObject::Container subMeshes;
	if (filterChildren(subMeshes, false, CC_TYPES::SUB_MESH) != 0)
	{
		ccLog::Warning("Has sub-meshes!");

		//create index map
		ccSubMesh::IndexMap indexMap;
		try
		{
			indexMap.reserve(triNum);

			//finish index map creation
			{
				unsigned newVisibleIndex = 0;
				unsigned newInvisibleIndex = 0;

				for (size_t i = 0; i < triNum; ++i)
				{
					const CCCoreLib::VerticesIndexes& tsi = m_triVertIndexes->at(i);

					//at least one hidden vertex --> we keep it
					if (verticesVisibility[tsi.i1] != CCCoreLib::POINT_VISIBLE ||
						verticesVisibility[tsi.i2] != CCCoreLib::POINT_VISIBLE ||
						verticesVisibility[tsi.i3] != CCCoreLib::POINT_VISIBLE)
					{
						indexMap.emplace_back(removeSelectedFaces ? newInvisibleIndex++ : static_cast<unsigned>(i));
					}
					else
					{
						indexMap.emplace_back(newVisibleIndex++);
					}
				}
			}

			for (size_t i = 0; i < subMeshes.size(); ++i)
			{
				ccSubMesh* subMesh = static_cast<ccSubMesh*>(subMeshes[i]);
				ccSubMesh* subMesh2 = subMesh->createNewSubMeshFromSelection(removeSelectedFaces, &indexMap);

				if (subMesh->size() == 0) //no more faces in current sub-mesh?
				{
					detachChild(subMesh); //FIXME: removeChild instead?
					subMesh = nullptr;
				}

				if (subMesh2 && newMesh)
				{
					subMesh2->setAssociatedMesh(newMesh);
					newMesh->addChild(subMesh2);
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory! Sub-meshes will be lost...");
			if (newMesh)
			{
				newMesh->setVisible(true); //force parent mesh visibility in this case!
			}

			for (size_t i = 0; i < subMeshes.size(); ++i)
			{
				removeChild(subMeshes[i]);
			}
		}
	}

	//shall we remove the selected faces from this mesh?
	if (removeSelectedFaces)
	{
		ccLog::Warning("Remove faces!");

		//we remove all fully visible faces
		size_t lastTri = 0;
		for (size_t i = 0; i < triNum; ++i)
		{
			const CCCoreLib::VerticesIndexes& tsi = m_triVertIndexes->at(i);

			//at least one hidden vertex --> we keep it
			if (verticesVisibility[tsi.i1] != CCCoreLib::POINT_VISIBLE ||
				verticesVisibility[tsi.i2] != CCCoreLib::POINT_VISIBLE ||
				verticesVisibility[tsi.i3] != CCCoreLib::POINT_VISIBLE)
			{
				if (i != lastTri)
				{
					m_triVertIndexes->setValue(lastTri, tsi);

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
		notifyGeometryUpdate();
	}

	return newMesh;
}

void ccMesh::shiftTriangleIndexes(unsigned shift)
{
	for (CCCoreLib::VerticesIndexes& ti : *m_triVertIndexes)
	{
		ti.i1 += shift;
		ti.i2 += shift;
		ti.i3 += shift;
	}
}

void ccMesh::flipTriangles()
{
	for (CCCoreLib::VerticesIndexes& ti : *m_triVertIndexes)
	{
		std::swap(ti.i2, ti.i3);
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
	m_triNormalIndexes = nullptr;
}

bool ccMesh::reservePerTriangleNormalIndexes()
{
	assert(!m_triNormalIndexes); //try to avoid doing this twice!
	if (!m_triNormalIndexes)
	{
		m_triNormalIndexes = new triangleNormalsIndexesSet();
		m_triNormalIndexes->link();
	}

	assert(m_triVertIndexes && m_triVertIndexes->isAllocated());

	return m_triNormalIndexes->reserveSafe(m_triVertIndexes->capacity());
}

void ccMesh::invertNormals()
{
	//per-triangle normals
	if (m_triNormals)
	{
		invertPerTriangleNormals();
	}

	//per-vertex normals
	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_associatedCloud);
	if (pc && pc->hasNormals())
	{
		pc->invertNormals();
	}
}

void ccMesh::invertPerTriangleNormals()
{
	if (m_triNormals)
	{
		for (CompressedNormType& n : *m_triNormals)
		{
			ccNormalCompressor::InvertNormal(n);
		}
	}
}

void ccMesh::addTriangleNormalIndexes(int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	m_triNormalIndexes->emplace_back(Tuple3i(i1, i2, i3));
}

void ccMesh::setTriangleNormalIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->size() > triangleIndex);
	m_triNormalIndexes->setValue(triangleIndex, Tuple3i(i1, i2, i3));
}

void ccMesh::getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const
{
	if (m_triNormalIndexes && m_triNormalIndexes->size() > triangleIndex)
	{
		const Tuple3i& indexes = m_triNormalIndexes->getValue(triangleIndex);
		i1 = indexes.u[0];
		i2 = indexes.u[1];
		i3 = indexes.u[2];
	}
	else
	{
		i1 = i2 = i3 = -1;
	}
}

bool ccMesh::getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const
{
	if (m_triNormals && m_triNormalIndexes && m_triNormalIndexes->size() > triangleIndex)
	{
		const Tuple3i& indexes = m_triNormalIndexes->getValue(triangleIndex);
		if (indexes.u[0] >= 0)
			Na = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes.u[0]));
		else
			Na = CCVector3(0, 0, 0);
		if (indexes.u[1] >= 0)
			Nb = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes.u[1]));
		else
			Nb = CCVector3(0, 0, 0);
		if (indexes.u[2] >= 0)
			Nc = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes.u[2]));
		else
			Nc = CCVector3(0, 0, 0);

		return true;
	}

	return false;
}

bool ccMesh::hasTriNormals() const
{
	return m_triNormals && m_triNormals->isAllocated() && m_triNormalIndexes && (m_triNormalIndexes->size() == m_triVertIndexes->size());
}

/*********************************************************/
/************    PER-TRIANGLE TEX COORDS    **************/
/*********************************************************/

void ccMesh::setTexCoordinatesTable(TextureCoordsContainer* texCoordsTable, bool autoReleaseOldTable/*=true*/)
{
	if (m_texCoords == texCoordsTable)
		return;

	if (m_texCoords && autoReleaseOldTable)
	{
		int childIndex = getChildIndex(m_texCoords);
		m_texCoords->release();
		m_texCoords = nullptr;
		if (childIndex >= 0)
			removeChild(childIndex);
	}

	m_texCoords = texCoordsTable;
	if (m_texCoords)
	{
		m_texCoords->link();
		int childIndex = getChildIndex(m_texCoords);
		if (childIndex < 0)
			addChild(m_texCoords);
	}
	else
	{
		removePerTriangleTexCoordIndexes(); //auto-remove per-triangle indexes (we don't need them anymore)
	}
}

void ccMesh::getTriangleTexCoordinates(unsigned triIndex, TexCoords2D* &tx1, TexCoords2D* &tx2, TexCoords2D* &tx3) const
{
	if (m_texCoords && m_texCoordIndexes)
	{
		const Tuple3i& txInd = m_texCoordIndexes->getValue(triIndex);
		tx1 = (txInd.u[0] >= 0 ? &m_texCoords->getValue(txInd.u[0]) : nullptr);
		tx2 = (txInd.u[1] >= 0 ? &m_texCoords->getValue(txInd.u[1]) : nullptr);
		tx3 = (txInd.u[2] >= 0 ? &m_texCoords->getValue(txInd.u[2]) : nullptr);
	}
	else
	{
		tx1 = tx2 = tx3;
	}
}

bool ccMesh::reservePerTriangleTexCoordIndexes()
{
	assert(!m_texCoordIndexes); //try to avoid doing this twice!
	if (!m_texCoordIndexes)
	{
		m_texCoordIndexes = new triangleTexCoordIndexesSet();
		m_texCoordIndexes->link();
	}

	assert(m_triVertIndexes && m_triVertIndexes->isAllocated());

	return m_texCoordIndexes->reserveSafe(m_triVertIndexes->capacity());
}

void ccMesh::removePerTriangleTexCoordIndexes()
{
	triangleTexCoordIndexesSet* texCoordIndexes = m_texCoordIndexes;
	m_texCoordIndexes = nullptr;

	if (texCoordIndexes)
		texCoordIndexes->release();
}

void ccMesh::addTriangleTexCoordIndexes(int i1, int i2, int i3)
{
	assert(m_texCoordIndexes && m_texCoordIndexes->isAllocated());
	m_texCoordIndexes->emplace_back(Tuple3i(i1, i2, i3));
}

void ccMesh::setTriangleTexCoordIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_texCoordIndexes && m_texCoordIndexes->size() > triangleIndex);
	m_texCoordIndexes->setValue(triangleIndex, Tuple3i(i1, i2, i3));
}

void ccMesh::getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const
{
	assert(m_texCoordIndexes && m_texCoordIndexes->size() > triangleIndex);

	const Tuple3i& tci = m_texCoordIndexes->getValue(triangleIndex);
	i1 = tci.u[0];
	i2 = tci.u[1];
	i3 = tci.u[2];
}

bool ccMesh::hasTextures() const
{
	return hasMaterials() && m_texCoords && m_texCoords->isAllocated() && m_texCoordIndexes && (m_texCoordIndexes->size() == m_triVertIndexes->size());
}

/*********************************************************/
/**************    PER-TRIANGLE MATERIALS    *************/
/*********************************************************/

bool ccMesh::hasMaterials() const
{
	return m_materials && !m_materials->empty() && m_triMtlIndexes && (m_triMtlIndexes->size() == m_triVertIndexes->size());
}

void ccMesh::setTriangleMtlIndexesTable(triangleMaterialIndexesSet* matIndexesTable, bool autoReleaseOldTable/*=true*/)
{
	if (m_triMtlIndexes == matIndexesTable)
		return;

	if (m_triMtlIndexes && autoReleaseOldTable)
	{
		m_triMtlIndexes->release();
		m_triMtlIndexes = nullptr;
	}

	m_triMtlIndexes = matIndexesTable;
	if (m_triMtlIndexes)
	{
		m_triMtlIndexes->link();
	}
}

bool ccMesh::reservePerTriangleMtlIndexes()
{
	assert(!m_triMtlIndexes); //try to avoid doing this twice!
	if (!m_triMtlIndexes)
	{
		m_triMtlIndexes = new triangleMaterialIndexesSet();
		m_triMtlIndexes->link();
	}

	assert(m_triVertIndexes && m_triVertIndexes->isAllocated());

	return m_triMtlIndexes->reserveSafe(m_triVertIndexes->capacity());
}

void ccMesh::removePerTriangleMtlIndexes()
{
	if (m_triMtlIndexes)
		m_triMtlIndexes->release();
	m_triMtlIndexes = nullptr;
}

void ccMesh::addTriangleMtlIndex(int mtlIndex)
{
	assert(m_triMtlIndexes && m_triMtlIndexes->isAllocated());
	m_triMtlIndexes->emplace_back(mtlIndex);
}

void ccMesh::setTriangleMtlIndex(unsigned triangleIndex, int mtlIndex)
{
	assert(m_triMtlIndexes && m_triMtlIndexes->size() > triangleIndex);
	m_triMtlIndexes->setValue(triangleIndex, mtlIndex);
}

int ccMesh::getTriangleMtlIndex(unsigned triangleIndex) const
{
	assert(m_triMtlIndexes && m_triMtlIndexes->size() > triangleIndex);
	return m_triMtlIndexes->at(triangleIndex);
}

bool ccMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericMesh::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple meshes)
	//so instead we save it's unique ID (dataVersion>=20)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	uint32_t vertUniqueID = (m_associatedCloud ? static_cast<uint32_t>(m_associatedCloud->getUniqueID()) : 0);
	if (out.write((const char*)&vertUniqueID, 4) < 0)
		return WriteError();

	//per-triangle normals array (dataVersion>=20)
	{
		//we can't save the normals array here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the normals array must be saved in the same BIN file! (responsibility of the caller)
		uint32_t normArrayID = (m_triNormals && m_triNormals->isAllocated() ? static_cast<uint32_t>(m_triNormals->getUniqueID()) : 0);
		if (out.write((const char*)&normArrayID, 4) < 0)
			return WriteError();
	}

	//texture coordinates array (dataVersion>=20)
	{
		//we can't save the texture coordinates array here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the texture coordinates array must be saved in the same BIN file! (responsibility of the caller)
		uint32_t texCoordArrayID = (m_texCoords && m_texCoords->isAllocated() ? static_cast<uint32_t>(m_texCoords->getUniqueID()) : 0);
		if (out.write((const char*)&texCoordArrayID, 4) < 0)
			return WriteError();
	}

	//materials
	{
		//we can't save the material set here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the material set must be saved in the same BIN file! (responsibility of the caller)
		uint32_t matSetID = (m_materials ? static_cast<uint32_t>(m_materials->getUniqueID()) : 0);
		if (out.write((const char*)&matSetID, 4) < 0)
			return WriteError();
	}

	//triangles indexes (dataVersion>=20)
	if (!m_triVertIndexes)
		return ccLog::Error("Internal error: mesh has no triangles array! (not enough memory?)");
	if (!ccSerializationHelper::GenericArrayToFile<CCCoreLib::VerticesIndexes, 3, unsigned>(*m_triVertIndexes, out))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = hasPerTriangleMtlIndexes();
	if (out.write((const char*)&hasTriMtlIndexes, sizeof(bool)) < 0)
		return WriteError();
	if (hasTriMtlIndexes)
	{
		assert(m_triMtlIndexes);
		if (!ccSerializationHelper::GenericArrayToFile<int, 1, int>(*m_triMtlIndexes, out))
			return false;
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = hasPerTriangleTexCoordIndexes();
	if (out.write((const char*)&hasTexCoordIndexes, sizeof(bool)) < 0)
		return WriteError();
	if (hasTexCoordIndexes)
	{
		assert(m_texCoordIndexes);
		if (!ccSerializationHelper::GenericArrayToFile<Tuple3i, 3, int>(*m_texCoordIndexes, out))
			return false;
	}

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = (m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	if (out.write((const char*)&hasTriNormalIndexes, sizeof(bool)) < 0)
		return WriteError();
	if (hasTriNormalIndexes)
	{
		assert(m_triNormalIndexes);
		if (!ccSerializationHelper::GenericArrayToFile<Tuple3i, 3, int>(*m_triNormalIndexes, out))
			return false;
	}

	return true;
}

bool ccMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericMesh::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by multiple meshes)
	//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_associatedCloud) = vertUniqueID;

	//per-triangle normals array (dataVersion>=20)
	{
		//as the associated normals array can't be saved directly (as it may be shared by multiple meshes)
		//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
		//is the responsibility of the caller to make sure that all dependencies are saved together)
		uint32_t normArrayID = 0;
		if (in.read((char*)&normArrayID, 4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the array unique ID in the 'm_triNormals' pointer!!!
		*(uint32_t*)(&m_triNormals) = normArrayID;
	}

	//texture coordinates array (dataVersion>=20)
	{
		//as the associated texture coordinates array can't be saved directly (as it may be shared by multiple meshes)
		//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
		//is the responsibility of the caller to make sure that all dependencies are saved together)
		uint32_t texCoordArrayID = 0;
		if (in.read((char*)&texCoordArrayID, 4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the array unique ID in the 'm_texCoords' pointer!!!
		*(uint32_t*)(&m_texCoords) = texCoordArrayID;
	}

	//materials
	{
		//as the associated materials can't be saved directly (as it may be shared by multiple meshes)
		//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
		//is the responsibility of the caller to make sure that all dependencies are saved together)
		uint32_t matSetID = 0;
		if (in.read((char*)&matSetID, 4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the array unique ID in the 'm_materials' pointer!!!
		*(uint32_t*)(&m_materials) = matSetID;
	}

	//triangles indexes (dataVersion>=20)
	if (!m_triVertIndexes)
		return false;
	if (!ccSerializationHelper::GenericArrayFromFile<CCCoreLib::VerticesIndexes, 3, unsigned>(*m_triVertIndexes, in, dataVersion))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = false;
	if (in.read((char*)&hasTriMtlIndexes, sizeof(bool)) < 0)
		return ReadError();
	if (hasTriMtlIndexes)
	{
		if (!m_triMtlIndexes)
		{
			m_triMtlIndexes = new triangleMaterialIndexesSet();
			m_triMtlIndexes->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile<int, 1, int>(*m_triMtlIndexes, in, dataVersion))
		{
			m_triMtlIndexes->release();
			m_triMtlIndexes = nullptr;
			return false;
		}
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = false;
	if (in.read((char*)&hasTexCoordIndexes, sizeof(bool)) < 0)
		return ReadError();
	if (hasTexCoordIndexes)
	{
		if (!m_texCoordIndexes)
		{
			m_texCoordIndexes = new triangleTexCoordIndexesSet();
			m_texCoordIndexes->link();
		}
		if (!ccSerializationHelper::GenericArrayFromFile<Tuple3i, 3, int>(*m_texCoordIndexes, in, dataVersion))
		{
			m_texCoordIndexes->release();
			m_texCoordIndexes = nullptr;
			return false;
		}
	}

	//'materials shown' state (dataVersion>=20 && dataVersion<29))
	if (dataVersion < 29)
	{
		bool materialsShown = false;
		if (in.read((char*)&materialsShown, sizeof(bool)) < 0)
			return ReadError();
		showMaterials(materialsShown);
	}

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = false;
	if (in.read((char*)&hasTriNormalIndexes, sizeof(bool)) < 0)
		return ReadError();
	if (hasTriNormalIndexes)
	{
		if (!m_triNormalIndexes)
		{
			m_triNormalIndexes = new triangleNormalsIndexesSet();
			m_triNormalIndexes->link();
		}
		assert(m_triNormalIndexes);
		if (!ccSerializationHelper::GenericArrayFromFile<Tuple3i, 3, int>(*m_triNormalIndexes, in, dataVersion))
		{
			removePerTriangleNormalIndexes();
			return false;
		}
	}

	if (dataVersion < 29)
	{
		//'per-triangle normals shown' state (dataVersion>=20 && dataVersion<29))
		bool triNormsShown = false;
		if (in.read((char*)&triNormsShown, sizeof(bool)) < 0)
			return ReadError();
		showTriNorms(triNormsShown);

		//'polygon stippling' state (dataVersion>=20 && dataVersion<29))
		bool stippling = false;
		if (in.read((char*)&stippling, sizeof(bool)) < 0)
			return ReadError();
		enableStippling(stippling);
	}

	notifyGeometryUpdate();

	return true;
}

void ccMesh::computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const
{
	assert(triIndex < m_triVertIndexes->size());

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->at(triIndex);
	return computeInterpolationWeights(tri, P, weights);
}

void ccMesh::computeInterpolationWeights(const CCCoreLib::VerticesIndexes& vertIndexes, const CCVector3& P, CCVector3d& weights) const
{
	const CCVector3 *A = m_associatedCloud->getPoint(vertIndexes.i1);
	const CCVector3 *B = m_associatedCloud->getPoint(vertIndexes.i2);
	const CCVector3 *C = m_associatedCloud->getPoint(vertIndexes.i3);

	//barycentric interpolation weights
	weights.x = sqrt(((P - *B).cross(*C - *B)).norm2d())/*/2*/;
	weights.y = sqrt(((P - *C).cross(*A - *C)).norm2d())/*/2*/;
	weights.z = sqrt(((P - *A).cross(*B - *A)).norm2d())/*/2*/;

	//normalize weights
	double sum = weights.x + weights.y + weights.z;
	weights /= sum;
}

bool ccMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	assert(triIndex < size());

	if (!hasNormals())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	//interpolation weights
	CCVector3d w;
	computeInterpolationWeights(tri, P, w);

	return interpolateNormals(tri, w, N, hasTriNormals() ? &m_triNormalIndexes->at(triIndex) : nullptr);
}

bool ccMesh::interpolateNormalsBC(unsigned triIndex, const CCVector3d& w, CCVector3& N)
{
	assert(triIndex < size());

	if (!hasNormals())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	return interpolateNormals(tri, w, N, hasTriNormals() ? &m_triNormalIndexes->at(triIndex) : nullptr);
}

bool ccMesh::interpolateNormals(const CCCoreLib::VerticesIndexes& vertIndexes, const CCVector3d& w, CCVector3& N, const Tuple3i* triNormIndexes/*=0*/)
{
	CCVector3d Nd(0, 0, 0);
	{
		if (!triNormIndexes || triNormIndexes->u[0] >= 0)
		{
			const CCVector3& N1 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes->u[0])) : m_associatedCloud->getPointNormal(vertIndexes.i1);
			Nd += N1.toDouble() * w.u[0];
		}

		if (!triNormIndexes || triNormIndexes->u[1] >= 0)
		{
			const CCVector3& N2 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes->u[1])) : m_associatedCloud->getPointNormal(vertIndexes.i2);
			Nd += N2.toDouble() * w.u[1];
		}

		if (!triNormIndexes || triNormIndexes->u[2] >= 0)
		{
			const CCVector3& N3 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes->u[2])) : m_associatedCloud->getPointNormal(vertIndexes.i3);
			Nd += N3.toDouble() * w.u[2];
		}
		Nd.normalize();
	}

	N = Nd.toPC();

	return true;
}

bool ccMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& color)
{
	assert(triIndex < size());

	if (!hasColors())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	//interpolation weights
	CCVector3d w;
	computeInterpolationWeights(tri, P, w);

	return interpolateColors(tri, w, color);
}

bool ccMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color)
{
	assert(triIndex < size());

	if (!hasColors())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	//interpolation weights
	CCVector3d w;
	computeInterpolationWeights(tri, P, w);

	return interpolateColors(tri, w, color);
}

bool ccMesh::interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgb& color)
{
	assert(triIndex < size());

	if (!hasColors())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	return interpolateColors(tri, w, color);
}

bool ccMesh::interpolateColors(const CCCoreLib::VerticesIndexes& vertIndexes, const CCVector3d& w, ccColor::Rgb& color)
{
	const ccColor::Rgba& C1 = m_associatedCloud->getPointColor(vertIndexes.i1);
	const ccColor::Rgba& C2 = m_associatedCloud->getPointColor(vertIndexes.i2);
	const ccColor::Rgba& C3 = m_associatedCloud->getPointColor(vertIndexes.i3);

	color.r = static_cast<ColorCompType>(floor(C1.r * w.u[0] + C2.r * w.u[1] + C3.r * w.u[2]));
	color.g = static_cast<ColorCompType>(floor(C1.g * w.u[0] + C2.g * w.u[1] + C3.g * w.u[2]));
	color.b = static_cast<ColorCompType>(floor(C1.b * w.u[0] + C2.b * w.u[1] + C3.b * w.u[2]));

	return true;
}

bool ccMesh::interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgba& color)
{
	assert(triIndex < size());

	if (!hasColors())
		return false;

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	return interpolateColors(tri, w, color);
}

bool ccMesh::interpolateColors(const CCCoreLib::VerticesIndexes& vertIndexes, const CCVector3d& w, ccColor::Rgba& color)
{
	const ccColor::Rgba& C1 = m_associatedCloud->getPointColor(vertIndexes.i1);
	const ccColor::Rgba& C2 = m_associatedCloud->getPointColor(vertIndexes.i2);
	const ccColor::Rgba& C3 = m_associatedCloud->getPointColor(vertIndexes.i3);

	color.r = static_cast<ColorCompType>(floor(C1.r * w.u[0] + C2.r * w.u[1] + C3.r * w.u[2]));
	color.g = static_cast<ColorCompType>(floor(C1.g * w.u[0] + C2.g * w.u[1] + C3.g * w.u[2]));
	color.b = static_cast<ColorCompType>(floor(C1.b * w.u[0] + C2.b * w.u[1] + C3.b * w.u[2]));
	color.a = static_cast<ColorCompType>(floor(C1.a * w.u[0] + C2.a * w.u[1] + C3.a * w.u[2]));

	return true;
}

bool ccMesh::getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgba& color, bool returnColorIfNoTexture)
{
	assert(triIndex < size());

	assert(vertIndex < 3);
	if (vertIndex > 2)
	{
		ccLog::Error("[ccMesh::getVertexColorFromMaterial] Internal error: invalid vertex index!");
		return false;
	}

	int matIndex = -1;

	if (hasMaterials())
	{
		assert(m_materials);
		matIndex = m_triMtlIndexes->getValue(triIndex);
		assert(matIndex < static_cast<int>(m_materials->size()));
	}

	const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(triIndex);

	//do we need to change material?
	bool foundMaterial = false;
	if (matIndex >= 0)
	{
		ccMaterial::CShared material = (*m_materials)[matIndex];
		if (material->hasTexture())
		{
			assert(m_texCoords && m_texCoordIndexes);
			const Tuple3i& txInd = m_texCoordIndexes->getValue(triIndex);
			const TexCoords2D* T = (txInd.u[vertIndex] >= 0 ? &m_texCoords->getValue(txInd.u[vertIndex]) : nullptr);
			if (T)
			{
				//get the texture coordinates between 0 and 1
				float temp;
				float tx = std::modf(T->tx, &temp);
				if (tx < 0)
					tx = 1.0f + tx;
				float ty = std::modf(T->ty, &temp);
				if (ty < 0)
					ty = 1.0f + ty;

				//get color from texture image
				const QImage texture = material->getTexture();
				int xPix = std::min(static_cast<int>(floor(tx * texture.width())), texture.width() - 1);
				int yPix = std::min(static_cast<int>(floor(ty * texture.height())), texture.height() - 1);

				QRgb pixel = texture.pixel(xPix, yPix);

				color = ccColor::FromQRgba(pixel);
				foundMaterial = true;
			}
		}
		else
		{
			const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
			color = ccColor::FromRgbafToRgba(diffuse);

			foundMaterial = true;
		}
	}

	if (!foundMaterial && returnColorIfNoTexture && hasColors())
	{
		color = m_associatedCloud->getPointColor(tri.i[vertIndex]);
		foundMaterial = true;
	}

	return foundMaterial;
}

bool ccMesh::getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color, bool interpolateColorIfNoTexture)
{
	assert(triIndex < size());

	int matIndex = -1;

	if (hasMaterials())
	{
		assert(m_materials);
		matIndex = m_triMtlIndexes->getValue(triIndex);
		assert(matIndex < static_cast<int>(m_materials->size()));
	}

	//do we need to change material?
	if (matIndex < 0)
	{
		if (interpolateColorIfNoTexture)
			return interpolateColors(triIndex, P, color);
		return false;
	}

	ccMaterial::CShared material = (*m_materials)[matIndex];

	if (!material->hasTexture())
	{
		const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
		color = ccColor::FromRgbafToRgba(diffuse);
		return true;
	}

	assert(m_texCoords && m_texCoordIndexes);
	const Tuple3i& txInd = m_texCoordIndexes->getValue(triIndex);
	const TexCoords2D* T1 = (txInd.u[0] >= 0 ? &m_texCoords->getValue(txInd.u[0]) : nullptr);
	const TexCoords2D* T2 = (txInd.u[1] >= 0 ? &m_texCoords->getValue(txInd.u[1]) : nullptr);
	const TexCoords2D* T3 = (txInd.u[2] >= 0 ? &m_texCoords->getValue(txInd.u[2]) : nullptr);

	//interpolation weights
	CCVector3d w;
	computeInterpolationWeights(triIndex, P, w);

	if (	(!T1 && CCCoreLib::GreaterThanEpsilon( w.u[0] ) )
		||	(!T2 && CCCoreLib::GreaterThanEpsilon( w.u[1] ) )
		||	(!T3 && CCCoreLib::GreaterThanEpsilon( w.u[2] ) ) )
	{
		//assert(false);
		if (interpolateColorIfNoTexture)
			return interpolateColors(triIndex, P, color);
		return false;
	}

	double x = (T1 ? T1->tx*w.u[0] : 0.0) + (T2 ? T2->tx * w.u[1] : 0.0) + (T3 ? T3->tx * w.u[2] : 0.0);
	double y = (T1 ? T1->ty*w.u[0] : 0.0) + (T2 ? T2->ty * w.u[1] : 0.0) + (T3 ? T3->ty * w.u[2] : 0.0);

	//DGM: we mut handle texture coordinates below 0 or above 1 (i.e. repetition)
	//if (x < 0 || x > 1.0 || y < 0 || y > 1.0)
	if (x > 1.0)
	{
		double xFrac = 0.0;
		double xInt = 0.0;
		xFrac = std::modf(x, &xInt);
		x = xFrac;
	}
	else if (x < 0.0)
	{
		double xFrac = 0.0;
		double xInt = 0.0;
		xFrac = std::modf(x, &xInt);
		x = 1.0 + xFrac;
	}

	//same thing for y
	if (y > 1.0)
	{
		double yFrac = 0.0;
		double yInt = 0.0;
		yFrac = std::modf(y, &yInt);
		y = yFrac;
	}
	else if (y < 0.0)
	{
		double yFrac = 0.0;
		double yInt = 0.0;
		yFrac = std::modf(y, &yInt);
		y = 1.0 + yFrac;
	}

	//get color from texture image
	{
		const QImage texture = material->getTexture();
		int xPix = std::min(static_cast<int>(floor(x*texture.width())), texture.width() - 1);
		int yPix = std::min(static_cast<int>(floor(y*texture.height())), texture.height() - 1);

		QRgb pixel = texture.pixel(xPix, yPix);

		const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
		color.r = static_cast<ColorCompType>(diffuse.r * qRed(pixel));
		color.g = static_cast<ColorCompType>(diffuse.g * qGreen(pixel));
		color.b = static_cast<ColorCompType>(diffuse.b * qBlue(pixel));
		color.a = static_cast<ColorCompType>(diffuse.a * qAlpha(pixel));
	}

	return true;
}

//we use as many static variables as we can to limit the size of the heap used by each recursion...
static const unsigned s_defaultSubdivideGrowRate = 50;
static PointCoordinateType s_maxSubdivideArea = 1;
static QMap<qint64,unsigned> s_alreadyCreatedVertices; //map to store already created edges middle points

static qint64 GenerateKey(unsigned edgeIndex1, unsigned edgeIndex2)
{
	if (edgeIndex1 > edgeIndex2)
		std::swap(edgeIndex1, edgeIndex2);

	return (static_cast<qint64>(edgeIndex1) << 32) | static_cast<qint64>(edgeIndex2);
}

bool ccMesh::pushSubdivide(/*PointCoordinateType maxArea, */unsigned indexA, unsigned indexB, unsigned indexC)
{
	if ( s_maxSubdivideArea <= CCCoreLib::ZERO_TOLERANCE_POINT_COORDINATE )
	{
		ccLog::Error("[ccMesh::pushSubdivide] Invalid input argument!");
		return false;
	}

	if (!getAssociatedCloud() || !getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Error("[ccMesh::pushSubdivide] Vertices set must be a true point cloud!");
		return false;
	}
	ccPointCloud* vertices = static_cast<ccPointCloud*>(getAssociatedCloud());
	assert(vertices);
	const CCVector3* A = vertices->getPoint(indexA);
	const CCVector3* B = vertices->getPoint(indexB);
	const CCVector3* C = vertices->getPoint(indexC);

	//do we need to subdivide this triangle?
	PointCoordinateType area = ((*B - *A)*(*C - *A)).norm() / 2;
	if (area > s_maxSubdivideArea/*maxArea*/)
	{
		//we will add 3 new vertices, so we must be sure to have enough memory
		if (vertices->size() + 2 >= vertices->capacity())
		{
			assert(s_defaultSubdivideGrowRate > 2);
			if (!vertices->reserve(vertices->size() + s_defaultSubdivideGrowRate))
			{
				ccLog::Error("[ccMesh::pushSubdivide] Not enough memory!");
				return false;
			}
			//We have to update pointers as they may have been wrangled by the 'reserve' call
			A = vertices->getPoint(indexA);
			B = vertices->getPoint(indexB);
			C = vertices->getPoint(indexC);
		}

		//add new vertices
		unsigned indexG1 = 0;
		{
			qint64 key = GenerateKey(indexA, indexB);
			QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(key);
			if (it == s_alreadyCreatedVertices.constEnd())
			{
				//generate new vertex
				indexG1 = vertices->size();
				CCVector3 G1 = (*A + *B) / 2;
				vertices->addPoint(G1);
				//interpolate other features?
				//if (vertices->hasNormals())
				//{
				//	//vertices->reserveTheNormsTable();
				//	CCVector3 N(0.0, 0.0, 1.0);
				//	interpolateNormals(indexA, indexB, indexC, G1, N);
				//	vertices->addNorm(N);
				//}
				if (vertices->hasColors())
				{
					CCCoreLib::VerticesIndexes tri(indexA, indexB, indexC);
					CCVector3d w1;
					computeInterpolationWeights(tri, G1, w1);
					ccColor::Rgba color;
					interpolateColors(CCCoreLib::VerticesIndexes(indexA, indexB, indexC), w1, color);
					vertices->addColor(color);
				}
				//and add it to the map
				s_alreadyCreatedVertices.insert(key, indexG1);
			}
			else
			{
				indexG1 = it.value();
			}
		}
		unsigned indexG2 = 0;
		{
			qint64 key = GenerateKey(indexB, indexC);
			QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(key);
			if (it == s_alreadyCreatedVertices.constEnd())
			{
				//generate new vertex
				indexG2 = vertices->size();
				CCVector3 G2 = (*B + *C) / 2;
				vertices->addPoint(G2);
				//interpolate other features?
				//if (vertices->hasNormals())
				//{
				//	//vertices->reserveTheNormsTable();
				//	CCVector3 N(0.0, 0.0, 1.0);
				//	interpolateNormals(indexA, indexB, indexC, G2, N);
				//	vertices->addNorm(N);
				//}
				if (vertices->hasColors())
				{
					CCCoreLib::VerticesIndexes tri(indexA, indexB, indexC);
					CCVector3d w2;
					computeInterpolationWeights(tri, G2, w2);
					ccColor::Rgba colors;
					interpolateColors(CCCoreLib::VerticesIndexes(indexA, indexB, indexC), w2, colors);
					vertices->addColor(colors);
				}
				//and add it to the map
				s_alreadyCreatedVertices.insert(key, indexG2);
			}
			else
			{
				indexG2 = it.value();
			}
		}
		unsigned indexG3 = vertices->size();
		{
			qint64 key = GenerateKey(indexC, indexA);
			QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(key);
			if (it == s_alreadyCreatedVertices.constEnd())
			{
				//generate new vertex
				indexG3 = vertices->size();
				CCVector3 G3 = (*C + *A) / 2.0;
				vertices->addPoint(G3);
				//interpolate other features?
				//if (vertices->hasNormals())
				//{
				//	//vertices->reserveTheNormsTable();
				//	CCVector3 N(0.0, 0.0, 1.0);
				//	interpolateNormals(indexA, indexB, indexC, G3, N);
				//	vertices->addNorm(N);
				//}
				if (vertices->hasColors())
				{
					CCCoreLib::VerticesIndexes tri(indexA, indexB, indexC);
					CCVector3d w3;
					computeInterpolationWeights(tri, G3, w3);
					ccColor::Rgba colors;
					interpolateColors(CCCoreLib::VerticesIndexes(indexA, indexB, indexC), w3, colors);
					vertices->addColor(colors);
				}
				//and add it to the map
				s_alreadyCreatedVertices.insert(key, indexG3);
			}
			else
			{
				indexG3 = it.value();
			}
		}

		//add new triangles
		if (!pushSubdivide(/*maxArea, */indexA, indexG1, indexG3))
			return false;
		if (!pushSubdivide(/*maxArea, */indexB, indexG2, indexG1))
			return false;
		if (!pushSubdivide(/*maxArea, */indexC, indexG3, indexG2))
			return false;
		if (!pushSubdivide(/*maxArea, */indexG1, indexG2, indexG3))
			return false;
	}
	else
	{
		//we will add one triangle, so we must be sure to have enough memory
		if (size() == capacity())
		{
			if (!reserve(size() + 3 * s_defaultSubdivideGrowRate))
			{
				ccLog::Error("[ccMesh::pushSubdivide] Not enough memory!");
				return false;
			}
		}

		//we keep this triangle as is
		addTriangle(indexA, indexB, indexC);
	}

	return true;
}

ccMesh* ccMesh::subdivide(PointCoordinateType maxArea) const
{
	if ( maxArea <= CCCoreLib::ZERO_TOLERANCE_POINT_COORDINATE )
	{
		ccLog::Error("[ccMesh::subdivide] Invalid input argument!");
		return nullptr;
	}
	s_maxSubdivideArea = maxArea;

	unsigned triCount = size();
	ccGenericPointCloud* vertices = getAssociatedCloud();
	unsigned vertCount = (vertices ? vertices->size() : 0);
	if (!vertices || vertCount*triCount == 0)
	{
		ccLog::Error("[ccMesh::subdivide] Invalid mesh: no face or no vertex!");
		return nullptr;
	}

	ccPointCloud* resultVertices = vertices->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(vertices)->cloneThis() : ccPointCloud::From(vertices,vertices);
	if (!resultVertices)
	{
		ccLog::Error("[ccMesh::subdivide] Not enough memory!");
		return nullptr;
	}

	ccMesh* resultMesh = new ccMesh(resultVertices);
	resultMesh->addChild(resultVertices);

	if (!resultMesh->reserve(triCount))
	{
		ccLog::Error("[ccMesh::subdivide] Not enough memory!");
		delete resultMesh;
		return nullptr;
	}

	s_alreadyCreatedVertices.clear();

	try
	{
		for (unsigned i = 0; i < triCount; ++i)
		{
			const CCCoreLib::VerticesIndexes& tri = m_triVertIndexes->getValue(i);
			if (!resultMesh->pushSubdivide(/*maxArea,*/tri.i1, tri.i2, tri.i3))
			{
				ccLog::Error("[ccMesh::subdivide] Not enough memory!");
				delete resultMesh;
				return nullptr;
			}
		}
	}
	catch(...)
	{
		ccLog::Error("[ccMesh::subdivide] An error occurred!");
		delete resultMesh;
		return nullptr;
	}

	//we must also 'fix' the triangles that share (at least) an edge with a subdivided triangle!
	try
	{
		unsigned newTriCount = resultMesh->size();
		for (unsigned i = 0; i < newTriCount; ++i)
		{
			CCCoreLib::VerticesIndexes& tri = resultMesh->m_triVertIndexes->getValue(i); //warning: array might change at each call to reallocate!
			unsigned indexA = tri.i1;
			unsigned indexB = tri.i2;
			unsigned indexC = tri.i3;

			//test all edges
			int indexG1 = -1;
			{
				QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(GenerateKey(indexA, indexB));
				if (it != s_alreadyCreatedVertices.constEnd())
					indexG1 = (int)it.value();
			}
			int indexG2 = -1;
			{
				QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(GenerateKey(indexB, indexC));
				if (it != s_alreadyCreatedVertices.constEnd())
					indexG2 = (int)it.value();
			}
			int indexG3 = -1;
			{
				QMap<qint64, unsigned>::const_iterator it = s_alreadyCreatedVertices.constFind(GenerateKey(indexC, indexA));
				if (it != s_alreadyCreatedVertices.constEnd())
					indexG3 = (int)it.value();
			}

			//at least one edge is 'wrong'
			unsigned brokenEdges =	(indexG1 < 0 ? 0:1)
								+	(indexG2 < 0 ? 0:1)
								+	(indexG3 < 0 ? 0:1);

			if (brokenEdges == 1)
			{
				int indexG = indexG1;
				unsigned char i1 = 2; //relative index facing the broken edge
				if (indexG2 >= 0)
				{
					indexG = indexG2;
					i1 = 0;
				}
				else if (indexG3 >= 0)
				{
					indexG = indexG3;
					i1 = 1;
				}
				assert(indexG >= 0);
				assert(i1<3);

				unsigned indexes[3] = { indexA, indexB, indexC };

				//replace current triangle by one half
				tri.i1 = indexes[i1];
				tri.i2 = indexG;
				tri.i3 = indexes[(i1 + 2) % 3];
				//and add the other half (we can use pushSubdivide as the area should already be ok!)
				if (!resultMesh->pushSubdivide(/*maxArea,*/indexes[i1], indexes[(i1 + 1) % 3], indexG))
				{
					ccLog::Error("[ccMesh::subdivide] Not enough memory!");
					delete resultMesh;
					return nullptr;
				}
			}
			else if (brokenEdges == 2)
			{
				if (indexG1 < 0) //broken edges: BC and CA
				{
					//replace current triangle by the 'pointy' part
					tri.i1 = indexC;
					tri.i2 = indexG3;
					tri.i3 = indexG2;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexA, indexG2, indexG3) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexA, indexB, indexG2))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return nullptr;
					}
				}
				else if (indexG2 < 0) //broken edges: AB and CA
				{
					//replace current triangle by the 'pointy' part
					tri.i1 = indexA;
					tri.i2 = indexG1;
					tri.i3 = indexG3;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexB, indexG3, indexG1) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexB, indexC, indexG3))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return nullptr;
					}
				}
				else /*if (indexG3 < 0)*/ //broken edges: AB and BC
				{
					//replace current triangle by the 'pointy' part
					tri.i1 = indexB;
					tri.i2 = indexG2;
					tri.i3 = indexG1;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexC, indexG1, indexG2) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexC, indexA, indexG1))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return nullptr;
					}
				}
			}
			else if (brokenEdges == 3) //works just as a standard subdivision in fact!
			{
				//replace current triangle by one quarter
				tri.i1 = indexA;
				tri.i2 = indexG1;
				tri.i3 = indexG3;
				//and add the other 3 quarters (we can use pushSubdivide as the area should already be ok!)
				if (!resultMesh->pushSubdivide(/*maxArea, */indexB, indexG2, indexG1) ||
					!resultMesh->pushSubdivide(/*maxArea, */indexC, indexG3, indexG2) ||
					!resultMesh->pushSubdivide(/*maxArea, */indexG1, indexG2, indexG3))
				{
					ccLog::Error("[ccMesh::subdivide] Not enough memory!");
					delete resultMesh;
					return nullptr;
				}
			}
		}
	}
	catch (...)
	{
		ccLog::Error("[ccMesh::subdivide] An error occurred!");
		delete resultMesh;
		return nullptr;
	}

	s_alreadyCreatedVertices.clear();

	resultMesh->shrinkToFit();
	resultVertices->shrinkToFit();

	//we import from the original mesh... what we can
	if (hasNormals())
	{
		if (hasNormals()) //normals interpolation doesn't work well...
			resultMesh->computeNormals(!hasTriNormals());
		resultMesh->showNormals(normalsShown());
	}
	if (hasColors())
	{
		resultMesh->showColors(colorsShown());
	}
	resultMesh->setVisible(isVisible());

	return resultMesh;
}

bool ccMesh::convertMaterialsToVertexColors()
{
	if (!hasMaterials())
	{
		ccLog::Warning("[ccMesh::convertMaterialsToVertexColors] Mesh has no material!");
		return false;
	}

	if (!m_associatedCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Warning("[ccMesh::convertMaterialsToVertexColors] Need a true point cloud as vertices!");
		return false;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_associatedCloud);
	if (!cloud->resizeTheRGBTable(true))
	{
		ccLog::Warning("[ccMesh::convertMaterialsToVertexColors] Failed to resize vertices color table! (not enough memory?)");
		return false;
	}

	//now scan all faces and get the vertex color each time
	unsigned faceCount = size();

	placeIteratorAtBeginning();
	for (unsigned i = 0; i < faceCount; ++i)
	{
		const CCCoreLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
		for (unsigned char j = 0; j < 3; ++j)
		{
			ccColor::Rgba color;
			if (getVertexColorFromMaterial(i, j, color, true))
			{
				//FIXME: could we be smarter? (we process each point several times! And we assume the color is always the same...)
				cloud->setPointColor(tsi->i[j], color);
			}
		}
	}

	return true;
}

static bool TagDuplicatedVertices(	const CCCoreLib::DgmOctree::octreeCell& cell,
									void** additionalParameters,
									CCCoreLib::NormalizedProgress* nProgress/*=0*/)
{
	std::vector<int>* equivalentIndexes = static_cast<std::vector<int>*>(additionalParameters[0]);

	//we look for points very close to the others (only if not yet tagged!)

	//structure for nearest neighbors search
	CCCoreLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	nNSS.level = cell.level;
	static const PointCoordinateType c_defaultSearchRadius = static_cast<PointCoordinateType>(sqrt(CCCoreLib::ZERO_TOLERANCE_F));
	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	unsigned n = cell.points->size(); //number of points in the current cell

	//we already know some of the neighbours: the points in the current cell!
	try
	{
		nNSS.pointsInNeighbourhood.resize(n);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	//init structure with cell points
	{
		CCCoreLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
		for (unsigned i = 0; i < n; ++i, ++it)
		{
			it->point = cell.points->getPointPersistentPtr(i);
			it->pointIndex = cell.points->getPointGlobalIndex(i);
		}
		nNSS.alreadyVisitedNeighbourhoodSize = 1;
	}

	//for each point in the cell
	for (unsigned i = 0; i < n; ++i)
	{
		int thisIndex = static_cast<int>(cell.points->getPointGlobalIndex(i));
		if (equivalentIndexes->at(thisIndex) < 0) //has no equivalent yet 
		{
			cell.points->getPoint(i, nNSS.queryPoint);

			//look for neighbors in a (very small) sphere
			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
			unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, c_defaultSearchRadius, false);

			//if there are some very close points
			if (k > 1)
			{
				for (unsigned j = 0; j < k; ++j)
				{
					//all the other points are equivalent to the query point
					const unsigned& otherIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
					if (static_cast<int>(otherIndex) != thisIndex)
						equivalentIndexes->at(otherIndex) = thisIndex;
				}
			}

			//and the query point is always root
			equivalentIndexes->at(thisIndex) = thisIndex;
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

bool ccMesh::mergeDuplicatedVertices(unsigned char octreeLevel/*=10*/, QWidget* parentWidget/*=nullptr*/)
{
	if (!m_associatedCloud)
	{
		assert(false);
		return false;
	}

	unsigned vertCount = m_associatedCloud->size();
	unsigned faceCount = size();
	if (vertCount == 0 || faceCount == 0)
	{
		ccLog::Warning("[ccMesh::mergeDuplicatedVertices] No triangle or no vertex");
		return false;
	}

	try
	{
		std::vector<int> equivalentIndexes;
		const int razValue = -1;
		equivalentIndexes.resize(vertCount, razValue);

		// tag the duplicated vertices
		{
			QScopedPointer<ccProgressDialog> pDlg(nullptr);
			if (parentWidget)
			{
				pDlg.reset(new ccProgressDialog(true, parentWidget));
			}

			// try to build the octree
			ccOctree::Shared octree = ccOctree::Shared(new ccOctree(m_associatedCloud));
			if (!octree->build(pDlg.data()))
			{
				ccLog::Warning("[MergeDuplicatedVertices] Not enough memory");
				return false;
			}

			void* additionalParameters[] = { static_cast<void*>(&equivalentIndexes) };
			unsigned result = octree->executeFunctionForAllCellsAtLevel(10,
				TagDuplicatedVertices,
				additionalParameters,
				false,
				pDlg.data(),
				"Tag duplicated vertices");

			if (result == 0)
			{
				ccLog::Warning("[MergeDuplicatedVertices] Duplicated vertices removal algorithm failed?!");
				return false;
			}
		}

		unsigned remainingCount = 0;
		for (unsigned i = 0; i < vertCount; ++i)
		{
			int eqIndex = equivalentIndexes[i];
			assert(eqIndex >= 0);
			if (eqIndex == static_cast<int>(i)) //root point
			{
				// we replace the root index by its 'new' index (+ vertCount, to differentiate it later)
				int newIndex = static_cast<int>(vertCount + remainingCount);
				equivalentIndexes[i] = newIndex;
				++remainingCount;
			}
		}

		CCCoreLib::ReferenceCloud newVerticesRef(m_associatedCloud);
		if (!newVerticesRef.reserve(remainingCount))
		{
			ccLog::Warning("[MergeDuplicatedVertices] Not enough memory");
			return false;
		}

		//copy root points in a new cloud
		{
			for (unsigned i = 0; i < vertCount; ++i)
			{
				int eqIndex = equivalentIndexes[i];
				if (eqIndex >= static_cast<int>(vertCount)) //root point
					newVerticesRef.addPointIndex(i);
				else
					equivalentIndexes[i] = equivalentIndexes[eqIndex]; //and update the other indexes
			}
		}

		ccPointCloud* newVertices = nullptr;
		if (m_associatedCloud->isKindOf(CC_TYPES::POINT_CLOUD))
		{
			newVertices = static_cast<ccPointCloud*>(m_associatedCloud)->partialClone(&newVerticesRef);
		}
		else
		{
			newVertices = ccPointCloud::From(&newVerticesRef, m_associatedCloud);
		}
		if (!newVertices)
		{
			ccLog::Warning("[MergeDuplicatedVertices] Not enough memory");
			return false;
		}

		//update face indexes
		{
			unsigned newFaceCount = 0;
			for (unsigned i = 0; i < faceCount; ++i)
			{
				CCCoreLib::VerticesIndexes* tri = getTriangleVertIndexes(i);
				tri->i1 = static_cast<unsigned>(equivalentIndexes[tri->i1]) - vertCount;
				tri->i2 = static_cast<unsigned>(equivalentIndexes[tri->i2]) - vertCount;
				tri->i3 = static_cast<unsigned>(equivalentIndexes[tri->i3]) - vertCount;

				//very small triangles (or flat ones) may be implicitly removed by vertex fusion!
				if (tri->i1 != tri->i2 && tri->i1 != tri->i3 && tri->i2 != tri->i3)
				{
					if (newFaceCount != i)
						swapTriangles(i, newFaceCount);
					++newFaceCount;
				}
			}

			if (newFaceCount == 0)
			{
				ccLog::Warning("[MergeDuplicatedVertices] After vertex fusion, all triangles would collapse! We'll keep the non-fused version...");
				delete newVertices;
				newVertices = nullptr;
			}
			else
			{
				resize(newFaceCount);
			}
		}

		// update the mesh vertices
		int childPos = getChildIndex(m_associatedCloud);
		if (childPos >= 0)
		{
			removeChild(childPos);
		}
		else
		{
			delete m_associatedCloud;
			m_associatedCloud = nullptr;
		}
		setAssociatedCloud(newVertices);
		if (childPos >= 0)
		{
			addChild(m_associatedCloud);
		}
		vertCount = (m_associatedCloud ? m_associatedCloud->size() : 0);
		ccLog::Print("[MergeDuplicatedVertices] Remaining vertices after auto-removal of duplicate ones: %i", vertCount);
		ccLog::Print("[MergeDuplicatedVertices] Remaining faces after auto-removal of duplicate ones: %i", size());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[MergeDuplicatedVertices] Not enough memory: could not remove duplicated vertices!");
	}

	return true;
}
