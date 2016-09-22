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
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"
#include "ccSubMesh.h"
#include "ccScalarField.h"
#include "ccColorScalesManager.h"
#include "ccGenericGLDisplay.h"
#include "ccProgressDialog.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <ReferenceCloud.h>

//System
#include <string.h>
#include <assert.h>
#include <math.h> //for modf

static CCVector3 s_blankNorm(0,0,0);

ccMesh::ccMesh(ccGenericPointCloud* vertices)
	: ccGenericMesh("Mesh")
	, m_associatedCloud(0)
	, m_triNormals(0)
	, m_texCoords(0)
	, m_materials(0)
	, m_triVertIndexes(0)
	, m_globalIterator(0)
	, m_triMtlIndexes(0)
	, m_texCoordIndexes(0)
	, m_triNormalIndexes(0)
{
	setAssociatedCloud(vertices);

	m_triVertIndexes = new triangleIndexesContainer();
	m_triVertIndexes->link();
}

ccMesh::ccMesh(CCLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices)
	: ccGenericMesh("Mesh")
	, m_associatedCloud(0)
	, m_triNormals(0)
	, m_texCoords(0)
	, m_materials(0)
	, m_triVertIndexes(0)
	, m_globalIterator(0)
	, m_triMtlIndexes(0)
	, m_texCoordIndexes(0)
	, m_triNormalIndexes(0)
{
	setAssociatedCloud(giVertices);

	m_triVertIndexes = new triangleIndexesContainer();
	m_triVertIndexes->link();

	unsigned triNum = giMesh->size();
	if (!reserve(triNum))
		return;

	giMesh->placeIteratorAtBegining();
	for (unsigned i=0; i<triNum; ++i)
	{
		const CCLib::VerticesIndexes* tsi = giMesh->getNextTriangleVertIndexes();
		addTriangle(tsi->i1,tsi->i2,tsi->i3);
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
	setMaterialSet(0);
	setTexCoordinatesTable(0);

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
		setAssociatedCloud(0);

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
		theNorms.resize(vertCount,s_blankNorm);
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
	placeIteratorAtBegining();
	{
		for (unsigned i=0; i<triCount; ++i)
		{
			CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();

			assert(tsi->i1<vertCount && tsi->i2<vertCount && tsi->i3<vertCount);
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
		for (unsigned i=0; i<vertCount; i++)
		{
			CCVector3& N = theNorms[i];
			//normalize the 'mean' normal
			N.normalize();
			cloud->setPointNormal(i,N);
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
	setTriNormsTable(0);

	NormsIndexesTableType* normIndexes = new NormsIndexesTableType();
	if (!normIndexes->reserve(triCount))
	{
		normIndexes->release();
		ccLog::Warning("[ccMesh::computePerTriangleNormals] Not enough memory!");
		return false;
	}

	//for each triangle
	{
		for (unsigned i=0; i<triCount; ++i)
		{
			const unsigned* tri = m_triVertIndexes->getValue(i);
			const CCVector3* A = m_associatedCloud->getPoint(tri[0]);
			const CCVector3* B = m_associatedCloud->getPoint(tri[1]);
			const CCVector3* C = m_associatedCloud->getPoint(tri[2]);

			//compute face normal (right hand rule)
			CCVector3 N = (*B-*A).cross(*C-*A);

			CompressedNormType nIndex = ccNormalVectors::GetNormIndex(N.u);
			normIndexes->addElement(nIndex);
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

		for (int i=0; i<static_cast<int>(triCount); ++i)
			addTriangleNormalIndexes(i,i,i);
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
		for (unsigned i=0; i<nPts; ++i)
		{
			meanSF[i] = m_associatedCloud->getPointScalarValue(i);
			count[i] = 1;
		}
	}

	//for each triangle
	unsigned nTri = size();
	{
		placeIteratorAtBegining();
		for (unsigned i=0; i<nTri; ++i)
		{
			const CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes(); //DGM: getNextTriangleVertIndexes is faster for mesh groups!

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
		for (unsigned i=0; i<nPts; ++i)
			meanSF[i] /= (ScalarType)count[i];
	}

	switch (process)
	{
	case SMOOTH_MESH_SF:
		{
			//Smooth = mean value
			for (unsigned i=0; i<nPts; ++i)
				m_associatedCloud->setPointScalarValue(i,meanSF[i]);
		}
		break;
	case ENHANCE_MESH_SF:
		{
			//Enhance = old value + (old value - mean value)
			for (unsigned i=0; i<nPts; ++i)
			{
				ScalarType v = 2.0f*m_associatedCloud->getPointScalarValue(i) - meanSF[i];
				m_associatedCloud->setPointScalarValue(i,v > 0.0f ? v : 0.0f);
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
		m_triNormals = 0;
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
		m_materials = 0;
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
		unsigned numTriNormals = m_triNormals->currentSize();

#if 0 //no use to use memory for this!
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
						newNorms->addElement(newNormIndex);
					}
				}

				//recode
                m_triNormals->placeIteratorAtBegining();
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
            m_triNormals->placeIteratorAtBegining();
            for (unsigned i=0; i<numTriNormals; i++)
            {
                CompressedNormType* _theNormIndex = m_triNormals->getCurrentValuePtr();
                CCVector3 new_n(ccNormalVectors::GetNormal(*_theNormIndex));
                trans.applyRotation(new_n);
                *_theNormIndex = ccNormalVectors::GetNormIndex(new_n.u);
                m_triNormals->forwardIterator();
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

	GenericChunkedArray<3, PointCoordinateType>* verticesDisplacement = new GenericChunkedArray < 3, PointCoordinateType > ;
	if (!verticesDisplacement->resize(vertCount))
	{
		//not enough memory
		verticesDisplacement->release();
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
		verticesDisplacement->release();
		return false;
	}

	placeIteratorAtBegining();
	for(unsigned j=0; j<faceCount; j++)
	{
		const CCLib::VerticesIndexes* tri = getNextTriangleVertIndexes();
		edgesCount[tri->i1] += 2;
		edgesCount[tri->i2] += 2;
		edgesCount[tri->i3] += 2;
	}

	//progress dialog
	CCLib::NormalizedProgress nProgress(progressCb, nbIteration);
	if (progressCb)
	{
		progressCb->setMethodTitle(QObject::tr("Laplacian smooth"));
		progressCb->setInfo(QObject::tr("Iterations: %1\nVertices: %2\nFaces: %3").arg(nbIteration).arg(vertCount).arg(faceCount));
		progressCb->start();
	}

	//repeat Laplacian smoothing iterations
	for (unsigned iter = 0; iter < nbIteration; iter++)
	{
		verticesDisplacement->fill(0);

		//for each triangle
		placeIteratorAtBegining();
		for(unsigned j=0; j<faceCount; j++)
		{
			const CCLib::VerticesIndexes* tri = getNextTriangleVertIndexes();

			const CCVector3* A = m_associatedCloud->getPoint(tri->i1);
			const CCVector3* B = m_associatedCloud->getPoint(tri->i2);
			const CCVector3* C = m_associatedCloud->getPoint(tri->i3);

			CCVector3 dAB = (*B-*A);
			CCVector3 dAC = (*C-*A);
			CCVector3 dBC = (*C-*B);

			CCVector3* dA = (CCVector3*)verticesDisplacement->getValue(tri->i1);
			(*dA) += dAB+dAC;
			CCVector3* dB = (CCVector3*)verticesDisplacement->getValue(tri->i2);
			(*dB) += dBC-dAB;
			CCVector3* dC = (CCVector3*)verticesDisplacement->getValue(tri->i3);
			(*dC) -= dAC+dBC;
		}

		if (!nProgress.oneStep())
		{
			//cancelled by user
			break;
		}

		//apply displacement
		verticesDisplacement->placeIteratorAtBegining();
		for (unsigned i=0; i<vertCount; i++)
		{
			//this is a "persistent" pointer and we know what type of cloud is behind ;)
			CCVector3* P = const_cast<CCVector3*>(m_associatedCloud->getPointPersistentPtr(i));
			const CCVector3* d = (const CCVector3*)verticesDisplacement->getValue(i);
			(*P) += (*d) * (factor / edgesCount[i]);
		}
	}

	m_associatedCloud->notifyGeometryUpdate();

	if (hasNormals())
		computeNormals(!hasTriNormals());

	if (verticesDisplacement)
		verticesDisplacement->release();
	verticesDisplacement = 0;

	return true;
}

ccMesh* ccMesh::cloneMesh(	ccGenericPointCloud* vertices/*=0*/,
							ccMaterialSet* clonedMaterials/*=0*/,
							NormsIndexesTableType* clonedNormsTable/*=0*/,
							TextureCoordsContainer* cloneTexCoords/*=0*/)
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
			return 0;
		}

		//flag used vertices
		{
			placeIteratorAtBegining();
			for (unsigned i=0; i<triNum; ++i)
			{
				const CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
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
			newVertices = m_associatedCloud->clone(0,true);
		}
		else
		{
			//we create a temporary entity with used vertices only
			CCLib::ReferenceCloud rc(m_associatedCloud);
			if (rc.reserve(realVertCount))
			{
				for (unsigned i=0; i<vertNum; ++i)
					if (usedVerts[i] != vertNum)
						rc.addPointIndex(i); //can't fail, see above

				//and the associated vertices set
				assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
				newVertices = static_cast<ccPointCloud*>(m_associatedCloud)->partialClone(&rc);
				if (newVertices && newVertices->size() < rc.size())
				{
					//not enough memory!
					delete newVertices;
					newVertices = 0;
				}
			}
		}
	}

	//failed to create a new vertices set!
	if (!newVertices)
	{
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return 0;
	}

	//mesh clone
	ccMesh* cloneMesh = new ccMesh(newVertices);
	if (!cloneMesh->reserve(triNum))
	{
		if (!vertices)
			delete newVertices;
		delete cloneMesh;
		ccLog::Error("[ccMesh::clone] Not enough memory!");
		return 0;
	}

	//let's create the new triangles
	if (!usedVerts.empty()) //in case we have an equivalence table
	{
		placeIteratorAtBegining();
		for (unsigned i=0; i<triNum; ++i)
		{
			const CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
			cloneMesh->addTriangle(usedVerts[tsi->i1],usedVerts[tsi->i2],usedVerts[tsi->i3]);
		}
		usedVerts.clear();
	}
	else
	{
		placeIteratorAtBegining();
		for (unsigned i=0; i<triNum; ++i)
		{
			const CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
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


ccMesh* ccMesh::Triangulate(ccGenericPointCloud* cloud,
							CC_TRIANGULATION_TYPES type,
							bool updateNormals/*=false*/,
							PointCoordinateType maxEdgeLength/*=0*/,
							unsigned char dim/*=2*/)
{
	if (!cloud || dim > 2)
	{
		ccLog::Warning("[ccMesh::Triangulate] Invalid input parameters!");
		return 0;
	}
	if (cloud->size() < 3)
	{
		ccLog::Warning("[ccMesh::Triangulate] Cloud has not enough points!");
		return 0;
	}
	
	//compute raw mesh
	char errorStr[1024];
	CCLib::GenericIndexedMesh* dummyMesh = CCLib::PointProjectionTools::computeTriangulation(	cloud,
																								type,
																								maxEdgeLength,
																								dim,
																								errorStr);
	if (!dummyMesh)
	{
		ccLog::Warning(QString("[ccMesh::Triangulate] Failed to construct Delaunay mesh (Triangle lib error: %1)").arg(errorStr));
		return 0;
	}

	//convert raw mesh to ccMesh
	ccMesh* mesh = new ccMesh(dummyMesh, cloud);

	//don't need this anymore
	delete dummyMesh;
	dummyMesh = 0;

	if (!mesh)
	{
		ccLog::Warning("[ccMesh::Triangulate] An error occurred while computing mesh! (not enough memory?)");
		return 0;
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
	if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud() != cloud)
	{
		mesh->getAssociatedCloud()->setGlobalShift(cloud->getGlobalShift());
		mesh->getAssociatedCloud()->setGlobalScale(cloud->getGlobalScale());
	}

	return mesh;
}

bool ccMesh::merge(const ccMesh* mesh)
{
	if (!mesh)
	{
		assert(false);
		ccLog::Warning("[ccMesh::merge] Internal error: invalid input!");
		return false;
	}
	if (	!mesh->getAssociatedCloud() || !mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD)
		||	!m_associatedCloud || !m_associatedCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		ccLog::Warning("[ccMesh::merge] Requires meshes with standard vertices!");
		return false;
	}
	ccPointCloud* vertices = static_cast<ccPointCloud*>(mesh->getAssociatedCloud());

	//vertices
	unsigned vertNumBefore = m_associatedCloud->size();
	//triangles
	unsigned triNumBefore = size();

	bool success = false;

	while (true) //fake loop for easy breaking/cleaning
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

			for (unsigned i=0; i<triAdded; ++i)
			{
				const CCLib::VerticesIndexes* tsi = mesh->getTriangleVertIndexes(i);
				addTriangle(vertIndexShift+tsi->i1, vertIndexShift+tsi->i2, vertIndexShift+tsi->i3);
			}
		}

		//triangle normals
		bool hasTriangleNormals = m_triNormals && m_triNormalIndexes;
		if (hasTriangleNormals || otherMeshHasTriangleNormals)
		{
			//1st: does the other mesh has triangle normals
			if (otherMeshHasTriangleNormals)
			{
				unsigned triIndexShift = 0;
				if (m_triNormals != mesh->m_triNormals)
				{
					//reserve mem for triangle normals
					if (!m_triNormals)
					{
						NormsIndexesTableType* normsTable = new NormsIndexesTableType();
						setTriNormsTable(normsTable);
					}
					assert(m_triNormals);
					unsigned triNormalsCountBefore = m_triNormals->currentSize();
					if (!m_triNormals->reserve(triNormalsCountBefore + mesh->m_triNormals->currentSize()))
					{
						ccLog::Warning("[ccMesh::merge] Not enough memory!");
						break;
					}
					//copy the values
					{
						for (unsigned i=0; i<mesh->m_triNormals->currentSize(); ++i)
							m_triNormals->addElement(mesh->m_triNormals->getValue(i));
					}
					triIndexShift = triNormalsCountBefore;
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triNormalIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i=0; i<mesh->m_triNormalIndexes->currentSize(); ++i)
					{
						const int* indexes = mesh->m_triNormalIndexes->getValue(i);
						int newIndexes[3] = {	indexes[0] < 0 ? -1 : indexes[0] + static_cast<int>(triIndexShift),
												indexes[1] < 0 ? -1 : indexes[1] + static_cast<int>(triIndexShift),
												indexes[2] < 0 ? -1 : indexes[2] + static_cast<int>(triIndexShift) };
						m_triNormalIndexes->addElement(newIndexes);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triNormalIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					int defaultElement[3] = {-1,-1,-1};
					for (unsigned i=0; i<mesh->m_triNormalIndexes->currentSize(); ++i)
						m_triNormalIndexes->addElement(defaultElement);
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
					for (size_t m=0; m!=otherMatSetSize; ++m)
					{
						materialIndexMap[m] = m_materials->addMaterial(mesh->m_materials->at(m));
					}
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triMtlIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i=0; i<mesh->m_triMtlIndexes->currentSize(); ++i)
					{
						int index = mesh->m_triMtlIndexes->getValue(i);
						assert(index < static_cast<int>(materialIndexMap.size()));
						int newIndex = (index < 0 ? -1 : materialIndexMap[index]);
						m_triMtlIndexes->addElement(newIndex);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_triMtlIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					int defaultValue = -1;
					for (unsigned i=0; i<mesh->m_triMtlIndexes->currentSize(); ++i)
						m_triMtlIndexes->addElement(defaultValue);
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
				unsigned texCoordIndexShift = 0;
				if (m_texCoords != mesh->m_texCoords)
				{
					//reserve mem for triangle normals
					if (!m_texCoords)
					{
						TextureCoordsContainer* texCoordsTable = new TextureCoordsContainer;
						setTexCoordinatesTable(texCoordsTable);
					}
					assert(m_texCoords);
					unsigned texCoordCountBefore = m_texCoords->currentSize();
					if (!m_texCoords->reserve(texCoordCountBefore + mesh->m_texCoords->currentSize()))
					{
						ccLog::Warning("[ccMesh::merge] Not enough memory!");
						break;
					}
					//copy the values
					{
						static const float TxDef[2] = {-1.0f,-1.0f};
						for (unsigned i=0; i<mesh->m_texCoords->currentSize(); ++i)
						{
							const float* Tx = mesh->m_texCoords->getValue(i);
							assert(Tx);
							m_texCoords->addElement(Tx ? Tx : TxDef);
						}
					}
					texCoordIndexShift = texCoordCountBefore;
				}

				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_texCoordIndexes->capacity() >= triNumBefore + triAdded);
				//copy the values
				{
					for (unsigned i=0; i<mesh->m_texCoordIndexes->currentSize(); ++i)
					{
						const int* indexes = mesh->m_texCoordIndexes->getValue(i);
						int newIndexes[3] = {	!indexes || indexes[0] < 0 ? -1 : indexes[0] + static_cast<int>(texCoordIndexShift),
												!indexes || indexes[1] < 0 ? -1 : indexes[1] + static_cast<int>(texCoordIndexShift),
												!indexes || indexes[2] < 0 ? -1 : indexes[2] + static_cast<int>(texCoordIndexShift) };
						m_texCoordIndexes->addElement(newIndexes);
					}
				}
			}
			else
			{
				//the indexes should have already been resized by the call to 'reserve'!
				assert(m_texCoordIndexes->capacity() >= triNumBefore + triAdded);
				//fill the indexes table with default values
				{
					int defaultElement[3] = {-1,-1,-1};
					for (unsigned i=0; i<mesh->m_texCoordIndexes->currentSize(); ++i)
						m_texCoordIndexes->addElement(defaultElement);
				}
			}
		}

		//the end!
		showWired(this->isShownAsWire() || mesh->isShownAsWire());
		enableStippling(this->stipplingEnabled() || mesh->stipplingEnabled());
		success = true;
		break;
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
	return m_triVertIndexes->currentSize();
}

unsigned ccMesh::capacity() const
{
	return m_triVertIndexes->capacity();
}

void ccMesh::forEach(genericTriangleAction& action)
{
	if (!m_associatedCloud)
		return;

	m_triVertIndexes->placeIteratorAtBegining();
	for (unsigned i=0; i<m_triVertIndexes->currentSize(); ++i)
	{
		const unsigned* tri = m_triVertIndexes->getCurrentValue();
		m_currentTriangle.A = m_associatedCloud->getPoint(tri[0]);
		m_currentTriangle.B = m_associatedCloud->getPoint(tri[1]);
		m_currentTriangle.C = m_associatedCloud->getPoint(tri[2]);
		action(m_currentTriangle);
		m_triVertIndexes->forwardIterator();
	}
}

void ccMesh::placeIteratorAtBegining()
{
	m_globalIterator = 0;
}

CCLib::GenericTriangle* ccMesh::_getNextTriangle()
{
	if (m_globalIterator < m_triVertIndexes->currentSize())
		return _getTriangle(m_globalIterator++);

	return NULL;
}

CCLib::GenericTriangle* ccMesh::_getTriangle(unsigned triangleIndex) //temporary
{
	assert(triangleIndex < m_triVertIndexes->currentSize());

	const unsigned* tri = m_triVertIndexes->getValue(triangleIndex);
	m_currentTriangle.A = m_associatedCloud->getPoint(tri[0]);
	m_currentTriangle.B = m_associatedCloud->getPoint(tri[1]);
	m_currentTriangle.C = m_associatedCloud->getPoint(tri[2]);

	return &m_currentTriangle;
}

void ccMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(triangleIndex < m_triVertIndexes->currentSize());

	const unsigned* tri = m_triVertIndexes->getValue(triangleIndex);
	m_associatedCloud->getPoint(tri[0],A);
	m_associatedCloud->getPoint(tri[1],B);
	m_associatedCloud->getPoint(tri[2],C);
}

void ccMesh::refreshBB()
{
	if (!m_associatedCloud || m_bBox.isValid())
		return;

	m_bBox.clear();

	unsigned count = m_triVertIndexes->currentSize();
	m_triVertIndexes->placeIteratorAtBegining();
	for (unsigned i=0; i<count; ++i)
	{
		const unsigned* tri = m_triVertIndexes->getCurrentValue();
		assert(tri[0] < m_associatedCloud->size() && tri[1] < m_associatedCloud->size() && tri[2] < m_associatedCloud->size());
		m_bBox.add(*m_associatedCloud->getPoint(tri[0]));
		m_bBox.add(*m_associatedCloud->getPoint(tri[1]));
		m_bBox.add(*m_associatedCloud->getPoint(tri[2]));
		m_triVertIndexes->forwardIterator();
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
	CCLib::VerticesIndexes t(i1,i2,i3);
	m_triVertIndexes->addElement(t.i);
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

	return m_triVertIndexes->reserve(n);
}

bool ccMesh::resize(unsigned n)
{
	m_bBox.setValidity(false);
	notifyGeometryUpdate();

	if (m_triMtlIndexes)
	{
		if (!m_triMtlIndexes->resize(n,true,-1))
			return false;
	}

	if (m_texCoordIndexes)
	{
		int defaultValues[3] = {-1,-1,-1};
		if (!m_texCoordIndexes->resize(n,true,defaultValues))
			return false;
	}

	if (m_triNormalIndexes)
	{
		int defaultValues[3] = {-1,-1,-1};
		if (!m_triNormalIndexes->resize(n,true,defaultValues))
			return false;
	}

	return m_triVertIndexes->resize(n);
}

void ccMesh::swapTriangles(unsigned index1, unsigned index2)
{
	assert(std::max(index1,index2) < size());

	m_triVertIndexes->swap(index1,index2);
	if (m_triMtlIndexes)
		m_triMtlIndexes->swap(index1,index2);
	if (m_texCoordIndexes)
		m_texCoordIndexes->swap(index1,index2);
	if (m_triNormalIndexes)
		m_triNormalIndexes->swap(index1,index2);
}

CCLib::VerticesIndexes* ccMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	return reinterpret_cast<CCLib::VerticesIndexes*>(m_triVertIndexes->getValue(triangleIndex));
}

const CCLib::VerticesIndexes* ccMesh::getTriangleVertIndexes(unsigned triangleIndex) const
{
	return reinterpret_cast<CCLib::VerticesIndexes*>(m_triVertIndexes->getValue(triangleIndex));
}

CCLib::VerticesIndexes* ccMesh::getNextTriangleVertIndexes()
{
	if (m_globalIterator<m_triVertIndexes->currentSize())
		return getTriangleVertIndexes(m_globalIterator++);

	return NULL;
}

unsigned ccMesh::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
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
		return;

	//3D pass
	if (MACRO_Draw3D(context))
	{
		//any triangle?
		unsigned n, triNum = m_triVertIndexes->currentSize();
		if (triNum == 0)
			return;

		//L.O.D.
		bool lodEnabled = (triNum > context.minLODTriangleCount && context.decimateMeshOnMove && MACRO_LODActivated(context));
		unsigned decimStep = (lodEnabled ? static_cast<unsigned>(ceil(static_cast<double>(triNum*3) / context.minLODTriangleCount)) : 1);

		//display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);
		//no normals shading without light!
		if (!MACRO_LightIsEnabled(context))
			glParams.showNorms = false;

		//vertices visibility
		const ccGenericPointCloud::VisibilityTableType* verticesVisibility = m_associatedCloud->getTheVisibilityArray();
		bool visFiltering = (verticesVisibility && verticesVisibility->isAllocated());

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
		ccScalarField* currentDisplayedScalarField = 0;
		bool greyForNanScalarValues = true;
		//unsigned colorRampSteps = 0;
		ccColorScale::Shared colorScale(0);

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
				assert(m_associatedCloud->isA(CC_TYPES::POINT_CLOUD));
				rgbColorsTable = static_cast<ccPointCloud*>(m_associatedCloud)->rgbColors();
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
#define OPTIM_MEM_CPY //use optimized mem. transfers
#ifdef OPTIM_MEM_CPY
			const unsigned step = 3*(decimStep-1);
#else
			const unsigned step = 3*decimStep;
#endif
			//the GL type depends on the PointCoordinateType 'size' (float or double)
			GLenum GL_COORD_TYPE = sizeof(PointCoordinateType) == 4 ? GL_FLOAT : GL_DOUBLE;

			glFunc->glEnableClientState(GL_VERTEX_ARRAY);
			glFunc->glVertexPointer(3, GL_COORD_TYPE, 0, GetVertexBuffer());

			if (glParams.showNorms)
			{
				glFunc->glEnableClientState(GL_NORMAL_ARRAY);
				glFunc->glNormalPointer(GL_COORD_TYPE,0,GetNormalsBuffer());
			}
			if (glParams.showSF || glParams.showColors)
			{
				glFunc->glEnableClientState(GL_COLOR_ARRAY);
				glFunc->glColorPointer(3,GL_UNSIGNED_BYTE,0,GetColorsBuffer());
			}

			//we can scan and process each chunk separately in an optimized way
			unsigned chunks = m_triVertIndexes->chunksCount();
			for (unsigned k=0; k<chunks; ++k)
			{
				const unsigned chunkSize = m_triVertIndexes->chunkSize(k);

				//vertices
				const unsigned* _vertIndexes = m_triVertIndexes->chunkStartPtr(k);
				PointCoordinateType* _vertices = GetVertexBuffer();
#ifdef OPTIM_MEM_CPY
				for (n=0; n<chunkSize; n+=decimStep,_vertIndexes+=step)
				{
					const CCVector3* P1 = m_associatedCloud->getPoint(*_vertIndexes++);
					*(_vertices)++ = P1->x;
					*(_vertices)++ = P1->y;
					*(_vertices)++ = P1->z;
					const CCVector3* P2 = m_associatedCloud->getPoint(*_vertIndexes++);
					*(_vertices)++ = P2->x;
					*(_vertices)++ = P2->y;
					*(_vertices)++ = P2->z;
					const CCVector3* P3 = m_associatedCloud->getPoint(*_vertIndexes++);
					*(_vertices)++ = P3->x;
					*(_vertices)++ = P3->y;
					*(_vertices)++ = P3->z;
				}
#else
				for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
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
					ColorCompType* _rgbColors = GetColorsBuffer();
					_vertIndexes = m_triVertIndexes->chunkStartPtr(k);
					assert(colorScale);
#ifdef OPTIM_MEM_CPY
					for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
					{
						const ColorCompType* col = currentDisplayedScalarField->getValueColor(*_vertIndexes++);
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;

						col = currentDisplayedScalarField->getValueColor(*_vertIndexes++);
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;

						col = currentDisplayedScalarField->getValueColor(*_vertIndexes++);
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;
						*(_rgbColors)++ = *(col)++;
					}
#else
					for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
					{
						const ColorCompType* col = currentDisplayedScalarField->getValueColor(_vertIndexes[0]);
						memcpy(_rgbColors,col,sizeof(ColorCompType)*3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(_vertIndexes[1]);
						memcpy(_rgbColors,col,sizeof(ColorCompType)*3);
						_rgbColors += 3;
						col = currentDisplayedScalarField->getValueColor(_vertIndexes[2]);
						memcpy(_rgbColors,col,sizeof(ColorCompType)*3);
						_rgbColors += 3;
					}
#endif
				}
				//colors
				else if (glParams.showColors)
				{
					ColorCompType* _rgbColors = GetColorsBuffer();
					_vertIndexes = m_triVertIndexes->chunkStartPtr(k);
#ifdef OPTIM_MEM_CPY
					for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
					{
						const ColorCompType* col = rgbColorsTable->getValue(*_vertIndexes++);
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
					for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
					{
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[0]),sizeof(ColorCompType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[1]),sizeof(ColorCompType)*3);
						_rgbColors += 3;
						memcpy(_rgbColors,rgbColorsTable->getValue(_vertIndexes[2]),sizeof(ColorCompType)*3);
						_rgbColors += 3;
					}
#endif
				}

				//normals
				if (glParams.showNorms)
				{
					PointCoordinateType* _normals = GetNormalsBuffer();
					if (showTriNormals)
					{
						assert(m_triNormalIndexes);
						int* _triNormalIndexes = m_triNormalIndexes->chunkStartPtr(k);
#ifdef OPTIM_MEM_CPY
						for (n=0; n<chunkSize; n+=decimStep, _triNormalIndexes+=step)
						{
							assert(*_triNormalIndexes < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N1 = (*_triNormalIndexes >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = N1.x;
							*(_normals)++ = N1.y;
							*(_normals)++ = N1.z;

							assert(*_triNormalIndexes < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N2 = (*_triNormalIndexes >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = N2.x;
							*(_normals)++ = N2.y;
							*(_normals)++ = N2.z;

							assert(*_triNormalIndexes < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N3 = (*_triNormalIndexes >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(*_triNormalIndexes)) : s_blankNorm);
							++_triNormalIndexes;
							*(_normals)++ = N3.x;
							*(_normals)++ = N3.y;
							*(_normals)++ = N3.z;
						}
#else
						for (n=0; n<chunkSize; n+=decimStep, _triNormalIndexes+=step)
						{
							assert(_triNormalIndexes[0] < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N1 = (_triNormalIndexes[0] >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[0])) : s_blankNorm);
							memcpy(_normals,N1.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							assert(_triNormalIndexes[1] < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N2 = (_triNormalIndexes[0]==_triNormalIndexes[1] ? N1 : _triNormalIndexes[1] >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[1])) : s_blankNorm);
							memcpy(_normals,N2.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							assert(_triNormalIndexes[2] < static_cast<int>(m_triNormals->currentSize()));
							const CCVector3& N3 = (_triNormalIndexes[0]==_triNormalIndexes[2] ? N1 : _triNormalIndexes[2] >= 0 ? compressedNormals->getNormal(m_triNormals->getValue(_triNormalIndexes[2])) : s_blankNorm);
							memcpy(_normals,N3.u,sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
#endif
					}
					else
					{
						_vertIndexes = m_triVertIndexes->chunkStartPtr(k);
#ifdef OPTIM_MEM_CPY
						for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
						{
							const CCVector3& N1 = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = N1.x;
							*(_normals)++ = N1.y;
							*(_normals)++ = N1.z;

							const CCVector3& N2 = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = N2.x;
							*(_normals)++ = N2.y;
							*(_normals)++ = N2.z;

							const CCVector3& N3 = compressedNormals->getNormal(normalsIndexesTable->getValue(*_vertIndexes++));
							*(_normals)++ = N3.x;
							*(_normals)++ = N3.y;
							*(_normals)++ = N3.z;
						}
#else
						for (n=0; n<chunkSize; n+=decimStep, _vertIndexes+=step)
						{
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[0])).u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[1])).u,sizeof(PointCoordinateType)*3);
							_normals+=3;
							memcpy(_normals,compressedNormals->getNormal(normalsIndexesTable->getValue(_vertIndexes[2])).u,sizeof(PointCoordinateType)*3);
							_normals+=3;
						}
#endif
					}
				}

				if (!showWired)
				{
					glFunc->glDrawArrays(lodEnabled ? GL_POINTS : GL_TRIANGLES, 0, (chunkSize / decimStep) * 3);
					//glFunc->glDrawElements(lodEnabled ? GL_POINTS : GL_TRIANGLES,(chunkSize/decimStep)*3,GL_UNSIGNED_INT,s_vertIndexes);
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
			const float *Tx1 = 0, *Tx2 = 0, *Tx3 = 0;

			//loop on all triangles
			m_triVertIndexes->placeIteratorAtBegining();

			int lasMtlIndex = -1;

			if (showTextures)
			{
				glFunc->glPushAttrib(GL_ENABLE_BIT);
				glFunc->glEnable(GL_TEXTURE_2D);
			}

			GLenum triangleDisplayType = lodEnabled ? GL_POINTS : showWired ? GL_LINE_LOOP : GL_TRIANGLES;
			glFunc->glBegin(triangleDisplayType);

			GLuint currentTexID = 0;

			for (n = 0; n < triNum; ++n)
			{
				//current triangle vertices
				const CCLib::VerticesIndexes* tsi = (CCLib::VerticesIndexes*)m_triVertIndexes->getCurrentValue();
				m_triVertIndexes->forwardIterator();

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
						assert(m_triNormalIndexes);
						const int* idx = m_triNormalIndexes->getValue(n);
						assert(idx[0] < static_cast<int>(m_triNormals->currentSize()));
						assert(idx[1] < static_cast<int>(m_triNormals->currentSize()));
						assert(idx[2] < static_cast<int>(m_triNormals->currentSize()));
						N1 = (idx[0] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[0])).u : 0);
						N2 = (idx[0] == idx[1] ? N1 : idx[1] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[1])).u : 0);
						N3 = (idx[0] == idx[2] ? N1 : idx[2] >= 0 ? ccNormalVectors::GetNormal(m_triNormals->getValue(idx[2])).u : 0);
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
						const int* txInd = m_texCoordIndexes->getValue(n);
						assert(txInd[0] < static_cast<int>(m_texCoords->currentSize()));
						assert(txInd[1] < static_cast<int>(m_texCoords->currentSize()));
						assert(txInd[2] < static_cast<int>(m_texCoords->currentSize()));
						Tx1 = (txInd[0] >= 0 ? m_texCoords->getValue(txInd[0]) : 0);
						Tx2 = (txInd[1] >= 0 ? m_texCoords->getValue(txInd[1]) : 0);
						Tx3 = (txInd[2] >= 0 ? m_texCoords->getValue(txInd[2]) : 0);
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
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi->i1)->u);

				//vertex 2
				if (N2)
					ccGL::Normal3v(glFunc, N2);
				if (col2)
					glFunc->glColor3ubv(col2);
				if (Tx2)
					glFunc->glTexCoord2fv(Tx2);
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi->i2)->u);

				//vertex 3
				if (N3)
					ccGL::Normal3v(glFunc, N3);
				if (col3)
					glFunc->glColor3ubv(col3);
				if (Tx3)
					glFunc->glTexCoord2fv(Tx3);
				ccGL::Vertex3v(glFunc, m_associatedCloud->getPoint(tsi->i3)->u);
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

		if (m_stippling)
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

ccMesh* ccMesh::createNewMeshFromSelection(bool removeSelectedFaces)
{
	if (!m_associatedCloud)
	{
		return NULL;
	}

	ccGenericPointCloud::VisibilityTableType* verticesVisibility = m_associatedCloud->getTheVisibilityArray();
	if (!verticesVisibility || !verticesVisibility->isAllocated())
	{
		ccLog::Error(QString("[Mesh %1] Internal error: vertex visibility table not instantiated!").arg(getName()));
		return NULL;
	}

	//create vertices for the new mesh
	ccGenericPointCloud* newVertices = m_associatedCloud->createNewCloudFromVisibilitySelection(false);
	if (!newVertices)
	{
		ccLog::Error(QString("[Mesh %1] Failed to create segmented mesh vertices! (not enough memory?)").arg(getName()));
		return NULL;
	}
	assert(newVertices);

	//create a 'reference' cloud if none was provided
	CCLib::ReferenceCloud* rc = 0;
	{
		//we create a temporary entity with the visible vertices only
		rc = new CCLib::ReferenceCloud(m_associatedCloud);

		for (unsigned i=0; i<m_associatedCloud->size(); ++i)
			if (verticesVisibility->getValue(i) == POINT_VISIBLE)
				if (!rc->addPointIndex(i))
				{
					ccLog::Error("Not enough memory!");
					delete rc;
					return 0;
				}
	}

	//nothing to do
	if (rc->size() == 0 || (removeSelectedFaces && rc->size() == m_associatedCloud->size()))
	{
		delete rc;
		return 0;
	}

	//we create a new mesh with the current selection
	CCLib::GenericIndexedMesh* result = CCLib::ManualSegmentationTools::segmentMesh(this, rc, true, NULL, newVertices);

	//don't use this anymore
	delete rc;
	rc = 0;

	ccMesh* newMesh = NULL;
	if (result)
	{
		newMesh = new ccMesh(result, newVertices);
		if (!newMesh)
		{
			delete newVertices;
			newVertices = NULL;
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
				NormsIndexesTableType* newTriNormals = 0;
				if (m_triNormals && m_triNormalIndexes)
				{
					assert(m_triNormalIndexes->currentSize() == m_triVertIndexes->currentSize());
					//create new 'minimal' subset
					newTriNormals = new NormsIndexesTableType();
					newTriNormals->link();
					try
					{
						newNormIndexes.resize(m_triNormals->currentSize(), -1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new normals subset! (not enough memory)");
						newMesh->removePerTriangleNormalIndexes();
						newTriNormals->release();
						newTriNormals = 0;
					}
				}

				//temporary structure for texture indexes mapping
				std::vector<int> newTexIndexes;
				TextureCoordsContainer* newTriTexIndexes = 0;
				if (m_texCoords && m_texCoordIndexes)
				{
					assert(m_texCoordIndexes->currentSize() == m_triVertIndexes->currentSize());
					//create new 'minimal' subset
					newTriTexIndexes = new TextureCoordsContainer();
					newTriTexIndexes->link();
					try
					{
						newTexIndexes.resize(m_texCoords->currentSize(), -1);
					}
					catch (const std::bad_alloc&)
					{
						ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new texture indexes subset! (not enough memory)");
						newMesh->removePerTriangleTexCoordIndexes();
						newTriTexIndexes->release();
						newTriTexIndexes = 0;
					}
				}

				//temporary structure for material indexes mapping
				std::vector<int> newMatIndexes;
				ccMaterialSet* newMaterials = 0;
				if (m_materials && m_triMtlIndexes)
				{
					assert(m_triMtlIndexes->currentSize() == m_triVertIndexes->currentSize());
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
						newMaterials = 0;
						if (newTriTexIndexes) //we can release texture coordinates as well (as they depend on materials!)
						{
							newMesh->removePerTriangleTexCoordIndexes();
							newTriTexIndexes->release();
							newTriTexIndexes = 0;
							newTexIndexes.clear();
						}
					}
				}

				unsigned triNum = m_triVertIndexes->currentSize();
				m_triVertIndexes->placeIteratorAtBegining();
				for (unsigned i = 0; i < triNum; ++i)
				{
					const CCLib::VerticesIndexes* tsi = (CCLib::VerticesIndexes*)m_triVertIndexes->getCurrentValue();
					m_triVertIndexes->forwardIterator();

					//all vertices must be visible
					if (verticesVisibility->getValue(tsi->i1) == POINT_VISIBLE &&
						verticesVisibility->getValue(tsi->i2) == POINT_VISIBLE &&
						verticesVisibility->getValue(tsi->i3) == POINT_VISIBLE)
					{
						//import per-triangle normals?
						if (newTriNormals)
						{
							assert(m_triNormalIndexes);

							//current triangle (compressed) normal indexes
							const int* triNormIndexes = m_triNormalIndexes->getValue(i);

							//for each triangle of this mesh, try to determine if its normals are already in use
							//(otherwise add them to the new container and increase its index)
							for (unsigned j = 0; j < 3; ++j)
							{
								if (triNormIndexes[j] >= 0 && newNormIndexes[triNormIndexes[j]] < 0)
								{
									if (newTriNormals->currentSize() == newTriNormals->capacity() 
										&& !newTriNormals->reserve(newTriNormals->currentSize() + 1000)) //auto expand
									{
										ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new normals subset! (not enough memory)");
										newMesh->removePerTriangleNormalIndexes();
										newTriNormals->release();
										newTriNormals = 0;
										break;
									}

									//import old normal to new subset (create new index)
									newNormIndexes[triNormIndexes[j]] = (int)newTriNormals->currentSize(); //new element index = new size - 1 = old size!
									newTriNormals->addElement(m_triNormals->getValue(triNormIndexes[j]));
								}
							}

							if (newTriNormals) //structure still exists?
							{
								newMesh->addTriangleNormalIndexes(	triNormIndexes[0] < 0 ? -1 : newNormIndexes[triNormIndexes[0]],
																	triNormIndexes[1] < 0 ? -1 : newNormIndexes[triNormIndexes[1]],
																	triNormIndexes[2] < 0 ? -1 : newNormIndexes[triNormIndexes[2]]);
							}
						}

						//import texture coordinates?
						if (newTriTexIndexes)
						{
							assert(m_texCoordIndexes);

							//current triangle texture coordinates indexes
							const int* triTexIndexes = m_texCoordIndexes->getValue(i);

							//for each triangle of this mesh, try to determine if its textures coordinates are already in use
							//(otherwise add them to the new container and increase its index)
							for (unsigned j=0; j<3; ++j)
							{
								if (triTexIndexes[j] >= 0 && newTexIndexes[triTexIndexes[j]] < 0)
								{
									if (newTriTexIndexes->currentSize() == newTriTexIndexes->capacity() 
										&& !newTriTexIndexes->reserve(newTriTexIndexes->currentSize() + 500)) //auto expand
									{
										ccLog::Error("Failed to create new texture coordinates subset! (not enough memory)");
										newMesh->removePerTriangleTexCoordIndexes();
										newTriTexIndexes->release();
										newTriTexIndexes = 0;
										break;
									}
									//import old texture coordinate to new subset (create new index)
									newTexIndexes[triTexIndexes[j]] = static_cast<int>(newTriTexIndexes->currentSize()); //new element index = new size - 1 = old size!
									newTriTexIndexes->addElement(m_texCoords->getValue(triTexIndexes[j]));
								}
							}

							if (newTriTexIndexes) //structure still exists?
							{
								newMesh->addTriangleTexCoordIndexes(triTexIndexes[0] < 0 ? -1 : newTexIndexes[triTexIndexes[0]],
																	triTexIndexes[1] < 0 ? -1 : newTexIndexes[triTexIndexes[1]],
																	triTexIndexes[2] < 0 ? -1 : newTexIndexes[triTexIndexes[2]]);
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
									newMaterials->push_back(m_materials->at(triMatIndex));
								}
								catch (const std::bad_alloc&)
								{
									ccLog::Warning("[ccMesh::createNewMeshFromSelection] Failed to create new materials subset! (not enough memory)");
									newMesh->removePerTriangleMtlIndexes();
									newMaterials->release();
									newMaterials = 0;
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
					newTriNormals->resize(newTriNormals->currentSize()); //smaller so it should always be ok!
					newMesh->setTriNormsTable(newTriNormals);
					newTriNormals->release();
					newTriNormals = 0;
				}

				if (newTriTexIndexes)
				{
					newMesh->setTexCoordinatesTable(newTriTexIndexes);
					newTriTexIndexes->release();
					newTriTexIndexes = 0;
				}

				if (newMaterials)
				{
					newMesh->setMaterialSet(newMaterials);
					newMaterials->release();
					newMaterials = 0;
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
		result = 0;
	}

	unsigned triNum = m_triVertIndexes->currentSize();

	//we must modify eventual sub-meshes!
	ccHObject::Container subMeshes;
	if (filterChildren(subMeshes,false,CC_TYPES::SUB_MESH) != 0)
	{
		//create index map
		ccSubMesh::IndexMap* indexMap = new ccSubMesh::IndexMap;
		if (!indexMap->reserve(triNum))
		{
			ccLog::Error("Not enough memory! Sub-meshes will be lost...");
			newMesh->setVisible(true); //force parent mesh visibility in this case!

			for (size_t i=0; i<subMeshes.size(); ++i)
				removeChild(subMeshes[i]);
		}
		else
		{
			//finish index map creation
			{
				unsigned newVisibleIndex = 0;
				unsigned newInvisibleIndex = 0;

				m_triVertIndexes->placeIteratorAtBegining();
				for (unsigned i=0; i<triNum; ++i)
				{
					const CCLib::VerticesIndexes* tsi = reinterpret_cast<CCLib::VerticesIndexes*>(m_triVertIndexes->getCurrentValue());
					m_triVertIndexes->forwardIterator();

					//at least one hidden vertex --> we keep it
					if (verticesVisibility->getValue(tsi->i1) != POINT_VISIBLE ||
						verticesVisibility->getValue(tsi->i2) != POINT_VISIBLE ||
						verticesVisibility->getValue(tsi->i3) != POINT_VISIBLE)
					{
						indexMap->addElement(removeSelectedFaces ? newInvisibleIndex++ : i);
					}
					else
					{
						indexMap->addElement(newVisibleIndex++);
					}
				}
			}

			for (size_t i=0; i<subMeshes.size(); ++i)
			{
				ccSubMesh* subMesh = static_cast<ccSubMesh*>(subMeshes[i]);
				ccSubMesh* subMesh2 = subMesh->createNewSubMeshFromSelection(removeSelectedFaces,indexMap);

				if (subMesh->size() == 0) //no more faces in current sub-mesh?
				{
					detachChild(subMesh); //FIXME: removeChild instead?
					subMesh = 0;
				}

				if (subMesh2)
				{
					subMesh2->setAssociatedMesh(newMesh);
					newMesh->addChild(subMesh2);
				}
			}
		}

		indexMap->release();
		indexMap = 0;
	}

	//shall we remove the selected faces from this mesh?
	if (removeSelectedFaces)
	{
		//we remove all fully visible faces
		unsigned lastTri = 0;
		m_triVertIndexes->placeIteratorAtBegining();
		for (unsigned i=0; i<triNum; ++i)
		{
			const CCLib::VerticesIndexes* tsi = reinterpret_cast<CCLib::VerticesIndexes*>(m_triVertIndexes->getCurrentValue());
			m_triVertIndexes->forwardIterator();

			//at least one hidden vertex --> we keep it
			if (verticesVisibility->getValue(tsi->i1) != POINT_VISIBLE ||
				verticesVisibility->getValue(tsi->i2) != POINT_VISIBLE ||
				verticesVisibility->getValue(tsi->i3) != POINT_VISIBLE)
			{
				if (i != lastTri)
				{
					m_triVertIndexes->setValue(lastTri, (unsigned*)tsi);

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
	m_triVertIndexes->placeIteratorAtBegining();
	for (unsigned i=0; i<m_triVertIndexes->currentSize(); ++i)
	{
		unsigned *ti = m_triVertIndexes->getCurrentValue();
		ti[0] += shift;
		ti[1] += shift;
		ti[2] += shift;
		m_triVertIndexes->forwardIterator();
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
	m_triNormalIndexes = 0;
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

	return m_triNormalIndexes->reserve(m_triVertIndexes->capacity());
}

void ccMesh::addTriangleNormalIndexes(int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	int indexes[3] = { i1, i2, i3 };
	m_triNormalIndexes->addElement(indexes);
}

void ccMesh::setTriangleNormalIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_triNormalIndexes && m_triNormalIndexes->currentSize() > triangleIndex);
	int indexes[3] = { i1, i2, i3 };
	m_triNormalIndexes->setValue(triangleIndex,indexes);
}

void ccMesh::getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const
{
	if (m_triNormalIndexes && m_triNormalIndexes->currentSize() > triangleIndex)
	{
		int* indexes = m_triNormalIndexes->getValue(triangleIndex);
		i1 = indexes[0];
		i2 = indexes[1];
		i3 = indexes[2];
	}
	else
	{
		i1 = i2 = i3 = -1;
	}
}

bool ccMesh::getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const
{
	if (m_triNormals && m_triNormalIndexes && m_triNormalIndexes->currentSize() > triangleIndex)
	{
		int* indexes = m_triNormalIndexes->getValue(triangleIndex);
		if (indexes[0] >= 0)
			Na = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes[0]));
		else
			Na = CCVector3(0,0,0);
		if (indexes[1] >= 0)
			Nb = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes[1]));
		else
			Nb = CCVector3(0,0,0);
		if (indexes[2] >= 0)
			Nc = ccNormalVectors::GetUniqueInstance()->getNormal(m_triNormals->getValue(indexes[2]));
		else
			Nc = CCVector3(0,0,0);

		return true;
	}

	return false;
}

bool ccMesh::hasTriNormals() const
{
	return m_triNormals && m_triNormals->isAllocated() && m_triNormalIndexes && (m_triNormalIndexes->currentSize() == m_triVertIndexes->currentSize());
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
		m_texCoords = 0;
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

void ccMesh::getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const
{
	if (m_texCoords && m_texCoordIndexes)
	{
		const int* txInd = m_texCoordIndexes->getValue(triIndex);
		tx1 = (txInd[0] >= 0 ? m_texCoords->getValue(txInd[0]) : 0);
		tx2 = (txInd[1] >= 0 ? m_texCoords->getValue(txInd[1]) : 0);
		tx3 = (txInd[2] >= 0 ? m_texCoords->getValue(txInd[2]) : 0);
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

	return m_texCoordIndexes->reserve(m_triVertIndexes->capacity());
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
	int indexes[3] = { i1, i2, i3 };
	m_texCoordIndexes->addElement(indexes);
}

void ccMesh::setTriangleTexCoordIndexes(unsigned triangleIndex, int i1, int i2, int i3)
{
	assert(m_texCoordIndexes && m_texCoordIndexes->currentSize() > triangleIndex);
	int indexes[3] = { i1, i2, i3 };
	m_texCoordIndexes->setValue(triangleIndex,indexes);
}

void ccMesh::getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const
{
	assert(m_texCoordIndexes && m_texCoordIndexes->currentSize() > triangleIndex);

	const int* tci = m_texCoordIndexes->getValue(triangleIndex);
	i1 = tci[0];
	i2 = tci[1];
	i3 = tci[2];
}

bool ccMesh::hasTextures() const
{
	return hasMaterials() && m_texCoords && m_texCoords->isAllocated() && m_texCoordIndexes && (m_texCoordIndexes->currentSize() == m_triVertIndexes->currentSize());
}

/*********************************************************/
/**************    PER-TRIANGLE MATERIALS    *************/
/*********************************************************/

bool ccMesh::hasMaterials() const
{
	return m_materials && !m_materials->empty() && m_triMtlIndexes && (m_triMtlIndexes->currentSize() == m_triVertIndexes->currentSize());
}

void ccMesh::setTriangleMtlIndexesTable(triangleMaterialIndexesSet* matIndexesTable, bool autoReleaseOldTable/*=true*/)
{
	if (m_triMtlIndexes == matIndexesTable)
		return;

	if (m_triMtlIndexes && autoReleaseOldTable)
	{
		m_triMtlIndexes->release();
		m_triMtlIndexes = 0;
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

	return m_triMtlIndexes->reserve(m_triVertIndexes->capacity());
}

void ccMesh::removePerTriangleMtlIndexes()
{
	if (m_triMtlIndexes)
		m_triMtlIndexes->release();
	m_triMtlIndexes = 0;
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

int ccMesh::getTriangleMtlIndex(unsigned triangleIndex) const
{
	assert(m_triMtlIndexes && m_triMtlIndexes->currentSize() > triangleIndex);
	return m_triMtlIndexes->getValue(triangleIndex);
}

bool ccMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericMesh::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple meshes)
	//so instead we save it's unique ID (dataVersion>=20)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	uint32_t vertUniqueID = (m_associatedCloud ? static_cast<uint32_t>(m_associatedCloud->getUniqueID()) : 0);
	if (out.write((const char*)&vertUniqueID,4) < 0)
		return WriteError();

	//per-triangle normals array (dataVersion>=20)
	{
		//we can't save the normals array here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the normals array must be saved in the same BIN file! (responsibility of the caller)
		uint32_t normArrayID = (m_triNormals && m_triNormals->isAllocated() ? static_cast<uint32_t>(m_triNormals->getUniqueID()) : 0);
		if (out.write((const char*)&normArrayID,4) < 0)
			return WriteError();
	}

	//texture coordinates array (dataVersion>=20)
	{
		//we can't save the texture coordinates array here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the texture coordinates array must be saved in the same BIN file! (responsibility of the caller)
		uint32_t texCoordArrayID = (m_texCoords && m_texCoords->isAllocated() ? static_cast<uint32_t>(m_texCoords->getUniqueID()) : 0);
		if (out.write((const char*)&texCoordArrayID,4) < 0)
			return WriteError();
	}

	//materials
	{
		//we can't save the material set here (as it may be shared by multiple meshes)
		//so instead we save it's unique ID (dataVersion>=20)
		//WARNING: the material set must be saved in the same BIN file! (responsibility of the caller)
		uint32_t matSetID = (m_materials ? static_cast<uint32_t>(m_materials->getUniqueID()) : 0);
		if (out.write((const char*)&matSetID,4) < 0)
			return WriteError();
	}

	//triangles indexes (dataVersion>=20)
	if (!m_triVertIndexes)
		return ccLog::Error("Internal error: mesh has no triangles array! (not enough memory?)");
	if (!ccSerializationHelper::GenericArrayToFile(*m_triVertIndexes,out))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = hasPerTriangleMtlIndexes();
	if (out.write((const char*)&hasTriMtlIndexes,sizeof(bool)) < 0)
		return WriteError();
	if (hasTriMtlIndexes)
	{
		assert(m_triMtlIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_triMtlIndexes,out))
			return false;
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = hasPerTriangleTexCoordIndexes();
	if (out.write((const char*)&hasTexCoordIndexes,sizeof(bool)) < 0)
		return WriteError();
	if (hasTexCoordIndexes)
	{
		assert(m_texCoordIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_texCoordIndexes,out))
			return false;
	}

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = (m_triNormalIndexes && m_triNormalIndexes->isAllocated());
	if (out.write((const char*)&hasTriNormalIndexes,sizeof(bool)) < 0)
		return WriteError();
	if (hasTriNormalIndexes)
	{
		assert(m_triNormalIndexes);
		if (!ccSerializationHelper::GenericArrayToFile(*m_triNormalIndexes,out))
			return false;
	}

	return true;
}

bool ccMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericMesh::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by multiple meshes)
	//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID,4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_associatedCloud) = vertUniqueID;

	//per-triangle normals array (dataVersion>=20)
	{
		//as the associated normals array can't be saved directly (as it may be shared by multiple meshes)
		//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
		//is the responsibility of the caller to make sure that all dependencies are saved together)
		uint32_t normArrayID = 0;
		if (in.read((char*)&normArrayID,4) < 0)
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
		if (in.read((char*)&texCoordArrayID,4) < 0)
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
		if (in.read((char*)&matSetID,4) < 0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the array unique ID in the 'm_materials' pointer!!!
		*(uint32_t*)(&m_materials) = matSetID;
	}

	//triangles indexes (dataVersion>=20)
	if (!m_triVertIndexes)
		return false;
	if (!ccSerializationHelper::GenericArrayFromFile(*m_triVertIndexes,in,dataVersion))
		return false;

	//per-triangle materials (dataVersion>=20))
	bool hasTriMtlIndexes = false;
	if (in.read((char*)&hasTriMtlIndexes,sizeof(bool)) < 0)
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
			m_triMtlIndexes = 0;
			return false;
		}
	}

	//per-triangle texture coordinates indexes (dataVersion>=20))
	bool hasTexCoordIndexes = false;
	if (in.read((char*)&hasTexCoordIndexes,sizeof(bool)) < 0)
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
			m_texCoordIndexes = 0;
			return false;
		}
	}

	//'materials shown' state (dataVersion>=20 && dataVersion<29))
	if (dataVersion<29)
	{
		bool materialsShown = false;
		if (in.read((char*)&materialsShown,sizeof(bool)) < 0)
			return ReadError();
		showMaterials(materialsShown);
	}

	//per-triangle normals  indexes (dataVersion>=20))
	bool hasTriNormalIndexes = false;
	if (in.read((char*)&hasTriNormalIndexes,sizeof(bool)) < 0)
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

	if (dataVersion<29)
	{
		//'per-triangle normals shown' state (dataVersion>=20 && dataVersion<29))
		bool triNormsShown = false;
		if (in.read((char*)&triNormsShown,sizeof(bool)) < 0)
			return ReadError();
		showTriNorms(triNormsShown);

		//'polygon stippling' state (dataVersion>=20 && dataVersion<29))
		bool stippling = false;
		if (in.read((char*)&stippling,sizeof(bool)) < 0)
			return ReadError();
		enableStippling(stippling);
	}

	notifyGeometryUpdate();

	return true;
}

void ccMesh::computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const
{
	assert(triIndex < m_triVertIndexes->currentSize());

	const unsigned* tri = m_triVertIndexes->getValue(triIndex);
	return computeInterpolationWeights(tri[0],tri[1],tri[2],P,weights);
}

void ccMesh::computeInterpolationWeights(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, CCVector3d& weights) const
{
	const CCVector3 *A = m_associatedCloud->getPoint(i1);
	const CCVector3 *B = m_associatedCloud->getPoint(i2);
	const CCVector3 *C = m_associatedCloud->getPoint(i3);

	//barcyentric intepolation weights
	weights.x = sqrt(((P-*B).cross(*C-*B)).norm2d())/*/2*/;
	weights.y = sqrt(((P-*C).cross(*A-*C)).norm2d())/*/2*/;
	weights.z = sqrt(((P-*A).cross(*B-*A)).norm2d())/*/2*/;

	//normalize weights
	double sum = weights.x + weights.y + weights.z;
	weights /= sum;
}

bool ccMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	assert(triIndex < size());

	if (!hasNormals())
		return false;

	const unsigned* tri = m_triVertIndexes->getValue(triIndex);

	return interpolateNormals(tri[0],tri[1],tri[2],P,N, hasTriNormals() ? m_triNormalIndexes->getValue(triIndex) : 0);
}

bool ccMesh::interpolateNormals(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, CCVector3& N, const int* triNormIndexes/*=0*/)
{
	//intepolation weights
	CCVector3d w;
	computeInterpolationWeights(i1,i2,i3,P,w);

	CCVector3d Nd(0,0,0);
	{
		if (!triNormIndexes || triNormIndexes[0] >= 0)
		{
			const CCVector3& N1 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes[0])) : m_associatedCloud->getPointNormal(i1);
			Nd += CCVector3d(N1.x,N1.y,N1.z) * w.u[0];
		}

		if (!triNormIndexes || triNormIndexes[1] >= 0)
		{
			const CCVector3& N2 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes[1])) : m_associatedCloud->getPointNormal(i2);
			Nd += CCVector3d(N2.x,N2.y,N2.z) * w.u[1];
		}

		if (!triNormIndexes || triNormIndexes[2] >= 0)
		{
			const CCVector3& N3 = triNormIndexes ? ccNormalVectors::GetNormal(m_triNormals->getValue(triNormIndexes[2])) : m_associatedCloud->getPointNormal(i3);
			Nd += CCVector3d(N3.x,N3.y,N3.z) * w.u[2];
		}
		Nd.normalize();
	}

	N = CCVector3::fromArray(Nd.u);

	return true;
}

bool ccMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C)
{
	assert(triIndex < size());

	if (!hasColors())
		return false;

	const unsigned* tri = m_triVertIndexes->getValue(triIndex);

	return interpolateColors(tri[0],tri[1],tri[2],P,C);
}

bool ccMesh::interpolateColors(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, ccColor::Rgb& rgb)
{
	//intepolation weights
	CCVector3d w;
	computeInterpolationWeights(i1,i2,i3,P,w);

	const ColorCompType* C1 = m_associatedCloud->getPointColor(i1);
	const ColorCompType* C2 = m_associatedCloud->getPointColor(i2);
	const ColorCompType* C3 = m_associatedCloud->getPointColor(i3);

	rgb.r = static_cast<ColorCompType>(floor(C1[0]*w.u[0] + C2[0]*w.u[1] + C3[0]*w.u[2]));
	rgb.g = static_cast<ColorCompType>(floor(C1[1]*w.u[0] + C2[1]*w.u[1] + C3[1]*w.u[2]));
	rgb.b = static_cast<ColorCompType>(floor(C1[2]*w.u[0] + C2[2]*w.u[1] + C3[2]*w.u[2]));

	return true;
}

bool ccMesh::getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& rgb, bool returnColorIfNoTexture)
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

	const unsigned* tri = m_triVertIndexes->getValue(triIndex);
	assert(tri);

	//do we need to change material?
	bool foundMaterial = false;
	if (matIndex >= 0)
	{
		ccMaterial::CShared material = (*m_materials)[matIndex];
		if (material->hasTexture())
		{
			assert(m_texCoords && m_texCoordIndexes);
			const int* txInd = m_texCoordIndexes->getValue(triIndex);
			const float* Tx = (txInd[vertIndex] >= 0 ? m_texCoords->getValue(txInd[vertIndex]) : 0);
			if (Tx)
			{
				if (Tx[0] >= 0 && Tx[0] <= 1.0f && Tx[1] >= 0 && Tx[1] <= 1.0f)
				{
					//get color from texture image
					const QImage texture = material->getTexture();
					int xPix = std::min(static_cast<int>(floor(Tx[0]*texture.width())),texture.width()-1);
					int yPix = std::min(static_cast<int>(floor(Tx[1]*texture.height())),texture.height()-1);

					QRgb pixel = texture.pixel(xPix,yPix);

					rgb.r = static_cast<ColorCompType>(qRed(pixel));
					rgb.g = static_cast<ColorCompType>(qGreen(pixel));
					rgb.b = static_cast<ColorCompType>(qBlue(pixel));

					foundMaterial = true;
				}
			}
		}
		else
		{
			const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
			rgb.r = static_cast<ColorCompType>(diffuse.r * ccColor::MAX);
			rgb.g = static_cast<ColorCompType>(diffuse.g * ccColor::MAX);
			rgb.b = static_cast<ColorCompType>(diffuse.b * ccColor::MAX);

			foundMaterial = true;
		}
	}

	if (!foundMaterial && returnColorIfNoTexture && hasColors())
	{
		rgb = ccColor::Rgb(m_associatedCloud->getPointColor(tri[vertIndex]));
		foundMaterial = true;
	}

	return foundMaterial;
}

bool ccMesh::getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb, bool interpolateColorIfNoTexture)
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
			return interpolateColors(triIndex,P,rgb);
		return false;
	}

	ccMaterial::CShared material = (*m_materials)[matIndex];

	if (!material->hasTexture())
	{
		const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
		rgb.r = static_cast<ColorCompType>(diffuse.r * ccColor::MAX);
		rgb.g = static_cast<ColorCompType>(diffuse.g * ccColor::MAX);
		rgb.b = static_cast<ColorCompType>(diffuse.b * ccColor::MAX);
		return true;
	}

	assert(m_texCoords && m_texCoordIndexes);
	const int* txInd = m_texCoordIndexes->getValue(triIndex);
	const float* Tx1 = (txInd[0] >= 0 ? m_texCoords->getValue(txInd[0]) : 0);
	const float* Tx2 = (txInd[1] >= 0 ? m_texCoords->getValue(txInd[1]) : 0);
	const float* Tx3 = (txInd[2] >= 0 ? m_texCoords->getValue(txInd[2]) : 0);

	//intepolation weights
	CCVector3d w;
	computeInterpolationWeights(triIndex,P,w);

	if (	(!Tx1 && w.u[0] > ZERO_TOLERANCE)
		||	(!Tx2 && w.u[1] > ZERO_TOLERANCE)
		||	(!Tx3 && w.u[2] > ZERO_TOLERANCE) )
	{
		//assert(false);
		if (interpolateColorIfNoTexture)
			return interpolateColors(triIndex,P,rgb);
		return false;
	}

	double x = (Tx1 ? Tx1[0]*w.u[0] : 0.0) + (Tx2 ? Tx2[0]*w.u[1] : 0.0) + (Tx3 ? Tx3[0]*w.u[2] : 0.0);
	double y = (Tx1 ? Tx1[1]*w.u[0] : 0.0) + (Tx2 ? Tx2[1]*w.u[1] : 0.0) + (Tx3 ? Tx3[1]*w.u[2] : 0.0);

	//DGM: we mut handle texture coordinates below 0 or above 1 (i.e. repetition)
	//if (x < 0 || x > 1.0 || y < 0 || y > 1.0)
	if (x > 1.0)
	{
		double xFrac,xInt;
		xFrac = modf(x,&xInt);
		x = xFrac;
	}
	else if (x < 0.0)
	{
		double xFrac,xInt;
		xFrac = modf(x,&xInt);
		x = 1.0 + xFrac;
	}

	//same thing for y
	if (y > 1.0)
	{
		double yFrac,yInt;
		yFrac = modf(y,&yInt);
		y = yFrac;
	}
	else if (y < 0.0)
	{
		double yFrac,yInt;
		yFrac = modf(y,&yInt);
		y = 1.0 + yFrac;
	}

	//get color from texture image
	{
		const QImage texture = material->getTexture();
		int xPix = std::min(static_cast<int>(floor(x*texture.width())),texture.width()-1);
		int yPix = std::min(static_cast<int>(floor(y*texture.height())),texture.height()-1);

		QRgb pixel = texture.pixel(xPix,yPix);

		const ccColor::Rgbaf& diffuse = material->getDiffuseFront();
		rgb.r = static_cast<ColorCompType>(diffuse.r * qRed  (pixel));
		rgb.g = static_cast<ColorCompType>(diffuse.g * qGreen(pixel));
		rgb.b = static_cast<ColorCompType>(diffuse.b * qBlue (pixel));
	}

	return true;
}

//we use as many static variables as we can to limit the size of the heap used by each recursion...
static const unsigned s_defaultSubdivideGrowRate = 50;
static PointCoordinateType s_maxSubdivideArea = 1;
static QMap<qint64,unsigned> s_alreadyCreatedVertices; //map to store already created edges middle points

static qint64 GenerateKey(unsigned edgeIndex1, unsigned edgeIndex2)
{
	if (edgeIndex1>edgeIndex2)
		std::swap(edgeIndex1,edgeIndex2);

	return ((((qint64)edgeIndex1)<<32) | (qint64)edgeIndex2);
}

bool ccMesh::pushSubdivide(/*PointCoordinateType maxArea, */unsigned indexA, unsigned indexB, unsigned indexC)
{
	if (s_maxSubdivideArea/*maxArea*/ <= ZERO_TOLERANCE)
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

	//do we need to sudivide this triangle?
	PointCoordinateType area = ((*B-*A)*(*C-*A)).norm()/2;
	if (area > s_maxSubdivideArea/*maxArea*/)
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
			//We have to update pointers as they may have been wrangled by the 'reserve' call
			A = vertices->getPoint(indexA);
			B = vertices->getPoint(indexB);
			C = vertices->getPoint(indexC);
		}

		//add new vertices
		unsigned indexG1 = 0;
		{
			qint64 key = GenerateKey(indexA,indexB);
			QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG1 = vertices->size();
				CCVector3 G1 = (*A+*B)/(PointCoordinateType)2.0;
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
					ccColor::Rgb C;
					interpolateColors(indexA,indexB,indexC,G1,C);
					vertices->addRGBColor(C.rgb);
				}
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
			qint64 key = GenerateKey(indexB,indexC);
			QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG2 = vertices->size();
				CCVector3 G2 = (*B+*C)/(PointCoordinateType)2.0;
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
					ccColor::Rgb C;
					interpolateColors(indexA,indexB,indexC,G2,C);
					vertices->addRGBColor(C.rgb);
				}
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
			qint64 key = GenerateKey(indexC,indexA);
			QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(key);
			if (it == s_alreadyCreatedVertices.end())
			{
				//generate new vertex
				indexG3 = vertices->size();
				CCVector3 G3 = (*C+*A)/(PointCoordinateType)2.0;
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
					ccColor::Rgb C;
					interpolateColors(indexA,indexB,indexC,G3,C);
					vertices->addRGBColor(C.rgb);
				}
				//and add it to the map
				s_alreadyCreatedVertices.insert(key,indexG3);
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
			if (!reserve(size() + 3*s_defaultSubdivideGrowRate))
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

ccMesh* ccMesh::subdivide(PointCoordinateType maxArea) const
{
	if (maxArea <= ZERO_TOLERANCE)
	{
		ccLog::Error("[ccMesh::subdivide] Invalid input argument!");
		return 0;
	}
	s_maxSubdivideArea = maxArea;

	unsigned triCount = size();
	ccGenericPointCloud* vertices = getAssociatedCloud();
	unsigned vertCount = (vertices ? vertices->size() : 0);
	if (!vertices || vertCount*triCount == 0)
	{
		ccLog::Error("[ccMesh::subdivide] Invalid mesh: no face or no vertex!");
		return 0;
	}

	ccPointCloud* resultVertices = vertices->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(vertices)->cloneThis() : ccPointCloud::From(vertices,vertices);
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
		for (unsigned i=0; i<triCount; ++i)
		{
			const unsigned* tri = m_triVertIndexes->getValue(i);
			if (!resultMesh->pushSubdivide(/*maxArea,*/tri[0],tri[1],tri[2]))
			{
				ccLog::Error("[ccMesh::subdivide] Not enough memory!");
				delete resultMesh;
				return 0;
			}
		}
	}
	catch(...)
	{
		ccLog::Error("[ccMesh::subdivide] An error occurred!");
		delete resultMesh;
		return 0;
	}

	//we must also 'fix' the triangles that share (at least) an edge with a subdivided triangle!
	try
	{
		unsigned newTriCount = resultMesh->size();
		for (unsigned i=0; i<newTriCount; ++i)
		{
			unsigned* _face = resultMesh->m_triVertIndexes->getValue(i); //warning: array might change at each call to reallocate!
			unsigned indexA = _face[0];
			unsigned indexB = _face[1];
			unsigned indexC = _face[2];

			//test all edges
			int indexG1 = -1;
			{
				QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexA,indexB));
				if (it != s_alreadyCreatedVertices.end())
					indexG1 = (int)it.value();
			}
			int indexG2 = -1;
			{
				QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexB,indexC));
				if (it != s_alreadyCreatedVertices.end())
					indexG2 = (int)it.value();
			}
			int indexG3 = -1;
			{
				QMap<qint64,unsigned>::const_iterator it = s_alreadyCreatedVertices.find(GenerateKey(indexC,indexA));
				if (it != s_alreadyCreatedVertices.end())
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
				_face[0] = indexes[i1];
				_face[1] = indexG;
				_face[2] = indexes[(i1+2)%3];
				//and add the other half (we can use pushSubdivide as the area should alredy be ok!)
				if (!resultMesh->pushSubdivide(/*maxArea,*/indexes[i1],indexes[(i1+1)%3],indexG))
				{
					ccLog::Error("[ccMesh::subdivide] Not enough memory!");
					delete resultMesh;
					return 0;
				}
			}
			else if (brokenEdges == 2)
			{
				if (indexG1 < 0) //broken edges: BC and CA
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexC;
					_face[1] = indexG3;
					_face[2] = indexG2;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexA, indexG2, indexG3) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexA, indexB, indexG2))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return 0;
					}
				}
				else if (indexG2 < 0) //broken edges: AB and CA
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexA;
					_face[1] = indexG1;
					_face[2] = indexG3;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexB, indexG3, indexG1) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexB, indexC, indexG3))
					{
						ccLog::Error("[ccMesh::subdivide] Not enough memory!");
						delete resultMesh;
						return 0;
					}
				}
				else /*if (indexG3 < 0)*/ //broken edges: AB and BC
				{
					//replace current triangle by the 'pointy' part
					_face[0] = indexB;
					_face[1] = indexG2;
					_face[2] = indexG1;
					//split the remaining 'trapezoid' in 2
					if (!resultMesh->pushSubdivide(/*maxArea, */indexC, indexG1, indexG2) ||
						!resultMesh->pushSubdivide(/*maxArea, */indexC, indexA, indexG1))
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
				if (!resultMesh->pushSubdivide(/*maxArea, */indexB, indexG2, indexG1) ||
					!resultMesh->pushSubdivide(/*maxArea, */indexC, indexG3, indexG2) ||
					!resultMesh->pushSubdivide(/*maxArea, */indexG1, indexG2, indexG3))
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
		ccLog::Error("[ccMesh::subdivide] An error occurred!");
		delete resultMesh;
		return 0;
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

	placeIteratorAtBegining();
	for (unsigned i=0; i<faceCount; ++i)
	{
		const CCLib::VerticesIndexes* tsi = getNextTriangleVertIndexes();
		for (unsigned char j=0; j<3; ++j)
		{
			ccColor::Rgb C;
			if (getVertexColorFromMaterial(i,j,C,true))
			{
				//FIXME: could we be smarter? (we process each point several times! And we assume the color is always the same...)
				cloud->setPointColor(tsi->i[j],C.rgb);
			}
		}
	}

	return true;
}
