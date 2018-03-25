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

#include "ccSubMesh.h"

//Local
#include "ccMaterialSet.h"
#include "ccGenericPointCloud.h"
#include "ccMesh.h"

//Qt
#include <QString>

//CCLib
#include <ManualSegmentationTools.h>

//system
#include <string.h>
#include <assert.h>

ccSubMesh::ccSubMesh(ccMesh* parentMesh)
	: ccGenericMesh("Sub-mesh")
	, m_associatedMesh(0)
	, m_triIndexes(new ReferencesContainer())
	, m_globalIterator(0)
{
	m_triIndexes->link();

	setAssociatedMesh(parentMesh); //must be called so as to set the right dependency!

	showColors(parentMesh ? parentMesh->colorsShown() : true);
	showNormals(parentMesh ? parentMesh->normalsShown() : true);
	showSF(parentMesh ? parentMesh->sfShown() : true);
}

ccSubMesh::~ccSubMesh()
{
	if (m_triIndexes)
	{
		m_triIndexes->release();
		m_triIndexes = 0;
	}
}

void ccSubMesh::setAssociatedMesh(ccMesh* mesh, bool unlinkPreviousOne/*=true*/)
{
	if (m_associatedMesh == mesh)
		return;

	if (m_associatedMesh && unlinkPreviousOne)
		m_associatedMesh->removeDependencyWith(this);

	m_associatedMesh = mesh;

	if (m_associatedMesh)
		m_associatedMesh->addDependency(this,DP_NOTIFY_OTHER_ON_UPDATE);
}

void ccSubMesh::onUpdateOf(ccHObject* obj)
{
	if (obj == m_associatedMesh)
		m_bBox.setValidity(false);
}

void ccSubMesh::forEach(genericTriangleAction action)
{
	if (!m_associatedMesh)
		return;

	m_triIndexes->placeIteratorAtBeginning();
	for (unsigned i=0; i<m_triIndexes->currentSize(); ++i)
	{
		CCLib::GenericTriangle* tri = m_associatedMesh->_getTriangle(m_triIndexes->getCurrentValue());
		action(*tri);
		m_triIndexes->forwardIterator();
	}
}

ccGenericPointCloud* ccSubMesh::getAssociatedCloud() const
{
	return m_associatedMesh ? m_associatedMesh->getAssociatedCloud() : 0;
}

CCLib::GenericTriangle* ccSubMesh::_getNextTriangle() //temporary object
{
	return m_associatedMesh && m_globalIterator < size() ? m_associatedMesh->_getTriangle(m_triIndexes->getValue(m_globalIterator++)) : 0;
}

CCLib::VerticesIndexes* ccSubMesh::getNextTriangleVertIndexes()
{
	return m_associatedMesh && m_globalIterator < size() ? m_associatedMesh->getTriangleVertIndexes(m_triIndexes->getValue(m_globalIterator++)) : 0;
}

bool ccSubMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateNormals(getTriGlobalIndex(triIndex),P,N);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateColors(getTriGlobalIndex(triIndex),P,rgb);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb, bool interpolateColorIfNoTexture)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->getColorFromMaterial(getTriGlobalIndex(triIndex),P,rgb,interpolateColorIfNoTexture);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& rgb, bool returnColorIfNoTexture)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->getVertexColorFromMaterial(getTriGlobalIndex(triIndex),vertIndex,rgb,returnColorIfNoTexture);

	//shouldn't happen
	assert(false);
	return false;
}

CCLib::GenericTriangle* ccSubMesh::_getTriangle(unsigned triIndex) //temporary object
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->_getTriangle(getTriGlobalIndex(triIndex));

	//shouldn't happen
	assert(false);
	return 0;
}

void ccSubMesh::getTriangleVertices(unsigned triIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	if (m_associatedMesh && triIndex < size())
	{
		m_associatedMesh->getTriangleVertices(getTriGlobalIndex(triIndex),A,B,C);
	}
	else
	{
		//shouldn't happen
		assert(false);
	}
}

CCLib::VerticesIndexes* ccSubMesh::getTriangleVertIndexes(unsigned triIndex)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->getTriangleVertIndexes(getTriGlobalIndex(triIndex));

	//shouldn't happen
	assert(false);
	return 0;
}

ccSubMesh* ccSubMesh::createNewSubMeshFromSelection(bool removeSelectedFaces, IndexMap* indexMap/*=0*/)
{
	ccGenericPointCloud* vertices = getAssociatedCloud();
	assert(vertices && m_associatedMesh);
	if (!vertices || !m_associatedMesh)
	{
		return NULL;
	}

	ccGenericPointCloud::VisibilityTableType* verticesVisibility = vertices->getTheVisibilityArray();
	if (!verticesVisibility || !verticesVisibility->isAllocated())
	{
		ccLog::Error(QString("[Sub-mesh %1] Internal error: vertex visibility table not instantiated!").arg(getName()));
		return NULL;
	}

	//we count the number of remaining faces
	unsigned triNum = m_triIndexes->currentSize();
	unsigned visibleFaces = 0;
	{
		for (unsigned i=0; i<triNum; ++i)
		{
			const unsigned& globalIndex = m_triIndexes->getValue(i);
			const CCLib::VerticesIndexes* tsi = m_associatedMesh->getTriangleVertIndexes(globalIndex);
			//triangle is visible?
			if (   verticesVisibility->getValue(tsi->i1) == POINT_VISIBLE
				&& verticesVisibility->getValue(tsi->i2) == POINT_VISIBLE
				&& verticesVisibility->getValue(tsi->i3) == POINT_VISIBLE)
			{
				++visibleFaces;
			}
		}
	}

	//nothing to do
	if (visibleFaces == 0)
	{
		if (indexMap) //we still have to translate global indexes!
		{
			for (unsigned i=0; i<triNum; ++i)
			{
				unsigned globalIndex = m_triIndexes->getValue(i);
				globalIndex = indexMap->getValue(globalIndex);
				m_triIndexes->setValue(i,globalIndex);
			}
		}
		return 0;
	}

	ccSubMesh* newSubMesh = new ccSubMesh(m_associatedMesh);
	if (!newSubMesh->reserve(size()))
	{
		ccLog::Error("[ccSubMesh::createNewSubMeshFromSelection] Not enough memory!");
		return NULL;
	}

	//create sub-mesh
	{
		unsigned lastTri = 0;
		for (unsigned i=0; i<triNum; ++i)
		{
			unsigned globalIndex = m_triIndexes->getValue(i);
			const CCLib::VerticesIndexes* tsi = m_associatedMesh->getTriangleVertIndexes(globalIndex);

			if (indexMap) //translate global index?
				globalIndex = indexMap->getValue(globalIndex);

			//triangle is visible?
			if (   verticesVisibility->getValue(tsi->i1) == POINT_VISIBLE
				&& verticesVisibility->getValue(tsi->i2) == POINT_VISIBLE
				&& verticesVisibility->getValue(tsi->i3) == POINT_VISIBLE)
			{
				newSubMesh->addTriangleIndex(globalIndex);
			}
			else if (removeSelectedFaces) //triangle is not visible? It stays in the original mesh!
			{
				//we replace the current triangle by the 'last' valid one
				assert(lastTri <= i);
				m_triIndexes->setValue(lastTri++,globalIndex);
			}
		}

		//resize original mesh
		if (removeSelectedFaces && lastTri < triNum)
		{
			if (lastTri == 0)
				m_triIndexes->clear(true);
			else
				resize(lastTri);

			m_bBox.setValidity(false);
			notifyGeometryUpdate();
		}
	}

	if (newSubMesh->size())
	{
		newSubMesh->setName(getName()+QString(".part"));
		newSubMesh->resize(newSubMesh->size());
		newSubMesh->setDisplay(getDisplay());
		newSubMesh->showColors(colorsShown());
		newSubMesh->showNormals(normalsShown());
		newSubMesh->showMaterials(materialsShown());
		newSubMesh->showSF(sfShown());
		newSubMesh->enableStippling(stipplingEnabled());
		newSubMesh->showWired(isShownAsWire());
	}
	else
	{
		assert(false);
		delete newSubMesh;
		newSubMesh = 0;
	}

	return newSubMesh;
}

bool ccSubMesh::normalsShown() const
{
	return (ccHObject::normalsShown() || triNormsShown());
}

#define CC_SUB_MESH_TRANSIENT_CONST_TEST(method) bool ccSubMesh::method() const { return m_associatedMesh ? m_associatedMesh->method() : false; }

CC_SUB_MESH_TRANSIENT_CONST_TEST(hasColors);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasNormals);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasScalarFields);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasDisplayedScalarField);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasMaterials);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasTextures);
CC_SUB_MESH_TRANSIENT_CONST_TEST(hasTriNormals);

const ccMaterialSet* ccSubMesh::getMaterialSet() const
{
	return m_associatedMesh ? m_associatedMesh->getMaterialSet() : 0;
}

int ccSubMesh::getTriangleMtlIndex(unsigned triIndex) const
{
	return m_associatedMesh ? m_associatedMesh->getTriangleMtlIndex(getTriGlobalIndex(triIndex)) : -1;
}

TextureCoordsContainer* ccSubMesh::getTexCoordinatesTable() const
{
	return m_associatedMesh ? m_associatedMesh->getTexCoordinatesTable() : 0;
}

void ccSubMesh::getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const
{
	if (m_associatedMesh && triIndex < size())
	{
		m_associatedMesh->getTriangleTexCoordinates(getTriGlobalIndex(triIndex), tx1, tx2, tx3);
	}
	else
	{
		//shouldn't happen
		tx1 = tx2 = tx3;
		assert(false);
	}
}

bool ccSubMesh::hasPerTriangleTexCoordIndexes() const
{
	return m_associatedMesh ? m_associatedMesh->hasPerTriangleTexCoordIndexes() : false;
}

void ccSubMesh::getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const
{
	if (m_associatedMesh && triangleIndex < size())
	{
		m_associatedMesh->getTriangleTexCoordinatesIndexes(getTriGlobalIndex(triangleIndex), i1, i2, i3);
	}
	else
	{
		i1 = i2 = i3 = -1;
	}
}

void ccSubMesh::getTriangleNormalIndexes(unsigned triIndex, int& i1, int& i2, int& i3) const
{
	if (m_associatedMesh && triIndex < size())
	{
		m_associatedMesh->getTriangleNormalIndexes(getTriGlobalIndex(triIndex), i1, i2, i3);
	}
	else
	{
		i1 = i2 = i3 = -1;
	}
}

bool ccSubMesh::getTriangleNormals(unsigned triIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const
{
	return (m_associatedMesh && triIndex < size() ? m_associatedMesh->getTriangleNormals(getTriGlobalIndex(triIndex), Na, Nb, Nc) : false);
}

NormsIndexesTableType* ccSubMesh::getTriNormsTable() const
{
	return m_associatedMesh ? m_associatedMesh->getTriNormsTable() : 0;
}

void ccSubMesh::clear(bool releaseMemory)
{
	m_triIndexes->clear(releaseMemory);
	m_bBox.setValidity(false);
}

bool ccSubMesh::addTriangleIndex(unsigned globalIndex)
{
	if (m_triIndexes->capacity() == m_triIndexes->currentSize())
		if (!m_triIndexes->reserve(m_triIndexes->capacity() + std::min<unsigned>(std::max<unsigned>(1, m_triIndexes->capacity() / 2), 1024))) //not enough space --> +50% (or 1024)
			return false;

	m_triIndexes->addElement(globalIndex);
	m_bBox.setValidity(false);

	return true;
}

bool ccSubMesh::addTriangleIndex(unsigned firstIndex, unsigned lastIndex)
{
	if (firstIndex >= lastIndex)
	{
		assert(false);
		return false;
	}

	unsigned range = lastIndex-firstIndex; //lastIndex is excluded
	unsigned pos = size();

	if (size()<pos+range && !m_triIndexes->resize(pos+range))
		return false;
	
	for (unsigned i = 0; i < range; ++i, ++firstIndex)
		m_triIndexes->setValue(pos++, firstIndex);

	m_bBox.setValidity(false);

	return true;
}

void ccSubMesh::setTriangleIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex < size());
	m_triIndexes->setValue(localIndex, globalIndex);
	m_bBox.setValidity(false);
}

bool ccSubMesh::reserve(unsigned n)
{
	return m_triIndexes->reserve(n);
}

bool ccSubMesh::resize(unsigned n)
{
	return m_triIndexes->resize(n);
}

unsigned ccSubMesh::capacity() const
{
	return m_triIndexes->capacity();
}

void ccSubMesh::refreshBB()
{
	m_bBox.clear();
	
	for (unsigned i = 0; i < size(); ++i)
	{
		CCLib::GenericTriangle* tri = _getTriangle(i);
		m_bBox.add(*tri->_getA());
		m_bBox.add(*tri->_getB());
		m_bBox.add(*tri->_getC());
	}

	notifyGeometryUpdate();
}

ccBBox ccSubMesh::getOwnBB(bool withGLFeatures/*=false*/)
{
	//force BB refresh if necessary
	if (!m_bBox.isValid() && size() != 0)
	{
		refreshBB();
	}

	return m_bBox;
}

void ccSubMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	//force BB refresh if necessary
	if (!m_bBox.isValid() && size() != 0)
	{
		refreshBB();
	}

	bbMin = m_bBox.minCorner();
	bbMax = m_bBox.maxCorner();
}

bool ccSubMesh::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericMesh::toFile_MeOnly(out))
		return false;

	//we can't save the associated mesh here (as it may already be saved)
	//so instead we save it's unique ID (dataVersion>=29)
	//WARNING: the mesh must be saved in the same BIN file! (responsibility of the caller)
	uint32_t meshUniqueID = (m_associatedMesh ? (uint32_t)m_associatedMesh->getUniqueID() : 0);
	if (out.write((const char*)&meshUniqueID, 4) < 0)
		return WriteError();

	//references (dataVersion>=29)
	if (!ccSerializationHelper::GenericArrayToFile(*m_triIndexes,out))
		return WriteError();

	return true;
}

bool ccSubMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericMesh::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//as the associated mesh can't be saved directly
	//we only store its unique ID (dataVersion>=29) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t meshUniqueID = 0;
	if (in.read((char*)&meshUniqueID,4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the mesh unique ID in the 'm_associatedMesh' pointer!!!
	*(uint32_t*)(&m_associatedMesh) = meshUniqueID;

	//references (dataVersion>=29)
	if (!ccSerializationHelper::GenericArrayFromFile(*m_triIndexes, in, dataVersion))
		return ReadError();

	return true;
}
