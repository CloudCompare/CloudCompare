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
#include "ccGenericPointCloud.h"
#include "ccMaterialSet.h"
#include "ccMesh.h"

//Qt
#include <QString>

//CCLib
#include <ManualSegmentationTools.h>

//system
#include <cassert>
#include <cstring>

ccSubMesh::ccSubMesh(ccMesh* parentMesh)
	: ccGenericMesh("Sub-mesh")
	, m_associatedMesh(nullptr)
	, m_globalIterator(0)
{
	setAssociatedMesh(parentMesh); //must be called so as to set the right dependency!

	showColors(parentMesh ? parentMesh->colorsShown() : true);
	showNormals(parentMesh ? parentMesh->normalsShown() : true);
	showSF(parentMesh ? parentMesh->sfShown() : true);
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

	for (unsigned int index : m_triIndexes)
	{
		CCLib::GenericTriangle* tri = m_associatedMesh->_getTriangle(index);
		action(*tri);
	}
}

ccGenericPointCloud* ccSubMesh::getAssociatedCloud() const
{
	return m_associatedMesh ? m_associatedMesh->getAssociatedCloud() : nullptr;
}

CCLib::GenericTriangle* ccSubMesh::_getNextTriangle() //temporary object
{
	return m_associatedMesh && m_globalIterator < size() ? m_associatedMesh->_getTriangle(m_triIndexes[m_globalIterator++]) : nullptr;
}

CCLib::VerticesIndexes* ccSubMesh::getNextTriangleVertIndexes()
{
	return m_associatedMesh && m_globalIterator < size() ? m_associatedMesh->getTriangleVertIndexes(m_triIndexes[m_globalIterator++]) : nullptr;
}

bool ccSubMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateNormals(getTriGlobalIndex(triIndex), P, N);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateNormalsBC(unsigned triIndex, const CCVector3d& w, CCVector3& N)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateNormalsBC(getTriGlobalIndex(triIndex), w, N);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& color)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateColors(getTriGlobalIndex(triIndex), P, color);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgb& color)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateColorsBC(getTriGlobalIndex(triIndex), w, color);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateColors(getTriGlobalIndex(triIndex), P, color);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgba& color)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->interpolateColorsBC(getTriGlobalIndex(triIndex), w, color);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color, bool interpolateColorIfNoTexture)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->getColorFromMaterial(getTriGlobalIndex(triIndex), P, color, interpolateColorIfNoTexture);

	//shouldn't happen
	assert(false);
	return false;
}

bool ccSubMesh::getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgba& color, bool returnColorIfNoTexture)
{
	if (m_associatedMesh && triIndex < size())
		return m_associatedMesh->getVertexColorFromMaterial(getTriGlobalIndex(triIndex), vertIndex, color, returnColorIfNoTexture);

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
	return nullptr;
}

void ccSubMesh::getTriangleVertices(unsigned triIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	if (m_associatedMesh && triIndex < size())
	{
		m_associatedMesh->getTriangleVertices(getTriGlobalIndex(triIndex), A, B, C);
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
	return nullptr;
}

ccSubMesh* ccSubMesh::createNewSubMeshFromSelection(bool removeSelectedFaces, IndexMap* indexMap/*=0*/)
{
	ccGenericPointCloud* vertices = getAssociatedCloud();
	assert(vertices && m_associatedMesh);
	if (!vertices || !m_associatedMesh)
	{
		return nullptr;
	}

	const ccGenericPointCloud::VisibilityTableType& verticesVisibility = vertices->getTheVisibilityArray();
	if (verticesVisibility.size() < vertices->size())
	{
		ccLog::Error(QString("[Sub-mesh %1] Internal error: vertex visibility table not instantiated!").arg(getName()));
		return nullptr;
	}

	//we count the number of remaining faces
	size_t triNum = m_triIndexes.size();
	size_t visibleFaces = 0;
	{
		for (unsigned globalIndex : m_triIndexes)
		{
			const CCLib::VerticesIndexes* tsi = m_associatedMesh->getTriangleVertIndexes(globalIndex);
			//triangle is visible?
			if (   verticesVisibility[tsi->i1] == POINT_VISIBLE
				&& verticesVisibility[tsi->i2] == POINT_VISIBLE
				&& verticesVisibility[tsi->i3] == POINT_VISIBLE)
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
			for (unsigned& globalIndex : m_triIndexes)
			{
				globalIndex = indexMap->at(globalIndex);
			}
		}
		return nullptr;
	}

	ccSubMesh* newSubMesh = new ccSubMesh(m_associatedMesh);
	if (!newSubMesh->reserve(size()))
	{
		ccLog::Error("[ccSubMesh::createNewSubMeshFromSelection] Not enough memory!");
		return nullptr;
	}

	//create sub-mesh
	{
		unsigned lastTri = 0;
		for (size_t i = 0; i < triNum; ++i)
		{
			unsigned globalIndex = m_triIndexes[i];
			const CCLib::VerticesIndexes* tsi = m_associatedMesh->getTriangleVertIndexes(globalIndex);

			if (indexMap) //translate global index?
				globalIndex = indexMap->at(globalIndex);

			//triangle is visible?
			if (	verticesVisibility[tsi->i1] == POINT_VISIBLE
				&&	verticesVisibility[tsi->i2] == POINT_VISIBLE
				&&	verticesVisibility[tsi->i3] == POINT_VISIBLE)
			{
				newSubMesh->addTriangleIndex(globalIndex);
			}
			else if (removeSelectedFaces) //triangle is not visible? It stays in the original mesh!
			{
				//we replace the current triangle by the 'last' valid one
				assert(lastTri <= i);
				m_triIndexes[lastTri++] = globalIndex;
			}
		}

		//resize original mesh
		if (removeSelectedFaces && lastTri < triNum)
		{
			resize(lastTri);
			m_bBox.setValidity(false);
			notifyGeometryUpdate();
		}
	}

	if (newSubMesh->size())
	{
		newSubMesh->setName(getName() + QString(".part"));
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
		newSubMesh = nullptr;
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
	return m_associatedMesh ? m_associatedMesh->getMaterialSet() : nullptr;
}

int ccSubMesh::getTriangleMtlIndex(unsigned triIndex) const
{
	return m_associatedMesh ? m_associatedMesh->getTriangleMtlIndex(getTriGlobalIndex(triIndex)) : -1;
}

TextureCoordsContainer* ccSubMesh::getTexCoordinatesTable() const
{
	return m_associatedMesh ? m_associatedMesh->getTexCoordinatesTable() : nullptr;
}

void ccSubMesh::getTriangleTexCoordinates(unsigned triIndex, TexCoords2D* &tx1, TexCoords2D* &tx2, TexCoords2D* &tx3) const
{
	if (m_associatedMesh && triIndex < size())
	{
		m_associatedMesh->getTriangleTexCoordinates(getTriGlobalIndex(triIndex), tx1, tx2, tx3);
	}
	else
	{
		//shouldn't happen
		tx1 = tx2 = tx3 = nullptr;
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
	return m_associatedMesh ? m_associatedMesh->getTriNormsTable() : nullptr;
}

void ccSubMesh::clear(bool releaseMemory)
{
	if (releaseMemory)
		m_triIndexes.resize(0);
	else
		m_triIndexes.clear();
	m_bBox.setValidity(false);
}

bool ccSubMesh::addTriangleIndex(unsigned globalIndex)
{
	try
	{
		m_triIndexes.emplace_back(globalIndex);
	}
	catch (const std::bad_alloc&)
	{
		//not engough memory
		return false;
	}

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

	unsigned range = lastIndex - firstIndex; //lastIndex is excluded

	try
	{
		m_triIndexes.reserve(m_triIndexes.size() + range);
	}
	catch (const std::bad_alloc&)
	{
		//not engough memory
		return false;
	}
	
	for (unsigned i = firstIndex; i < lastIndex; ++i)
	{
		m_triIndexes.emplace_back(i);
	}

	m_bBox.setValidity(false);

	return true;
}

void ccSubMesh::setTriangleIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex < size());
	m_triIndexes[localIndex] = globalIndex;
	m_bBox.setValidity(false);
}

bool ccSubMesh::reserve(size_t n)
{
	try
	{
		m_triIndexes.reserve(n);
	}
	catch (const std::bad_alloc&)
	{
		//not engough memory
		return false;
	}
	return true;
}

bool ccSubMesh::resize(size_t n)
{
	try
	{
		m_triIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		//not engough memory
		return false;
	}
	return true;
}

unsigned ccSubMesh::capacity() const
{
	return static_cast<unsigned>(m_triIndexes.capacity());
}

void ccSubMesh::refreshBB()
{
	m_bBox.clear();

	if (m_associatedMesh)
	{
		for (unsigned globalIndex : m_triIndexes)
		{
			CCLib::GenericTriangle* tri = m_associatedMesh->_getTriangle(globalIndex);
			m_bBox.add(*tri->_getA());
			m_bBox.add(*tri->_getB());
			m_bBox.add(*tri->_getC());
		}
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
	uint32_t meshUniqueID = (m_associatedMesh ? static_cast<uint32_t>(m_associatedMesh->getUniqueID()) : 0);
	if (out.write((const char*)&meshUniqueID, 4) < 0)
		return WriteError();

	//references (dataVersion>=29)
	if (!ccSerializationHelper::GenericArrayToFile<unsigned, 1, unsigned>(m_triIndexes, out))
		return WriteError();

	return true;
}

bool ccSubMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericMesh::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//as the associated mesh can't be saved directly
	//we only store its unique ID (dataVersion>=29) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t meshUniqueID = 0;
	if (in.read((char*)&meshUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the mesh unique ID in the 'm_associatedMesh' pointer!!!
	*(uint32_t*)(&m_associatedMesh) = meshUniqueID;

	//references (dataVersion>=29)
	if (!ccSerializationHelper::GenericArrayFromFile<unsigned, 1, unsigned>(m_triIndexes, in, dataVersion))
		return ReadError();

	return true;
}
