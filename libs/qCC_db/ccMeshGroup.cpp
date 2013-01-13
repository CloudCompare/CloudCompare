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
//$Rev:: 2213                                                              $
//$LastChangedDate:: 2012-07-18 19:39:09 +0200 (mer., 18 juil. 2012)       $
//**************************************************************************
//

#include "ccMeshGroup.h"

//Local
#include "ccMaterialSet.h"
#include "ccGenericPointCloud.h"
#include "ccMesh.h"

//Qt
#include <QString>

//CCLib
#include <ManualSegmentationTools.h>

//system
#include <assert.h>

#define CC_MESH_RECURSIVE_CALL(method) for (unsigned i=0;i<m_children.size();++i){if (m_children[i]->isKindOf(CC_MESH)) static_cast<ccGenericMesh*>(m_children[i])->method;}
#define CC_MESH_RECURSIVE_TEST(method) for (unsigned i=0;i<m_children.size();++i) {if (m_children[i]->isKindOf(CC_MESH)) {if (static_cast<ccGenericMesh*>(m_children[i])->method()) return true;}} return false;

ccMeshGroup::ccMeshGroup(ccGenericPointCloud* vertices)
    : ccGenericMesh(vertices, "Mesh Group")
	, currentChildIndex(-1)
{
    showColors(true);
    showNormals(true);

    //we automatically lock the cloud, to prevent vertices sharing issues!
    if (vertices)
        vertices->setLocked(true);
}

unsigned ccMeshGroup::size() const
{
    unsigned i,s=0;
    for (i=0;i<m_children.size();++i)
        if (m_children[i]->isKindOf(CC_MESH))
            s+= static_cast<ccGenericMesh*>(m_children[i])->size();

    return s;
}

void ccMeshGroup::setVisible(bool state)
{
    ccHObject::setVisible(state);
    CC_MESH_RECURSIVE_CALL(setVisible(state));
}

void ccMeshGroup::showNormals(bool state)
{
	showTriNorms(state);
	ccHObject::showNormals(state);
	CC_MESH_RECURSIVE_CALL(showNormals(state));
}

void ccMeshGroup::showColors(bool state)
{
    ccHObject::showColors(state);
    CC_MESH_RECURSIVE_CALL(showColors(state));
}

void ccMeshGroup::showTriNorms(bool state)
{
	CC_MESH_RECURSIVE_CALL(showTriNorms(state));
}

void ccMeshGroup::showMaterials(bool state)
{
    CC_MESH_RECURSIVE_CALL(showMaterials(state));
}

bool ccMeshGroup::triNormsShown() const
{
	CC_MESH_RECURSIVE_TEST(triNormsShown);
}

bool ccMeshGroup::materialsShown() const
{
	CC_MESH_RECURSIVE_TEST(materialsShown);
}

bool ccMeshGroup::hasTriNormals() const
{
	CC_MESH_RECURSIVE_TEST(hasTriNormals);
}

void ccMeshGroup::clearTriNormals()
{
    CC_MESH_RECURSIVE_CALL(clearTriNormals());

	setTriNormsTable(0);

	//due to an old bug on mesh groups we have to look for 'm_triNormals' as a child)
	for (unsigned i=0;i<getChildrenNumber();++i)
	{
		if (getChild(i)->isA(CC_NORMAL_INDEXES_ARRAY))
		{
			removeChild((int)i);
			break;
		}
	}
}

bool ccMeshGroup::hasMaterials() const
{
	CC_MESH_RECURSIVE_TEST(hasMaterials);
}

void ccMeshGroup::showSF(bool state)
{
    ccHObject::showSF(state);
    CC_MESH_RECURSIVE_CALL(showSF(state));
}

void ccMeshGroup::showWired(bool state)
{
    ccGenericMesh::showWired(state);
    CC_MESH_RECURSIVE_CALL(showWired(state));
}

void ccMeshGroup::setTempColor(const colorType* col, bool autoActivate /*= true*/)
{
    ccGenericMesh::setTempColor(col,autoActivate);
    CC_MESH_RECURSIVE_CALL(setTempColor(col,autoActivate));
}

void ccMeshGroup::enableTempColor(bool state)
{
    ccGenericMesh::enableTempColor(state);
    CC_MESH_RECURSIVE_CALL(enableTempColor(state));
}

void ccMeshGroup::setAssociatedCloud(ccGenericPointCloud* cloud)
{
    ccGenericMesh::setAssociatedCloud(cloud);
    CC_MESH_RECURSIVE_CALL(setAssociatedCloud(cloud));
}

void ccMeshGroup::shiftTriangleIndexes(unsigned shift)
{
    CC_MESH_RECURSIVE_CALL(shiftTriangleIndexes(shift));
}

void ccMeshGroup::forEach(genericTriangleAction& anAction)
{
    CC_MESH_RECURSIVE_CALL(forEach(anAction));
}

void ccMeshGroup::placeIteratorAtBegining()
{
	currentChildIndex=-1;
    //let's look for the first sub-mesh!
    while (++currentChildIndex<(int)m_children.size())
    {
        if (m_children[currentChildIndex]->isKindOf(CC_MESH))
        {
            static_cast<ccGenericMesh*>(m_children[currentChildIndex])->placeIteratorAtBegining();
            break;
        }
    }
}

CCLib::GenericTriangle* ccMeshGroup::_getNextTriangle() //temporary object
{
    if (currentChildIndex<(int)m_children.size())
    {
        CCLib::GenericTriangle* tri = static_cast<ccGenericMesh*>(m_children[currentChildIndex])->_getNextTriangle();
        if (tri)
            return tri;

        while (++currentChildIndex<(int)m_children.size())
        {
            if (m_children[currentChildIndex]->isKindOf(CC_MESH))
            {
                static_cast<ccGenericMesh*>(m_children[currentChildIndex])->placeIteratorAtBegining();
                return _getNextTriangle();
            }
        }
    }

    return NULL;
}

CCLib::TriangleSummitsIndexes* ccMeshGroup::getNextTriangleIndexes()
{
    if (currentChildIndex<int(m_children.size()))
    {
        CCLib::TriangleSummitsIndexes* tsi = static_cast<ccGenericMesh*>(m_children[currentChildIndex])->getNextTriangleIndexes();
        if (tsi)
            return tsi;

        while (++currentChildIndex<(int)m_children.size())
        {
            if (m_children[currentChildIndex]->isKindOf(CC_MESH))
            {
                static_cast<ccGenericMesh*>(m_children[currentChildIndex])->placeIteratorAtBegining();
                return getNextTriangleIndexes();
            }
        }
    }

    return NULL;
}

//FIXME!
static CCLib::GenericTriangle* s_triangle; //to avoid heap overflow
CCLib::GenericTriangle* ccMeshGroup::_getTriangle_recursive(unsigned& index)
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
            if (m_children[i]->isA(CC_MESH_GROUP))
            {
                if ((s_triangle = static_cast<ccMeshGroup*>(m_children[i])->_getTriangle_recursive(index)))
                    return s_triangle;
            }
            else
            {
                ccMesh* child = static_cast<ccMesh*>(m_children[i]);
                unsigned cs = child->size();
                if (index < cs)
                    return child->_getTriangle(index);
                else
                    index -= cs;
            }
        }
    }

    return NULL;
}

bool ccMeshGroup::getTriangleSummits_recursive(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
            if (m_children[i]->isA(CC_MESH_GROUP))
            {
                if (static_cast<ccMeshGroup*>(m_children[i])->getTriangleSummits_recursive(triangleIndex,A,B,C))
                    return true;
            }
            else
            {
                ccMesh* child = static_cast<ccMesh*>(m_children[i]);
                unsigned cs = child->size();
                if (triangleIndex < cs)
				{
                    child->getTriangleSummits(triangleIndex,A,B,C);
					return true;
				}
                else
				{
                    triangleIndex -= cs;
				}
            }
        }
    }

	return false;
}


//FIXME: so slow on big mesh groups!!!
static CCLib::TriangleSummitsIndexes* s_triSummits; //to avoid heap overflow
CCLib::TriangleSummitsIndexes* ccMeshGroup::getTriangleIndexes_recursive(unsigned& triangleIndex)
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
            if (m_children[i]->isGroup())
            {
                if ((s_triSummits = static_cast<ccMeshGroup*>(m_children[i])->getTriangleIndexes_recursive(triangleIndex)))
                    return s_triSummits;
            }
            else
            {
                ccMesh* child = static_cast<ccMesh*>(m_children[i]);
                unsigned cs = child->size();
                if (triangleIndex < cs)
                    return child->getTriangleIndexes(triangleIndex);
                else
                    triangleIndex -= cs;
            }
        }
    }

    return NULL;
}

bool ccMeshGroup::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
			ccGenericMesh* subMesh = static_cast<ccGenericMesh*>(m_children[i]);
			unsigned triCount = subMesh->size();
			if (triIndex<triCount)
				return subMesh->interpolateNormals(triIndex,P,N);
			else
				triIndex-=triCount;
		}
	}

	//shouldn't happen
	assert(false);
	return false;
}

bool ccMeshGroup::interpolateColors(unsigned triIndex, const CCVector3& P, colorType rgb[])
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
			ccGenericMesh* subMesh = static_cast<ccGenericMesh*>(m_children[i]);
			unsigned triCount = subMesh->size();
			if (triIndex<triCount)
				return subMesh->interpolateColors(triIndex,P,rgb);
			else
				triIndex-=triCount;
		}
	}

	//shouldn't happen
	assert(false);
	return false;
}

bool ccMeshGroup::getColorFromTexture(unsigned triIndex, const CCVector3& P, colorType rgb[], bool interpolateColorIfNoTexture)
{
    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
			ccGenericMesh* subMesh = static_cast<ccGenericMesh*>(m_children[i]);
			unsigned triCount = subMesh->size();
			if (triIndex<triCount)
				return subMesh->getColorFromTexture(triIndex,P,rgb,interpolateColorIfNoTexture);
			else
				triIndex-=triCount;
		}
	}

	//shouldn't happen
	assert(false);
	return false;
}

//FIXME!
CCLib::GenericTriangle* ccMeshGroup::_getTriangle(unsigned index) //temporary object
{
    return _getTriangle_recursive(index);
}

//FIXME!
void ccMeshGroup::getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
    getTriangleSummits_recursive(triangleIndex,A,B,C);
}

//FIXME!
CCLib::TriangleSummitsIndexes* ccMeshGroup::getTriangleIndexes(unsigned triangleIndex)
{
    return getTriangleIndexes_recursive(triangleIndex);
}

void ccMeshGroup::getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[])
{
    ccBBox bb = getBB();

    memcpy(Mins,bb.minCorner().u,3*sizeof(PointCoordinateType));
    memcpy(Maxs,bb.maxCorner().u,3*sizeof(PointCoordinateType));
}

void ccMeshGroup::addChild(ccHObject* anObject, bool dependant/* = true*/)
{
    assert(anObject->isKindOf(CC_MESH) ? static_cast<ccGenericMesh*>(anObject)->getAssociatedCloud()==m_associatedCloud : true);

    ccHObject::addChild(anObject,dependant);
}

ccGenericMesh* ccMeshGroup::createNewMeshFromSelection(bool removeSelectedVertices, CCLib::ReferenceCloud* selection/*=NULL*/, ccGenericPointCloud* vertices/*=NULL*/)
{
    assert(m_associatedCloud);

    ccGenericPointCloud* newVertices = NULL;
    if (!vertices)
    {
        newVertices = m_associatedCloud->createNewCloudFromVisibilitySelection(false);
        if (!newVertices)
        {
            //ccConsole::Error("An error occured: not enough memory ?");
            return NULL;
        }
    }
    else
    {
        newVertices = vertices;
    }
    assert(newVertices);

    CCLib::ReferenceCloud* rc = selection;
    if (!rc)
    {
        rc = m_associatedCloud->getTheVisiblePoints();
        if (!rc || rc->size()==0)
        {
            //ccConsole::Error("No Points in selection!\n");
            if (rc)
				delete rc;
            return 0;
        }
    }
    assert(rc);

    //new mesh group
    ccMeshGroup* mg = new ccMeshGroup(newVertices);

    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
            ccGenericMesh* childTri = static_cast<ccGenericMesh*>(m_children[i]);
            ccGenericMesh* newTri = childTri->createNewMeshFromSelection(removeSelectedVertices, rc, newVertices);
            if (newTri)
                mg->addChild(newTri);
        }
    }

    if (!selection) //if we have created a selection for this mesh
    {
        delete rc;
        rc=0;
    }

    if (mg->getChildrenNumber())
    {
        mg->setName(getName()+QString(".part"));
        if (!vertices) //if we have created vertices for this mesh
        {
            mg->addChild(newVertices);
            mg->setDisplay_recursive(getDisplay());
            mg->showColors(colorsShown());
            mg->showNormals(normalsShown());
			mg->showMaterials(materialsShown());
            mg->showSF(sfShown());
            newVertices->setEnabled(false);
            //newVertices->setLocked(true);
        }
    }
    else
    {
        delete mg;
        mg=NULL;

        if (!vertices) //if we have created vertices for this mesh
        {
            delete newVertices;
            newVertices=0;
        }
    }

	return mg;
}

ccGenericMesh* ccMeshGroup::clone(ccGenericPointCloud* vertices/*=0*/,
								  ccMaterialSet* clonedMaterials/*=0*/,
								  NormsIndexesTableType* clonedNormsTable/*=0*/,
								  TextureCoordsContainer* cloneTexCoords/*=0*/)
{
    assert(m_associatedCloud);

    ccGenericPointCloud* newVertices = vertices;
    if (!newVertices)
    {
        newVertices = m_associatedCloud->clone();
        if (!newVertices)
        {
            ccLog::Error("[ccGenericMesh::clone] Failed to clone vertices! (not enough memory?)");
            return 0;
        }
    }

	//materials
	assert(!clonedMaterials);
	if (getMaterialSet())
	{
		clonedMaterials = getMaterialSet()->clone();
		if (!clonedMaterials)
			ccLog::Error("[ccMeshGroup::clone] Failed to clone materials set!");
	}
	//normals
	assert(!clonedNormsTable);
	if (getTriNormsTable())
	{
		clonedNormsTable=getTriNormsTable()->clone();
		if (!clonedNormsTable)
			ccLog::Error("[ccMeshGroup::clone] Failed to clone (per-triangle) normals!");
	}
	//texture coordinates
	assert(!cloneTexCoords);
	if (getTexCoordinatesTable())
	{
		cloneTexCoords=getTexCoordinatesTable()->clone();
		if (!cloneTexCoords)
			ccLog::Error("[ccMeshGroup::clone] Failed to clone texture coordinates!");
	}

    //new mesh group
    ccMeshGroup* mg = new ccMeshGroup(newVertices);

    for (unsigned i=0;i<m_children.size();++i)
    {
        if (m_children[i]->isKindOf(CC_MESH))
        {
            ccGenericMesh* childTri = static_cast<ccGenericMesh*>(m_children[i]);
            ccGenericMesh* newTri = childTri->clone(newVertices,clonedMaterials,clonedNormsTable,cloneTexCoords);
            if (newTri)
                mg->addChild(newTri);
        }
    }

    if (mg->getChildrenNumber())
    {
        mg->setName(getName()+QString(".clone"));

        if (!vertices) //if we have created vertices for this mesh
        {
            mg->addChild(newVertices);
            mg->setDisplay_recursive(getDisplay());
            //newVertices->setLocked(true);
        }

		if (clonedMaterials)
		{
			mg->setMaterialSet(clonedMaterials);
			mg->addChild(clonedMaterials);
		}
		if (clonedNormsTable)
		{
			mg->setTriNormsTable(clonedNormsTable);
			mg->addChild(clonedNormsTable);
		}
		if (cloneTexCoords)
		{
			mg->setTexCoordinatesTable(cloneTexCoords);
			mg->addChild(cloneTexCoords);
		}
		mg->showColors(colorsShown());
		mg->showNormals(normalsShown());
		mg->showSF(sfShown());
		mg->setVisible(isVisible());
		mg->setEnabled(isEnabled());
    }
    else
    {
        delete mg;
        mg=0;

        if (!vertices) //if we have created vertices for this mesh
        {
            delete newVertices;
            newVertices=0;
        }
    }

    return mg;
}

void ccMeshGroup::refreshBB()
{
    for (unsigned i=0;i<m_children.size();++i)
        if (m_children[i]->isKindOf(CC_MESH))
            static_cast<ccGenericMesh*>(m_children[i])->refreshBB();
}
