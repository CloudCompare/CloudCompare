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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_MESH_GROUP_HEADER
#define CC_MESH_GROUP_HEADER

#include "ccGenericMesh.h"

//! A group of meshes sharing vertices (and associated properties) in a unique cloud
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccMeshGroup : public ccGenericMesh
#else
class ccMeshGroup : public ccGenericMesh
#endif
{
public:

    //! Default constructor
	ccMeshGroup(ccGenericPointCloud* vertices);

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_MESH_GROUP;};

    //inherited methods (ccGenericMesh)
	virtual void showWired(bool state);
	virtual ccGenericMesh* createNewMeshFromSelection(bool removeSelectedVertices=false, CCLib::ReferenceCloud* selection=NULL, ccGenericPointCloud* vertices=NULL);
	virtual ccGenericMesh* clone(ccGenericPointCloud* vertices = 0, ccMaterialSet* clonedMaterials = 0, NormsIndexesTableType* clonedNormsTable = 0, TextureCoordsContainer* cloneTexCoords =0);
    virtual void refreshBB();
	virtual void showTriNorms(bool state);
	virtual bool triNormsShown() const;
	virtual bool hasTriNormals() const;
	virtual void clearTriNormals();
    virtual void showMaterials(bool state);
	virtual bool materialsShown() const;
	virtual bool hasMaterials() const;
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N);
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, colorType rgb[]);
	virtual bool getColorFromTexture(unsigned triIndex, const CCVector3& P, colorType rgb[], bool interpolateColorIfNoTexture);

	//inherited methods (ccHObject)
    virtual void addChild(ccHObject* anObject, bool dependant = true);

	//inherited methods (ccDrawableObject)
    virtual void setVisible(bool state);
    virtual void showNormals(bool state);
    virtual void showColors(bool state);
    virtual void showSF(bool state);
    virtual void setTempColor(const colorType* col, bool autoActivate = true);
    virtual void enableTempColor(bool state);

	//inherited methods (GenericIndexedMesh)
	virtual unsigned size() const;
	virtual void forEach(genericTriangleAction& anAction);
	virtual void placeIteratorAtBegining();
	virtual CCLib::GenericTriangle* _getNextTriangle(); //temporary object
	virtual CCLib::GenericTriangle* _getTriangle(unsigned index); //temporary object
	virtual CCLib::TriangleSummitsIndexes* getNextTriangleIndexes();
	virtual CCLib::TriangleSummitsIndexes* getTriangleIndexes(unsigned triangleIndex);
	virtual void getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C);
    virtual void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]);
	virtual void shiftTriangleIndexes(unsigned shift);
	virtual void setAssociatedCloud(ccGenericPointCloud* cloud);

protected:

	CCLib::GenericTriangle* _getTriangle_recursive(unsigned& index); //temporary object
	CCLib::TriangleSummitsIndexes* getTriangleIndexes_recursive(unsigned& triangleIndex);
	bool getTriangleSummits_recursive(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C);

	//iterator
	int currentChildIndex;
};

#endif
