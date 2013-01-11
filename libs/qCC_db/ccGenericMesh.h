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

#ifndef CC_GENERIC_MESH_HEADER
#define CC_GENERIC_MESH_HEADER

//CCLib
#include <GenericIndexedMesh.h>
#include <ReferenceCloud.h>
#include <GenericProgressCallback.h>

//Local
#include "ccHObject.h"
#include "ccAdvancedTypes.h"

class ccGenericPointCloud;
class ccMaterialSet;

//! Generic mesh interface
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGenericMesh : public CCLib::GenericIndexedMesh, public ccHObject
#else
class ccGenericMesh : public CCLib::GenericIndexedMesh, public ccHObject
#endif
{

public:

    //! Default ccGenericMesh constructor
    /** \param associatedCloud the vertices cloud
        \param name object name
    **/
    ccGenericMesh(ccGenericPointCloud* associatedCloud, QString name=QString());

	//! Default destructor
	virtual ~ccGenericMesh();

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_MESH;};

    //! Sets whether mesh should be displayed as a wire or with plain facets
	virtual void showWired(bool state);

    //! Returns whether the mesh is displayed as wired or with plain facets
	virtual bool isShownAsWire() const;

    //! Computes per-vertex normals
    virtual bool computeNormals();

	//! Mesh scalar field processes
	enum MESH_SCALAR_FIELD_PROCESS {	SMOOTH_MESH_SF,		/**< Smooth **/
										ENHANCE_MESH_SF,	/**< Enhance **/
	};

	//! Applies process to the mesh scalar field (the one associated to its vertices in fact)
    /** A very simple smoothing/enhancement algorithm based on
        each vertex direct neighbours. Prior to calling this method,
        one should check first that the vertices are associated to a
        scalar field.
		Warning: the processed scalar field must be enabled for both
		INPUT & OUTPUT! (see ccGenericCloud::setCurrentScalarField)
        \param process either 'smooth' or 'enhance'
    **/
	bool processScalarField(MESH_SCALAR_FIELD_PROCESS process);

    //! Creates a new mesh with the selected vertices only
    /** Virtual method called after a graphical segmentation.
        It should create a new mesh structure with the vertices that are
        tagged as "visible" (see ccGenericPointCloud::visibilityArray). This method
        may also have to update this mesh if removeSelectedFaces is true.
        In this case, all "selected" triangles should be removed from this mesh.
        \param removeSelectedVertices specifies if the faces composed of one or more selected vertices should be removed or not
        \param selection the point selection (recomputed if NULL)
        \param  vertices the new mesh vertices (recomputed if NULL)
    **/
	virtual ccGenericMesh* createNewMeshFromSelection(bool removeSelectedVertices=false, CCLib::ReferenceCloud* selection=NULL, ccGenericPointCloud* vertices=NULL)=0;

    //inherited from ccDrawableObject
    virtual bool hasColors() const;
    virtual bool hasNormals() const;
    virtual bool hasScalarFields() const;
    virtual bool hasDisplayedScalarField() const;
    virtual void showNormals(bool state);
    virtual bool normalsShown() const;

    //! Returns the vertices cloud
	virtual ccGenericPointCloud* getAssociatedCloud() const;

    //! Sets the associated vertices cloud (warning)
	virtual void setAssociatedCloud(ccGenericPointCloud* cloud);

	//! Shifts all triangles indexes
	/** \param shift index shift (positive)
	**/
	virtual void shiftTriangleIndexes(unsigned shift)=0;

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree
        \param vertices vertices set to use (will be automatically - AND OPTIMALLY - cloned if 0)
		\param clonedMaterials for internal use
		\param clonedNormsTable for internal use
		\param cloneTexCoords for internal use
		\return a copy of this entity
	**/
	virtual ccGenericMesh* clone(ccGenericPointCloud* vertices = 0,
									ccMaterialSet* clonedMaterials = 0,
									NormsIndexesTableType* clonedNormsTable = 0,
									TextureCoordsContainer* cloneTexCoords =0) = 0;

    //! Forces bounding-box update
    virtual void refreshBB()=0;

	//! Returns whether the mesh has materials/textures
	virtual bool hasMaterials() const=0;

	//! Sets whether textures should be displayed or not
	virtual void showMaterials(bool state)=0;

	//! Sets whether textures/material should be displayed or not
	virtual bool materialsShown() const=0;

	/*********************************************************/
	/**************    PER-TRIANGLE NORMALS    ***************/
	/*********************************************************/

	//! Returns whether the mesh has per-triangle normals
	virtual bool hasTriNormals() const=0;

	//! Sets whether to show or not per-triangle normals
	virtual void showTriNorms(bool state)=0;

	//! Returns whether per-triangle normals are shown or not 
	virtual bool triNormsShown() const=0;

	//! Sets per-triangle normals array (may be shared)
	virtual void setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable = true);

	//! Returns per-triangle normals array
	virtual NormsIndexesTableType* getTriNormsTable() const {return m_triNormals;}

	//! Removes per-triangle normals
	virtual void clearTriNormals() { setTriNormsTable(0); }

	/*********************************************************/
	/*********  PER-TRIANGLE TEXTURE COORDINATES  ************/
	/*********************************************************/

	//! Sets per-triangle texture coordinates array (may be shared)
	virtual void setTexCoordinatesTable(TextureCoordsContainer* texCoordsTable, bool autoReleaseOldTable = true);

	//! Returns per-triangle texture coordinates array
	virtual TextureCoordsContainer* getTexCoordinatesTable() const {return m_texCoords;}

	//! Sets associated material set (may be shared)
	virtual void setMaterialSet(ccMaterialSet* materialSet, bool autoReleaseOldMaterialSet = true);

	//! Returns associated material set
	const ccMaterialSet* getMaterialSet() const {return m_materials;}

	//! Laplacian smoothing
	/** \param nbIteration smoothing iterations
		\param factor smoothing 'force'
	**/
	bool laplacianSmooth(unsigned nbIteration=100, float factor=0.01, CCLib::GenericProgressCallback* progressCb=0);

	//inherited from ccHObject
	virtual bool isSerializable() const { return true; }

	//! Interpolates normal(s) inside a given triangle
	/** \param triIndex triangle index
		\param P point where to interpolate (should be inside the triangle!)
		\param[out] N interpolated normal
		\return success
	**/
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) = 0;

	//! Interpolates RGB colors inside a given triangle
	/** \param triIndex triangle index
		\param P point where to interpolate (should be inside the triangle!)
		\param[out] rgb interpolated color
		\return success
	**/
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, colorType rgb[]) = 0;

	//! Get RGB color fom a given triangle texture
	/** \param triIndex triangle index
		\param P point where to grab color (should be inside the triangle!)
		\param[out] rgb texel color
		\param interpolateColorIfNoTexture whether to get color from the RGB field if no texture is associated to the given triangles
		\return success
	**/
	virtual bool getColorFromTexture(unsigned triIndex, const CCVector3& P, colorType rgb[], bool interpolateColorIfNoTexture) = 0;

protected:

    //inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
    virtual void applyGLTransformation(const ccGLMatrix& trans);
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

	//! display
	bool m_showWired;

	//! associated cloud (vertices)
	ccGenericPointCloud* m_associatedCloud;

	//! Per-triangle normals
	NormsIndexesTableType* m_triNormals;

	//! Texture coordinates
	TextureCoordsContainer* m_texCoords;

	//! Materials
	ccMaterialSet* m_materials;
};

#endif
