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

#ifndef CC_MESH_HEADER
#define CC_MESH_HEADER

//! Max number of displayed triangles (per entity) in "low detail" display
const unsigned MAX_LOD_FACES_NUMBER = 250000;

//CCLib
#include <SimpleTriangle.h>

#include "ccGenericMesh.h"
#include "ccMaterial.h"

//! Triangular mesh
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccMesh : public ccGenericMesh
#else
class ccMesh : public ccGenericMesh
#endif
{
public:

	//! Default ccMesh constructor
    /** \param vertices the vertices cloud
    **/
	ccMesh(ccGenericPointCloud* vertices);

    //! ccMesh constructor (from a CCLib::GenericIndexedMesh)
    /** The GenericIndexedMesh should refer to a known ccGenericPointCloud.
        \param giMesh the GenericIndexedMesh
        \param giVertices giMesh vertices
    **/
    ccMesh(CCLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices);

	//! Default destructor
	virtual ~ccMesh();

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_MESH;};

	//inherited methods (ccGenericMesh)
	virtual ccGenericMesh* createNewMeshFromSelection(bool removeSelectedVertices=false, CCLib::ReferenceCloud* selection=NULL, ccGenericPointCloud* vertices=NULL);
	virtual ccGenericMesh* clone(ccGenericPointCloud* vertices = 0, ccMaterialSet* clonedMaterials = 0, NormsIndexesTableType* clonedNormsTable = 0, TextureCoordsContainer* cloneTexCoords =0);
    virtual void refreshBB();
	virtual bool hasMaterials() const;
	virtual void showMaterials(bool state) {m_materialsShown = state;}
	virtual bool materialsShown() const {return m_materialsShown;}
	virtual bool hasTriNormals() const;
	virtual void showTriNorms(bool state) {m_triNormsShown=state;}
	virtual bool triNormsShown() const {return m_triNormsShown;}
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N);
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, colorType rgb[]);
	virtual bool getColorFromTexture(unsigned triIndex, const CCVector3& P, colorType rgb[], bool interpolateColorIfNoTexture);
	virtual void setTexCoordinatesTable(TextureCoordsContainer* texCoordsTable, bool autoReleaseOldTable = true);

	//inherited methods (GenericIndexedMesh)
	virtual void forEach(genericTriangleAction& anAction);
	virtual void placeIteratorAtBegining();
	virtual CCLib::GenericTriangle* _getNextTriangle(); //temporary
	virtual CCLib::GenericTriangle* _getTriangle(unsigned triangleIndex); //temporary
	virtual CCLib::TriangleSummitsIndexes* getNextTriangleIndexes();
	virtual CCLib::TriangleSummitsIndexes* getTriangleIndexes(unsigned triangleIndex);
	virtual void getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C);
	virtual unsigned size() const;
	virtual void getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[]);
	virtual void shiftTriangleIndexes(unsigned shift);

	//const version of getTriangleIndexes
	const virtual CCLib::TriangleSummitsIndexes* getTriangleIndexes(unsigned triangleIndex) const;

	//inherited methods (ccHObject)
    virtual ccBBox getMyOwnBB();

	//inherited methods (ccDrawableObject)
	virtual void setDisplay(ccGenericGLDisplay* win);

	//! Returns max capacity
	virtual unsigned maxSize() const;

	//! Adds a triangle to the mesh
	/** Warning: bounding box validity is broken after a call to this method.
        However, for the sake of performance, no call to updateModificationTime
        is made automatically. Make sure to do so when all modifications are done.
        \param i1 first summit index (relatively to the vertex cloud)
		\param i2 second summit index (relatively to the vertex cloud)
		\param i3 third summit index (relatively to the vertex cloud)
	**/
	void addTriangle(unsigned i1, unsigned i2, unsigned i3);

	//! Reserves the memory to store the triangles (as 3 indexes each)
	/** \param n the number of triangles to reserve
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned n);

	//! Resizes the mesh database
	/** If the new number of elements is smaller than the actual size,
		the overflooding elements will be deleted.
		\param n the new number of triangles
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned n);

	/*********************************************************/
	/**************    PER-TRIANGLE NORMALS    ***************/
	/*********************************************************/

	//! Returns whether per triangle normals are enabled
	/** To enable per triangle normals, you should:
		- first, reserve memory for triangles (this is always the first thing to do)
		- associate this mesh to a triangle normals array (see ccGenericMesh::setTriNormsTable)
		- reserve memory to store per-triangle normal indexes with ccMesh::reservePerTriangleNormalIndexes
		- add for each triangle a triplet of indexes (referring to stored normals)
	**/
	bool arePerTriangleNormalsEnabled() const;

	//! Reserves memory to store per-triangle triplets of normal indexes
	/** Before adding per-triangle normal indexes triplets to
		the mesh (with ccMesh::addTriangleNormalsIndexes()) be
		sure to reserve the  necessary amount of memory with
		this method. This method reserves memory for as many
		normals indexes triplets as the number of triangles
		in the mesh (effictively stored or reserved - a call to
		ccMesh::reserve prior to this one is mandatory).
		\return true if ok, false if there's not enough memory
	**/
	bool reservePerTriangleNormalIndexes();

	//! Adds a triplet of normal indexes for next triangle
	/** Make sure per-triangle normal indexes array is allocated
		(see reservePerTriangleNormalIndexes)
		\param i1 first summit normal index
		\param i2 first summit normal index
		\param i3 first summit normal index
	**/
	void addTriangleNormalIndexes(int i1, int i2, int i3);

	//! Sets a triplet of normal indexes for a given triangle
	/** \param triangleIndex triangle index
		\param i1 first summit normal index
		\param i2 first summit normal index
		\param i3 first summit normal index
	**/
	void setTriangleNormalIndexes(unsigned triangleIndex, int i1, int i2, int i3);

	//! Removes any per-triangle triplets of normal indexes
	void removePerTriangleNormalIndexes();

	//inherited from ccGenericMesh
	/** Re-implemented to automatically release 'per triangle normal indexes'
		if per-triangle normals table is removed (triNormsTable==0).
	**/
	virtual void setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable = true);

	/********************************************************/
	/************    PER-TRIANGLE MATERIAL    ***************/
	/********************************************************/

	//! Reserves memory to store per-triangle material index
	/** Before adding per-triangle material index to
		the mesh (with ccMesh::addTriangleMtlIndex()) be sure
		to reserve the  necessary amount of memory with this
		method. This method reserves memory for as many
		material descriptors as the number of triangles in
		the mesh (effictively stored or reserved - a call to
		ccMesh::reserve prior to this one is mandatory).
		\return true if ok, false if there's not enough memory
	**/
	bool reservePerTriangleMtlIndexes();

	//! Removes any per-triangle material indexes
	void removePerTriangleMtlIndexes();

	//! Adds triangle material index for next triangle
	/** Cf. ccMesh::reservePerTriangleMtlIndexes.
		\param mtlIndex triangle material index
	**/
	void addTriangleMtlIndex(int mtlIndex);

	//! Sets triangle material indexes
	/** Cf. ccMesh::reservePerTriangleMtlIndexes.
		\param triangleIndex triangle index
		\param mtlIndex triangle material index
	**/
	void setTriangleMtlIndex(unsigned triangleIndex, int mtlIndex);

	/******************************************************************/
	/************    PER-TRIANGLE TEXTURE COORDINATE    ***************/
	/******************************************************************/
	//! Reserves memory to store per-triangle triplets of tex coords indexes
	/** Before adding per-triangle tex coords indexes triplets to
		the mesh (with ccMesh::addTriangleTexCoordIndexes()) be
		sure to reserve the  necessary amount of memory with
		this method. This method reserves memory for as many
		tex coords indexes triplets as the number of triangles
		in the mesh (effictively stored or reserved - a call to
		ccMesh::reserve prior to this one is mandatory).
		\return true if ok, false if there's not enough memory
	**/
	bool reservePerTriangleTexCoordIndexes();

	//! Remove per-triangle tex coords indexes
	void removePerTriangleTexCoordIndexes();

	//! Adds a triplet of tex coords indexes for next triangle
	/** Make sure per-triangle tex coords indexes array is allocated
		(see reservePerTriangleTexCoordIndexes)
		\param i1 first summit tex coords index
		\param i2 first summit tex coords index
		\param i3 first summit tex coords index
	**/
	void addTriangleTexCoordIndexes(int i1, int i2, int i3);

	//! Sets a triplet of tex coords indexes for a given triangle
	/** \param triangleIndex triangle index
		\param i1 first summit tex coords index
		\param i2 first summit tex coords index
		\param i3 first summit tex coords index
	**/
	void setTriangleTexCoordIndexes(unsigned triangleIndex, int i1, int i2, int i3);

	//! Returns whether textures are available for this mesh
	bool hasTextures() const;

	//! Enables polygon stippling
	void enableStippling(bool state) {m_stippling = state;}

	//! Returns whether polygon stippling is enabled or not
	bool stipplingEnabled() const {return m_stippling;}

	//! Subdivides mesh (so as to ensure that all triangles are falls below 'maxArea')
	/** \return subdivided mesh (if successfull)
	**/
	ccMesh* subdivide(float maxArea) const;

protected:

    //inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

    //inherited from ccGenericMesh
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

	//! Used internally by 'subdivide'
	bool pushSubdivide(PointCoordinateType maxArea, unsigned indexA, unsigned indexB, unsigned indexC);

	//! Container of per-triangle vertices indexes (3)
	typedef GenericChunkedArray<3,unsigned> triangleIndexesContainer;
	//! Triangles indexes
	triangleIndexesContainer* m_triIndexes;

	//! Iterator on the list of triangle summits indexes
	unsigned m_globalIterator;
	//! Dump triangle structure to transmit temporary data
	CCLib::SimpleRefTriangle m_currentTriangle;

    //! Bounding-box
    ccBBox m_bBox;

	//! Container of per-triangle material descriptors
	typedef GenericChunkedArray<1,int> triangleMaterialIndexesSet;
	//! Per-triangle material indexes
	triangleMaterialIndexesSet* m_triMtlIndexes;
	//! Texture/material display flag
	bool m_materialsShown;

	//! Set of triplets of indexes referring to mesh texture coordinates
	typedef GenericChunkedArray<3,int> triangleTexCoordIndexesSet;
	//! Mesh tex coords indexes (per-triangle)
	triangleTexCoordIndexesSet* m_texCoordIndexes;

	//! Set of triplets of indexes referring to mesh normals
	typedef GenericChunkedArray<3,int> triangleNormalsIndexesSet;
	//! Mesh normals indexes (per-triangle)
	triangleNormalsIndexesSet* m_triNormalIndexes;
	//! Per-triangle normals display flag
	bool m_triNormsShown;

	//! Polygon stippling state
	bool m_stippling;
};

#endif
