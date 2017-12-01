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

#ifndef CC_MESH_HEADER
#define CC_MESH_HEADER

//CCLib
#include <SimpleTriangle.h>
#include <PointProjectionTools.h>

//Local
#include "ccGenericMesh.h"

class ccProgressDialog;
class ccPolyline;

//! Triangular mesh
class QCC_DB_LIB_API ccMesh : public ccGenericMesh
{
public:

	//! Default ccMesh constructor
	/** \param vertices the vertices cloud
	**/
	explicit ccMesh(ccGenericPointCloud* vertices);

	//! ccMesh constructor (from a CCLib::GenericIndexedMesh)
	/** The GenericIndexedMesh should refer to a known ccGenericPointCloud.
		\param giMesh the GenericIndexedMesh
		\param giVertices giMesh vertices
	**/
	explicit ccMesh(CCLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices);

	//! Default destructor
	virtual ~ccMesh();

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::MESH; }

	//! Sets the associated vertices cloud (warning)
	void setAssociatedCloud(ccGenericPointCloud* cloud);

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree
		\param vertices vertices set to use (will be automatically - AND OPTIMALLY - cloned if 0)
		\param clonedMaterials for internal use
		\param clonedNormsTable for internal use
		\param cloneTexCoords for internal use
		\return a copy of this entity
	**/
	ccMesh* cloneMesh(	ccGenericPointCloud* vertices = 0,
								ccMaterialSet* clonedMaterials = 0,
								NormsIndexesTableType* clonedNormsTable = 0,
								TextureCoordsContainer* cloneTexCoords = 0);

	//! Creates a Delaunay 2.5D mesh from a point cloud
	/** See CCLib::PointProjectionTools::computeTriangulation.
	**/
	static ccMesh* Triangulate( ccGenericPointCloud* cloud,
								CC_TRIANGULATION_TYPES type,
								bool updateNormals = false,
								PointCoordinateType maxEdgeLength = 0,
								unsigned char dim = 2);

	//! Creates a Delaunay 2.5D mesh from two polylines
	static ccMesh* TriangulateTwoPolylines(ccPolyline* p1, ccPolyline* p2, CCVector3* projectionDir = 0);

	//! Merges another mesh into this one
	/** \param mesh mesh to be merged in this one
		\param createSubMesh whether to create a submesh entity corresponding to the added mesh
		\return success
	**/
	bool merge(const ccMesh* mesh, bool createSubMesh);

	//inherited methods (ccHObject)
	virtual unsigned getUniqueIDForDisplay() const override;
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;
	virtual bool isSerializable() const override { return true; }
	virtual const ccGLMatrix& getGLTransformationHistory() const override;

	//inherited methods (ccGenericMesh)
	inline virtual ccGenericPointCloud* getAssociatedCloud() const override { return m_associatedCloud; }
	virtual void refreshBB() override;
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) override;
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C) override;
	virtual void computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const override;
	virtual bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C, bool interpolateColorIfNoTexture) override;
	virtual bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& C, bool returnColorIfNoTexture) override;
	virtual unsigned capacity() const override;

	//inherited methods (GenericIndexedMesh)
	virtual void forEach(genericTriangleAction& action) override;
	virtual void placeIteratorAtBegining() override;
	virtual CCLib::GenericTriangle* _getNextTriangle() override; //temporary
	virtual CCLib::GenericTriangle* _getTriangle(unsigned triangleIndex) override; //temporary
	virtual CCLib::VerticesIndexes* getNextTriangleVertIndexes() override;
	virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
	virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) override;
	virtual unsigned size() const override;
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;

	//const version of getTriangleVertIndexes
	const virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) const;

	//inherited methods (ccDrawableObject)
	virtual bool hasColors() const override;
	virtual bool hasNormals() const override;
	virtual bool hasScalarFields() const override;
	virtual bool hasDisplayedScalarField() const override;
	virtual bool normalsShown() const override;
	virtual void toggleMaterials() override { showMaterials(!materialsShown()); }

	//! Shifts all triangles indexes
	/** \param shift index shift (positive)
	**/
	void shiftTriangleIndexes(unsigned shift);

	//! Adds a triangle to the mesh
	/** \warning Bounding-box validity is broken after a call to this method.
		However, for the sake of performance, no call to notifyGeometryUpdate
		is made automatically. Make sure to do so when all modifications are done!
		\param i1 first vertex index (relatively to the vertex cloud)
		\param i2 second vertex index (relatively to the vertex cloud)
		\param i3 third vertex index (relatively to the vertex cloud)
	**/
	void addTriangle(unsigned i1, unsigned i2, unsigned i3);

	//! Reserves the memory to store the vertex indexes (3 per triangle)
	/** \param n the number of triangles to reserve
		\return true if the method succeeds, false otherwise
	**/
	bool reserve(unsigned n);

	//! Resizes the array of vertex indexes (3 per triangle)
	/** If the new number of elements is smaller than the actual size,
		the overflooding elements will be deleted.
		\param n the new number of triangles
		\return true if the method succeeds, false otherwise
	**/
	bool resize(unsigned n);

	//! Removes unused capacity
	inline void shrinkToFit() { if (size() < capacity()) resize(size()); }

	/*********************************************************/
	/**************    PER-TRIANGLE NORMALS    ***************/
	/*********************************************************/

	//inherited from ccGenericMesh
	virtual bool hasTriNormals() const override;
	virtual void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;
	virtual bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const override;
	virtual NormsIndexesTableType* getTriNormsTable() const override { return m_triNormals; }

	//! Sets per-triangle normals array (may be shared)
	void setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable = true);

	//! Removes per-triangle normals
	void clearTriNormals() { setTriNormsTable(0); }

	//! Returns whether per triangle normals are enabled
	/** To enable per triangle normals, you should:
		- first, reserve memory for triangles (this is always the first thing to do)
		- associate this mesh to a triangle normals array (see ccMesh::setTriNormsTable)
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
		\param i1 first vertex normal index
		\param i2 second vertex normal index
		\param i3 third vertex normal index
	**/
	void addTriangleNormalIndexes(int i1, int i2, int i3);

	//! Sets a triplet of normal indexes for a given triangle
	/** \param triangleIndex triangle index
		\param i1 first vertex normal index
		\param i2 second vertex normal index
		\param i3 third vertex normal index
	**/
	void setTriangleNormalIndexes(unsigned triangleIndex, int i1, int i2, int i3);

	//! Removes any per-triangle triplets of normal indexes
	void removePerTriangleNormalIndexes();

	/********************************************************/
	/************    PER-TRIANGLE MATERIAL    ***************/
	/********************************************************/

	//inherited from ccGenericMesh
	virtual bool hasMaterials() const override;
	virtual const ccMaterialSet* getMaterialSet() const override { return m_materials; }
	virtual int getTriangleMtlIndex(unsigned triangleIndex) const override;

	//! Converts materials to vertex colors
	/** Warning: this method will overwrite colors (if any)
	**/
	bool convertMaterialsToVertexColors();

	//! Returns whether this mesh as per-triangle material index
	bool hasPerTriangleMtlIndexes() const { return m_triMtlIndexes && m_triMtlIndexes->isAllocated(); }

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

	//! Container of per-triangle material descriptors
	typedef GenericChunkedArray<1,int> triangleMaterialIndexesSet;

	//! Sets per-triangle material indexes array
	void setTriangleMtlIndexesTable(triangleMaterialIndexesSet* matIndexesTable, bool autoReleaseOldTable = true);

	//! Returns the per-triangle material indexes array
	inline const triangleMaterialIndexesSet* getTriangleMtlIndexesTable() const { return m_triMtlIndexes; }

	//! Sets triangle material indexes
	/** Cf. ccMesh::reservePerTriangleMtlIndexes.
		\param triangleIndex triangle index
		\param mtlIndex triangle material index
	**/
	void setTriangleMtlIndex(unsigned triangleIndex, int mtlIndex);

	//! Sets associated material set (may be shared)
	void setMaterialSet(ccMaterialSet* materialSet, bool autoReleaseOldMaterialSet = true);

	/******************************************************************/
	/************    PER-TRIANGLE TEXTURE COORDINATE    ***************/
	/******************************************************************/

	//inherited from ccGenericMesh
	virtual bool hasTextures() const override;
	virtual TextureCoordsContainer* getTexCoordinatesTable() const override { return m_texCoords; }
	virtual void getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const override;
	virtual bool hasPerTriangleTexCoordIndexes() const override { return m_texCoordIndexes && m_texCoordIndexes->isAllocated(); }
	virtual void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;

	//! Sets per-triangle texture coordinates array (may be shared)
	void setTexCoordinatesTable(TextureCoordsContainer* texCoordsTable, bool autoReleaseOldTable = true);

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
		\param i1 first vertex tex coords index
		\param i2 second vertex tex coords index
		\param i3 third vertex tex coords index
	**/
	void addTriangleTexCoordIndexes(int i1, int i2, int i3);

	//! Sets a triplet of tex coords indexes for a given triangle
	/** \param triangleIndex triangle index
		\param i1 first vertex tex coords index
		\param i2 second vertex tex coords index
		\param i3 third vertex tex coords index
	**/
	void setTriangleTexCoordIndexes(unsigned triangleIndex, int i1, int i2, int i3);

	//! Computes normals
	/** \param perVertex whether normals should be computed per-vertex or per-triangle
		\return success
	**/
	bool computeNormals(bool perVertex);

	//! Computes per-vertex normals
	bool computePerVertexNormals();

	//! Computes per-triangle normals
	bool computePerTriangleNormals();

	//! Laplacian smoothing
	/** \param nbIteration smoothing iterations
		\param factor smoothing 'force'
		\param progressCb progress dialog callback
	**/
	bool laplacianSmooth(	unsigned nbIteration = 100,
							PointCoordinateType factor = static_cast<PointCoordinateType>(0.01),
							ccProgressDialog* progressCb = 0);

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

	//! Subdivides mesh (so as to ensure that all triangles are falls below 'maxArea')
	/** \return subdivided mesh (if successful)
	**/
	ccMesh* subdivide(PointCoordinateType maxArea) const;

	//! Creates a new mesh with the selected vertices only
	/** This method is called after a graphical segmentation.
		It creates a new mesh structure with the vertices that are
		tagged as "visible" (see ccGenericPointCloud::visibilityArray).
		This method will also update this mesh if removeSelectedFaces is true.
		In this case, all "selected" triangles will be removed from this mesh's instance.

		\param removeSelectedFaces specifies if the faces composed only of 'selected' vertices should be removed or not
	**/
	ccMesh* createNewMeshFromSelection(bool removeSelectedFaces);

	//! Swaps two triangles
	/** Automatically updates internal structures (i.e. lookup tables for
		material, normals, etc.).
	**/
	void swapTriangles(unsigned index1, unsigned index2);

	//! Transforms the mesh per-triangle normals
	void transformTriNormals(const ccGLMatrix& trans);

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
	virtual void applyGLTransformation(const ccGLMatrix& trans) override;
	virtual void onUpdateOf(ccHObject* obj) override;
	virtual void onDeletionOf(const ccHObject* obj) override;

	//! Same as other 'computeInterpolationWeights' method with a set of 3 vertices indexes
	void computeInterpolationWeights(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, CCVector3d& weights) const;
	//! Same as other 'interpolateNormals' method with a set of 3 vertices indexes
	bool interpolateNormals(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, CCVector3& N, const int* triNormIndexes = 0);
	//! Same as other 'interpolateColors' method with a set of 3 vertices indexes
	bool interpolateColors(unsigned i1, unsigned i2, unsigned i3, const CCVector3& P, ccColor::Rgb& C);

	//! Used internally by 'subdivide'
	bool pushSubdivide(/*PointCoordinateType maxArea, */unsigned indexA, unsigned indexB, unsigned indexC);

	/*** EXTENDED CALL SCRIPTS (FOR CC_SUB_MESHES) ***/
	
	//0 parameter
	#define ccMesh_extended_call0(baseName,recursiveName) \
	inline virtual void recursiveName() \
	{ \
		baseName(); \
		for (Container::iterator it = m_children.begin(); it != m_children.end(); ++it) \
			if ((*it)->isA(CC_TYPES::SUB_MESH)) \
				static_cast<ccGenericMesh*>(*it)->baseName(); \
	} \

	//1 parameter
	#define ccMesh_extended_call1(baseName,param1Type,recursiveName) \
	inline virtual void recursiveName(param1Type p) \
	{ \
		baseName(p); \
		for (Container::iterator it = m_children.begin(); it != m_children.end(); ++it) \
			if ((*it)->isA(CC_TYPES::SUB_MESH)) \
				static_cast<ccGenericMesh*>(*it)->baseName(p); \
	} \

	//recursive equivalents of some of ccGenericMesh methods (applied to sub-meshes as well)
	ccMesh_extended_call1(showNormals, bool, showNormals_extended)

	//! associated cloud (vertices)
	ccGenericPointCloud* m_associatedCloud;

	//! Per-triangle normals
	NormsIndexesTableType* m_triNormals;

	//! Texture coordinates
	TextureCoordsContainer* m_texCoords;

	//! Materials
	ccMaterialSet* m_materials;

	//! Container of per-triangle vertices indexes (3)
	typedef GenericChunkedArray<3,unsigned> triangleIndexesContainer;
	//! Triangles' vertices indexes (3 per triangle)
	triangleIndexesContainer* m_triVertIndexes;

	//! Iterator on the list of triangles
	unsigned m_globalIterator;
	//! Dump triangle structure to transmit temporary data
	CCLib::SimpleRefTriangle m_currentTriangle;

	//! Bounding-box
	ccBBox m_bBox;

	//! Per-triangle material indexes
	triangleMaterialIndexesSet* m_triMtlIndexes;

	//! Set of triplets of indexes referring to mesh texture coordinates
	typedef GenericChunkedArray<3,int> triangleTexCoordIndexesSet;
	//! Mesh tex coords indexes (per-triangle)
	triangleTexCoordIndexesSet* m_texCoordIndexes;

	//! Set of triplets of indexes referring to mesh normals
	typedef GenericChunkedArray<3,int> triangleNormalsIndexesSet;
	//! Mesh normals indexes (per-triangle)
	triangleNormalsIndexesSet* m_triNormalIndexes;
};

#endif //CC_MESH_HEADER
