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
#include <PointProjectionTools.h>
#include <SimpleTriangle.h>

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
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccMesh(ccGenericPointCloud* vertices, unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! ccMesh constructor (from a CCLib::GenericIndexedMesh)
	/** The GenericIndexedMesh should refer to a known ccGenericPointCloud.
		\param giMesh the GenericIndexedMesh
		\param giVertices giMesh vertices
	**/
	explicit ccMesh(CCLib::GenericIndexedMesh* giMesh, ccGenericPointCloud* giVertices);

	//! Default destructor
	~ccMesh() override;
	
	//! Returns class ID
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::MESH; }

	//! Sets the associated vertices cloud (warning)
	void setAssociatedCloud(ccGenericPointCloud* cloud);

	//! Clones this entity
	/** All the main features of the entity are cloned, except from the octree
		\param vertices vertices set to use (will be automatically - AND OPTIMALLY - cloned if nullptr)
		\param clonedMaterials for internal use
		\param clonedNormsTable for internal use
		\param cloneTexCoords for internal use
		\return a copy of this entity
	**/
	ccMesh* cloneMesh(	ccGenericPointCloud* vertices = nullptr,
						ccMaterialSet* clonedMaterials = nullptr,
						NormsIndexesTableType* clonedNormsTable = nullptr,
						TextureCoordsContainer* cloneTexCoords = nullptr);

	//! Creates a Delaunay 2.5D mesh from a point cloud
	/** See CCLib::PointProjectionTools::computeTriangulation.
	**/
	static ccMesh* Triangulate( ccGenericPointCloud* cloud,
								CC_TRIANGULATION_TYPES type,
								bool updateNormals = false,
								PointCoordinateType maxEdgeLength = 0,
								unsigned char dim = 2);

	//! Creates a Delaunay 2.5D mesh from two polylines
	static ccMesh* TriangulateTwoPolylines(ccPolyline* p1, ccPolyline* p2, CCVector3* projectionDir = nullptr);

	//! Merges another mesh into this one
	/** \param mesh mesh to be merged in this one
		\param createSubMesh whether to create a submesh entity corresponding to the added mesh
		\return success
	**/
	bool merge(const ccMesh* mesh, bool createSubMesh);

	//inherited methods (ccHObject)
	unsigned getUniqueIDForDisplay() const override;
	ccBBox getOwnBB(bool withGLFeatures = false) override;
	bool isSerializable() const override { return true; }
	const ccGLMatrix& getGLTransformationHistory() const override;

	//inherited methods (ccGenericMesh)
	inline ccGenericPointCloud* getAssociatedCloud() const override { return m_associatedCloud; }
	void refreshBB() override;
	bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) override;
	bool interpolateNormalsBC(unsigned triIndex, const CCVector3d& w, CCVector3& N) override;
	bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C) override;
	bool interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgb& C) override;
	bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgba& C) override;
	bool interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgba& C) override;
	void computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const override;
	bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgba& C, bool interpolateColorIfNoTexture) override;
	bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgba& color, bool returnColorIfNoTexture) override;
	unsigned capacity() const override;

	//inherited methods (GenericIndexedMesh)
	void forEach(genericTriangleAction action) override;
	void placeIteratorAtBeginning() override;
	CCLib::GenericTriangle* _getNextTriangle() override; //temporary
	CCLib::GenericTriangle* _getTriangle(unsigned triangleIndex) override; //temporary
	CCLib::VerticesIndexes* getNextTriangleVertIndexes() override;
	CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
	void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;
	unsigned size() const override;
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;

	//const version of getTriangleVertIndexes
	const virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) const;

	//inherited methods (ccDrawableObject)
	bool hasColors() const override;
	bool hasNormals() const override;
	bool hasScalarFields() const override;
	bool hasDisplayedScalarField() const override;
	bool normalsShown() const override;
	void toggleMaterials() override { showMaterials(!materialsShown()); }

	//! Shifts all triangles indexes
	/** \param shift index shift (positive)
	**/
	void shiftTriangleIndexes(unsigned shift);

	//! Flips the triangle
	/** Swaps the second and third vertices indexes
	**/
	void flipTriangles();

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
	bool reserve(size_t n);

	//! Resizes the array of vertex indexes (3 per triangle)
	/** If the new number of elements is smaller than the actual size,
		the overflooding elements will be deleted.
		\param n the new number of triangles
		\return true if the method succeeds, false otherwise
	**/
	bool resize(size_t n);

	//! Removes unused capacity
	inline void shrinkToFit() { if (size() < capacity()) resize(size()); }

	/*********************************************************/
	/**************    PER-TRIANGLE NORMALS    ***************/
	/*********************************************************/

	//inherited from ccGenericMesh
	bool hasTriNormals() const override;
	void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;
	bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const override;
	NormsIndexesTableType* getTriNormsTable() const override { return m_triNormals; }

	//! Sets per-triangle normals array (may be shared)
	void setTriNormsTable(NormsIndexesTableType* triNormsTable, bool autoReleaseOldTable = true);

	//! Removes per-triangle normals
	void clearTriNormals() { setTriNormsTable(nullptr); }

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
	bool hasMaterials() const override;
	const ccMaterialSet* getMaterialSet() const override { return m_materials; }
	int getTriangleMtlIndex(unsigned triangleIndex) const override;

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
	using triangleMaterialIndexesSet = ccArray<int, 1, int>;

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
	bool hasTextures() const override;
	TextureCoordsContainer* getTexCoordinatesTable() const override { return m_texCoords; }
	void getTriangleTexCoordinates(unsigned triIndex, TexCoords2D* &tx1, TexCoords2D* &tx2, TexCoords2D* &tx3) const override;
	bool hasPerTriangleTexCoordIndexes() const override { return m_texCoordIndexes && m_texCoordIndexes->isAllocated(); }
	void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;

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
							ccProgressDialog* progressCb = nullptr);

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

protected: //methods

	//inherited from ccHObject
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	void applyGLTransformation(const ccGLMatrix& trans) override;
	void onUpdateOf(ccHObject* obj) override;
	void onDeletionOf(const ccHObject* obj) override;

	//! Same as other 'computeInterpolationWeights' method with a set of 3 vertices indexes
	void computeInterpolationWeights(const CCLib::VerticesIndexes& vertIndexes, const CCVector3& P, CCVector3d& weights) const;
	//! Same as other 'interpolateNormals' method with a set of 3 vertices indexes
	bool interpolateNormals(const CCLib::VerticesIndexes& vertIndexes, const CCVector3d& w, CCVector3& N, const Tuple3i* triNormIndexes = nullptr);
	//! Same as other 'interpolateColors' method with a set of 3 vertices indexes
	bool interpolateColors(const CCLib::VerticesIndexes& vertIndexes, const CCVector3d& w, ccColor::Rgb& C);
	//! Same as other 'interpolateColors' method with a set of 3 vertices indexes
	bool interpolateColors(const CCLib::VerticesIndexes& vertIndexes, const CCVector3d& w, ccColor::Rgba& C);

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

protected: //members

	//! associated cloud (vertices)
	ccGenericPointCloud* m_associatedCloud;

	//! Per-triangle normals
	NormsIndexesTableType* m_triNormals;

	//! Texture coordinates
	TextureCoordsContainer* m_texCoords;

	//! Materials
	ccMaterialSet* m_materials;

	//! Container of per-triangle vertices indexes (3)
	using triangleIndexesContainer = ccArray<CCLib::VerticesIndexes, 3, unsigned>;
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
	using triangleTexCoordIndexesSet = ccArray<Tuple3i, 3, int>;
	//! Mesh tex coords indexes (per-triangle)
	triangleTexCoordIndexesSet* m_texCoordIndexes;

	//! Set of triplets of indexes referring to mesh normals
	using triangleNormalsIndexesSet = ccArray<Tuple3i, 3, int>;
	//! Mesh normals indexes (per-triangle)
	triangleNormalsIndexesSet* m_triNormalIndexes;
};

#endif //CC_MESH_HEADER
