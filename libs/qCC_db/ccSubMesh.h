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

#ifndef CC_SUB_MESH_HEADER
#define CC_SUB_MESH_HEADER

//Local
#include "ccGenericMesh.h"
#include "ccBBox.h"

class ccMesh;

//! A sub-mesh
/** Equivalent to a CCLib::ReferenceCloud for a mesh
**/
class QCC_DB_LIB_API ccSubMesh : public ccGenericMesh
{
public:

	//! Default constructor
	explicit ccSubMesh(ccMesh* parentMesh);
	//! Destructor
	virtual ~ccSubMesh();

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SUB_MESH; }

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;
	virtual bool isSerializable() const override { return true; }

	//inherited methods (ccGenericMesh)
	virtual ccGenericPointCloud* getAssociatedCloud() const override;
	virtual void refreshBB() override;
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) override;
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb) override;
	virtual bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb, bool interpolateColorIfNoTexture) override;
	virtual bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& rgb, bool returnColorIfNoTexture) override;
	virtual bool hasMaterials() const override;
	virtual const ccMaterialSet* getMaterialSet() const override;
	virtual int getTriangleMtlIndex(unsigned triangleIndex) const override;
	virtual bool hasTextures() const override;
	virtual TextureCoordsContainer* getTexCoordinatesTable() const override;
	virtual void getTriangleTexCoordinates(unsigned triIndex, TexCoords2D* &tx1, TexCoords2D* &tx2, TexCoords2D* &tx3) const override;
	virtual bool hasPerTriangleTexCoordIndexes() const override;
	virtual void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;
	virtual bool hasTriNormals() const override;
	virtual void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override;
	virtual bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const override;
	virtual NormsIndexesTableType* getTriNormsTable() const override;
	virtual unsigned capacity() const override;

	//inherited methods (ccDrawableObject)
	virtual bool hasColors() const override;
	virtual bool hasNormals() const override;
	virtual bool hasScalarFields() const override;
	virtual bool hasDisplayedScalarField() const override;
	virtual bool normalsShown() const override;

	//inherited methods (GenericIndexedMesh)
	inline virtual unsigned size() const override { return static_cast<unsigned>(m_triIndexes.size()); }
	virtual void forEach(genericTriangleAction action) override;
	inline virtual void placeIteratorAtBeginning() override { m_globalIterator = 0; }
	virtual CCLib::GenericTriangle* _getNextTriangle() override; //temporary object
	virtual CCLib::GenericTriangle* _getTriangle(unsigned index) override; //temporary object
	virtual CCLib::VerticesIndexes* getNextTriangleVertIndexes() override;
	virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
	virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;

	//! Returns global index (i.e. relative to the associated mesh) of a given element
	/** \param localIndex local index (i.e. relative to the internal index container)
	**/
	inline unsigned getTriGlobalIndex(unsigned localIndex) const { return m_triIndexes[localIndex]; }

	//! Returns the global index of the triangle pointed by the current element
	inline unsigned getCurrentTriGlobalIndex() const { assert(m_globalIterator < size()); return m_triIndexes[m_globalIterator]; }

	//! Forwards the local element iterator
	inline void forwardIterator() { ++m_globalIterator; }

	//! Clears the mesh
	void clear(bool releaseMemory);

	//! Triangle global index insertion mechanism
	/** \param globalIndex a triangle global index
		\return false if not enough memory
	**/
	bool addTriangleIndex(unsigned globalIndex);

	//! Triangle global index insertion mechanism (range)
	/** \param firstIndex first triangle global index of range
		\param lastIndex last triangle global index of range (excluded)
		\return false if not enough memory
	**/
	bool addTriangleIndex(unsigned firstIndex, unsigned lastIndex);

	//! Sets global index for a given element
	/** \param localIndex local index
		\param globalIndex global index
	**/
	void setTriangleIndex(unsigned localIndex, unsigned globalIndex);

	//! Reserves some memory for hosting the triangle references
	/** \param n the number of triangles (references)
	**/
	bool reserve(size_t n);

	//! Presets the size of the vector used to store triangle references
	/** \param n the number of triangles (references)
	**/
	bool resize(size_t n);

	//! Returns the associated mesh
	inline ccMesh* getAssociatedMesh() { return m_associatedMesh; }

	//! Returns the associated mesh (const version)
	inline const ccMesh* getAssociatedMesh() const { return m_associatedMesh; }

	//! Sets the associated mesh
	/** \param mesh parent mesh
		\param unlinkPreviousOne whether to remove any dependency with the previous parent mesh (if any)
	**/
	void setAssociatedMesh(ccMesh* mesh, bool unlinkPreviousOne = true);

	//! Indexes map for createNewSubMeshFromSelection
	typedef std::vector<unsigned> IndexMap;

	//! Creates a new sub mesh with the selected vertices only
	/** This method is called after a graphical segmentation
		or by ccMesh::createNewMeshFromSelection.
		It creates a new sub-mesh structure with the vertices that are
		tagged as "visible" (see ccGenericPointCloud::visibilityArray).
		This method will also update this sub-mesh if removeSelectedFaces is true.
		In this case, all "selected" triangles will be removed from this sub-mesh's instance.

		\param removeSelectedFaces specifies if the faces composed only of 'selected' vertices should be removed or not
		\param indexMap if an index map is provided, it will be used to 'translate' global indexes
	**/
	ccSubMesh* createNewSubMeshFromSelection(bool removeSelectedFaces, IndexMap* indexMap = 0);

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
	virtual void onUpdateOf(ccHObject* obj) override;

	//! Associated mesh
	ccMesh* m_associatedMesh;

	//! Container of 3D triangles indexes
	typedef std::vector<unsigned> ReferencesContainer;

	//! Indexes of (some of) the associated mesh triangles
	ReferencesContainer m_triIndexes;

	//! Iterator on the triangles references container
	unsigned m_globalIterator;

	//! Bounding-box
	ccBBox m_bBox;
};

#endif //CC_SUB_MESH_HEADER
