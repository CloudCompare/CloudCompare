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

#ifndef CC_SUB_MESH_HEADER
#define CC_SUB_MESH_HEADER

//Local
#include "qCC_db.h"
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
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::SUB_MESH; }

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false);
	virtual bool isSerializable() const { return true; }

	//inherited methods (ccGenericMesh)
	virtual ccGenericPointCloud* getAssociatedCloud() const;
	//virtual ccGenericMesh* clone(ccGenericPointCloud* vertices = 0, ccMaterialSet* clonedMaterials = 0, NormsIndexesTableType* clonedNormsTable = 0, TextureCoordsContainer* cloneTexCoords =0);
	virtual void refreshBB();
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N);
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb);
	virtual bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb, bool interpolateColorIfNoTexture);
	virtual bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& rgb, bool returnColorIfNoTexture);
	virtual bool hasMaterials() const;
	virtual const ccMaterialSet* getMaterialSet() const;
	virtual int getTriangleMtlIndex(unsigned triangleIndex) const;
	virtual bool hasTextures() const;
	virtual TextureCoordsContainer* getTexCoordinatesTable() const;
	virtual void getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const;
	virtual bool hasPerTriangleTexCoordIndexes() const;
	virtual void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const;
	virtual bool hasTriNormals() const;
	virtual void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const;
	virtual bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const;
	virtual NormsIndexesTableType* getTriNormsTable() const;
	virtual unsigned capacity() const;

	//inherited methods (ccDrawableObject)
	virtual bool hasColors() const;
	virtual bool hasNormals() const;
	virtual bool hasScalarFields() const;
	virtual bool hasDisplayedScalarField() const;
	virtual bool normalsShown() const;

	//inherited methods (GenericIndexedMesh)
	inline virtual unsigned size() const { return m_triIndexes->currentSize(); }
	virtual void forEach(genericTriangleAction& action);
	inline virtual void placeIteratorAtBegining() { m_globalIterator = 0; }
	virtual CCLib::GenericTriangle* _getNextTriangle(); //temporary object
	virtual CCLib::GenericTriangle* _getTriangle(unsigned index); //temporary object
	virtual CCLib::VerticesIndexes* getNextTriangleVertIndexes();
	virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex);
	virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C);
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax);

	//! Returns global index (i.e. relative to the associated mesh) of a given element
	/** \param localIndex local index (i.e. relative to the internal index container)
	**/
	inline virtual unsigned getTriGlobalIndex(unsigned localIndex) const { return m_triIndexes->getValue(localIndex); }

	//! Returns the global index of the triangle pointed by the current element
	inline virtual unsigned getCurrentTriGlobalIndex() const { assert(m_globalIterator < size()); return m_triIndexes->getValue(m_globalIterator); }

	//! Forwards the local element iterator
	inline virtual void forwardIterator() { ++m_globalIterator; }

	//! Clears the mesh
	virtual void clear(bool releaseMemory);

	//! Triangle global index insertion mechanism
	/** \param globalIndex a triangle global index
		\return false if not enough memory
	**/
	virtual bool addTriangleIndex(unsigned globalIndex);

	//! Triangle global index insertion mechanism (range)
	/** \param firstIndex first triangle global index of range
		\param lastIndex last triangle global index of range (excluded)
		\return false if not enough memory
	**/
	virtual bool addTriangleIndex(unsigned firstIndex, unsigned lastIndex);

	//! Sets global index for a given element
	/** \param localIndex local index
		\param globalIndex global index
	**/
	virtual void setTriangleIndex(unsigned localIndex, unsigned globalIndex);

	//! Reserves some memory for hosting the triangle references
	/** \param n the number of triangles (references)
	**/
	virtual bool reserve(unsigned n);

	//! Presets the size of the vector used to store triangle references
	/** \param n the number of triangles (references)
	**/
	virtual bool resize(unsigned n);

	//! Returns the associated mesh
	inline virtual ccMesh* getAssociatedMesh() { return m_associatedMesh; }

	//! Returns the associated mesh (const version)
	inline virtual const ccMesh* getAssociatedMesh() const { return m_associatedMesh; }

	//! Sets the associated mesh
	/** \param mesh parent mesh
		\param unlinkPreviousOne whether to remove any dependency with the previous parent mesh (if any)
	**/
	virtual void setAssociatedMesh(ccMesh* mesh, bool unlinkPreviousOne = true);

	//! Indexes map for createNewSubMeshFromSelection
	typedef GenericChunkedArray<1,unsigned> IndexMap;

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
	virtual ccSubMesh* createNewSubMeshFromSelection(bool removeSelectedFaces, IndexMap* indexMap = 0);

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
	virtual void onUpdateOf(ccHObject* obj);

	//! Associated mesh
	ccMesh* m_associatedMesh;

	//! Container of 3D triangles indexes
	typedef GenericChunkedArray<1,unsigned> ReferencesContainer;

	//! Indexes of (some of) the associated mesh triangles
	ReferencesContainer* m_triIndexes;

	//! Iterator on the triangles references container
	unsigned m_globalIterator;

	//! Bounding-box
	ccBBox m_bBox;
};

#endif //CC_SUB_MESH_HEADER
