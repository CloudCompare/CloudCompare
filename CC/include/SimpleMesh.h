//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef SIMPLE_MESH_HEADER
#define SIMPLE_MESH_HEADER

//Local
#include "BoundingBox.h"
#include "GenericIndexedMesh.h"
#include "SimpleTriangle.h"

//System
#include <vector>

namespace CCLib
{

class GenericIndexedCloud;

//! A simple mesh structure, with index-based vertex access
/** Implements the GenericIndexedMesh interface. This mesh is always associated
	to a (index based) point cloud that stores the mesh vertexes.
**/
class CC_CORE_LIB_API SimpleMesh : public GenericIndexedMesh
{
public: //constructors

	//! SimpleMesh Constructor
	/** \param _theVertices the point cloud containing the vertices
		\param linkVerticesWithMesh specifies if the vertex cloud should be deleted when the SimpleMesh object is destructed
	**/
	SimpleMesh(GenericIndexedCloud* _theVertices, bool linkVerticesWithMesh = false);

	//! SimpleMesh destructor
	~SimpleMesh() override;

public: //inherited methods

	void forEach(genericTriangleAction action) override;
	void placeIteratorAtBeginning() override;
	GenericTriangle* _getNextTriangle() override; //temporary
	GenericTriangle* _getTriangle(unsigned triangleIndex) override; //temporary
	VerticesIndexes* getNextTriangleVertIndexes() override;
	VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
	unsigned size() const override { return static_cast<unsigned>(m_triIndexes.size()); }
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
	void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;

public: //specific methods

	//! Returns the mesh capacity
	inline unsigned capacity() const { return static_cast<unsigned>(m_triIndexes.capacity()); }

	//! Returns the vertices
	inline const GenericIndexedCloud* vertices() const { return theVertices; }

	//! Clears the mesh
	inline void clear() { m_triIndexes.resize(0); }

	//! Adds a triangle to the mesh
	/** Vertex indexes are expresesed relatively to the vertex cloud.
		\param i1 first vertex index
		\param i2 second vertex index
		\param i3 third vertex index
	**/
	virtual void addTriangle(unsigned i1, unsigned i2, unsigned i3);

	//! Reserves the memory to store the triangles (as 3 indexes each)
	/** \param n the number of triangles to reserve
		\return true if the method succeeds, false otherwise
	**/
	virtual bool reserve(unsigned n);

	//! Resizes the mesh database
	/** If the new number of elements is smaller than the actual size,
		the overflooding elements will be deleted.
		\param n the new number of triangles
		\return true if the method succeeds, false otherwise
	**/
	virtual bool resize(unsigned n);

protected:

	//! A triangle vertices indexes container
	using TriangleIndexesContainer = std::vector<VerticesIndexes>;
	//! The triangles indexes
	TriangleIndexesContainer m_triIndexes;

	//! Iterator on the list of triangles
	unsigned globalIterator;
	//! Dump triangle structure to transmit temporary data
	SimpleTriangle dummyTriangle;

	//! The associated point cloud (vertices)
	GenericIndexedCloud* theVertices;
	//! Specifies if the associated cloud should be deleted when the mesh is deleted
	bool verticesLinked;

	//! Bounding-box
	BoundingBox m_bbox;
};

}

#endif //SIMPLE_MESH_HEADER
