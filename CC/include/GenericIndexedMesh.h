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

#ifndef GENERIC_INDEXED_MESH_HEADER
#define GENERIC_INDEXED_MESH_HEADER

//Local
#include "GenericMesh.h"

namespace CCLib
{

//! Triangle described by the indexes of its 3 vertices
struct VerticesIndexes
{
	union
	{
		struct
		{
			unsigned i1, i2, i3;
		};
		unsigned i[3];
	};

	//! Constructor with specified indexes
	VerticesIndexes(unsigned _i1, unsigned _i2, unsigned _i3)
		: i1(_i1)
		, i2(_i2)
		, i3(_i3)
	{}

	//! Default constructor
	VerticesIndexes()
		: i1(0)
		, i2(0)
		, i3(0)
	{}
};

//! A generic mesh with index-based vertex access
/** Implements the GenericMehs interface.
**/
class CC_CORE_LIB_API GenericIndexedMesh : public GenericMesh
{
public:

	//! Default destructor
	~GenericIndexedMesh() override = default;

	//! Returns the ith triangle
	/**	Virtual method to request a triangle with a specific index.
		The returned object can be temporary.
		\param triangleIndex of the requested triangle (between 0 and the mesh size-1)
		\return the requested triangle, or 0 if index value is not valid
	**/
	virtual GenericTriangle* _getTriangle(unsigned triangleIndex) = 0;

	//! Returns the indexes of the vertices of a given triangle
	/**	\param triangleIndex index of the triangle (between 0 and size(mesh)-1)
		\return the triangle indexes (or 0 if index value is not valid)
	**/
	virtual VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) = 0;

	//! Returns the vertices of a given triangle
	/**	\param[in] triangleIndex index of the triangle (between 0 and the size(mesh)-1)
		\param[out] A first vertex
		\param[out] B second vertex
		\param[out] C third vertex
	**/
	virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const = 0;

	//! Returns the indexes of the vertices of the next triangle (relatively to the global iterator position)
	/**	\return the triangle indexes (or 0 if the global iterator is out of bounds)
	**/
	virtual VerticesIndexes* getNextTriangleVertIndexes() = 0;
};

}

#endif //GENERIC_INDEXED_MESH_HEADER
