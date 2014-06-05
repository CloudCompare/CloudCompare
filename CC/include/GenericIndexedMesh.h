//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GENERIC_INDEXED_MESH_HEADER
#define GENERIC_INDEXED_MESH_HEADER

//Local
#include "CCCoreLib.h"
#include "GenericMesh.h"

namespace CCLib
{

//! A triangle descriptor
/** The triangle is described by the 3 indexes
	of its summits.
**/
struct TriangleSummitsIndexes
{
	union
	{
		struct
		{
			unsigned i1,i2,i3;
		};
		unsigned i[3];
	};

	//! Constructor with specified indexes
	TriangleSummitsIndexes(unsigned _i1, unsigned _i2, unsigned _i3)
		: i1(_i1)
		, i2(_i2)
		, i3(_i3)
	{
	}

	//! Default constructor
	TriangleSummitsIndexes()
		: i1(0)
		, i2(0)
		, i3(0)
	{
	}
};

//! A generic mesh with index-based vertex access
/** Implements the GenericMehs interface.
**/
class CC_CORE_LIB_API GenericIndexedMesh : public GenericMesh
{
public:

	//! Default destructor
	virtual ~GenericIndexedMesh() {}

	//! Returns the ith triangle
	/**	Virtual method to request a triangle with a specific index.
		The returned object can be temporary.
		\param triangleIndex of the requested triangle (between 0 and the mesh size-1)
		\return the requested triangle, or 0 if index value is not valid
	**/
	virtual GenericTriangle* _getTriangle(unsigned triangleIndex)=0;

	//! Returns the summits indexes of the ith triangle
	/**	Virtual method to request the 3 summits indexes of a triangle with a
		specific index.
		\param triangleIndex index of the requested triangle (between 0 and the mesh size-1)
		\return the requested indexes (a 3-size array), or 0 if index value is not valid
	**/
	virtual TriangleSummitsIndexes* getTriangleIndexes(unsigned triangleIndex)=0;

	//! Returns the summits of the ith triangle
	/**	Virtual method to request the 3 summits of a triangle with a
		specific index.
		\param triangleIndex index of the requested triangle (between 0 and the mesh size-1)
		\param A the first requested summit
		\param B the second requested summit
		\param C the third requested summit
	**/
	virtual void getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)=0;

	//! Returns the next triangle summit indexes (relatively to the global iterator position)
	/**	Virtual method to handle the mesh global iterator.
		Global iterator position should be increased each time
		this method is called.
		\return a triangle
	**/
	virtual TriangleSummitsIndexes* getNextTriangleIndexes()=0;

protected:
};

}

#endif //GENERIC_INDEXED_MESH_HEADER
