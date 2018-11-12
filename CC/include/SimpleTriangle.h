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

#ifndef SIMPLE_TRIANGLE_HEADER
#define SIMPLE_TRIANGLE_HEADER

//Local
#include "GenericTriangle.h"

namespace CCLib
{

//! A simple triangle class
/** Implements the GenericTriangle class with references to 3D points.
	WARNING: make sure that references don't point to temporary objects!
	WARNING: not compatible with parallelization.
**/
class CC_CORE_LIB_API SimpleRefTriangle : public GenericTriangle
{
public:

	//! Default constructor
	SimpleRefTriangle()
		: A(nullptr)
		, B(nullptr)
		, C(nullptr)
	{}

	//! Constructor from 3 vertices (references to)
	/** \param _A first vertex
		\param _B second vertex
		\param _C third vertex
	**/
	SimpleRefTriangle(const CCVector3* _A, const CCVector3* _B, const CCVector3* _C)
		: A(_A)
		, B(_B)
		, C(_C)
	{}

	//inherited methods (see GenericDistribution)
	inline const CCVector3* _getA() const override { return A; }
	inline const CCVector3* _getB() const override { return B; }
	inline const CCVector3* _getC() const override { return C; }

	//! A vertex (ref)
	const CCVector3 *A;
	//! B vertex (ref)
	const CCVector3 *B;
	//! C vertex (ref)
	const CCVector3 *C;
};

//! A simple triangle class
/** Implements the GenericTriangle class with a triplet of 3D points.
	Relies on direct storage for speed enhancement and parallelization!
**/
class CC_CORE_LIB_API SimpleTriangle : public GenericTriangle
{
public:

	//! Default constructor
	SimpleTriangle()
		: A(0,0,0)
		, B(0,0,0)
		, C(0,0,0)
	{}

	//! Constructor from 3 vertices
	/** \param _A first vertex
		\param _B second vertex
		\param _C third vertex
	**/
	SimpleTriangle(const CCVector3& _A, const CCVector3& _B, const CCVector3& _C)
		: A(_A)
		, B(_B)
		, C(_C)
	{}

	//inherited methods (see GenericDistribution)
	inline const CCVector3* _getA() const override { return &A; }
	inline const CCVector3* _getB() const override { return &B; }
	inline const CCVector3* _getC() const override { return &C; }

	//! A vertex
	CCVector3 A;
	//! B vertex
	CCVector3 B;
	//! C vertex
	CCVector3 C;
};

}
#endif //SIMPLE_TRIANGLE_HEADER
