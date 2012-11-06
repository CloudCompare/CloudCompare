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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef SIMPLE_TRIANGLE_HEADER
#define SIMPLE_TRIANGLE_HEADER

#include "GenericTriangle.h"

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
#endif

namespace CCLib
{

//! A simple triangle class
/** Implements the GenericTriangle class with references to 3D points.
	WARNING: make sure that references are not point to temporary objects!
**/
#ifdef CC_USE_AS_DLL
class CC_DLL_API SimpleRefTriangle : public GenericTriangle
#else
class SimpleRefTriangle : public GenericTriangle
#endif
{
public:

	//! Default constructor
	SimpleRefTriangle()
		: A(0)
		, B(0)
		, C(0)
	{}

	//! Constructor with 3 summits
	/** \param _A first summit
		\param _B second summit
		\param _C third summit
	**/
	SimpleRefTriangle(const CCVector3* _A, const CCVector3* _B, const CCVector3* _C)
		: A(_A)
		, B(_B)
		, C(_C)
	{
	}

	//inherited methods (see GenericDistribution)
	virtual const CCVector3* _getA() const {return A;};
	virtual const CCVector3* _getB() const {return B;};
	virtual const CCVector3* _getC() const {return C;};

	//direct access for speed enhancement
	//! A summit
	const CCVector3 *A;
	//! B summit
	const CCVector3 *B;
	//! C summit
	const CCVector3 *C;
};

//! A simple triangle class
/** Implements the GenericTriangle class with a triplet of 3D points **/
#ifdef CC_USE_AS_DLL
class CC_DLL_API SimpleTriangle : public GenericTriangle
#else
class SimpleTriangle : public GenericTriangle
#endif
{
public:

	//! Default constructor
	SimpleTriangle()
		: A(0.0)
		, B(0.0)
		, C(0.0)
	{}

	//! Constructor with 3 summits
	/** \param _A first summit
		\param _B second summit
		\param _C third summit
	**/
	SimpleTriangle(const CCVector3& _A, const CCVector3& _B, const CCVector3& _C)
		: A(_A)
		, B(_B)
		, C(_C)
	{
	}

	//inherited methods (see GenericDistribution)
	virtual const CCVector3* _getA() const {return &A;};
	virtual const CCVector3* _getB() const {return &B;};
	virtual const CCVector3* _getC() const {return &C;};

	//direct storage for speed enhancement and parallel strategies compatibility!
	//! A summit
	CCVector3 A;
	//! B summit
	CCVector3 B;
	//! C summit
	CCVector3 C;
};

}
#endif
