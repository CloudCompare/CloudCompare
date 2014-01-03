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

#ifndef CC_INDEXED_TRANSFORMATION_BUFFER_HEADER
#define CC_INDEXED_TRANSFORMATION_BUFFER_HEADER

//Local
#include "ccIndexedTransformation.h"

//system
#include <float.h>
#include <vector>

//! Indexed Transformation buffer
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccIndexedTransformationBuffer : public std::vector< ccIndexedTransformation >
#else
class ccIndexedTransformationBuffer : public std::vector< ccIndexedTransformation >
#endif
{
public:
    
	//! Sorts transformations based on their index
	/** Ascending sort.
	**/
	void sort();

	//! Returns the nearest indexed transformation(s) to a given index
	/** This method returns the preceding and following transformations.
		
		\warning Binary search: buffer must be sorted! (see ccIndexedTransformationBuffer::sort)
		
		\param index query index (e.g. timestamp)
		\param trans1 directly preceding transformation (if any - null otherwise)
		\param trans2 directly following transformation (if any - null otherwise)
		\param trans1IndexInBuffer (optional) index of trans1 in buffer
		\param trans2IndexInBuffer (optional) index of trans2 in buffer
		\return success
	**/
	bool findNearest(	double index,
						const ccIndexedTransformation* &trans1,
						const ccIndexedTransformation* &trans2,
						size_t* trans1IndexInBuffer = 0,
						size_t* trans2IndexInBuffer = 0) const;

	//! Returns the indexed transformation at a given index (interpolates it if necessary)
	/** \warning Binary search: buffer must be sorted! (see ccIndexedTransformationBuffer::sort)

		\param index query index (e.g. timestamp)
		\param trans output transformation (if successful)
		\param maxIndexDistForInterpolation max 'distance' between query index and existing indexes to actually interpolate/output a transformation
		\return success
	**/
	bool getInterpolatedTransformation(	double index,
										ccIndexedTransformation& trans,
										double maxIndexDistForInterpolation = DBL_MAX) const;

protected:

};

#endif
