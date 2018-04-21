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

#ifndef CHAMFER_DISTANCE_TRANSFORM_HEADER
#define CHAMFER_DISTANCE_TRANSFORM_HEADER

//Local
#include "Grid3D.h"
#include "MathTools.h"

namespace CCLib
{

class GenericProgressCallback;
class NormalizedProgress;

//! Class to compute a Chamfer distance field on a 3D grid
/** Internally we use 'unsigned short' value to limit memory consumption.
	For computational reasons, the max computable 'distance' is 0xFAFA = 64250.
**/
class CC_CORE_LIB_API ChamferDistanceTransform : public Grid3D<unsigned short>, public MathTools
{

public:

	//! Max possible 'distance'
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const unsigned short MAX_DIST = 0xFAFA;

	//! Initializes the grid
	/** 'Zero' cells must be initialized with setValue(0).
		The grid must be explicitelty initialized prior to any action.
		\return true if the initialization succeeded
	**/
	inline bool init(const Tuple3ui& gridSize)
	{
		return Grid3D<GridElement>::init(gridSize.x, gridSize.y, gridSize.z, 1, MAX_DIST);
	}

	//! Computes the Chamfer distance on the whole grid
	/** Propagates the distances on the whole grid. The 'zeros' should
		have already been initialized before calling this method (see
		ChamferDistanceTransform::setZero).
		\param type the Chamfer distance type
		\param progressCb the client application can get some notification of the process
			progress through this callback mechanism (see GenericProgressCallback)
		\return max distance (or -1 if an error occurred)
	**/
	int propagateDistance(CC_CHAMFER_DISTANCE_TYPE type, GenericProgressCallback* progressCb = nullptr);

};

}

#endif //CHAMFER_DISTANCE_TRANSFORM_HEADER
