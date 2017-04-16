#ifndef CCLIBALGORITHMS_H
#define CCLIBALGORITHMS_H
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccHObject.h"

class QWidget;

class ccGenericPointCloud;


namespace ccLibAlgorithms
{
	//! Returns a default first guess for algorithms kernel size (one cloud)
	PointCoordinateType GetDefaultCloudKernelSize(ccGenericPointCloud* cloud, unsigned knn = 12);
	
	//! Returns a default first guess for algorithms kernel size (several clouds)
	PointCoordinateType GetDefaultCloudKernelSize(const ccHObject::Container& entities, unsigned knn = 12);
	
	/*** CCLib "standalone" algorithms ***/
	
	//CCLib algorithms handled by the 'ApplyCCLibAlgorithm' method
	enum CC_LIB_ALGORITHM { CCLIB_ALGO_CURVATURE = 1,
							CCLIB_ALGO_SF_GRADIENT,
							CCLIB_ALGO_ROUGHNESS,
							CCLIB_ALGO_APPROX_DENSITY,
							CCLIB_ALGO_ACCURATE_DENSITY,
							CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST = 255,
						};
	
	//! Applies a standard CCLib algorithm (see CC_LIB_ALGORITHM) on a set of entities
	bool ApplyCCLibAlgorithm(	CC_LIB_ALGORITHM algo,
								ccHObject::Container& entities,
								QWidget* parent = 0,
								void** additionalParameters = 0);
	
	//! Scale matching algorithms
	enum ScaleMatchingAlgorithm { BB_MAX_DIM, BB_VOLUME, PCA_MAX_DIM, ICP_SCALE };
	
	//! Applies a standard CCLib algorithm (see CC_LIB_ALGORITHM) on a set of entities
	bool ApplyScaleMatchingAlgorithm(ScaleMatchingAlgorithm algo,
												ccHObject::Container& entities,
												double icpRmsDiff,
												int icpFinalOverlap,
												unsigned refEntityIndex = 0,
												QWidget* parent = 0);
}

#endif
