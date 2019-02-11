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

#include <GeometricalAnalysisTools.h>

class QWidget;

class ccGenericPointCloud;
class ccProgressDialog;

namespace ccLibAlgorithms
{
	//! Returns a default first guess for algorithms kernel size (one cloud)
	PointCoordinateType GetDefaultCloudKernelSize(ccGenericPointCloud* cloud, unsigned knn = 12);
	
	//! Returns a default first guess for algorithms kernel size (several clouds)
	PointCoordinateType GetDefaultCloudKernelSize(const ccHObject::Container& entities, unsigned knn = 12);
	
	/*** CCLib "standalone" algorithms ***/

	//! Geometric characteristic (with sub option)
	struct GeomCharacteristic
	{
		GeomCharacteristic(CCLib::GeometricalAnalysisTools::GeomCharacteristic c, int option = 0)
			: charac(c)
			, subOption(option)
		{}
		
		CCLib::GeometricalAnalysisTools::GeomCharacteristic charac;
		int subOption = 0;
	};

	//! Set of GeomCharacteristic instances
	typedef std::vector<GeomCharacteristic> GeomCharacteristicSet;

	//! Computes geometrical characteristics (see GeometricalAnalysisTools::GeomCharacteristic) on a set of entities
	bool ComputeGeomCharacteristics(const GeomCharacteristicSet& characteristics,
									PointCoordinateType radius,
									ccHObject::Container& entities,
									QWidget* parent = nullptr);
	
	//! Computes a geometrical characteristic (see GeometricalAnalysisTools::GeomCharacteristic) on a set of entities
	bool ComputeGeomCharacteristic(	CCLib::GeometricalAnalysisTools::GeomCharacteristic algo,
									int subOption,
									PointCoordinateType radius,
									ccHObject::Container& entities,
									QWidget* parent = nullptr,
									ccProgressDialog* progressDialog = nullptr);

	//CCLib algorithms handled by the 'ApplyCCLibAlgorithm' method
	enum CC_LIB_ALGORITHM { CCLIB_ALGO_SF_GRADIENT,
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
