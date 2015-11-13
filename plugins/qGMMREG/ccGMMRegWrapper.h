//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qGMMReg                     #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#ifndef CC_GMMREG_WRAPPER_HEADER
#define CC_GMMREG_WRAPPER_HEADER

//CCLib
#include <CCGeom.h>
#include <GenericCloud.h>

//System
#include <vector>
#include <stdlib.h>

class ccMainAppInterface;
class QWidget;

//! CloudCompare wrapper for the GMMReg library
class ccGMMRegWrapper
{
public:
	struct StepValues
	{
		StepValues()
			: enabled(false)
			, scale(0)
			, lambda(0)
			, maxIter(0)
		{}

		bool enabled;
		double scale;
		double lambda;
		int maxIter;
	};

	//! Performs the non-rigid registration between d (deformed) and m (model)
	static bool PerformRegistration(CCLib::GenericCloud* d,
									CCLib::GenericCloud* m,
									const std::vector<StepValues>& steps,
									std::vector<CCVector3>& displacedPoints,
									bool useTPS = true,
									size_t controlPointsCount = 0,
									ccMainAppInterface* app = 0,
									QWidget* parentWidget = 0);
};

#endif
