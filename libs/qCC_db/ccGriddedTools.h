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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GRIDDED_CLOUD_TOOLS_HEADER
#define CC_GRIDDED_CLOUD_TOOLS_HEADER

//Local
#include "ccPointCloud.h"


class ccGBLSensor;
class ccGLMatrix;

//! Tools dedicated to gridded clouds
class QCC_DB_LIB_API ccGriddedTools
{
public:

	//! Grid (angular) parameters
	struct GridParameters
	{
		GridParameters()
			: minPhi(0)
			, maxPhi(0)
			, minTheta(0)
			, maxTheta(0)
			, deltaPhiRad(0)
			, deltaThetaRad(0)
			, maxRange(0)
		{}

		PointCoordinateType minPhi, maxPhi;
		PointCoordinateType minTheta, maxTheta;
		PointCoordinateType deltaPhiRad, deltaThetaRad;
		PointCoordinateType maxRange;
	};

	//! Detects the given grid parameters (angular span, etc.)
	/** \param cloud associated cloud
		\param grid scan grid
		\param parameters output parameters
		\param verbose whether the process should output some detailed information in the log/console or not
		\param cloudToSensorTrans transformation from cloud coordinate system to the sensor coordinate system (optional)
		\return success
	**/
	static bool DetectParameters(	const ccPointCloud* cloud,
									const ccPointCloud::Grid::Shared grid,
									GridParameters& parameters,
									bool verbose = false,
									ccGLMatrix* cloudToSensorTrans = 0);

	//! Determines the (TLS) sensor parameters from the relative position of gridded points
	/** \param cloud cloud on which to compute the sensor parameters (should be a single grid)
		\param grid scan grid
		\param cloudToSensorTrans transformation from cloud coordinate system to the sensor coordinate system (optional)
		\return sensor (if successful) or 0 otherwise
	**/
	static ccGBLSensor* ComputeBestSensor(	ccPointCloud* cloud,
											ccPointCloud::Grid::Shared grid,
											ccGLMatrix* cloudToSensorTrans = 0);
};

#endif
