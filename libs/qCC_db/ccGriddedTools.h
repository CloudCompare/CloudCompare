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

#ifndef CC_GRIDDED_CLOUD_TOOLS_HEADER
#define CC_GRIDDED_CLOUD_TOOLS_HEADER

//Local
#include "qCC_db.h"
#include "ccPointCloud.h"

//system
#include <vector>

class ccGBLSensor;
class ccGLMatrix;

//! Tools dedicated to gridded clouds
class QCC_DB_LIB_API ccGriddedTools
{
public:

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
