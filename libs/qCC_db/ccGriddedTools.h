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

//system
#include <vector>

class ccPointCloud;
class ccGBLSensor;
class ccGLMatrix;

//! Tools dedicated to gridded clouds
class QCC_DB_LIB_API ccGriddedTools
{
public:

	//! Computes normals on a gridded cloud
	/** \param cloud cloud on which to compute the normals (should be a single grid)
		\param indexes for each grid cell, this table gives the index of the corresponding point (or -1)
		\param width grid width
		\param height grid height
		\param kernelWidth neighborhood size (number of cells)
		\param[out] canceledByUser whether the process has been canceled by the user or not (optional)
		\return success
	**/
	static bool ComputeNormals(	ccPointCloud* cloud,
								const std::vector<int>& indexGrid,
								int width,
								int height,
								bool* canceledByUser = 0,
								int kernelWidth = 3 );

	//! Determines the (TLS) sensor parameters from the relative position of gridded points
	/** \param cloud cloud on which to compute the sensor parameters (should be a single grid)
		\param indexes for each grid cell, this table gives the index of the corresponding point (or -1)
		\param width grid width
		\param height grid height
		\param cloudToSensorTrans transformation from cloud coordinate system to the sensor coordinate system (optional)
		\return sensor (if successful) or 0 otherwise
	**/
	static ccGBLSensor* ComputeBestSensor(ccPointCloud* cloud, const std::vector<int>& indexGrid, unsigned width, unsigned height, ccGLMatrix* cloudToSensorTrans = 0);

	//! Behavior regarding whether normals should be automatically computed or not at loading time
	/** Only for formats supporting this (i.e. gridded clouds: PTX, etc.)
	**/
	enum ComputeNormalsBehavior { ALWAYS, ASK_USER, NEVER};

	//! Returns whether normals should be computed depending on the specified behavior
	/** \warning May change the input behavior
		\return whether normals should be computed or not
	**/
	static bool HandleAutoComputeNormalsFeature(ComputeNormalsBehavior& behavior);

};

#endif
