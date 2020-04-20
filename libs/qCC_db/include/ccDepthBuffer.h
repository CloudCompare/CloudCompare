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

#ifndef CC_DEPTH_BUFFER_HEADER
#define CC_DEPTH_BUFFER_HEADER

//local
#include "qCC_db.h"
#include <CCGeom.h>

//System
#include <vector>

//! Sensor "depth map"
/** Contains an array of depth values (along each scanned direction) and its dimensions.
	This array corresponds roughly to what have been "seen" by the sensor during
	acquisition (the 3D points are simply projected in the sensor frame).
**/
class QCC_DB_LIB_API ccDepthBuffer
{
public:
	//! Default constructor
	ccDepthBuffer();
	//! Destructor
	virtual ~ccDepthBuffer();

	//! Z-Buffer grid
	std::vector<PointCoordinateType> zBuff;
	//! Pitch step (may differ from the sensor's)
	PointCoordinateType deltaPhi;
	//! Yaw step (may differ from the sensor's)
	PointCoordinateType deltaTheta;
	//! Buffer width
	unsigned width;
	//! Buffer height
	unsigned height;

	//! Clears the buffer
	void clear();

	//! Applies a mean filter to fill small holes (= lack of information) of the depth map.
	/**	The depth buffer must have been created before (see GroundBasedLidarSensor::computeDepthBuffer).
	\return a negative value if an error occurs, 0 otherwise
	**/
	int fillHoles();
};

#endif //CC_DEPTH_BUFFER_HEADER
