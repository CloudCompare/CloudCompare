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

#ifndef CC_SENSOR_HEADER
#define CC_SENSOR_HEADER

#include "ccHObject.h"

//! Sensor types
enum CC_SENSOR_TYPE {UNKNOWN_SENSOR,
						GROUND_BASED_LIDAR,
						AIRBORNE_LIDAR,
						PHOTOGRAMMETRY_SENSOR,
						AIRBORNE_RADAR};

//! A generic sensor interface
class ccSensor : public ccHObject
{
public:

	//! Default constructor
	ccSensor() : ccHObject("Sensor") {};

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const { return CC_SENSOR; };

	//! Returns the sensor type
	/** Should be re-implemented by any sub-class.
        \return the sensor type
	**/
	virtual CC_SENSOR_TYPE getType() const { return UNKNOWN_SENSOR; }

	//! Returns the "visibility type" of a point
	/** Precise definition of point's visibility can be found in Daniel Girardeau-Montaut's
		PhD manuscript (Chapter 2, section 2-3-3). In fact it can be anything, assuming that
		a point that is not POINT_VISIBLE won't be compared during a point-to-cloud distance
		computation process.
		\param P a 3D point
		\return the visibility of the point
	**/
	virtual uchar checkVisibility(const CCVector3& P) const = 0;

};

#endif //CC_SENSOR_HEADER
