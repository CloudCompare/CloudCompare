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

//CCLib
#include <GenericSensor.h>

#include "ccHObject.h"

//! a GenericSensor class "encapsulated" in a ccHObject class
class ccSensor : public CCLib::GenericSensor, public ccHObject
{
public:

	//! Default constructor
	ccSensor() : CCLib::GenericSensor(), ccHObject("Sensor") {};

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_SENSOR;};

    //inherited from GenericSensor
    virtual CC_VISIBILITY_TYPE checkVisibility(const CCVector3& aPoint) {return VIEWED;};
};

#endif //CC_SENSOR_HEADER
