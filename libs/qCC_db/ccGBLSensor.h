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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1735                                                              $
//$LastChangedDate:: 2010-12-02 17:11:39 +0100 (jeu., 02 déc. 2010)       $
//**************************************************************************
//

#ifndef CC_GROUND_LIDAR_SENSOR_HEADER
#define CC_GROUND_LIDAR_SENSOR_HEADER

//CCLib
#include <GroundBasedLidarSensor.h>

#include "ccSensor.h"

//! a GroundLidarSensor "encapsulated" in a ccHObject class
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGBLSensor : public CCLib::GroundBasedLidarSensor, public ccSensor
#else
class ccGBLSensor : public CCLib::GroundBasedLidarSensor, public ccSensor
#endif
{
public:

	//! Default constructor
	ccGBLSensor(CCLib::CC_SENSOR_ROTATION_ORDER r = CCLib::GBL_THETA_PHI);

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_GBL_SENSOR;};

	//inherited methods (GroundBasedLidarSensor)
	using CCLib::GroundBasedLidarSensor::checkVisibility;

	//! Updates graphic representation to reflect current sensor parameters
	void updateGraphicRepresentation();

    //! Sets the sensor graphic representation scale
	void setGraphicScale(double scale);

    //! Returns the sensor graphic representation scale
	double getGraphicScale();

    //Inherited from ccHObject
    //virtual ccBBox getMyOwnBB();
    virtual ccBBox getDisplayBB();

protected:

    //Inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

    //! Sensor graphic representation scale
    double scale;

    //! Bounding-box (body)
    ccBBox bBox;
};

#endif
