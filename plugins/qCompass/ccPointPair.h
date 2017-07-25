//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CC_POINTPAIR_HEADER
#define CC_POINTPAIR_HEADER

#include <ccPolyline.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <GenericIndexedCloudPersist.h>
#include <ccPointCloud.h>


#include "ccMeasurement.h"

class ccPointPair : 
	public ccPolyline, 
	public ccMeasurement
{
public:
	ccPointPair(ccPointCloud* associatedCloud);
	ccPointPair(ccPolyline* obj); //used to construct from a polyline with the correct data

	virtual ~ccPointPair() {}

	virtual void updateMetadata() { }; //overriding by actual data classes (see bottom of file)

	//get the direction of this pair (not normalized) 
	CCVector3 getDirection();

protected:
	float m_relMarkerScale = 5.0f;

	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

//static functions
public:
	static bool isPointPair(ccHObject* object);
};

#endif
