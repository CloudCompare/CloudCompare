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

#ifndef CC_LINEATION_HEADER
#define CC_LINEATION_HEADER

#include <ccPolyline.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <GenericIndexedCloudPersist.h>
#include <ccPointCloud.h>


#include "ccMeasurement.h"

class ccLineation : 
	public ccPolyline, 
	public ccMeasurement
{
public:
	ccLineation(ccPointCloud* associatedCloud);
	ccLineation(ccPolyline* obj); //used to construct from a polyline with the correct data

	virtual ~ccLineation() {}

	void updateMetadata();

	//inherited from ccHObject
	//inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POLY_LINE; }

	//get the direction of this lineation (not normalized) 
	CCVector3 getDirection()
	{
		if (size() != 2)
		{
			return CCVector3();
		} else
		{
			const CCVector3 start = *getPoint(0);
			const CCVector3 end = *getPoint(1);
			return end - start;
		}
	}

protected:
	float m_relMarkerScale = 5.0f;

	//overidden from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	
private:
	void init(); //used by ctors

//static functions
public:
	static bool ccLineation::isLineation(ccHObject* object);

};

#endif
