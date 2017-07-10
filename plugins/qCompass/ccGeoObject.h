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


#ifndef CC_GEOOBJECT_HEADER
#define CC_GEOOBJECT_HEADER

#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMainAppInterface.h>

#include "ccTrace.h"

class ccGeoObject : public ccHObject
{
public:
	ccGeoObject(QString name, ccMainAppInterface* app);
	ccGeoObject(ccHObject* obj, ccMainAppInterface* app);

	//returns the pointCloud associated with this ccGeoObject's interior (or null if the interior is undefined)
	ccPointCloud* getAssociatedCloud();
	
	//returns the ccHObject parent of the specified mapping region (see below for mapping region consts)
	ccHObject* getRegion(int mappingRegion);

	//draws all children objects in "highlighted" mode
	void setActive(bool active);
	void recurseChildren(ccHObject* par, bool highlight);

	//flags defining different mapping regions
	static const int INTERIOR = 0;
	static const int UPPER_BOUNDARY = 1;
	static const int LOWER_BOUNDARY = 2;

protected:
	ccMainAppInterface* m_app;
	ccPointCloud* m_associatedCloud = nullptr; //the dataset this object "belongs" too. Assigned when an "interior" gets defined.
	ccHObject* m_interior; //group containing interior point set and associated measurments
	int m_interior_id = -1;

	ccHObject* m_upper; //group containing upper boundary polylines and associated measurments
	int m_upper_id = -1;

	ccHObject* m_lower; //group containing lower boundary polylines/traces and associated measurments
	int m_lower_id = -1;

	void generateInterior();
	void generateUpper();
	void generateLower();

private:
	void init(QString name, ccMainAppInterface* app);

//static functions
public:
	static bool isGeoObject(ccHObject* object);
	static bool isGeoObjectUpper(ccHObject* object);
	static bool isGeoObjectLower(ccHObject* object);
	static bool isGeoObjectInterior(ccHObject* object);
};


#endif
