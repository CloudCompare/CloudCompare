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
#include "ccPointPair.h"
#include "ccFitPlane.h"

class ccTopologyRelation;

/*
ccGeoObjects are a data-organisation structure comprised of four ccHObjects, a parent (representing the "GeoObject") and 3 children,
representing the "upper boundary", "lower boundary" and "interior" of the GeoObject.
*/

class ccGeoObject : public ccHObject
{
public:
	ccGeoObject(QString name, ccMainAppInterface* app, bool singleSurface);
	ccGeoObject(ccHObject* obj, ccMainAppInterface* app);

	//returns the pointCloud associated with this ccGeoObject's interior (or null if the interior is undefined)
	ccPointCloud* getAssociatedCloud();
	
	//returns the ccHObject parent of the specified mapping region (see below for mapping region consts)
	ccHObject* getRegion(int mappingRegion);

	//draws all children objects in "highlighted" mode
	void setActive(bool active);

	//gets the topological relationship between this GeoObject and another
	int getRelationTo(ccGeoObject* obj, ccTopologyRelation** out = nullptr);

	//adds a topological relationship between this GeoObject and another
	ccTopologyRelation* addRelationTo(ccGeoObject* obj2, int type, ccMainAppInterface* app);

	//returns the GID of this geo-object
	unsigned int getGID() { return _gID; }

	//flags defining different mapping regions
	static const int INTERIOR = 0;
	static const int UPPER_BOUNDARY = 1;
	static const int LOWER_BOUNDARY = 2;

protected:
	//link back to the main plugin interface
	ccMainAppInterface* m_app;

	//the dataset this object "belongs" too. Assigned when an "interior" gets defined.
	ccPointCloud* m_associatedCloud = nullptr;

	//group containing interior point set and associated measurments
	ccHObject* m_interior = nullptr; 
	int m_interior_id = -1;

	//group containing upper boundary polylines and associated measurments
	ccHObject* m_upper = nullptr; 
	int m_upper_id = -1;

	//group containing lower boundary polylines/traces and associated measurments
	ccHObject* m_lower = nullptr; 
	int m_lower_id = -1;

	//protected functions for generating the above objects
	void generateInterior();
	void generateUpper();
	void generateLower();

private:
	//builds this GeoObject and its ccHObject components
	void init(bool singleSurface);

	//searches and activates/disactivates children belonging to the ccHObject. This is used when the DBTree selection changes.
	void recurseChildren(ccHObject* par, bool highlight);

	//return the topology relationship between this ccHObject and another (WIP).
	ccTopologyRelation* getRelation(ccHObject* obj, int id1, int id2);

	void assignGID();

	unsigned int _gID; //unique and persistent id for this geoObject

public://static functions

   //returns true if object is a ccGeoObject
	static bool isGeoObject(ccHObject* object);

	//returns true if object is the upper component of a ccGeoObject
	static bool isGeoObjectUpper(ccHObject* object);

	//returns true if object is the lower component of a ccGeoObject
	static bool isGeoObjectLower(ccHObject* object);

	//returns ture if object is the interior component of a ccGeoObject
	static bool isGeoObjectInterior(ccHObject* object);

	//returns true if object is a single-surface GeoObject rather than a generic GeoObject
	static bool isSingleSurfaceGeoObject(ccHObject* object);

	//traverses up the DbTree, starting at object, until a ccGeoObject is found. If none is found this will return null.
	static ccGeoObject* getGeoObjectParent(ccHObject* object);

	//traverses up the DbTree, starting at object, until a ccGeoObject representing the interior, upper or lower surface. 
	//returns -1 if no ccGeoObject is found, otherwise ccGeoObject::INTERIOR, ccGeoObject::UPPER_BOUNDARY or ccGeoObject::LOWER_BOUNDARY
	static int getGeoObjectRegion(ccHObject* object);
};

#endif
