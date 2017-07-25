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

#include "ccThickness.h"

//pass ctors straight to PointPair
ccThickness::ccThickness(ccPointCloud* associatedCloud)
	: ccPointPair(associatedCloud)
{ 
	updateMetadata();
}

ccThickness::ccThickness(ccPolyline* obj)
	: ccPointPair(obj)
{ 
	updateMetadata();
}

void ccThickness::updateMetadata()
{
	QVariantMap* map = new QVariantMap();

	//add metadata tag defining the ccCompass class type
	map->insert("ccCompassType", "Thickness");

	//add metadata tag defining length
	float length = getDirection().norm();
	map->insert("length", length);

	//update name
	setName(QString::asprintf("%.3fT", length));

	//store
	setMetaData(*map, true);
}

//returns true if object is a lineation
bool ccThickness::isThickness(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("Thickness");
	}
	return false;
}
