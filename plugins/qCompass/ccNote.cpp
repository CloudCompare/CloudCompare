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

#include "ccNote.h"

//pass ctors straight to PointPair
ccNote::ccNote(ccPointCloud* associatedCloud)
	: ccPointPair(associatedCloud)
{
	updateMetadata();
}

ccNote::ccNote(ccPolyline* obj)
	: ccPointPair(obj)
{ 
	updateMetadata();
}

void ccNote::updateMetadata()
{
	//add metadata tag defining the ccCompass class type
	QVariantMap* map = new QVariantMap();
	map->insert("ccCompassType", "Note");
	setMetaData(*map, true);

	//update drawing stuff
	showNameIn3D(true);
	setDefaultColor(ccColor::cyan);
	setActiveColor(ccColor::red);
}

//returns true if object is a lineation
bool ccNote::isNote(ccHObject* object)
{
	if (object->hasMetaData("ccCompassType"))
	{
		return object->getMetaData("ccCompassType").toString().contains("Note");
	}
	return false;
}
