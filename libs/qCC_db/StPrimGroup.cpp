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

//Always first
#include "StPrimGroup.h"

StPrimGroup::StPrimGroup(const QString& name)
	: ccHObject(name)
{
}

StPrimGroup::StPrimGroup(StPrimGroup& obj)
	: ccHObject(obj)
{
	
}

StPrimGroup::StPrimGroup(ccHObject& obj)
	: ccHObject(obj)
{
}

StPrimGroup::~StPrimGroup()
{
}

ccHObject::Container StPrimGroup::getValidPlanes()
{
	ccHObject::Container valid;
	for (size_t i = 0; i < getChildrenNumber(); i++) {
		ccHObject* child_cloud = getChild(i);

		if (!child_cloud->isEnabled() || !child_cloud->isA(CC_TYPES::POINT_CLOUD)) continue;

		for (size_t j = 0; j < child_cloud->getChildrenNumber(); j++) {
			ccHObject* pl = child_cloud->getChild(j);
			if (pl->isEnabled() && !pl->getName().endsWith("-del")) {
				valid.push_back(pl);
				break;
			}
		}
	}
	return valid;
}
