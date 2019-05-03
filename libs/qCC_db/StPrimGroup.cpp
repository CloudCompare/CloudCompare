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
	for (unsigned int i = 0; i < getChildrenNumber(); i++) {
		ccHObject* child_cloud = getChild(i);

		if (!child_cloud->isEnabled() || !child_cloud->isA(CC_TYPES::POINT_CLOUD) || child_cloud->getName().endsWith("-del")) continue;

		for (unsigned int j = 0; j < child_cloud->getChildrenNumber(); j++) {
			ccHObject* pl = child_cloud->getChild(j);
			if (pl->isEnabled() && pl->isA(CC_TYPES::PLANE)) {
				valid.push_back(pl);
				break;
			}
		}
	}
	return valid;
}

ccPlane * StPrimGroup::getPlaneByName(QString name)
{
	for (unsigned int i = 0; i < getChildrenNumber(); i++) {
		ccHObject* child_cloud = getChild(i);
		if (!(child_cloud->getName() == name) || !child_cloud->isA(CC_TYPES::POINT_CLOUD) || child_cloud->getName().endsWith("-del")) continue;

		for (unsigned int j = 0; j < child_cloud->getChildrenNumber(); j++) {
			ccHObject* pl = child_cloud->getChild(j);
			if (pl->isA(CC_TYPES::PLANE)) {
				return static_cast<ccPlane*>(pl);
			}
		}
	}
	return nullptr;
}

void StPrimGroup::filterByName(QStringList name_list)
{
	for (unsigned int i = 0; i < getChildrenNumber(); i++) {
		ccHObject* child_cloud = getChild(i); if (!child_cloud) return;
		child_cloud->setEnabled(name_list.indexOf(child_cloud->getName()) >= 0);
	}
}
