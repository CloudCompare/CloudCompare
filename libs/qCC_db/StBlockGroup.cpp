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
#include "StBlockGroup.h"

StBlockGroup::StBlockGroup(const QString& name)
	: ccHObject(name)
{
}

StBlockGroup::StBlockGroup(StBlockGroup& obj)
	: ccHObject(obj)
{
	
}

StBlockGroup::StBlockGroup(ccHObject& obj)
	: ccHObject(obj)
{
}

StBlockGroup::~StBlockGroup()
{
}

ccHObject::Container StBlockGroup::getFootPrints()
{
	ccHObject::Container fts;
	filterChildren(fts, false, CC_TYPES::ST_FOOTPRINT, true);
	return fts;
}

void StBlockGroup::groupFootPrint()
{
	ccHObject::Container footprintObjs = getFootPrints();

}

std::vector<std::vector<int>> StBlockGroup::getFootPrintGroup()
{
	return std::vector<std::vector<int>>();
}
