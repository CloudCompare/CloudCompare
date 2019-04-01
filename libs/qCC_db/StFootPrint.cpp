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
#include "StFootPrint.h"

StFootPrint::StFootPrint(GenericIndexedCloudPersist* associatedCloud)
	: ccPolyline(associatedCloud)
	, m_ground(0)
{
}

StFootPrint::StFootPrint(StFootPrint& obj)
	: ccPolyline(obj)
	, m_ground(obj.m_ground)
{
	
}

StFootPrint::StFootPrint(ccPolyline& obj)
	: ccPolyline(obj)
	, m_ground(0)
{
}

StFootPrint::~StFootPrint()
{
}

void StFootPrint::reverseVertexOrder()
{
	getAssociatedCloud();
	size_t last = size();
	size_t first = 0;
	while ((first != last) && first != --last) {
		///< swap first and last
		size_t fist_index = getPointGlobalIndex(first);
		setPointIndex(first, getPointGlobalIndex(last));
		setPointIndex(last, fist_index);
		++first;
	}
	setPointIndex(0, getPointGlobalIndex(0));
}

inline double StFootPrint::getHeight() const
{
	return getPoint(0)->z;
}

void StFootPrint::setHeight(double height)
{
	for (size_t i = 0; i < size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*getPoint(i));
		P.z = height;
	}
	invalidateBoundingBox();
}
