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
#include <iostream>

StFootPrint::StFootPrint(GenericIndexedCloudPersist* associatedCloud)
	: ccPolyline(associatedCloud)
	, m_ground(0)
	, m_hole(false)
{
}

StFootPrint::StFootPrint(StFootPrint& obj)
	: ccPolyline(obj)
	, m_ground(obj.m_ground)
	, m_hole(false)
{
	
}

StFootPrint::StFootPrint(ccPolyline& obj)
	: ccPolyline(obj)
	, m_ground(0)
	, m_hole(false)
{
}

StFootPrint::~StFootPrint()
{
}

bool StFootPrint::reverseVertexOrder()
{
	try {
		getAssociatedCloud();
		unsigned int last = size();
		unsigned int first = 0;
		while ((first != last) && first != --last) {
			///< swap first and last
			unsigned int fist_index = getPointGlobalIndex(first);
			setPointIndex(first, getPointGlobalIndex(last));
			setPointIndex(last, fist_index);
			++first;
		}
	}
	catch (std::runtime_error& e) {
		std::cout << "unknown error happens: " << e.what() << std::endl;
		return false;
	}
	return true;
}

inline double StFootPrint::getHeight() const
{
	return getPoint(0)->z;
}

void StFootPrint::setHeight(double height)
{
	for (unsigned int i = 0; i < size(); i++) {
		CCVector3& P = const_cast<CCVector3&>(*getPoint(i));
		P.z = height;
	}
	invalidateBoundingBox();
}

bool StFootPrint::toFile_MeOnly(QFile & out) const
{
	if (!ccPolyline::toFile_MeOnly(out)) {
		return false;
	}
	
	if (out.write((const char*)&m_ground, sizeof(double)) < 0)
		return WriteError();

	if (out.write((const char*)&m_hole, sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool StFootPrint::fromFile_MeOnly(QFile & in, short dataVersion, int flags)
{
	if (!ccPolyline::fromFile_MeOnly(in, dataVersion, flags)) {
		return false;
	}


	if (in.read((char*)&m_ground, sizeof(double)) < 0)
		return ReadError();

	if (in.read((char*)&m_hole, sizeof(bool)) < 0)
		return ReadError();

	return true;
}
