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

#include "ccShiftedObject.h"

//local
#include "ccLog.h"
#include "ccSerializableObject.h"

ccShiftedObject::ccShiftedObject(QString name, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccHObject(name, uniqueID)
	, m_globalShift(0, 0, 0)
	, m_globalScale(1.0)
{
}

void ccShiftedObject::setGlobalShift(const CCVector3d& shift)
{
	m_globalShift = shift;
}

void ccShiftedObject::setGlobalShift(double x, double y, double z)
{
	m_globalShift.x = x;
	m_globalShift.y = y;
	m_globalShift.z = z;
}

void ccShiftedObject::setGlobalScale(double scale)
{
	if (scale == 0)
	{
		ccLog::Warning("[setGlobalScale] Invalid scale (zero)!");
		m_globalScale = 1.0;
	}
	else
	{
		m_globalScale = scale;
	}
}

bool ccShiftedObject::saveShiftInfoToFile(QFile& out) const
{
	//'coordinates shift'
	if (out.write((const char*)m_globalShift.u, sizeof(double) * 3) < 0)
		return ccSerializableObject::WriteError();
	//'global scale'
	if (out.write((const char*)&m_globalScale, sizeof(double)) < 0)
		return ccSerializableObject::WriteError();

	return true;
}

bool ccShiftedObject::loadShiftInfoFromFile(QFile& in)
{
	//'coordinates shift'
	if (in.read((char*)m_globalShift.u, sizeof(double) * 3) < 0)
		return ccSerializableObject::ReadError();
	//'global scale'
	if (in.read((char*)&m_globalScale, sizeof(double)) < 0)
		return ccSerializableObject::ReadError();

	return true;
}

bool ccShiftedObject::getGlobalBB(CCVector3d& minCorner, CCVector3d& maxCorner)
{
	ccBBox box = getOwnBB(false);
	minCorner = toGlobal3d(box.minCorner());
	maxCorner = toGlobal3d(box.maxCorner());
	return box.isValid();
}
