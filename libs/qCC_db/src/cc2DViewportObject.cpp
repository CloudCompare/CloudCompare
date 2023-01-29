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

//Local
#include "cc2DViewportObject.h"

cc2DViewportObject::cc2DViewportObject(QString name/*=QString()*/)
	: ccHObject(name)
{}

cc2DViewportObject::cc2DViewportObject(const cc2DViewportObject& viewport)
	: ccHObject(viewport)
	, m_params(viewport.m_params)
{}

bool cc2DViewportObject::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 20)
	{
		assert(false);
		return false;
	}

	if (!ccHObject::toFile_MeOnly(out, dataVersion))
		return false;

	//ccViewportParameters (dataVersion>=20)
	if (!m_params.toFile(out, dataVersion))
		return false;

	return true;
}

bool cc2DViewportObject::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//ccViewportParameters (dataVersion>=20)
	if (!m_params.fromFile(in, dataVersion, flags, oldToNewIDMap))
		return false;

	return true;
}

short cc2DViewportObject::minimumFileVersion_MeOnly() const
{
	short minVersion = std::max(static_cast<short>(20), ccHObject::minimumFileVersion_MeOnly());
	return std::max(minVersion, m_params.minimumFileVersion());
}
