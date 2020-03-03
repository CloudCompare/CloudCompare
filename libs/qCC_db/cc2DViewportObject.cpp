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

bool cc2DViewportObject::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//ccViewportParameters (dataVersion>=20)
	if (!m_params.toFile(out))
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
