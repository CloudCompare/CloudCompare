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

#ifdef CC_SHP_SUPPORT

#include "ShpDBFFields.h"

//system
#include <assert.h>

bool IntegerDBFField::save(DBFHandle handle, int fieldIndex) const
{
	if (!handle || fieldIndex < 0)
	{
		assert(false);
		return false;
	}

	for (size_t i = 0; i < values.size(); ++i)
		DBFWriteIntegerAttribute(handle, static_cast<int>(i), fieldIndex, values[i]);

	return true;
}

bool DoubleDBFField::save(DBFHandle handle, int fieldIndex) const
{
	if (!handle || fieldIndex < 0)
	{
		assert(false);
		return false;
	}

	for (size_t i = 0; i < values.size(); ++i)
		DBFWriteDoubleAttribute(handle, static_cast<int>(i), fieldIndex, values[i]);

	return true;
}

bool DoubleDBFField3D::save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const
{
	if (!handle || xFieldIndex < 0 || yFieldIndex < 0 || zFieldIndex < 0)
	{
		assert(false);
		return false;
	}

	for (size_t i = 0; i < values.size(); ++i)
	{
		DBFWriteDoubleAttribute(handle, static_cast<int>(i), xFieldIndex, values[i].x);
		DBFWriteDoubleAttribute(handle, static_cast<int>(i), yFieldIndex, values[i].y);
		DBFWriteDoubleAttribute(handle, static_cast<int>(i), zFieldIndex, values[i].z);
	}

	return true;
}

#endif //CC_SHP_SUPPORT
