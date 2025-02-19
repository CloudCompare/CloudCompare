// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccVBOManager.h"
#include "ccPointCloud.h"

int ccVBO::init(int count, bool withColors, bool withNormals, bool* reallocated /*=nullptr*/)
{
	// required memory
	int totalSizeBytes = sizeof(PointCoordinateType) * count * 3;
	if (withColors)
	{
		rgbShift = totalSizeBytes;
		totalSizeBytes += sizeof(ColorCompType) * count * 4;
	}
	if (withNormals)
	{
		normalShift = totalSizeBytes;
		totalSizeBytes += sizeof(PointCoordinateType) * count * 3;
	}

	if (!isCreated())
	{
		if (!create())
		{
			// no message as it will probably happen on a lot on (old) graphic cards
			return -1;
		}

		setUsagePattern(QOpenGLBuffer::DynamicDraw); //"StaticDraw: The data will be set once and used many times for drawing operations."
		                                             //"DynamicDraw: The data will be modified repeatedly and used many times for drawing operations.
	}

	if (!bind())
	{
		ccLog::Warning("[ccPointCloud::VBO::init] Failed to bind VBO to active context!");
		destroy();
		return -1;
	}

	if (totalSizeBytes != size())
	{
		allocate(totalSizeBytes);
		if (reallocated)
			*reallocated = true;

		if (size() != totalSizeBytes)
		{
			ccLog::Warning("[ccPointCloud::VBO::init] Not enough (GPU) memory!");
			release();
			destroy();
			return -1;
		}
	}
	else
	{
		// nothing to do
	}

	release();

	return totalSizeBytes;
}
