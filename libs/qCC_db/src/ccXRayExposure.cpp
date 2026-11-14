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
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "..\include\ccXRayExposure.h"

double ccXRayExposure::ProjectedAreaRatio(const CCVector3d&  minimum,
                                          const CCVector3d&  maximum,
                                          const ccGLMatrixd& modelView,
                                          const ccGLMatrixd& projection)
{
	double minX = 0;
	double minY = 0;
	double maxX = 0;
	double maxY = 0;
	for (unsigned corner = 0; corner < 8; ++corner)
	{
		double position[4]{0, 0, 0, 1};
		for (unsigned axis = 0; axis < 3; ++axis)
		{
			position[axis] = (corner & (1 << axis)) ? maximum[axis] : minimum[axis];
		}

		double eye[4]{0, 0, 0, 0};
		for (unsigned row = 0; row < 4; ++row)
		{
			for (unsigned col = 0; col < 4; ++col)
			{
				eye[row] += modelView.data()[4 * col + row] * position[col];
			}
		}

		double clip[4]{0, 0, 0, 0};
		for (unsigned row = 0; row < 4; ++row)
		{
			for (unsigned col = 0; col < 4; ++col)
			{
				clip[row] += projection.data()[4 * col + row] * eye[col];
			}
		}

		// A box crossing the eye plane has an unbounded perspective footprint.
		// Use bounded close-up exposure instead of dividing by zero or flipping it.
		if (!std::isfinite(clip[3]) || CCCoreLib::LessThanEpsilon(clip[3]))
		{
			return 64.0;
		}
		double x = clip[0] / clip[3];
		double y = clip[1] / clip[3];

		if (corner != 0)
		{
			minX = std::min(minX, x);
			maxX = std::max(maxX, x);
			minY = std::min(minY, y);
			maxY = std::max(maxY, y);
		}
		else
		{
			minX = maxX = x;
			minY = maxY = y;
		}
	}
	// The viewport occupies [-1,1] x [-1,1] in normalized device coordinates.
	return std::clamp((maxX - minX) * (maxY - minY) / 4.0, 1.0e-8, 64.0);
}

float ccXRayExposure::Strength(double areaRatio, double exposure)
{
	// Point size controls coverage independently; enlarging points must not
	// dim them. Only the camera footprint and exposure change their strength.
	return static_cast<float>(0.02 * std::exp2(exposure) * areaRatio);
}
