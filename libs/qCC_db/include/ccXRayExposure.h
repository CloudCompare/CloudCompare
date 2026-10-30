#pragma once

// CloudCompare - GPL-2.0-or-later

#include <algorithm>
#include <cmath>
#include <limits>

//! Exposure compensation from the projected bounding box, independent of Qt/OpenGL.
namespace ccXRay
{
	//! Matrices are column-major, as in OpenGL. Extents are deliberately not clipped
	//! to the viewport: panning must not change exposure, even when the box is offscreen.
	inline double ProjectedAreaRatio(const double* minimum,
	                                 const double* maximum,
	                                 const double* modelView,
	                                 const double* projection)
	{
		double minX = std::numeric_limits<double>::infinity();
		double minY = minX;
		double maxX = -minX;
		double maxY = -minX;
		for (unsigned corner = 0; corner < 8; ++corner)
		{
			double position[4] = {0, 0, 0, 1};
			for (unsigned axis = 0; axis < 3; ++axis)
			{
				position[axis] = (corner & (1 << axis)) ? maximum[axis] : minimum[axis];
			}
			double eye[4]  = {};
			double clip[4] = {};
			for (unsigned row = 0; row < 4; ++row)
			{
				for (unsigned col = 0; col < 4; ++col)
				{
					eye[row] += modelView[4 * col + row] * position[col];
				}
			}
			for (unsigned row = 0; row < 4; ++row)
			{
				for (unsigned col = 0; col < 4; ++col)
				{
					clip[row] += projection[4 * col + row] * eye[col];
				}
			}
			// A box crossing the eye plane has an unbounded perspective footprint.
			// Use bounded close-up exposure instead of dividing by zero or flipping it.
			if (!std::isfinite(clip[3]) || clip[3] <= 1.0e-12)
			{
				return 64.0;
			}
			double x = clip[0] / clip[3];
			double y = clip[1] / clip[3];
			if (!std::isfinite(x) || !std::isfinite(y))
			{
				return 1.0;
			}
			minX = std::min(minX, x);
			maxX = std::max(maxX, x);
			minY = std::min(minY, y);
			maxY = std::max(maxY, y);
		}
		// The viewport occupies [-1,1] x [-1,1] in normalized device coordinates.
		return std::max(1.0e-8, std::min(64.0, (maxX - minX) * (maxY - minY) / 4.0));
	}

	inline float Strength(double areaRatio, double exposureStops)
	{
		// Point size controls coverage independently; enlarging points must not
		// dim them. Only the camera footprint and exposure change their strength.
		return static_cast<float>(0.02 * std::exp2(exposureStops) * areaRatio);
	}
} // namespace ccXRay
