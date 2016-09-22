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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef RAY_AND_BOX_HEADER
#define RAY_AND_BOX_HEADER

#include "CCGeom.h"

//! Simple Ray structure
template <typename T > struct Ray
{
	Ray(const Vector3Tpl<T>& rayAxis, const Vector3Tpl<T>& rayOrigin)
		: dir(rayAxis)
		, origin(rayOrigin)
		, invDir(0,0,0)
		, sign(0,0,0)
	{
		dir.normalize();
		invDir = Vector3Tpl<T>(1/rayAxis.x, 1/rayAxis.y, 1/rayAxis.z); // +/-infinity is acceptable here because we are mainly interested in the sign
		sign = Tuple3i(invDir.x < 0, invDir.y < 0, invDir.z < 0);
	}

	inline double radialSquareDistance(const Vector3Tpl<T>& P) const
	{
		Vector3Tpl<T> OP = P - origin;
		return OP.cross(dir).norm2d();
	}

	inline double squareDistanceToOrigin(const Vector3Tpl<T>& P) const
	{
		Vector3Tpl<T> OP = P - origin;
		return OP.norm2d();
	}

	inline void squareDistances(const Vector3Tpl<T>& P, double& radial, double& toOrigin) const
	{
		Vector3Tpl<T> OP = P - origin;
		radial = OP.cross(dir).norm2d();
		toOrigin = OP.norm2d();
	}

	Vector3Tpl<T> dir, origin;
	Vector3Tpl<T> invDir;
	Tuple3i sign;
};

//! Simple axis aligned box structure
template <typename T > struct AABB
{
	AABB(const Vector3Tpl<T>& minCorner, const Vector3Tpl<T>& maxCorner)
	{
		assert(minCorner.x <= maxCorner.x);
		assert(minCorner.y <= maxCorner.y);
		assert(minCorner.z <= maxCorner.z);

		corners[0] = minCorner;
		corners[1] = maxCorner;
	}

	/*
	* Ray-box intersection using IEEE numerical properties to ensure that the
	* test is both robust and efficient, as described in:
	*
	*      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
	*      "An Efficient and Robust Ray-Box Intersection Algorithm"
	*      Journal of graphics tools, 10(1):49-54, 2005
	*
	*/
	bool intersects(const Ray<T> &r, T* t0 = 0, T* t1 = 0) const
	{
		T tmin  = (corners[  r.sign.x].x - r.origin.x) * r.invDir.x;
		T tmax  = (corners[1-r.sign.x].x - r.origin.x) * r.invDir.x;
		T tymin = (corners[  r.sign.y].y - r.origin.y) * r.invDir.y;
		T tymax = (corners[1-r.sign.y].y - r.origin.y) * r.invDir.y;
		
		if (tmin > tymax || tymin > tmax) 
			return false;
		if (tymin > tmin)
			tmin = tymin;
		if (tymax < tmax)
			tmax = tymax;
		
		T tzmin = (corners[  r.sign.z].z - r.origin.z) * r.invDir.z;
		T tzmax = (corners[1-r.sign.z].z - r.origin.z) * r.invDir.z;
		
		if (tmin > tzmax || tzmin > tmax) 
			return false;
		if (tzmin > tmin)
			tmin = tzmin;
		if (tzmax < tmax)
			tmax = tzmax;

		if (t0)
			*t0 = tmin;
		if (t1)
			*t1 = tmax;
		
		return true;
	}

	Vector3Tpl<T> corners[2];
};

#endif //RAY_AND_BOX_HEADER
