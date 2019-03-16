//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

#ifndef _VEC3_H_
#define _VEC3_H_

//system
#include <cmath>

class Vec3 // a minimal vector class of 3 doubles and overloaded math operators
{
public:

	union
	{
		struct
		{
			double x, y, z;
		};
		double f[3];
	};

	Vec3(double _x, double _y, double _z)
		: x(_x)
		, y(_y)
		, z(_z)
	{}

	Vec3()
		: x(0)
		, y(0)
		, z(0)
	{}

	Vec3(const Vec3& v)
		: x(v.x)
		, y(v.y)
		, z(v.z)
	{}

	inline double length() const
	{
		return sqrt(x * x + y * y + z * z);
	}

	inline Vec3 normalized() const
	{
		double l = length();
		return Vec3(x / l, y / l, z / l);
	}

	inline void operator+= (const Vec3 &v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
	}

	inline Vec3 operator/ (const double &a) const
	{
		return Vec3(x / a, y / a, z / a);
	}

	inline Vec3 operator- (const Vec3 &v) const
	{
		return Vec3(x - v.x, y - v.y, z - v.z);
	}

	inline Vec3 operator+ (const Vec3 &v) const
	{
		return Vec3(x + v.x, y + v.y, z + v.z);
	}

	inline Vec3 operator* (const double &a) const
	{
		return Vec3(x * a, y * a, z * a);
	}

	inline Vec3 operator-() const
	{
		return Vec3(-x, -y, -z);
	}

	inline Vec3 cross(const Vec3 &v) const
	{
		return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	inline double dot(const Vec3 &v) const
	{
		return x * v.x + y * v.y + z * v.z;
	}
};

#endif //_VEC3_H_
