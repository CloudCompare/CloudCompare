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
