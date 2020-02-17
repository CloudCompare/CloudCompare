//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GEOM_HEADER
#define CC_GEOM_HEADER

//Local
#include "CCCoreLib.h"
#include "CCTypes.h"

//system
#include <cmath>
#include <limits>
#include <algorithm>

//! 2D Vector
template <typename Type> class Vector2Tpl
{
public:

	union
	{
		struct
		{
			Type x, y;
		};
		Type u[2];
	};

	//! Default constructor
	/** Inits vector to (0,0).
		\param s default init value for both coordinates
	**/
	inline explicit Vector2Tpl(Type s = 0) : x(s), y(s) {}

	//! Constructor from a couple of coordinates
	/** Inits vector to (x,y).
		\param _x x coordinate
		\param _y y coordinate
	**/
	inline Vector2Tpl(Type _x, Type _y) : x(_x), y(_y) {}

	//! Returns vector square norm
	inline Type norm2() const { return (x*x) + (y*y); }
	//! Returns vector norm
	inline Type norm() const { return std::sqrt(norm2()); }
	//! Sets vector norm to unity
	inline void normalize() { Type n = norm2(); if (n > 0) *this /= std::sqrt(n); }

	//! Dot product
	inline Type dot(const Vector2Tpl& v) const { return (x*v.x) + (y*v.y); }
	//! Cross product
	/** \return a positive value if (u,v) makes a counter-clockwise turn, negative for clockwise turn, and zero if the vectors are parallel
	**/
	inline Type cross(const Vector2Tpl& v) const { return x * v.y - y * v.x; }

	//! Inverse operator
	inline Vector2Tpl& operator - () { x = -x; y = -y; return *this; }
	//! In-place addition operator
	inline Vector2Tpl& operator += (const Vector2Tpl& v) { x += v.x; y += v.y; return *this; }
	//! In-place subtraction operator
	inline Vector2Tpl& operator -= (const Vector2Tpl& v) { x -= v.x; y -= v.y; return *this; }
	//! In-place multiplication (by a scalar) operator
	inline Vector2Tpl& operator *= (Type v) { x *= v; y *= v; return *this; }
	//! In-place division (by a scalar) operator
	inline Vector2Tpl& operator /= (Type v) { x /= v; y /= v; return *this; }
	//! Addition operator
	inline Vector2Tpl operator + (const Vector2Tpl& v) const { return Vector2Tpl(x + v.x, y + v.y); }
	//! Subtraction operator
	inline Vector2Tpl operator - (const Vector2Tpl& v) const { return Vector2Tpl(x - v.x, y - v.y); }
	//! Multiplication operator
	inline Vector2Tpl operator * (Type s) const { return Vector2Tpl(x*s, y*s); }
	//! Division operator
	inline Vector2Tpl operator / (Type s) const { return Vector2Tpl(x / s, y / s); }
	//! Direct coordinate access
	inline Type& operator [] (unsigned i) { return u[i]; }
	//! Direct coordinate access (const)
	inline const Type& operator [] (unsigned i) const { return u[i]; }
};

//! 3-Tuple structure (templated version)
template <class Type> class Tuple3Tpl
{
public:
	
	// The 3 tuple values as a union (array/separate values)
	union
	{
		struct
		{
			Type x, y, z;
		};
		Type u[3];
	};

	//! Default constructor
	/** Inits tuple to (0, 0, 0).
	**/
	inline Tuple3Tpl() : x(0), y(0), z(0) {}

	//! Constructor from a triplet of values
	/** Inits typle to (a,b,c).
	**/
	inline Tuple3Tpl(Type a, Type b, Type c) : x(a), y(b), z(c) {}

	//! Constructor from an array of 3 elements
	inline explicit Tuple3Tpl(const Type p[]) : x(p[0]), y(p[1]), z(p[2]) {}

	//! Inverse operator
	inline Tuple3Tpl operator - () const { Tuple3Tpl V(-x, -y, -z); return V; }
	//! In-place addition operator
	inline Tuple3Tpl& operator += (const Tuple3Tpl& v) { x += v.x; y += v.y; z += v.z; return *this; }
	//! In-place subtraction operator
	inline Tuple3Tpl& operator -= (const Tuple3Tpl& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
	//! In-place multiplication (by a scalar) operator
	inline Tuple3Tpl& operator *= (Type v) { x *= v; y *= v; z *= v; return *this; }
	//! In-place division (by a scalar) operator
	inline Tuple3Tpl& operator /= (Type v) { x /= v; y /= v; z /= v; return *this; }
	//! Addition operator
	inline Tuple3Tpl operator + (const Tuple3Tpl& v) const { return Tuple3Tpl(x + v.x, y + v.y, z + v.z); }
	//! Subtraction operator
	inline Tuple3Tpl operator - (const Tuple3Tpl& v) const { return Tuple3Tpl(x - v.x, y - v.y, z - v.z); }
	//! Multiplication operator
	inline Tuple3Tpl operator * (Type s) const { return Tuple3Tpl(x*s, y*s, z*s); }
	//! Division operator
	inline Tuple3Tpl operator / (Type s) const { return Tuple3Tpl(x/s, y/s, z/s); }
};

//! Tuple of 3 unsigned bytes
using Tuple3ub = Tuple3Tpl<unsigned char>;
//! Tuple of 3 short values
using Tuple3s = Tuple3Tpl<short>;
//! Tuple of 3 int values
using Tuple3i = Tuple3Tpl<int>;
//! Tuple of 3 unsigned int values
using Tuple3ui = Tuple3Tpl<unsigned int>;

//! 3D Vector (templated version)
template <typename Type> class Vector3Tpl : public Tuple3Tpl<Type>
{
public:

	//Don't ask me what other x, y, z or u members this class could
	//use but it seems necessary for compilation on some platforms...
	using Tuple3Tpl<Type>::x;
	using Tuple3Tpl<Type>::y;
	using Tuple3Tpl<Type>::z;
	using Tuple3Tpl<Type>::u;

	//! Default constructor
	/** Inits vector to (0, 0, 0).
	**/
	inline Vector3Tpl() : Tuple3Tpl<Type>() {}

	//! Constructor from a triplet of coordinates
	/** Inits vector to (x,y,z).
	**/
	inline Vector3Tpl(Type _x, Type _y, Type _z) : Tuple3Tpl<Type>(_x, _y, _z) {}

	//! Constructor from an array of 3 elements
	inline explicit Vector3Tpl(const Type p[]) : Tuple3Tpl<Type>(p) {}

	//! Constructor from a 2D vector (and a third value)
	inline explicit Vector3Tpl(const Vector2Tpl<Type>& t2D, Type c) : Tuple3Tpl<Type>(t2D.x, t2D.y, c) {}

	//! Constructor from an int array
	static inline Vector3Tpl fromArray(const int a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }
	//! Constructor from a float array
	static inline Vector3Tpl fromArray(const float a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }
	//! Constructor from a double array
	static inline Vector3Tpl fromArray(const double a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }

	//! Dot product
	inline Type dot(const Vector3Tpl& v) const { return x*v.x + y*v.y + z*v.z; }
	//! Cross product
	inline Vector3Tpl cross(const Vector3Tpl &v) const { return Vector3Tpl((y*v.z) - (z*v.y), (z*v.x) - (x*v.z), (x*v.y) - (y*v.x)); }
	//! Returns vector square norm
	inline Type norm2() const { return x*x + y*y + z*z; }
	//! Returns vector square norm (forces double precision output)
	inline double norm2d() const { return static_cast<double>(x)*x + static_cast<double>(y)*y + static_cast<double>(z)*z; }
	//! Returns vector norm
	inline Type norm() const { return static_cast<Type>(std::sqrt(norm2d())); }
	//! Returns vector norm (forces double precision output)
	inline double normd() const { return std::sqrt(norm2d()); }
	//! Sets vector norm to unity
	inline void normalize() { Type n = norm(); if (n > std::numeric_limits<Type>::epsilon()) *this /= n; }
	//! Returns a normalized vector which is orthogonal to this one
	inline Vector3Tpl orthogonal() const { Vector3Tpl ort; vorthogonal(u, ort.u); return ort; }

	//! Inverse operator
	inline Vector3Tpl operator - () const { Vector3Tpl V(-x, -y, -z); return V; }
	//! In-place addition operator
	inline Vector3Tpl& operator += (const Vector3Tpl& v) { x += v.x; y += v.y; z += v.z; return *this; }
	//! In-place subtraction operator
	inline Vector3Tpl& operator -= (const Vector3Tpl& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
	//! In-place multiplication (by a scalar) operator
	inline Vector3Tpl& operator *= (Type v) { x *= v; y *= v; z *= v; return *this; }
	//! In-place division (by a scalar) operator
	inline Vector3Tpl& operator /= (Type v) { x /= v; y /= v; z /= v; return *this; }
	//! Addition operator
	inline Vector3Tpl operator + (const Vector3Tpl& v) const { return Vector3Tpl(x + v.x, y + v.y, z + v.z); }
	//! Subtraction operator
	inline Vector3Tpl operator - (const Vector3Tpl& v) const { return Vector3Tpl(x - v.x, y - v.y, z - v.z); }
	//! Multiplication operator
	inline Vector3Tpl operator * (Type s) const { return Vector3Tpl(x*s, y*s, z*s); }
	//! Division operator
	inline Vector3Tpl operator / (Type s) const { return Vector3Tpl(x/s, y/s, z/s); }
	//! Cross product operator
	inline Vector3Tpl operator * (const Vector3Tpl& v) const { return cross(v); }
	//! Dot product operator
	inline Type operator && (const Vector3Tpl& v) const { return dot(v); }
	//! Direct coordinate access
	inline Type& operator [] (unsigned i) { return u[i]; }
	//! Direct coordinate access (const)
	inline const Type& operator [] (unsigned i) const { return u[i]; }
	//! Returns the angle to another vector (in radians - in [0, pi]
	Type angle_rad(const Vector3Tpl& v) const { return vangle_rad(u, v.u); }
	double angle_radd(const Vector3Tpl& v) const { return vangle_radd(u, v.u); }

	static inline void vdivide(const Type p[], Type s, Type r[]) { r[0] = p[0] / s; r[1] = p[1] / s; r[2] = p[2] / s; }
	static inline void vdivide(Type p[], Type s) { p[0] /= s; p[1] /= s; p[2] /= s; }
	static inline void vmultiply(const Type p[], Type s, Type r[]) { r[0] = p[0] * s; r[1] = p[1] * s; r[2] = p[2] * s; }
	static inline void vmultiply(Type p[], Type s) { p[0] *= s; p[1] *= s; p[2] *= s; }
	static inline Type vdot(const Type p[], const Type q[]) { return (p[0] * q[0]) + (p[1] * q[1]) + (p[2] * q[2]); }
	static inline double vdotd(const Type p[], const Type q[]) { return (static_cast<double>(p[0])* q[0]) + (static_cast<double>(p[1])* q[1]) + (static_cast<double>(p[2])* q[2]); }
	static inline void vcross(const Type p[], const Type q[], Type r[]) { r[0] = (p[1] * q[2]) - (p[2] * q[1]); r[1] = (p[2] * q[0]) - (p[0] * q[2]); r[2] = (p[0] * q[1]) - (p[1] * q[0]); }
	static inline void vcopy(const Type p[], Type q[]) { q[0] = p[0]; q[1] = p[1]; q[2] = p[2]; }
	static inline void vset(Type p[], Type s) { p[0] = p[1] = p[2] = s; }
	static inline void vset(Type p[], Type x, Type y, Type z) { p[0] = x; p[1] = y; p[2] = z; }
	static inline void vadd(const Type p[], const Type q[], Type r[]) { r[0] = p[0] + q[0]; r[1] = p[1] + q[1]; r[2] = p[2] + q[2]; }
	// note misspelling: should be vsubtract
	static inline void vsubstract(const Type p[], const Type q[], Type r[]) { r[0] = p[0] - q[0]; r[1] = p[1] - q[1]; r[2] = p[2] - q[2]; }
	static inline void vcombination(Type a, const Type p[], Type b, const Type q[], Type r[]) { r[0] = (a*p[0]) + (b*q[0]); r[1] = (a*p[1]) + (b*q[1]); r[2] = (a*p[2]) + (b*q[2]); }
	static inline void vcombination(const Type p[], Type b, const Type q[], Type r[]) { r[0] = p[0] + (b*q[0]); r[1] = p[1] + (b*q[1]); r[2] = p[2] + (b*q[2]); }
	static inline void vnormalize(Type p[]) { Type n = vnorm2(p); if (n > 0) vdivide(p, std::sqrt(n)); }
	static inline Type vnorm2(const Type p[]) { return (p[0] * p[0]) + (p[1] * p[1]) + (p[2] * p[2]); }
	static inline double vnorm2d(const Type p[]) { return (static_cast<double>(p[0]) * p[0]) + (static_cast<double>(p[1]) * p[1]) + (static_cast<double>(p[2]) * p[2]); }
	static inline Type vdistance2(const Type p[], const Type q[]) { return ((p[0] - q[0])*(p[0] - q[0])) + ((p[1] - q[1])*(p[1] - q[1])) + ((p[2] - q[2])*(p[2] - q[2])); }
	static inline double vdistance2d(const Type p[], const Type q[]) { return ((static_cast<double>(p[0]) - q[0]) * (static_cast<double>(p[0]) - q[0])) + ((static_cast<double>(p[1]) - q[1]) * (static_cast<double>(p[1]) - q[1])) + ((static_cast<double>(p[2]) - q[2]) * (static_cast<double>(p[2]) - q[2])); }
	static inline Type vnorm(const Type p[]) { return std::sqrt(vnorm2(p)); }
	static inline double vnormd(const Type p[]) { return std::sqrt(vnorm2d(p)); }
	static inline Type vdistance(const Type p[], const Type q[]) { return std::sqrt(vdistance2(p, q)); }
	static inline double vdistanced(const Type p[], const Type q[]) { return std::sqrt(vdistance2d(p, q)); }

	static void vorthogonal(const Type p[], Type q[])
	{
		if (std::abs(p[0]) <= std::abs(p[1]) && std::abs(p[0]) <= std::abs(p[2]))
		{
			q[0] = 0; q[1] = p[2]; q[2] = -p[1];
		}
		else if (std::abs(p[1]) <= std::abs(p[0]) && std::abs(p[1]) <= std::abs(p[2]))
		{
			q[0] = -p[2]; q[1] = 0; q[2] = p[0];
		}
		else
		{
			q[0] = p[1]; q[1] = -p[0]; q[2] = 0;
		}
		vnormalize(q);
	}

	static Type vangle_rad(const Type p[], const Type q[])
	{
		Type productNorm = vnorm(p) * vnorm(q);
		if (productNorm < std::numeric_limits<Type>::epsilon())
		{
			return std::numeric_limits<Type>::quiet_NaN();
		}

		Type cosAngle = vdot(p, q) / productNorm;
		return acos(std::max(std::min(cosAngle, static_cast<Type>(1.0)), static_cast<Type>(-1.0)));
	}

	static double vangle_radd(const Type p[], const Type q[])
	{
		double productNorm = vnormd(p) * vnormd(q);
		if (productNorm < std::numeric_limits<double>::epsilon())
		{
			return std::numeric_limits<double>::quiet_NaN();
		}

		double cosAngle = vdotd(p, q) / productNorm;
		return acos(std::max(std::min(cosAngle, 1.0), -1.0));
	}

};

//! 4-Tuple structure (templated version)
template <class Type> class Tuple4Tpl
{
public:

	// The 4 tuple values as a union (array/separate values)
	union
	{
		struct
		{
			Type x, y, z, w;
		};
		Type u[4];
	};

	//! Default constructor
	/** Inits tuple to (0, 0, 0, 0).
	**/
	inline Tuple4Tpl() : x(0), y(0), z(0), w(0) {}

	//! Constructor from a triplet of values
	/** Inits typle to (a,b,c).
	**/
	inline Tuple4Tpl(Type a, Type b, Type c, Type d) : x(a), y(b), z(c), w(d) {}

	//! Constructor from an array of 4 elements
	inline explicit Tuple4Tpl(const Type p[]) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}

	//! Inverse operator
	inline Tuple4Tpl operator - () const { Tuple4Tpl V(-x, -y, -z, -w); return V; }
	//! In-place addition operator
	inline Tuple4Tpl& operator += (const Tuple4Tpl& v) { x += v.x; y += v.y; z += v.z; w += v.w; return *this; }
	//! In-place subtraction operator
	inline Tuple4Tpl& operator -= (const Tuple4Tpl& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this; }
	//! In-place multiplication (by a scalar) operator
	inline Tuple4Tpl& operator *= (Type v) { x *= v; y *= v; z *= v; w *= v; return *this; }
	//! In-place division (by a scalar) operator
	inline Tuple4Tpl& operator /= (Type v) { x /= v; y /= v; z /= v; w /= v; return *this; }
	//! Addition operator
	inline Tuple4Tpl operator + (const Tuple4Tpl& v) const { return Tuple4Tpl(x + v.x, y + v.y, z + v.z, w + v.w); }
	//! Subtraction operator
	inline Tuple4Tpl operator - (const Tuple4Tpl& v) const { return Tuple4Tpl(x - v.x, y - v.y, z - v.z, w - v.w); }
	//! Multiplication operator
	inline Tuple4Tpl operator * (Type s) const { return Tuple4Tpl(x*s, y*s, z*s, w*s); }
	//! Division operator
	inline Tuple4Tpl operator / (Type s) const { return Tuple4Tpl(x / s, y / s, z / s, w / s); }
};

//! Default 2D Vector
using CCVector2 = Vector2Tpl<PointCoordinateType>;

//! Double 2D Vector
using CCVector2d = Vector2Tpl<double>;

//! Int 2D Vector
using CCVector2i = Vector2Tpl<int>;

//! Multiplication of a 3D vector by a scalar (front) operator (float version)
inline Vector3Tpl<float> operator * (float s, const Vector3Tpl<float> &v) { return v*s; }
// Multiplication of a 3D vector by a scalar (front) operator (double version)
inline Vector3Tpl<double> operator * (double s, const Vector3Tpl<double> &v) { return v*s; }

//! Default 3D Vector
using CCVector3 = Vector3Tpl<PointCoordinateType>;

//! Double 3D Vector
using CCVector3f = Vector3Tpl<float>;
//! Double 3D Vector
using CCVector3d = Vector3Tpl<double>;

#endif //CC_GEOM_HEADER
