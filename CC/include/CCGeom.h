//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef CC_GEOM_HEADER
#define CC_GEOM_HEADER

#include "CCTypes.h"
#include <math.h>

//! 3D Vector
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API CCVector3
#else
class CCVector3
#endif
{
public:

	union
	{
		struct
		{
			PointCoordinateType x,y,z;
		};
		PointCoordinateType u[3];
	};

    //! Default constructor
    /** Inits vector to (0,0,0).
    **/
	inline CCVector3(PointCoordinateType s=0.) :x(s),y(s),z(s) {}
	//! Constructor from a triplet of coordinates
    /** Inits vector to (x,y,z).
    **/
	inline CCVector3(PointCoordinateType _x, PointCoordinateType _y, PointCoordinateType _z) :x(_x),y(_y),z(_z) {}
	//! Constructor from an array of 3 elements
	inline CCVector3(const PointCoordinateType p[]) :x(p[0]),y(p[1]),z(p[2]) {}
	//! Copy constructor
	inline CCVector3(const CCVector3& v) :x(v.x),y(v.y),z(v.z) {}

    //! Dot product
    inline PointCoordinateType dot(const CCVector3& v) const {return (x*v.x)+(y*v.y)+(z*v.z);}
    //! Cross product
    inline CCVector3 cross(const CCVector3 &v) const {return CCVector3((y*v.z)-(z*v.y), (z*v.x)-(x*v.z), (x*v.y)-(y*v.x));}
    //! Returns vector square norm
    inline PointCoordinateType norm2() const {return (x*x)+(y*y)+(z*z);}
    //! Returns vector norm
    inline PointCoordinateType norm() const {return sqrt(norm2());}
    //! Sets vector norm to unity
    inline void normalize() {PointCoordinateType n = norm2(); if (n>0.0f) *this /= sqrt(n);}
	//! Returns a normalized vector which is orthogonal to this one
    inline CCVector3 orthogonal() const {CCVector3 ort; vorthogonal(u, ort.u); return ort;}

    //! Inverse operator
    inline CCVector3& operator - () {x=-x; y=-y; z=-z; return *this;}
    //! In-place addition operator
	inline CCVector3& operator += (const CCVector3& v) {x+=v.x; y+=v.y; z+=v.z; return *this;}
    //! In-place substraction operator
	inline CCVector3& operator -= (const CCVector3& v) {x-=v.x; y-=v.y; z-=v.z; return *this;}
    //! In-place multiplication (by a scalar) operator
	inline CCVector3& operator *= (PointCoordinateType v) {x*=v; y*=v; z*=v; return *this;}
    //! In-place division (by a scalar) operator
	inline CCVector3& operator /= (PointCoordinateType v) {x/=v; y/=v; z/=v; return *this;}
    //! Addition operator
	inline CCVector3 operator + (const CCVector3& v) const {return CCVector3(x+v.x, y+v.y, z+v.z);}
    //! Substraction operator
	inline CCVector3 operator - (const CCVector3& v) const {return CCVector3(x-v.x, y-v.y, z-v.z);}
    //! Multiplication operator
	inline CCVector3 operator * (PointCoordinateType s) const {return CCVector3(x*s, y*s, z*s);}
    //! Division operator
	inline CCVector3 operator / (PointCoordinateType s) const {return CCVector3(x/s, y/s, z/s);}
    //! Cross product operator
	inline CCVector3 operator * (const CCVector3& v) const {return cross(v);}
	//! Copy operator
	inline CCVector3& operator = (const CCVector3 &v) {x=v.x; y=v.y; z=v.z; return *this;}
    //! Dot product operator
	inline PointCoordinateType operator && (const CCVector3 &v) const {return dot(v);}
	//! Direct coordinate access
	inline PointCoordinateType& operator [] (unsigned i) {return u[i];}
	//! Direct coordinate access (const)
	inline const PointCoordinateType& operator [] (unsigned i) const {return u[i];}
	//! Multiplication by a scalar (front) operator
	friend CCVector3 operator * (PointCoordinateType s, const CCVector3 &v);

	template<class Type> static inline Type vdot(const Type p[], const Type q[]) {return (p[0]*q[0])+(p[1]*q[1])+(p[2]*q[2]);}
	template<class Type> static inline void vcross(const Type p[], const Type q[], Type r[]) {r[0]=(p[1]*q[2])-(p[2]*q[1]); r[1]=(p[2]*q[0])-(p[0]*q[2]); r[2]=(p[0]*q[1])-(p[1]*q[0]);}
	template<class Type> static inline void vcopy(const Type p[], Type q[]) {q[0]=p[0]; q[1]=p[1]; q[2]=p[2];}
	template<class Type> static inline void vset(Type p[], Type s) {p[0]=p[1]=p[2]=s;}
	template<class Type> static inline void vset(Type p[], Type x, Type y, Type z) {p[0]=x; p[1]=y; p[2]=z;}
	template<class Type> static inline void vmultiply(const Type p[], Type s, Type r[]) {r[0]=p[0]*s; r[1]=p[1]*s; r[2]=p[2]*s;}
	template<class Type> static inline void vmultiply(Type p[], Type s) {p[0]*=s; p[1]*=s; p[2]*=s;}
	template<class Type> static inline void vadd(const Type p[], const Type q[], Type r[]) {r[0]=p[0]+q[0]; r[1]=p[1]+q[1]; r[2]=p[2]+q[2];}
	template<class Type> static inline void vsubstract(const Type p[], const Type q[], Type r[]) {r[0]=p[0]-q[0]; r[1]=p[1]-q[1]; r[2]=p[2]-q[2];}
	template<class Type> static inline void vcombination(Type a, const Type p[], Type b, const Type q[], Type r[]) {r[0]=(a*p[0])+(b*q[0]); r[1]=(a*p[1])+(b*q[1]); r[2]=(a*p[2])+(b*q[2]);}
	template<class Type> static inline void vcombination(const Type p[], Type b, const Type q[], Type r[]) {r[0]=p[0]+(b*q[0]); r[1]=p[1]+(b*q[1]); r[2]=p[2]+(b*q[2]);}
	template<class Type> static inline float vnorm2(const Type p[]) {return (p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]);}
	template<class Type> static inline float vnorm(const Type p[]) {return sqrt(vnorm2(p));}
	template<class Type> static inline void vnormalize(Type p[]) {Type n = vnorm2(p); if (n>0.0) vmultiply<Type>(p, (Type)1.0/sqrt(n), p);}
	template<class Type> static inline float vdistance2(const Type p[], const Type q[]) {return ((p[0]-q[0])*(p[0]-q[0]))+((p[1]-q[1])*(p[1]-q[1]))+((p[2]-q[2])*(p[2]-q[2]));}
	template<class Type> static inline float vdistance(const Type p[], const Type q[]) {return sqrt(vdistance2(p, q));}

	template<class Type> static inline void vorthogonal(const Type p[], Type q[])
	{
		Type qq[3];
		if (fabs(p[0])<=fabs(p[1]) && fabs(p[0])<=fabs(p[2]))
		{
			qq[0]=0.0f;qq[1]=p[2];qq[2]=-p[1];
		}
		else if (fabs(p[1])<=fabs(p[0]) && fabs(p[1])<=fabs(p[2]))
		{
			qq[0]=-p[2];qq[1]=0.;qq[2]=p[0];
		}
		else
		{
			qq[0]=p[1];qq[1]=-p[0];qq[2]=0.0f;
		}
		vcopy<Type>(qq,q);
		vnormalize<Type>(q);
	}

};

//! Multiplication by a scalar (front) operator
inline CCVector3 operator * (PointCoordinateType s, const CCVector3 &v) {return v*s;}

//! 2D Vector
class CCVector2
{
public:

	union
	{
		struct
		{
			PointCoordinateType x,y;
		};
		PointCoordinateType u[2];
	};

    //! Default constructor
    /** Inits vector to (0,0).
        \param s default init value for both coordinates
    **/
	inline CCVector2(PointCoordinateType s=0.0) : x(s), y(s) {}

	//! Constructor from a couple of coordinates
    /** Inits vector to (x,y).
        \param _x x coordinate
        \param _y y coordinate
    **/
	inline CCVector2(PointCoordinateType _x, PointCoordinateType _y) : x(_x), y(_y) {}

	//! Returns vector square norm
    inline PointCoordinateType norm2() const {return (x*x)+(y*y);}
    //! Returns vector norm
    inline PointCoordinateType norm() const {return sqrt(norm2());}
    //! Sets vector norm to unity
    inline void normalize() {PointCoordinateType n = norm2(); if (n>0.0f) *this /= sqrt(n);}

    //! Inverse operator
    inline CCVector2& operator - () {x=-x; y=-y; return *this;}
    //! In-place addition operator
	inline CCVector2& operator += (const CCVector2& v) {x+=v.x; y+=v.y; return *this;}
    //! In-place substraction operator
	inline CCVector2& operator -= (const CCVector2& v) {x-=v.x; y-=v.y; return *this;}
    //! In-place multiplication (by a scalar) operator
	inline CCVector2& operator *= (PointCoordinateType v) {x*=v; y*=v; return *this;}
    //! In-place division (by a scalar) operator
	inline CCVector2& operator /= (PointCoordinateType v) {x/=v; y/=v; return *this;}
    //! Addition operator
	inline CCVector2 operator + (const CCVector2& v) const {return CCVector2(x+v.x, y+v.y);}
    //! Substraction operator
	inline CCVector2 operator - (const CCVector2& v) const {return CCVector2(x-v.x, y-v.y);}
    //! Multiplication operator
	inline CCVector2 operator * (PointCoordinateType s) const {return CCVector2(x*s, y*s);}
    //! Division operator
	inline CCVector2 operator / (PointCoordinateType s) const {return CCVector2(x/s, y/s);}
	//! Copy operator
	inline CCVector2& operator = (const CCVector2 &v) {x=v.x; y=v.y; return *this;}
	//! Direct coordinate access
	inline PointCoordinateType& operator [] (unsigned i) {return u[i];}
	//! Direct coordinate access (const)
	inline const PointCoordinateType& operator [] (unsigned i) const {return u[i];}
	//! Multiplication by a scalar (front) operator
	friend CCVector2 operator * (PointCoordinateType s, const CCVector2 &v);
};

#endif
