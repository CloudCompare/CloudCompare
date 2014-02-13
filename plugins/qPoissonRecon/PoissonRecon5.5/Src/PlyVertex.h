#ifndef __PLY_VERTEX_H__
#define __PLY_VERTEX_H__

#include "Geometry.h"

typedef struct PlyProperty {    /* description of a property */
	
	char *name;                           /* property name */
	int external_type;                    /* file's data type */
	int internal_type;                    /* program's data type */
	int offset;                           /* offset bytes of prop in a struct */
	
	int is_list;                          /* 1 = list, 0 = scalar */
	int count_external;                   /* file's count type */
	int count_internal;                   /* program's count type */
	int count_offset;                     /* offset byte for list count */
	
} PlyProperty;

template< class Real >
class PlyVertex
{
public:
	const static int Components=3;
	static PlyProperty Properties[];

	Point3D< Real > point;

#if 1
	PlyVertex( void ) { ; }
	PlyVertex( Point3D< Real > p ) { point=p; }
	PlyVertex operator + ( PlyVertex p ) const { return PlyVertex( point+p.point ); }
	PlyVertex operator - ( PlyVertex p ) const { return PlyVertex( point-p.point ); }
	PlyVertex operator * ( Real s ) const { return PlyVertex( point*s ); }
	PlyVertex operator / ( Real s ) const { return PlyVertex( point/s ); }
	PlyVertex& operator += ( PlyVertex p ) { point += p.point ; return *this; }
	PlyVertex& operator -= ( PlyVertex p ) { point -= p.point ; return *this; }
	PlyVertex& operator *= ( Real s ) { point *= s ; return *this; }
	PlyVertex& operator /= ( Real s ) { point /= s ; return *this; }
#else
	operator Point3D<Real>& ()					{return point;}
	operator const Point3D<Real>& () const		{return point;}
	PlyVertex(void)								{point.coords[0]=point.coords[1]=point.coords[2]=0;}
	PlyVertex(const Point3D<Real>& p)			{point=p;}
#endif
};

#endif /* !__PLY_VERTEX_H__ */
