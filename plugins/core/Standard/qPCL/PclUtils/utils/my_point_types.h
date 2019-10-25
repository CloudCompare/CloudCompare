//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_MY_POINT_TYPES_H
#define Q_PCL_PLUGIN_MY_POINT_TYPES_H

//PCL
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>

//! PCL custom point type used for reading RGB data
struct OnlyRGB
{
	union
	{
		union
		{
			struct
			{
				boost::uint8_t b;
				boost::uint8_t g;
				boost::uint8_t r;
				boost::uint8_t a;
			};
			float rgb;
		};
		boost::uint32_t rgba;
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

//! PCL custom point type used for reading intensity data
struct PointI
{
	float intensity;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct FloatScalar
{
	float S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct DoubleScalar
{
	double S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct IntScalar
{
	int S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct UIntScalar
{
	unsigned S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct ShortScalar
{
	short S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct UShortScalar
{
	unsigned short S5c4laR;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

//! PCL custom point type used for reading intensity data
struct OnlyNormals
{
	float normal_x;
	float normal_y;
	float normal_z;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct OnlyNormalsCurvature
{
	PCL_ADD_NORMAL4D;

	union
	{
		struct
		{
			float curvature;
		};
		float data_c[4];
	};
};

struct PointXYZScalar
{
	PCL_ADD_POINT4D;
	float scalar;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		// make sure our new allocators are aligned

} EIGEN_ALIGN16;						// enforce SSE padding for correct memory alignment

struct PointXYZScalarRGB
{
	PCL_ADD_POINT4D;
	float scalar;
	union
	{
		union
		{
			struct
			{
				boost::uint8_t b;
				boost::uint8_t g;
				boost::uint8_t r;
				boost::uint8_t _unused;
			};
			float rgb;
		};
		boost::uint32_t rgba;
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

struct PointXYZScalarRGBNormals
{
	PCL_ADD_NORMAL4D;
	//PCL_ADD_RGB;
	union
	{
		union
		{
			struct
			{
				boost::uint8_t b;
				boost::uint8_t g;
				boost::uint8_t r;
				boost::uint8_t a;
			};
			float rgb;
		};
		uint32_t rgba;
	};
	PCL_ADD_POINT4D;
	float curvature;
	float scalar;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT (OnlyRGB,
	(float, rgb, rgb)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointI,
	(float, intensity, intensity)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (FloatScalar,
	(float, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (DoubleScalar,
	(double, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (IntScalar,
	(int, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (UIntScalar,
	(unsigned int, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (ShortScalar,
	(short, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (UShortScalar,
	(unsigned short, S5c4laR, S5c4laR)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (OnlyNormals,
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (OnlyNormalsCurvature,
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	(float, curvature, curvature)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZScalar,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scalar, scalar)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZScalarRGB,
	(float, rgb, rgb)
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scalar, scalar)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZScalarRGBNormals,
	(float, rgb, rgb)
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scalar, scalar)
	(float, normal_x, normal_x)
	(float, normal_y, normal_y)
	(float, normal_z, normal_z)
	(float, curvature, curvature)
	)

#endif // Q_PCL_PLUGIN_MY_POINT_TYPES_H
