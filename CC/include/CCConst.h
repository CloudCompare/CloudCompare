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

#ifndef CC_CONST_HEADER
#define CC_CONST_HEADER

#include "CCTypes.h"

//system
#include <cfloat>
#include <cmath>
#include <limits>

//! Pi
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//! Pi/2
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

//! Square root of 3
#ifndef SQRT_3
#define SQRT_3 1.7320508075688772935274463415059
#endif

//! Conversion factor from radians to degrees
#ifndef CC_RAD_TO_DEG
#define CC_RAD_TO_DEG (180.0/M_PI)
#endif

//! Conversion factor from degrees to radians
#ifndef CC_DEG_TO_RAD
#define CC_DEG_TO_RAD (M_PI/180.0)
#endif

//! Numerical threshold for considering a value as "zero"
#ifndef ZERO_TOLERANCE
#define ZERO_TOLERANCE static_cast<double>(FLT_EPSILON)
#endif

//! '1' as a PointCoordinateType value
/** To avoid compiler warnings about 'possible loss of data' **/
const PointCoordinateType PC_ONE = static_cast<PointCoordinateType>(1.0);

//! 'NaN' as a PointCoordinateType value
/** \warning: handle with care! **/
const PointCoordinateType PC_NAN = std::numeric_limits<PointCoordinateType>::quiet_NaN();

//! NaN as a ScalarType value
/** \warning: handle with care! **/
const ScalarType NAN_VALUE = std::numeric_limits<ScalarType>::quiet_NaN();

// Point visibility states
// By default visibility is expressed relatively to the sensor point of view.
// Warning: 'visible' value must always be the lowest!
const unsigned char POINT_VISIBLE				=	 0;				/**< Point visibility state: visible **/
const unsigned char POINT_HIDDEN				=	 1;				/**< Point visibility state: hidden (e.g. behind other points) **/
const unsigned char POINT_OUT_OF_RANGE			=	 2;				/**< Point visibility state: out of range **/
const unsigned char POINT_OUT_OF_FOV			=	 4;				/**< Point visibility state: out of field of view **/

//! Chamfer distances types
enum CC_CHAMFER_DISTANCE_TYPE { CHAMFER_111		=	0,				/**< Chamfer distance <1-1-1> **/
								CHAMFER_345		=	1				/**< Chamfer distance <3-4-5> **/
};

//! Types of local models (no model, least square best fitting plan, Delaunay 2D1/2 triangulation, height function)
enum CC_LOCAL_MODEL_TYPES {NO_MODEL				=	0,				/**< No local model **/
							LS					=	1,				/**< Least Square best fitting plane **/
							TRI					=	2,				/**< 2.5D Delaunay triangulation **/
							QUADRIC				=	3				/**< 2.5D quadric function **/
};

//! Min number of points to compute local models (see CC_LOCAL_MODEL_TYPES)
const unsigned CC_LOCAL_MODEL_MIN_SIZE[] = {		1,				/**< for single point model (i.e. no model ;) **/
													3,				/**< for least Square best fitting plane **/
													3,				/**< for Delaunay triangulation (2.5D) **/
													6,				/**< for Quadratic 'height' function **/
};

#endif //CC_CONST_HEADER
