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

#ifndef CC_CONST_HEADER
#define CC_CONST_HEADER

#include "CCTypes.h"

#include <math.h>

//! Max float value
#ifndef FLOAT_MAX
const float FLOAT_MAX = 3.4e38f;
#endif

//! Max double value
#ifndef DOUBLE_MAX
const double DOUBLE_MAX = 1.7e308;
#endif

//! Point visibility states
/** Visibility is expressed relatively to the sensor point of view.
**/
enum CC_VISIBILITY_TYPE {VIEWED					=	0,				/**< Point is "viewed" by the sensor **/
						HIDDEN					=	1,				/**< Point is hidden (behind other points) **/
						OUT_OF_RANGE			=	2,				/**< Point is out of range **/
						OUT_OF_FOV				=	4,				/**< Point is out of field of view **/
						ALL						=	7				/**< All states at once **/
};

// Particular distance values (should always be negative, and always in increasing order)
const DistanceType SEGMENTED_VALUE				=	(DistanceType)-3.0;						/**< numerical value to code the "segmented" state with strictly positive scalar fields **/
const DistanceType OUT_VALUE					=	(DistanceType)-2.0f;					/**< numerical value to code the "out of range/fov" visiblity state with strictly positive scalar fields **/
const DistanceType HIDDEN_VALUE					=	(DistanceType)-1.0f;					/**< numerical value to code the "hidden" visiblity state with strictly positive scalar fields **/
const DistanceType BIG_VALUE					=	(DistanceType)(sqrt(FLOAT_MAX)-1.0f);	/**< numerical value to code the "hidden" visiblity state with NON strictly positive scalar fields **/

//! Chamfer distances types
enum CC_CHAMFER_DISTANCE_TYPE {CHAMFER_111		=	0,				/**< Chamfer distance <1-1-1> **/
								CHAMFER_345		=	1				/**< Chamfer distance <3-4-5> **/
};

//! Types of local models (no model, least square best fitting plan, Delaunay 2D1/2 triangulation, height function)
enum CC_LOCAL_MODEL_TYPES {NO_MODEL				=	0,				/**< No local model **/
							LS					=	1,				/**< Least Square best fitting plane **/
							TRI					=	2,				/**< Delaunay triangulation (2.5D) **/
							HF					=	3				/**< Quadratic 'height' function **/
};

//! Min number of points to compute local models (see CC_LOCAL_MODEL_TYPES)
const unsigned CC_LOCAL_MODEL_MIN_SIZE[] = {	1,				/**< for single point model (i.e. no model ;) **/
												3,				/**< for least Square best fitting plane **/
												3,				/**< for Delaunay triangulation (2.5D) **/
												6,				/**< for Quadratic 'height' function **/
};

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

//! Conversion factor from radians to degrees
#ifndef CC_RAD_TO_DEG
#define CC_RAD_TO_DEG 180.0/M_PI
#endif

//! Conversion factor from degrees to radians
#ifndef CC_DEG_TO_RAD
#define CC_DEG_TO_RAD M_PI/180.0
#endif

//! Numerical threshold for considering a value as "zero"
#ifndef ZERO_TOLERANCE
#define ZERO_TOLERANCE 1e-8
#endif

#endif
