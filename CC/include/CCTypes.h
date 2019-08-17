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

#ifndef CC_TYPES_HEADER
#define CC_TYPES_HEADER

//! Type of the coordinates of a (N-D) point
using PointCoordinateType = float;

//! Type of a single scalar field value
#if defined SCALAR_TYPE_DOUBLE
using ScalarType = double;
#elif defined SCALAR_TYPE_FLOAT
using ScalarType = float;
#else
static_assert(false, "type for ScalarType has not been declared");
#endif //SCALAR_TYPE_DOUBLE

#endif //CC_TYPES_HEADER
