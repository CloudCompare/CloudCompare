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

#ifndef CC_TYPES_HEADER
#define CC_TYPES_HEADER

#include <stdio.h>

#ifndef uchar
typedef unsigned char uchar;
#endif

//! Type of a ND point coordinate
typedef float PointCoordinateType;

//! Type of an octree cell code (need 3 bits per level)
//#define OCTREE_CODES_64_BITS
#ifdef OCTREE_CODES_64_BITS
typedef unsigned __int64 OctreeCellCodeType; //max 21 levels (but twice more memory!)
#else
typedef unsigned OctreeCellCodeType; //max 10 levels
#endif

//! Type of a distance value (or more generally of any scalar value)
typedef float DistanceType;

#endif
