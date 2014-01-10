//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_COORDINATES_SHIFT_MANAGER_HEADER
#define CC_COORDINATES_SHIFT_MANAGER_HEADER

//CCLib
#include <CCGeom.h>

//! Helper class to handle coordinates shift while loading entities (GUI, etc.)
class ccCoordinatesShiftManager
{
public:

	//! Handles coordinates shift/scale given the first 3D point and current related parameters
	static bool Handle(	const double* P,
						double diagonal,
						bool alwaysDisplayLoadDialog,
						bool coordinatesTransformationEnabled,
						CCVector3d& coordinatesShift,
						double* coordinatesScale,
						bool& applyAll);
};

#endif