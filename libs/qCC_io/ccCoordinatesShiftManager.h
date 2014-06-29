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

class ccHObject;

//! Helper class to handle coordinates shift while loading entities (GUI, etc.)
class ccCoordinatesShiftManager
{
public:

	//! Handles coordinates shift/scale given the first 3D point and current related parameters
	static bool Handle(	const CCVector3d& P,
						double diagonal,
						bool displayDialogIfNecessary,
						bool useInputCoordinatesShiftIfPossible,
						CCVector3d& coordinatesShift,
						double* coordinatesScale,
						bool* applyAll = 0,
						bool forceDialogDisplay = false);

	//! Returns whether a particular point (coordinates) is too big or not
	static bool NeedShift(const CCVector3d& P);
	//! Returns whether a particular point coordinate is too big or not
	static bool NeedShift(double d);
	//! Returns whether a particular dimension (e.g. diagonal) is too big or not
	static bool NeedRescale(double d);

	//! Suggests a shift for a given point expressed in global coordinate space
	static CCVector3d BestShift(const CCVector3d& P);
	//! Suggests a scale for a given dimension (e.g. diagonal) in global coordinate space
	static double BestScale(double d);

	//! Returns max coordinate (absolute) value
	static double MaxCoordinateAbsValue();
	//! Returns max bounding-box diagonal
	static double MaxBoundgBoxDiagonal();
};

#endif