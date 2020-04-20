//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GLOBAL_SHIFT_MANAGER_HEADER
#define CC_GLOBAL_SHIFT_MANAGER_HEADER

//CCLib
#include <CCGeom.h>

//local
#include "qCC_io.h"

//Qt
#include <QString>

//STL
#include <vector>

class ccHObject;

//! Helper class to handle big coordinates shift/scale (typically while loading entities)
class QCC_IO_LIB_API ccGlobalShiftManager
{
public:

	//! Strategy to handle coordinates shift/scale
	enum Mode { NO_DIALOG, NO_DIALOG_AUTO_SHIFT, DIALOG_IF_NECESSARY, ALWAYS_DISPLAY_DIALOG };

	//! Handles coordinates shift/scale given the first 3D point and current related parameters
	static bool Handle(	const CCVector3d& P,
						double diagonal,
						Mode mode,
						bool useInputCoordinatesShiftIfPossible,
						CCVector3d& coordinatesShift,
						bool* preserveCoordinateShift,
						double* coordinatesScale,
						bool* applyAll = 0);

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

	//! Returns the max coordinate (absolute) value
	static double MaxCoordinateAbsValue() { return MAX_COORDINATE_ABS_VALUE; }
	//! Sets the max coordinate (absolute) value
	static void SetMaxCoordinateAbsValue(double value) { MAX_COORDINATE_ABS_VALUE = value; }

	//! Returns max bounding-box diagonal
	static double MaxBoundgBoxDiagonal() { return MAX_DIAGONAL_LENGTH; }
	//! Sets the max bounding-box diagonal
	static void SetMaxBoundgBoxDiagonal(double value) { MAX_DIAGONAL_LENGTH = value; }

	//! Adds a new shift / scale couple
	static void StoreShift(const CCVector3d& shift, double scale, bool preserve = true);

public: //Shift and scale info

	//! Shift and scale info
	struct ShiftInfo
	{
		CCVector3d shift;
		double scale;
		QString name;
		bool preserve;

		//! Default constructor
		ShiftInfo(QString str = QString("unnamed")) : shift(0, 0, 0), scale(1.0), name(str), preserve(true) {}
		//! Constructor from a vector and a scale value
		ShiftInfo(QString str, const CCVector3d& T, double s = 1.0) : shift(T), scale(s), name(str), preserve(true) {}
	};

	static bool GetLast(ShiftInfo& info);
	static bool GetLast(std::vector<ShiftInfo>& infos);

protected:
	
	// Max acceptable coordinate value
	static double MAX_COORDINATE_ABS_VALUE;

	// Max acceptable diagonal length
	static double MAX_DIAGONAL_LENGTH;
};

#endif
