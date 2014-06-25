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

//Local
#include "ccCoordinatesShiftManager.h"
#include "ccShiftAndScaleCloudDlg.h"

//qCC_db
#include <ccHObject.h>

//System
#include <string.h>
#include <assert.h>

// Max acceptable coordinate value
#define MAX_COORDINATE_ABS_VALUE 1.0e5
// Max acceptable diagonal length
#define MAX_DIAGONAL_LENGTH 1.0e6

double ccCoordinatesShiftManager::MaxCoordinateAbsValue()
{
	return MAX_COORDINATE_ABS_VALUE;
}

double ccCoordinatesShiftManager::MaxBoundgBoxDiagonal()
{
	return MAX_DIAGONAL_LENGTH;
}

bool ccCoordinatesShiftManager::Handle(	const double* P,
										double diagonal,
										bool displayDialogIfNecessary,
										bool useInputCoordinatesShiftIfPossible,
										CCVector3d& coordinatesShift,
										double* coordinatesScale,
										bool* applyAll/*=0*/,
										bool forceDialogDisplay/*=false*/)
{
	assert(P);
	assert(diagonal >= 0);

	if (applyAll)
		*applyAll = false;

	//if we can't display a dialog and no usable shift is specified, there's nothing we can do...
	if (!displayDialogIfNecessary && !useInputCoordinatesShiftIfPossible)
	{
		coordinatesShift = CCVector3d(0,0,0);
		if (coordinatesScale)
			*coordinatesScale = 1.0;
		return false;
	}

	//default scale
	double scale = (coordinatesScale ? std::max(*coordinatesScale,ZERO_TOLERANCE) : 1.0);

	bool needShift =	fabs(P[0]) >= MAX_COORDINATE_ABS_VALUE
					||	fabs(P[1]) >= MAX_COORDINATE_ABS_VALUE
					||	fabs(P[2]) >= MAX_COORDINATE_ABS_VALUE;

	bool needRescale = diagonal > MAX_DIAGONAL_LENGTH;

	//is shift necessary?
	if ( needShift || needRescale || forceDialogDisplay )
	{
		//coordinates transformation information already provided? (typically from a previous entity)
		if (useInputCoordinatesShiftIfPossible)
		{
			//either we are in non interactive mode (which means that shift is 'forced' by caller)
			if (!displayDialogIfNecessary
				//or we are in interactive mode and existing shift is pertinent
				|| (fabs(P[0]*scale + coordinatesShift.x) < MAX_COORDINATE_ABS_VALUE
				&&  fabs(P[1]*scale + coordinatesShift.y) < MAX_COORDINATE_ABS_VALUE
				&&  fabs(P[2]*scale + coordinatesShift.z) < MAX_COORDINATE_ABS_VALUE
				&&  diagonal*scale < MAX_DIAGONAL_LENGTH
				&&	!forceDialogDisplay) )
			{
				//user should use the provided shift information
				return true;
			}
			else
			{
				//--> otherwise we (should) ask for a better one
			}
		}

		//let's ask the user for those values
		assert(displayDialogIfNecessary);

		ccShiftAndScaleCloudDlg sasDlg(P,diagonal);
		if (!applyAll)
			sasDlg.showApplyAllButton(false);
		if (!coordinatesScale)
			sasDlg.showScaleItems(false);
		//shift on load already provided? (typically from a previous file)
		if (useInputCoordinatesShiftIfPossible)
		{
			sasDlg.setShift(coordinatesShift);
			if (coordinatesScale)
				sasDlg.setScale(*coordinatesScale);
			sasDlg.showWarning(true); //if we are here, it means that the provided shift isn't concordant
		}
		else
		{
			if (needShift)
			{
				sasDlg.setShift(CCVector3d(	-P[0],-P[1],fabs(P[2]) >= MAX_COORDINATE_ABS_VALUE ? -P[2] : 0)); //Z is generally the altitude and is not so big than the others!
				sasDlg.firstPointFrame->setStyleSheet("color: red;");
			}
			if (needRescale)
			{
				sasDlg.setScale(diagonal > MAX_COORDINATE_ABS_VALUE ? pow(10.0,-static_cast<double>(ceil(log(diagonal/MAX_DIAGONAL_LENGTH)))) : 1.0);
				sasDlg.scaleInfoFrame->setStyleSheet("color: red;");
			}
		}

		if (sasDlg.exec())
		{
			coordinatesShift = sasDlg.getShift();
			if (coordinatesScale)
				*coordinatesScale = sasDlg.getScale();
			if (applyAll)
				*applyAll = sasDlg.applyAll();
			return true;
		}
	}

	coordinatesShift = CCVector3d(0,0,0);
	if (coordinatesScale)
		*coordinatesScale = 1.0;
	return false;
}
