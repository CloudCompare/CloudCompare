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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//
#ifndef CC_COORDINATES_SHIFT_MANAGER_HEADER
#define CC_COORDINATES_SHIFT_MANAGER_HEADER

//Local
#include "ccShiftAndScaleCloudDlg.h"

//CCLib
#include <CCGeom.h>

// Max acceptable coordinate value
#define MAX_COORDINATE_ABS_VALUE ((PointCoordinateType)1.0e6)

//! Helper class to handle coordinates shift while loading entities (GUI, etc.)
class ccCoordinatesShiftManager
{
public:

	//! Handles coordinates shift/scale given the first 3D point and current related parameters
	static bool Handle(const double* P,
						double diagonal,
						bool alwaysDisplayLoadDialog,
						bool coordinatesTransformationEnabled,
						double* coordinatesShift,
						double* coordinatesScale,
						bool& applyAll)
	{
		assert(P && coordinatesShift);
		assert(diagonal>=0.0);
		
		applyAll=false;

		//if we can't display a dialog and no shift is specified, there's nothing we can do...
		if (!alwaysDisplayLoadDialog && !coordinatesTransformationEnabled)
		{
			memset(coordinatesShift,0,sizeof(double)*3);
			if (coordinatesScale)
				*coordinatesScale=1.0;
			return false;
		}

		//default scale
		double scale = (coordinatesScale ? *coordinatesScale : 1.0);

		//is shift necessary?
		if (   fabs(P[0]) >= MAX_COORDINATE_ABS_VALUE
			|| fabs(P[1]) >= MAX_COORDINATE_ABS_VALUE
			|| fabs(P[2]) >= MAX_COORDINATE_ABS_VALUE
			|| diagonal > MAX_COORDINATE_ABS_VALUE )
		{
			//coordinates transformation information already provided? (typically from a precedent entity)
			if (coordinatesTransformationEnabled)
			{
				//either we are in non interactive mode (which means that shift is 'forced' by caller)
				if (!alwaysDisplayLoadDialog
					//or we are in interactive mode and existing shift is pertinent
					|| (fabs(P[0]*scale+coordinatesShift[0]) < MAX_COORDINATE_ABS_VALUE
					&&  fabs(P[1]*scale+coordinatesShift[1]) < MAX_COORDINATE_ABS_VALUE
					&&  fabs(P[2]*scale+coordinatesShift[2]) < MAX_COORDINATE_ABS_VALUE
					&&  diagonal*scale < MAX_COORDINATE_ABS_VALUE ))
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
			assert(alwaysDisplayLoadDialog);

			ccShiftAndScaleCloudDlg sasDlg(P,diagonal);
			if (!coordinatesScale)
				sasDlg.showScaleItems(false);
			//shift on load already provided? (typically from a precedent file)
			if (coordinatesTransformationEnabled)
			{
				sasDlg.setShift(coordinatesShift[0],coordinatesShift[1],coordinatesShift[2]);
				if (coordinatesScale)
					sasDlg.setScale(*coordinatesScale);
				sasDlg.showWarning(true); //if we are here, it means that the provided shift isn't concordant
			}
			else
			{
				sasDlg.setShift(-P[0],-P[1],-P[2]);
				sasDlg.setScale(diagonal > MAX_COORDINATE_ABS_VALUE ? pow(10.0,-(double)ceil(log(diagonal/MAX_COORDINATE_ABS_VALUE))) : 1.0);
			}

			if (sasDlg.exec())
			{
				sasDlg.getShift(coordinatesShift[0],coordinatesShift[1],coordinatesShift[2]);
				if (coordinatesScale)
					*coordinatesScale = sasDlg.getScale();
				applyAll = sasDlg.applyAll();
				return true;
			}
		}

		memset(coordinatesShift,0,sizeof(double)*3);
		if (coordinatesScale)
			*coordinatesScale = 1.0;
		return false;
	}
};

#endif