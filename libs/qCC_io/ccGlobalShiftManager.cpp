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

//Local
#include "ccGlobalShiftManager.h"
#include "ccShiftAndScaleCloudDlg.h"

//qCC_db
#include <ccHObject.h>

//System
#include <string.h>
#include <assert.h>

double ccGlobalShiftManager::MAX_COORDINATE_ABS_VALUE = 1.0e4;
double ccGlobalShiftManager::MAX_DIAGONAL_LENGTH = 1.0e6;

//semi-persistent settings
struct LastShiftInfo : ccGlobalShiftManager::ShiftInfo
{
	bool valid;

	LastShiftInfo()
		: ccGlobalShiftManager::ShiftInfo("Last input")
		, valid(false)
	{}
};
static LastShiftInfo s_lastInfo;

bool ccGlobalShiftManager::GetLast(ShiftInfo& info)
{
	info = s_lastInfo;
	return s_lastInfo.valid;
}

bool ccGlobalShiftManager::NeedShift(const CCVector3d& P)
{
	return	NeedShift(P.x) || NeedShift(P.y) || NeedShift(P.z);
}

bool ccGlobalShiftManager::NeedShift(double d)
{
	return fabs(d) >= MAX_COORDINATE_ABS_VALUE;
}

bool ccGlobalShiftManager::NeedRescale(double d)
{
	return fabs(d) >= MAX_DIAGONAL_LENGTH;
}

bool ccGlobalShiftManager::Handle(	const CCVector3d& P,
									double diagonal,
									Mode mode,
									bool useInputCoordinatesShiftIfPossible,
									CCVector3d& coordinatesShift,
									double* coordinatesScale,
									bool* applyAll/*=0*/)
{
	assert(diagonal >= 0);

	if (applyAll)
	{
		*applyAll = false;
	}

	//default scale
	double scale = (coordinatesScale ? std::max(*coordinatesScale, ZERO_TOLERANCE) : 1.0);

	bool needShift = NeedShift(P);
	bool needRescale = NeedRescale(diagonal);

	//if we can't display a dialog and no usable shift is specified, there's nothing we can do...
	if (mode == NO_DIALOG && !useInputCoordinatesShiftIfPossible)
	{
		coordinatesShift = CCVector3d(0, 0, 0);
		if (coordinatesScale)
		{
			*coordinatesScale = 1.0;
		}

		if (needShift || needRescale)
		{
			ccLog::Warning("[ccGlobalShiftManager] Entity has very big coordinates: original accuracy may be lost! (you should apply a Global Shift or Scale)");
		}

		return false;
	}

	//is shift necessary?
	if ( needShift || useInputCoordinatesShiftIfPossible || needRescale || mode == ALWAYS_DISPLAY_DIALOG )
	{
		//shift information already provided? (typically from a previous entity)
		if (useInputCoordinatesShiftIfPossible && mode != ALWAYS_DISPLAY_DIALOG)
		{
			//either we are in non interactive mode (which means that shift is 'forced' by caller)
			if (mode == NO_DIALOG
				//or we are in interactive mode and existing shift is pertinent
				|| (!NeedShift(P*scale + coordinatesShift) && !NeedRescale(diagonal*scale)) )
			{
				//do we have already stored shift info?
				if (mode == NO_DIALOG_AUTO_SHIFT && s_lastInfo.valid)
				{
					//in "auto shift" mode, we may want to use it (to synchronize multiple clouds!)
					if (!NeedShift(P*s_lastInfo.scale + s_lastInfo.shift) && !NeedRescale(diagonal*s_lastInfo.scale))
					{
						coordinatesShift = s_lastInfo.shift;
						if (coordinatesScale)
						{
							*coordinatesScale = scale;
						}
						return true;
					}
				}
				
				//save info for next time
				if (coordinatesShift.norm2() != 0 || scale != 1.0)
				{
					s_lastInfo.valid = true;
					s_lastInfo.shift = coordinatesShift;
					s_lastInfo.scale = scale;
				}
				//user should use the provided shift information
				return true;
			}
			//--> otherwise we (should) ask for a better one
		}

		//let's deduce the right values (AUTO mode)
		if (mode == NO_DIALOG_AUTO_SHIFT)
		{
			//guess best shift & scale info from input point/diagonal
			if (needShift)
			{
				coordinatesShift = BestShift(P);

				//save info for next time
				s_lastInfo.valid = true;
				s_lastInfo.shift = coordinatesShift;
				s_lastInfo.scale = 1.0;
			}
			if (coordinatesScale && needRescale)
			{
				*coordinatesScale = BestScale(diagonal);

				//save info for next time
				if (!s_lastInfo.valid)
				{
					s_lastInfo.shift = CCVector3d(0, 0, 0);
				}
				s_lastInfo.valid = true;
				s_lastInfo.scale = *coordinatesScale;
			}

			return true;
		}

		//otherwise let's ask the user for those values
		ccShiftAndScaleCloudDlg sasDlg(P, diagonal);
		if (!applyAll)
		{
			sasDlg.showApplyAllButton(false);
		}
		if (!coordinatesScale)
		{
			sasDlg.showScaleItems(false);
		}

		scale = 1.0;
		CCVector3d shift(0, 0, 0);
		if (useInputCoordinatesShiftIfPossible)
		{
			//shift on load already provided? (typically from a previous file)
			shift = coordinatesShift;
			if (coordinatesScale)
			{
				scale = *coordinatesScale;
			}
			if (mode != ALWAYS_DISPLAY_DIALOG)
			{
				sasDlg.showWarning(true); //if we are here, it means that the provided shift isn't concordant
			}
		}
		else
		{
			//guess best shift & scale info from input point/diagonal
			if (needShift)
			{
				shift = BestShift(P);
			}
			if (needRescale)
			{
				scale = BestScale(diagonal);
			}
		}

		//add "suggested" entry
		int index = sasDlg.addShiftInfo(ShiftInfo("Suggested", shift, scale));
		sasDlg.setCurrentProfile(index);
		//add "last" entry (if available)
		if (s_lastInfo.valid)
		{
			sasDlg.addShiftInfo(s_lastInfo);
		}
		//add entries from file (if any)
		sasDlg.addFileInfo();
		
		//automatically make the first available shift that works
		//(different than the suggested one) active
		{
			for (size_t i = static_cast<size_t>(std::max(0, index + 1)); i < sasDlg.infoCount(); ++i)
			{
				ShiftInfo info;
				if (sasDlg.getInfo(i, info))
				{
					//check if they work
					if (	!NeedShift((CCVector3d(P) + info.shift) * info.scale )
						&&  !NeedRescale(diagonal*info.scale) )
					{
						sasDlg.setCurrentProfile(static_cast<int>(i));
						break;
					}
				}
			}
		}
		sasDlg.showTitle(needShift || needRescale);
		if (sasDlg.exec())
		{
			//save info for next time
			s_lastInfo.valid = true;
			s_lastInfo.shift = sasDlg.getShift();
			s_lastInfo.scale = sasDlg.getScale();

			coordinatesShift = sasDlg.getShift();
			if (coordinatesScale)
			{
				*coordinatesScale = sasDlg.getScale();
			}
			if (applyAll)
			{
				*applyAll = sasDlg.applyAll();
			}
			
			return true;
		}
	}

	coordinatesShift = CCVector3d(0, 0, 0);
	if (coordinatesScale)
	{
		*coordinatesScale = 1.0;
	}

	return false;
}

CCVector3d ccGlobalShiftManager::BestShift(const CCVector3d& P)
{
	if (!NeedShift(P))
	{
		return CCVector3d(0, 0, 0);
	}

	CCVector3d shift(	fabs(P[0]) >= MAX_COORDINATE_ABS_VALUE ? -P[0] : 0,
						fabs(P[1]) >= MAX_COORDINATE_ABS_VALUE ? -P[1] : 0,
						fabs(P[2]) >= MAX_COORDINATE_ABS_VALUE ? -P[2] : 0 );

	//round-off to the nearest hundred
	shift.x = static_cast<int>(shift.x / 100) * 100.0;
	shift.y = static_cast<int>(shift.y / 100) * 100.0;
	shift.z = static_cast<int>(shift.z / 100) * 100.0;

	return shift;
}

double ccGlobalShiftManager::BestScale(double d)
{
	return d < MAX_DIAGONAL_LENGTH ? 1.0 : pow(10.0, -static_cast<double>(ceil(log(d / MAX_DIAGONAL_LENGTH))));
}

