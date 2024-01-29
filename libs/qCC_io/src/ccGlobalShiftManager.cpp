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

//Qt
#include <QCoreApplication>
#include <QFile>

//qCC_db
#include <ccHObject.h>

//System
#include <string.h>
#include <assert.h>

double ccGlobalShiftManager::MAX_COORDINATE_ABS_VALUE = 1.0e4;
double ccGlobalShiftManager::MAX_DIAGONAL_LENGTH = 1.0e6;

// default name for the Global Shift 'bookmarks' file
static QString s_defaultGlobalShiftListFilename("global_shift_list.txt");

// default and last input shift/scale entries (don't use it directly, use GetLast() instead)
static std::vector<ccGlobalShiftManager::ShiftInfo> s_lastInfoBuffer;
const std::vector<ccGlobalShiftManager::ShiftInfo>& ccGlobalShiftManager::GetLast()
{
	// the first time this method is called, load the default values from the 'bookmark' files
	static bool s_firstTime = true;
	if (s_firstTime)
	{
		LoadInfoFromFile(QCoreApplication::applicationDirPath() + QString("/") + s_defaultGlobalShiftListFilename, s_lastInfoBuffer);
		s_firstTime = false;
	}

	return s_lastInfoBuffer;
}

static bool IsDefaultShift(const CCVector3d& shift, double scale)
{
	return (scale == 1.0 && shift.norm2d() == 0);
}

static bool SameShift(const ccGlobalShiftManager::ShiftInfo& shiftInfo, const CCVector3d& shift, double scale)
{
	return (	(shiftInfo.shift - shift).norm() <= CCCoreLib::ZERO_TOLERANCE_D
			&&	std::abs(shiftInfo.scale - scale) <= CCCoreLib::ZERO_TOLERANCE_D );
}

void ccGlobalShiftManager::StoreShift(const CCVector3d& shift, double scale, bool preserve/*=true*/)
{
	if (IsDefaultShift(shift, scale))
	{
		// default shift and scale are ignored
		return;
	}

	// check if it's already stored
	for (const ShiftInfo& shiftInfo : s_lastInfoBuffer)
	{
		if (SameShift(shiftInfo, shift, scale))
		{
			//we already know this one
			return;
		}
	}

	static unsigned lastInputIndex = 0;
	ShiftInfo info("Previous input");
	if (lastInputIndex != 0)
	{
		info.name += QString(" (%1)").arg(lastInputIndex);
	}
	++lastInputIndex;

	info.scale = scale;
	info.shift = shift;
	info.preserve = preserve;
	s_lastInfoBuffer.emplace_back(info);
}

bool ccGlobalShiftManager::NeedShift(const CCVector3d& P)
{
	return	NeedShift(P.x) || NeedShift(P.y) || NeedShift(P.z);
}

bool ccGlobalShiftManager::NeedShift(double d)
{
	return std::abs(d) >= MAX_COORDINATE_ABS_VALUE;
}

bool ccGlobalShiftManager::NeedRescale(double d)
{
	return std::abs(d) >= MAX_DIAGONAL_LENGTH;
}

static bool ShiftAndScaleAreSimilar(double scale1,
									const CCVector3d& shift1,
									double scale2,
									const CCVector3d& shift2)
{
	return ((shift1 - shift2).norm() <= CCCoreLib::ZERO_TOLERANCE_D) && (std::abs(scale1 - scale2) <= CCCoreLib::ZERO_TOLERANCE_D);
}

bool ccGlobalShiftManager::Handle(	const CCVector3d& P,
									double diagonal,
									Mode mode,
									bool useInputCoordinatesShiftIfPossible,
									CCVector3d& coordinatesShift,
									bool* _preserveCoordinateShift/*=nullptr*/,
									double* _coordinatesScale/*=nullptr*/,
									bool* _applyAll/*=nullptr*/)
{
	assert(diagonal >= 0.0);
	bool preserveCoordinateShift = true;
	if (useInputCoordinatesShiftIfPossible && _preserveCoordinateShift)
	{
		// if shift info was provided as input
		preserveCoordinateShift = *_preserveCoordinateShift;
	}
	if (_applyAll)
	{
		*_applyAll = false;
	}

	//default scale
	double scale = 1.0;

	bool needShift = false;
	bool needRescale = false;

	// if shift info was provided as input (typically from a previous entity)
	bool canUseInputCoordinatesShift = false;
	CCVector3d inputCoordinatesShift(0, 0, 0);
	double inputScale = 1.0;
	if (useInputCoordinatesShiftIfPossible)
	{
		if (nullptr != _coordinatesScale)
		{
			// use the input scale if specified
			*_coordinatesScale = std::max(*_coordinatesScale, CCCoreLib::ZERO_TOLERANCE_D);
			scale = *_coordinatesScale;
		}

		inputCoordinatesShift = coordinatesShift;
		inputScale = scale;

		if (mode == NO_DIALOG)
		{
			// without a dialog, we don't have the choice, we will use the input shift...
			//canUseInputCoordinatesShift = true;
			return true;
		}
		else
		{
			needShift = NeedShift(P*scale + coordinatesShift);
			needRescale = NeedRescale(diagonal*scale);

			canUseInputCoordinatesShift = (!needShift && !needRescale);
		}
	}
	else
	{
		// no shift was provided
		coordinatesShift = CCVector3d(0, 0, 0);
		if (nullptr != _coordinatesScale)
		{
			*_coordinatesScale = 1.0;
		}

		if (mode == NO_DIALOG)
		{
			// it would be a bit strange to call this method with NO_DIALOG and no input shift
			// but that's theoretically possible ;)
			return false;
		}

		needShift = NeedShift(P);
		needRescale = NeedRescale(diagonal);

		if (	!needShift
			&&	!needRescale
			&&	mode != ALWAYS_DISPLAY_DIALOG)
		{
			return false; //no need to apply any shift
		}
	}

	// after this point, we should either determine a new global shift
	// or ask the user to review/provide one with a dialog
	assert(mode != NO_DIALOG);

	// if necessary, we can try with if a previously used shift works
	int bestPreviousShiftIndex = -1;
	if (needShift || needRescale)
	{
		preserveCoordinateShift = true; // if we fall here, this means that even the provided input doesn't work
		if (mode != NO_DIALOG_AUTO_SHIFT)
		{
			// at this point, we will display the dialog since the input info doesn't work
			mode = ALWAYS_DISPLAY_DIALOG;
		}
	}

	const std::vector<ShiftInfo>& lastInfoBuffer = ccGlobalShiftManager::GetLast();

	if (needShift || needRescale || mode == ALWAYS_DISPLAY_DIALOG)
	{
		// try to find an already used Global Shift that would work
		for (size_t i = 0; i < lastInfoBuffer.size(); ++i)
		{
			const ShiftInfo& shiftInfo = lastInfoBuffer[i];
			bool tempNeedShift = NeedShift(P*shiftInfo.scale + shiftInfo.shift);
			bool tempNeedRescale = NeedRescale(diagonal*shiftInfo.scale);
			if (!tempNeedShift && !tempNeedRescale)
			{
				// we found a valid candidate
				bestPreviousShiftIndex = static_cast<int>(i);
				if (!canUseInputCoordinatesShift)
				{
					// use this shift & scale if the input one was not working
					coordinatesShift = shiftInfo.shift;
					scale = shiftInfo.scale;
					needShift = tempNeedShift;
					needRescale = tempNeedRescale;
				}
				break;
			}
		}
	}

	if (mode == NO_DIALOG_AUTO_SHIFT)
	{
		// since we can't display a dialog, we'll just use the best shift found so far
		if (needShift)
		{
			coordinatesShift = BestShift(P);
		}
		if (needRescale)
		{
			scale = BestScale(diagonal);
		}
	}
	// should we still display the dialog?
	else if (	needShift
			||	needRescale
			||	mode == ALWAYS_DISPLAY_DIALOG)
	{
		ccShiftAndScaleCloudDlg sasDlg(P, diagonal);
		sasDlg.showApplyAllButton(_applyAll != nullptr);
		sasDlg.showScaleItems(_coordinatesScale != nullptr);
		sasDlg.showWarning(needShift || needRescale);
		sasDlg.setPreserveShiftOnSave(preserveCoordinateShift);
		sasDlg.showPreserveShiftOnSave(_preserveCoordinateShift != nullptr);
		sasDlg.showTitle(needShift || needRescale);

		// always add the "suggested" entry
		CCVector3d suggestedShift = BestShift(P);
		double suggestedScale = BestScale(diagonal);
		int index = sasDlg.addShiftInfo(ShiftInfo("Automatic", suggestedShift, suggestedScale));

		// add the input shift if any, and if it's different from the others
		if (useInputCoordinatesShiftIfPossible)
		{
			bool alreadyRegistered = false;
			for (size_t i = 0; i < lastInfoBuffer.size(); ++i)
			{
				const ShiftInfo& shiftInfo = lastInfoBuffer[i];
				if (ShiftAndScaleAreSimilar(shiftInfo.scale, shiftInfo.shift, inputScale, inputCoordinatesShift))
				{
					// found the same shift
					alreadyRegistered = true;
					if (canUseInputCoordinatesShift && bestPreviousShiftIndex < 0)
					{
						// we want to select this shift as we didn't find any better
						bestPreviousShiftIndex = static_cast<int>(i);
					}
					break;
				}
			}

			if (!alreadyRegistered)
			{
				// we need to add it to the list (and use it by default)
				int addedEntryIndex = sasDlg.addShiftInfo(ShiftInfo("Input", coordinatesShift, scale));
				if (canUseInputCoordinatesShift)
				{
					index = addedEntryIndex;
				}
			}
		}
		
		// add the previous entries (if any)
		if (!lastInfoBuffer.empty())
		{
			size_t countBefore = sasDlg.infoCount();
			sasDlg.addShiftInfo(lastInfoBuffer);
			if (bestPreviousShiftIndex >= 0)
			{
				index = static_cast<int>(countBefore) + bestPreviousShiftIndex;
			}
		}
		sasDlg.setCurrentProfile(index);

		if (!sasDlg.exec())
		{
			// process cancelled by the user
			coordinatesShift = CCVector3d(0, 0, 0);
			if (nullptr != _coordinatesScale)
			{
				*_coordinatesScale = 1.0;
			}
			return false;
		}

		coordinatesShift = sasDlg.getShift();
		if (_coordinatesScale)
		{
			scale = sasDlg.getScale();
		}
		if (_preserveCoordinateShift)
		{
			preserveCoordinateShift = sasDlg.preserveShiftOnSave();
		}
		if (_applyAll)
		{
			*_applyAll = sasDlg.applyAll();
		}
	}

	// save info for next time
	StoreShift(coordinatesShift, scale, preserveCoordinateShift);

	if (_coordinatesScale)
	{
		*_coordinatesScale = scale;
	}
	if (_preserveCoordinateShift)
	{
		*_preserveCoordinateShift = preserveCoordinateShift;
	}

	return true;
}

CCVector3d ccGlobalShiftManager::BestShift(const CCVector3d& P)
{
	if (!NeedShift(P))
	{
		return CCVector3d(0, 0, 0);
	}
	
	CCVector3d shift(	std::abs(P[0]) >= MAX_COORDINATE_ABS_VALUE ? -P[0] : 0,
						std::abs(P[1]) >= MAX_COORDINATE_ABS_VALUE ? -P[1] : 0,
						std::abs(P[2]) >= MAX_COORDINATE_ABS_VALUE ? -P[2] : 0 );

	//round-off the shift value
	{
		//make sure the round off scale is not larger than the max coordinate value ;)
		int roundOffScalePower = 3;
		assert(MAX_COORDINATE_ABS_VALUE >= 1.0);
		while (pow(10.0, roundOffScalePower) > MAX_COORDINATE_ABS_VALUE)
		{
			if (--roundOffScalePower == 0)
				break;
		}

		double roundOffScale = pow(10.0, 1.0 * roundOffScalePower);
		shift.x = static_cast<int>(shift.x / roundOffScale) * roundOffScale;
		shift.y = static_cast<int>(shift.y / roundOffScale) * roundOffScale;
		shift.z = static_cast<int>(shift.z / roundOffScale) * roundOffScale;
	}
	
	return shift;
}

double ccGlobalShiftManager::BestScale(double d)
{
	return d < MAX_DIAGONAL_LENGTH ? 1.0 : pow(10.0, -static_cast<double>(ceil(log(d / MAX_DIAGONAL_LENGTH))));
}

bool ccGlobalShiftManager::LoadInfoFromFile(QString filename, std::vector<ShiftInfo>& infos)
{
	QFile file(filename);
	if (!file.open(QFile::Text | QFile::ReadOnly))
		return false;

	QTextStream stream(&file);
	unsigned lineNumber = 0;

	while (true)
	{
		//read next line
		QString line = stream.readLine();
		if (line.isEmpty())
			break;
		++lineNumber;

		if (line.startsWith("//"))
			continue;

		//split line in 5 items
		QStringList tokens = line.split(";", QString::SkipEmptyParts);
		if (tokens.size() != 5)
		{
			//invalid file
			ccLog::Warning(QString("[ccGlobalShiftManager::LoadInfoFromFile] File '%1' is malformed (5 items expected per line)").arg(filename));
			return false;
		}

		//decode items
		bool ok = true;
		unsigned errors = 0;
		ccGlobalShiftManager::ShiftInfo info;
		info.name = tokens[0].trimmed();
		info.shift.x = tokens[1].toDouble(&ok);
		if (!ok) ++errors;
		info.shift.y = tokens[2].toDouble(&ok);
		if (!ok) ++errors;
		info.shift.z = tokens[3].toDouble(&ok);
		if (!ok) ++errors;
		info.scale = tokens[4].toDouble(&ok);
		if (!ok) ++errors;

		//process errors
		if (errors)
		{
			//invalid file
			ccLog::Warning(QString("[ccGlobalShiftManager::LoadInfoFromFile] File '%1' is malformed (wrong item type encountered on line %2)").arg(filename).arg(lineNumber));
			return false;
		}

		try
		{
			infos.push_back(info);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			ccLog::Warning(QString("[ccGlobalShiftManager::LoadInfoFromFile] Not enough memory to read file '%1'").arg(filename));
			return false;
		}
	}

	return true;
}

