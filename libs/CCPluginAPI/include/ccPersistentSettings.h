#pragma once
// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Qt
#include <QString>

//! Persistent settings key (to be used with QSettings)
namespace ccPS
{
	inline QString LoadFile()
	{
		return QStringLiteral("LoadFile");
	}
	inline QString SaveFile()
	{
		return QStringLiteral("SaveFile");
	}
	inline QString MainWinGeom()
	{
		return QStringLiteral("mainWindowGeometry");
	}
	inline QString MainWinState()
	{
		return QStringLiteral("mainWindowState");
	}
	inline QString DoNotRestoreWindowGeometry()
	{
		return QStringLiteral("doNotRestoreWindowGeometry");
	}
	inline QString AppStyle()
	{
		return QStringLiteral("AppStyle");
	}
	inline QString CurrentPath()
	{
		return QStringLiteral("currentPath");
	}
	inline QString SelectedInputFilter()
	{
		return QStringLiteral("selectedInputFilter");
	}
	inline QString SelectedOutputFilterCloud()
	{
		return QStringLiteral("selectedOutputFilterCloud");
	}
	inline QString SelectedOutputFilterMesh()
	{
		return QStringLiteral("selectedOutputFilterMesh");
	}
	inline QString SelectedOutputFilterImage()
	{
		return QStringLiteral("selectedOutputFilterImage");
	}
	inline QString SelectedOutputFilterPoly()
	{
		return QStringLiteral("selectedOutputFilterPoly");
	}
	inline QString DuplicatePointsGroup()
	{
		return QStringLiteral("duplicatePoints");
	}
	inline QString DuplicatePointsMinDist()
	{
		return QStringLiteral("minDist");
	}
	inline QString HeightGridGeneration()
	{
		return QStringLiteral("HeightGridGeneration");
	}
	inline QString VolumeCalculation()
	{
		return QStringLiteral("VolumeCalculation");
	}
	inline QString Console()
	{
		return QStringLiteral("Console");
	}
	inline QString GlobalShift()
	{
		return QStringLiteral("GlobalShift");
	}
	inline QString MaxAbsCoord()
	{
		return QStringLiteral("MaxAbsCoord");
	}
	inline QString MaxAbsDiag()
	{
		return QStringLiteral("MaxAbsDiag");
	}
	inline QString AutoPickRotationCenter()
	{
		return QStringLiteral("AutoPickRotationCenter");
	}
	inline QString View3dRotationAxisLocked()
	{
		return QStringLiteral("View3dRotationAxisLocked");
	}
	inline QString View3dLockedAxisRotation()
	{
		return QStringLiteral("View3dLockedAxisRotation");
	}
	inline QString Options()
	{
		return QStringLiteral("Options");
	}
	inline QString Plugins()
	{
		return QStringLiteral("Plugins");
	}
	inline QString Translation()
	{
		return QStringLiteral("Translation");
	}
	inline QString Shortcuts()
	{
		return QStringLiteral("Shortcuts");
	}
}; // namespace ccPS
