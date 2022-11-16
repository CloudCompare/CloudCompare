#pragma once
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

//Qt
#include <QString>

//! Persistent settings key (to be used with QSettings)
namespace ccPS
{	
	inline const QString LoadFile                    () { return QStringLiteral( "LoadFile" ); }
	inline const QString SaveFile                    () { return QStringLiteral( "SaveFile" ); }
	inline const QString MainWinGeom                 () { return QStringLiteral( "mainWindowGeometry" ); }
	inline const QString MainWinState                () { return QStringLiteral( "mainWindowState" ); }
	inline const QString DoNotRestoreWindowGeometry  () { return QStringLiteral( "doNotRestoreWindowGeometry" ); }
	inline const QString AppStyle                    () { return QStringLiteral( "AppStyle" ); }
	inline const QString CurrentPath                 () { return QStringLiteral( "currentPath" ); }
	inline const QString SelectedInputFilter         () { return QStringLiteral( "selectedInputFilter" ); }
	inline const QString SelectedOutputFilterCloud   () { return QStringLiteral( "selectedOutputFilterCloud" ); }
	inline const QString SelectedOutputFilterMesh    () { return QStringLiteral( "selectedOutputFilterMesh" ); }
	inline const QString SelectedOutputFilterImage   () { return QStringLiteral( "selectedOutputFilterImage" ); }
	inline const QString SelectedOutputFilterPoly    () { return QStringLiteral( "selectedOutputFilterPoly" ); }
	inline const QString DuplicatePointsGroup        () { return QStringLiteral( "duplicatePoints" ); }
	inline const QString DuplicatePointsMinDist      () { return QStringLiteral( "minDist" ); }
	inline const QString HeightGridGeneration        () { return QStringLiteral( "HeightGridGeneration" ); }
	inline const QString VolumeCalculation           () { return QStringLiteral( "VolumeCalculation" ); }
	inline const QString Console                     () { return QStringLiteral( "Console" ); }
	inline const QString GlobalShift                 () { return QStringLiteral( "GlobalShift" ); }
	inline const QString MaxAbsCoord                 () { return QStringLiteral( "MaxAbsCoord" ); }
	inline const QString MaxAbsDiag                  () { return QStringLiteral( "MaxAbsDiag" ); }
	inline const QString AutoPickRotationCenter      () { return QStringLiteral( "AutoPickRotationCenter" ); }
	inline const QString Options                     () { return QStringLiteral( "Options" ); }
	inline const QString Plugins                     () { return QStringLiteral( "Plugins" ); }
	inline const QString Translation                 () { return QStringLiteral( "Translation" ); }
};
