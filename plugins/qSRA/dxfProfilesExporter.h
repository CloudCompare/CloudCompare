//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#ifndef QSRA_DXF_PROFILES_EXPORTER_HEADER
#define QSRA_DXF_PROFILES_EXPORTER_HEADER

//Local
#include "distanceMapGenerationTool.h"

//Qt
#include <QSharedPointer>
#include <QString>

class ccPolyline;
class ccMainAppInterface;

//! DXF profiles (of a surface of revolution) exporter
/** Requires DXF lib support!
**/
class DxfProfilesExporter
{
public:

	//! Returns whether DXF support is enabled or not
	static bool IsEnabled();

	struct Parameters
	{
		QStringList profileTitles;
		QString legendTheoProfileTitle;
		QString legendRealProfileTitle;
		QString scaledDevUnits;
		double devLabelMultCoef;
		double devMagnifyCoef;
		int precision;
	};

	//! Exports vertical profiles (of a surface of revolution's map) as a DXF file
	static bool SaveVerticalProfiles(	const QSharedPointer<DistanceMapGenerationTool::Map>& map,
										ccPolyline* profile,
										QString filename,
										unsigned angularStepCount,
										double heightStep,
										double heightShift,
										const Parameters& params,
										ccMainAppInterface* app = 0 );

	//! Exports horizontal profiles (of a surface of revolution's map) as a DXF file
	static bool SaveHorizontalProfiles(	const QSharedPointer<DistanceMapGenerationTool::Map>& map,
										ccPolyline* profile,
										QString filename,
										unsigned heightStepCount,
										double heightShift,
										double angularStep_rad,
										double radToUnitConvFactor,
										QString angleUnit,
										const Parameters& params,
										ccMainAppInterface* app = 0 );
};

#endif //QSRA_DXF_PROFILES_EXPORTER_HEADER

