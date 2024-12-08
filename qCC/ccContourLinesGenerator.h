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

struct ccRasterGrid;
class ccPolyline;
class ccScalarField;
class QWidget;

//CCCoreLib
#include <CCGeom.h>

//system
#include <vector>

//! Contour lines generator
class ccContourLinesGenerator
{
public:

	//! Contour lines generation parameters
	struct Parameters
	{
		double startAltitude = 0.0;
		double maxAltitude = 0.0;
		double step = 0.0; //gap between levels
		ccScalarField* altitudes = nullptr; //optional scalar field that stores the 'altitudes' (may be null, in which case the grid 'h' values are used directly)
		int minVertexCount = 3; //minimum number of vertices per contour line
		bool projectContourOnAltitudes = false;
		double emptyCellsValue = std::numeric_limits<double>::quiet_NaN();

		/* The parameters below are only required if GDAL is not required */
		QWidget* parentWidget = nullptr; //for progress dialog
		bool ignoreBorders = false;

	};

	//! Generates contour lines
	/** \warning contour lines are always generated in the XY plane
	**/
	static bool GenerateContourLines(	ccRasterGrid* rasterGrid,
										const CCVector2d& gridMinCornerXY, //grid min corner (2D)
										const Parameters& params,
										std::vector<ccPolyline*>& contourLines);
};
