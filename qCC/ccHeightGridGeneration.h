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

#ifndef CC_HEIGHT_GRID_GENERATION
#define CC_HEIGHT_GRID_GENERATION

//CClib
#include <GenericProgressCallback.h>

//Qt
#include <QSharedPointer>

//qCC_db
#include <ccBBox.h>

class ccGenericPointCloud;
class ccPointCloud;

class ccHeightGridGeneration
{
public:

	//! Types of projection
	enum ProjectionType {	PROJ_MINIMUM_HEIGHT			= 0,
							PROJ_MAXIMUM_HEIGHT			= 1,
							PROJ_AVERAGE_HEIGHT			= 2,
							INVALID_PROJECTION_TYPE		= 255,
	};

	//! Option for handling empty cells
	enum EmptyCellFillOption {	LEAVE_EMPTY				= 0,
								FILL_MINIMUM_HEIGHT		= 1,
								FILL_MAXIMUM_HEIGHT		= 2,
								FILL_CUSTOM_HEIGHT		= 3,
								FILL_AVERAGE_HEIGHT		= 4,
	};

	//! Default constructor
	ccHeightGridGeneration();

	//! Computes height grid
	static ccPointCloud* Compute(	ccGenericPointCloud* cloud,
									double grid_step,
									const ccBBox& customBox,
									unsigned char proj_dimension,
									ProjectionType type_of_projection,
									EmptyCellFillOption fillEmptyCells = LEAVE_EMPTY,
									ProjectionType sfInterpolation = INVALID_PROJECTION_TYPE,
									double customEmptyCellsHeight = -1.0,
									bool generateCloud = true,
									bool generateImage = false,
									bool generateRaster = false,
									bool generateASCII = false,
									bool generateCountSF = false,
									bool resampleOriginalCloud = false,
									CCLib::GenericProgressCallback* progressCb=0);
};

#endif
