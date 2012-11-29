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
//$Author:: dgm                                                            $
//$Rev:: 2274                                                              $
//$LastChangedDate:: 2012-10-17 19:17:38 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_HEIGHT_GRID_GENERATION
#define CC_HEIGHT_GRID_GENERATION

// Includes CClib
#include <GenericProgressCallback.h>

class ccGenericPointCloud;
class ccPointCloud;

class ccHeightGridGeneration
{
public:

    //! Types of projection
    enum ProjectionType {   PROJ_MINIMUM_HEIGHT			= 0,
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

    //! Computs height grid
    static void Compute(ccGenericPointCloud* cloud,
                        float grid_step,
						unsigned char proj_dimension,
                        ProjectionType type_of_projection,
                        EmptyCellFillOption fillEmptyCells = LEAVE_EMPTY,
						ProjectionType sfInterpolation = INVALID_PROJECTION_TYPE,
                        double customEmptyCellsHeight = -1.0,
                        bool generateImage = true,
                        bool generateASCII = false,
                        ccPointCloud* cloudGrid=0,
                        CCLib::GenericProgressCallback* progressCb=0);
};

#endif
