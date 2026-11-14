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
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "qCC_db.h"

// CCCoreLib
#include <ccGLMatrix.h>

//! Exposure compensation from the projected bounding box, independent of Qt/OpenGL.
struct QCC_DB_LIB_API ccXRayExposure
{
	//! Computes the projected area ratio of a 3D box in normalized device coordinates.
	/** Matrices are column-major, as in OpenGL. Extents are deliberately not clipped
	    to the viewport: panning must not change exposure, even when the box is offscreen.
	**/
	static double ProjectedAreaRatio(const CCVector3d&  minimum,
	                                 const CCVector3d&  maximum,
	                                 const ccGLMatrixd& modelView,
	                                 const ccGLMatrixd& projection);

	//! Computes the strength of X-ray points from the projected area ratio and exposure.
	static float Strength(double areaRatio, double exposure);
};
