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

// Local
#include "ccDrawableObject.h"

// CCCoreLib
#include <BoundingBox.h>

//! Bounding box structure
/** Supports several operators such as addition (to a matrix or a vector) and
    multiplication (by a matrix or a scalar).
**/
class QCC_DB_LIB_API ccBBox : public CCCoreLib::BoundingBoxD
{
  public:
	//! Default constructor
	ccBBox()
	    : CCCoreLib::BoundingBoxD()
	{
	}
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCVector3d& bbMinCorner, const CCVector3d& bbMaxCorner, bool valid)
	    : CCCoreLib::BoundingBoxD(bbMinCorner, bbMaxCorner, valid)
	{
	}
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCCoreLib::BoundingBoxD& bbox)
	    : CCCoreLib::BoundingBoxD(bbox)
	{
	}

	//! Applies transformation to the bounding box
	ccBBox operator*(const ccGLMatrix& mat) const;
	//! Applies transformation to the bounding box
	ccBBox operator*(const ccGLMatrixd& mat) const;

	//! Draws bounding box (OpenGL)
	/** \param context OpenGL context
	 *  \param col (R,G,B) color
	 **/
	void draw(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) const;
};
