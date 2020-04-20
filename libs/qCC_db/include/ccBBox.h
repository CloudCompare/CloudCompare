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

#ifndef CC_BBOX_HEADER
#define CC_BBOX_HEADER

//Local
#include "ccDrawableObject.h"

//CCLib
#include <BoundingBox.h>

//! Bounding box structure
/** Supports several operators such as addition (to a matrix or a vector) and
	multiplication (by a matrix or a scalar).
**/
class QCC_DB_LIB_API ccBBox : public CCLib::BoundingBox
{
public:

	//! Default constructor
	ccBBox() : CCLib::BoundingBox() {}
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCVector3& bbMinCorner, const CCVector3& bbMaxCorner) : CCLib::BoundingBox(bbMinCorner, bbMaxCorner) {}
	//! Constructor from two vectors (lower min. and upper max. corners)
	ccBBox(const CCLib::BoundingBox& bbox) : CCLib::BoundingBox(bbox) {}

	//! Applies transformation to the bounding box
	const ccBBox operator * (const ccGLMatrix& mat);
	//! Applies transformation to the bounding box
	const ccBBox operator * (const ccGLMatrixd& mat);

	//! Draws bounding box (OpenGL)
	/** \param context OpenGL context
	 *  \param col (R,G,B) color
	**/
	void draw(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) const;
};

#endif //CC_BBOX_HEADER
