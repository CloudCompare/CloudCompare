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

#ifndef CC_CROP_TOOL_HEADER
#define CC_CROP_TOOL_HEADER

//qCC_db
#include <ccBBox.h>

class ccHObject;
class ccGLMatrix;

//! Cropping tool
/** Handles clouds and meshes for now
**/
class ccCropTool
{
public:
	//! Crops the input entity
	/** \param entity entity to be cropped (should be a cloud or a mesh)
		\param box cropping box
		\param inside whether to keep the points/triangles inside or outside the input box
		\param meshRotation optional rotation (for meshes only)
		\return cropped entity (if any)
	**/
	static ccHObject* Crop(ccHObject* entity, const ccBBox& box, bool inside = true, const ccGLMatrix* meshRotation = 0);

};

#endif //CC_CROP_TOOL_HEADER
