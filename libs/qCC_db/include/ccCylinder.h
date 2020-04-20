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

#ifndef CC_CYLINDER_PRIMITIVE_HEADER
#define CC_CYLINDER_PRIMITIVE_HEADER

//Local
#include "ccCone.h"

//! Cylinder (primitive)
/** 3D cylinder primitive
**/
class QCC_DB_LIB_API ccCylinder : public ccCone
{
public:

	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

	//! Default constructor
	/** Cylinder axis corresponds to the 'Z' dimension.
		Internally represented by a cone with the same top and bottom radius.
		\param radius cylinder radius
		\param height cylinder height (transformation should point to the axis center)
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
		\param uniqueID unique ID (handle with care)
	**/
	ccCylinder(	PointCoordinateType radius,
				PointCoordinateType height,
				const ccGLMatrix* transMat = 0,
				QString name = QString("Cylinder"),
				unsigned precision = DEFAULT_DRAWING_PRECISION,
				unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCylinder(QString name = QString("Cylinder"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CYLINDER; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Cylinder"; }
	virtual ccGenericPrimitive* clone() const override;

	//inherited from ccCone
	virtual void setBottomRadius(PointCoordinateType radius) override;
	inline virtual void setTopRadius(PointCoordinateType radius) override { return setBottomRadius(radius); }

};

#endif //CC_CYLINDER_PRIMITIVE_HEADER
