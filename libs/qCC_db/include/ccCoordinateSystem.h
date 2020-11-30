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
//#          COPYRIGHT: Chris Brown                                        #
//#                                                                        #
//##########################################################################

#pragma once

//Local
#include "ccGenericPrimitive.h"

//! Coordinate System (primitive)
/** Coordinate System primitive
**/
class QCC_DB_LIB_API ccCoordinateSystem : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Coordinate System is essentially just a way to visualize a transform matrix.
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccCoordinateSystem(PointCoordinateType displayScale, PointCoordinateType axisWidth, const ccGLMatrix* transMat = 0,
		QString name = QString("CoordinateSystem"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCoordinateSystem(QString name = QString("CoordinateSystem"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::COORDINATESYSTEM; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "CoordinateSystem"; }
	virtual ccGenericPrimitive* clone() const override;

protected:

	//inherited from ccDrawable
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual bool buildUp() override;

	//! CoordinateSystem options
	PointCoordinateType m_DisplayScale;
	PointCoordinateType m_width;

	


};


