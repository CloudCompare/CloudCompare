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

#ifndef CC_TORUS_PRIMITIVE_HEADER
#define CC_TORUS_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"


//! Torus (primitive)
/** 3D torus primitive (with circular or rectangular section)
**/
class QCC_DB_LIB_API ccTorus : public ccGenericPrimitive
{
public:

	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

	//! Default constructor
	/** Torus is defined in the XY plane by default
		\param insideRadius inside radius
		\param outsideRadius outside radius
		\param angle_rad subtended angle (in radians)
		\param rectangularSection whether section is rectangular or round
		\param rectSectionHeight section height (if rectangular torus)
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (main loop angular step = 360/(4*precision), circular section angular step = 360/precision)
		\param uniqueID unique ID (handle with care)
	**/
	ccTorus(PointCoordinateType insideRadius,
			PointCoordinateType outsideRadius,
			double angle_rad = 2.0*M_PI,
			bool rectangularSection = false,
			PointCoordinateType rectSectionHeight = 0,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Torus"),
			unsigned precision = DEFAULT_DRAWING_PRECISION,
			unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccTorus(QString name = QString("Torus"));

	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::TORUS; }

	//inherited from ccGenericPrimitive
	QString getTypeName() const override { return "Torus"; }
	bool hasDrawingPrecision() const override { return true; }
	ccGenericPrimitive* clone() const override;

protected:

	//inherited from ccGenericPrimitive
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	bool buildUp() override;

	//! Inside radius
	PointCoordinateType m_insideRadius;

	//! Outisde radius
	PointCoordinateType m_outsideRadius;

	//! Whether torus has a rectangular (true) or circular (false) section
	bool m_rectSection;

	//! Rectangular section height (along Y-axis) if applicable
	PointCoordinateType m_rectSectionHeight;

	//! Subtended angle (in radians)
	double m_angle_rad;
};

#endif //CC_TORUS_PRIMITIVE_HEADER
