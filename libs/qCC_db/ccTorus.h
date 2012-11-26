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
//$Rev:: 1854                                                              $
//$LastChangedDate:: 2011-05-13 23:56:42 +0200 (ven., 13 mai 2011)         $
//**************************************************************************
//

#ifndef CC_TORUS_PRIMITIVE_HEADER
#define CC_TORUS_PRIMITIVE_HEADER

#include "ccGenericPrimitive.h"

//CCLib
#include <CCGeom.h>

//! Torus (primitive)
/** 3D torus primitive (with circular or rectangular section)
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccTorus : public ccGenericPrimitive
#else
class ccTorus : public ccGenericPrimitive
#endif
{
public:

	//! Default constructor
	/** Torus is defined in the XY plane by default
		\param insideRadius inside radius
		\param outsideRadius outside radius
		\param height section height (if rectangular torus)
		\param angle subtended angle (in radians)
		\param yOff displacement of axes along Y-axis (Snout mode)
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (main loop angular step = 360/(4*precision), circular section angular step = 360/precision)
	**/
	ccTorus(PointCoordinateType insideRadius,
				PointCoordinateType outsideRadius,
				double angle_rad = 2.0*M_PI,
				bool rectangularSection = false,
				PointCoordinateType rectSectionHeight = 0,
				const ccGLMatrix* transMat = 0,
				QString name = QString("Torus"),
				unsigned precision = 24);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccTorus(QString name = QString("Torus"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_TORUS; }

	//inherited from ccGenericPrimitive
	virtual bool hasDrawingPrecision() const { return true; }
	virtual ccGenericPrimitive* clone() const;

protected:
    
    //inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
	virtual bool buildUp();

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

#endif
