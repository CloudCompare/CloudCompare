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

#ifndef CC_DISH_PRIMITIVE_HEADER
#define CC_DISH_PRIMITIVE_HEADER

#include "ccGenericPrimitive.h"

//CCLib
#include <CCGeom.h>

//! Dish
/** Either a section of a sphere, or half of an ellipsoid!
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccDish : public ccGenericPrimitive
#else
class ccDish : public ccGenericPrimitive
#endif
{
public:

	//! Default constructor
	/** The origin of the Dish is at the centre of the base, and the Z-axis lies along the normal to the base.
		\param radius base radius
		\param height maximum height of dished surface above base
		\param radius2 If radius2 is zero, dish is drawn as a section of sphere. If radius2 is >0, dish is defined as half of an ellipsoid.
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
	**/
	ccDish(PointCoordinateType radius,
		PointCoordinateType height,
		PointCoordinateType radius2 = 0,
		const ccGLMatrix* transMat = 0,
		QString name = QString("Dish"),
		unsigned precision = 24);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccDish(QString name = QString("Dish"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_DISH; }

	//inherited from ccGenericPrimitive
	virtual bool hasDrawingPrecision() const { return true; }
	virtual ccGenericPrimitive* clone() const;

protected:
    
    //inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
	virtual bool buildUp();

	//! Base radius
	PointCoordinateType m_baseRadius;
	//! Second radius
	PointCoordinateType m_secondRadius;
	//! Height
	PointCoordinateType m_height;
};

#endif
