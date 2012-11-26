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

#ifndef CC_CONE_PRIMITIVE_HEADER
#define CC_CONE_PRIMITIVE_HEADER

#include "ccGenericPrimitive.h"

//CCLib
#include <CCGeom.h>

//! Cone (primitive)
/** 3D cone primitive
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccCone : public ccGenericPrimitive
#else
class ccCone : public ccGenericPrimitive
#endif
{
public:

	//! Default constructor
	/** Cone axis corresponds to the 'Z' dimension by default
		\param bottomRadius cone bottom radius
		\param topRadius cone top radius
		\param height cone height (transformation should point to the axis center)
		\param xOff displacement of axes along X-axis (Snout mode)
		\param yOff displacement of axes along Y-axis (Snout mode)
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
	**/
	ccCone(PointCoordinateType bottomRadius,
				PointCoordinateType topRadius,
				PointCoordinateType height,
				PointCoordinateType xOff = 0,
				PointCoordinateType yOff = 0,
				const ccGLMatrix* transMat = 0,
				QString name = QString("Cone"),
				unsigned precision = 24);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCone(QString name = QString("Cone"));

    //! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_CONE; }

	//inherited from ccGenericPrimitive
	virtual bool hasDrawingPrecision() const { return true; }
	virtual ccGenericPrimitive* clone() const;

protected:
    
    //inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
	virtual bool buildUp();

	//! Bottom radius
	PointCoordinateType m_bottomRadius;

	//! Top radius
	PointCoordinateType m_topRadius;

	//! Displacement of axes along X-axis (Snout mode)
	PointCoordinateType m_xOff;

	//! Displacement of axes along Y-axis (Snout mode)
	PointCoordinateType m_yOff;

	//! Height
	PointCoordinateType m_height;
};

#endif
