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

#ifndef CC_CYLINDER_PRIMITIVE_HEADER
#define CC_CYLINDER_PRIMITIVE_HEADER

#include "ccCone.h"

//! Cylinder (primitive)
/** 3D cylinder primitive
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccCylinder : public ccCone
#else
class ccCylinder : public ccCone
#endif
{
public:

	//! Default constructor
	/** Cylinder axis corresponds to the 'Z' dimension.
		Internally represented by a cone with the same top and bottom radius.
		\param radius cylinder radius
		\param height cylinder height (transformation should point to the axis center)
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
	**/
	ccCylinder(PointCoordinateType radius,
				PointCoordinateType height,
				const ccGLMatrix* transMat = 0,
				QString name = QString("Cylinder"),
				unsigned precision = 24);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCylinder(QString name = QString("Cylinder"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_CYLINDER; }

	//inherited from ccGenericPrimitive
	virtual ccGenericPrimitive* clone() const;

protected:
    
};

#endif
