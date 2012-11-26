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

#ifndef CC_BOX_PRIMITIVE_HEADER
#define CC_BOX_PRIMITIVE_HEADER

#include "ccGenericPrimitive.h"

//CCLib
#include <CCGeom.h>

//! Box (primitive)
/** 3D box primitive
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccBox : public ccGenericPrimitive
#else
class ccBox : public ccGenericPrimitive
#endif
{
public:

	//! Default constructor
	/** Box dimensions axis along each dimension are defined in a single 3D vector.
		A box is in fact composed of 6 planes (ccPlane).
		\param dims box dimensions
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccBox(const CCVector3& dims,
				const ccGLMatrix* transMat = 0,
				QString name = QString("Box"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccBox(QString name = QString("Box"));

    //! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_BOX; }

	//inherited from ccGenericPrimitive
	virtual ccGenericPrimitive* clone() const;

protected:

    //inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
	virtual bool buildUp();

	//! Box dimensions
	CCVector3 m_dims;
    
};

#endif
