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

#ifndef CC_PLANE_PRIMITIVE_HEADER
#define CC_PLANE_PRIMITIVE_HEADER

#include "ccGenericPrimitive.h"

//Qt
#include <QImage>

//CCLib
#include <CCGeom.h>

//! Plane (primitive)
/** 3D plane primitive
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccPlane : public ccGenericPrimitive
#else
class ccPlane : public ccGenericPrimitive
#endif
{
public:

	//! Default constructor
	/** Plane normal corresponds to 'Z' dimension
		\param xWidth plane width along 'X' dimension
		\param yWidth plane width along 'Y' dimension
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccPlane(PointCoordinateType xWidth,
			PointCoordinateType yWidth,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Plane"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccPlane(QString name = QString("Plane"));

    //! create a plane as best fitting plane of any CCLib::GenericIndexedCloudPersist-derived object.
    static ccPlane * fromFit(CCLib::GenericIndexedCloudPersist * cloud, double* rms = 0);

    //! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const { return CC_PLANE; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const { return "Plane"; }
	virtual ccGenericPrimitive* clone() const;

	//inherited from ccDrawableObject
	virtual ccBBox getFitBB(ccGLMatrix& trans);

	//! Returns 'X' width
	PointCoordinateType getXWidth() const { return m_xWidth; }

	//! Returns 'Y' width
	PointCoordinateType getYWidth() const { return m_yWidth; }

	//! Returns normal
	CCVector3 getNormal() const { return CCVector3(m_transformation.getColumn(2)); }

	//! Sets an image as texture
	bool setAsTexture(QImage image);

protected:
    
    //inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
	virtual bool buildUp();

	//! Width along 'X' dimension
	PointCoordinateType m_xWidth;

	//! Width along 'Y' dimension
	PointCoordinateType m_yWidth;
};

#endif //CC_PLANE_PRIMITIVE_HEADER
