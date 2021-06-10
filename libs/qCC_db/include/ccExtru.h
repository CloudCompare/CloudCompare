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

#ifndef CC_EXTRU_PRIMITIVE_HEADER
#define CC_EXTRU_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"


//! Profile extrusion (primitive)
/** It is defined by sweeping a profile representing the extrusion's shape (polyline)
	through a given distance (equivalent to the extrusion thickness).
**/
class QCC_DB_LIB_API ccExtru : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Input (2D) polyline represents the profile in (X,Y) plane.
		Extrusion is always done ine the 'Z' dimension.
		\param profile 2D profile to extrude
		\param height extrusion thickness
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccExtru(const std::vector<CCVector2>& profile,
			PointCoordinateType height,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Extrusion"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccExtru(QString name = QString("Extrusion"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::EXTRU; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Extrusion"; }
	virtual ccGenericPrimitive* clone() const override;

	//! Returns extrusion thickness
	const PointCoordinateType getThickness() const { return m_height; }

	//! Returns profile
	const std::vector<CCVector2>& getProfile() const { return m_profile; }

protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual bool buildUp() override;

	//! Extrusion thickness
	PointCoordinateType m_height;

	//! Profile
	std::vector<CCVector2> m_profile;
};

#endif //CC_EXTRU_PRIMITIVE_HEADER
