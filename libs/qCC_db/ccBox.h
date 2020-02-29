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

#ifndef CC_BOX_PRIMITIVE_HEADER
#define CC_BOX_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"

//! Box (primitive)
/** 3D box primitive
**/
class QCC_DB_LIB_API ccBox : public ccGenericPrimitive
{
public:

	//! Default constructor
	/** Box dimensions axis along each dimension are defined in a single 3D vector.
		A box is in fact composed of 6 planes (ccPlane).
		\param dims box dimensions
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/
	ccBox(	const CCVector3& dims,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Box"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccBox(QString name = QString("Box"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::BOX; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Box"; }
	virtual ccGenericPrimitive* clone() const override;

	//! Sets box dimensions
	inline void setDimensions(CCVector3& dims) { m_dims = dims; updateRepresentation(); }

	//! Returns box dimensions
	const CCVector3& getDimensions() const { return m_dims; }

protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual bool buildUp() override;

	//! Box dimensions
	CCVector3 m_dims;

};

#endif //CC_BOX_PRIMITIVE_HEADER
