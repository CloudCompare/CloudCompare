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

#ifndef CC_CONE_PRIMITIVE_HEADER
#define CC_CONE_PRIMITIVE_HEADER

//Local
#include "ccGenericPrimitive.h"


//! Cone (primitive)
/** 3D cone primitive
**/
class QCC_DB_LIB_API ccCone : public ccGenericPrimitive
{
public:

	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

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
		\param uniqueID unique ID (handle with care)
	**/
	ccCone(	PointCoordinateType bottomRadius,
			PointCoordinateType topRadius,
			PointCoordinateType height,
			PointCoordinateType xOff = 0,
			PointCoordinateType yOff = 0,
			const ccGLMatrix* transMat = 0,
			QString name = QString("Cone"),
			unsigned precision = DEFAULT_DRAWING_PRECISION,
			unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccCone(QString name = QString("Cone"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CONE; }

	//! Returns height
	inline PointCoordinateType getHeight() const { return m_height; }
	//! Sets height
	/** \warning changes primitive content (calls ccGenericPrimitive::updateRepresentation)
	**/
	void setHeight(PointCoordinateType height);

	//! Returns bottom radius
	inline PointCoordinateType getBottomRadius() const { return m_bottomRadius; }
	//! Sets bottom radius
	/** \warning changes primitive content (calls ccGenericPrimitive::updateRepresentation)
	**/
	virtual void setBottomRadius(PointCoordinateType radius);

	//! Returns top radius
	inline PointCoordinateType getTopRadius() const { return m_topRadius; }
	//! Sets top radius
	/** \warning changes primitive content (calls ccGenericPrimitive::updateRepresentation)
	**/
	virtual void setTopRadius(PointCoordinateType radius);

	//! Returns cone axis bottom end point after applying transformation
	virtual CCVector3 getBottomCenter() const;
	//! Returns cone axis top end point after applying transformation
	virtual CCVector3 getTopCenter() const;

	//! Returns cone axis end point associated with whichever radii is smaller
	virtual CCVector3 getSmallCenter() const;
	//! Returns cone axis end point associated with whichever radii is larger
	virtual CCVector3 getLargeCenter() const;

	//! Returns whichever cone radii is smaller
	virtual PointCoordinateType getSmallRadius() const;
	//! Returns whichever cone radii is larger
	virtual PointCoordinateType getLargeRadius() const;

	//! Returns true if the Cone was created in snout mode 
	virtual bool isSnoutMode() const { return (m_xOff != 0 || m_yOff != 0); }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Cone"; }
	virtual bool hasDrawingPrecision() const override { return true; }
	virtual ccGenericPrimitive* clone() const override;

protected:

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	virtual bool buildUp() override;

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

#endif //CC_CONE_PRIMITIVE_HEADER
