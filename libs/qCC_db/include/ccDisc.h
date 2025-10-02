// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CNRS / OSERen / University of Rennes, France       #
// #                                                                        #
// ##########################################################################

#ifndef CCDISC_H
#define CCDISC_H

// Local
#include <ccGenericPrimitive.h>

//! Disc (primitive)
/** 3D disc primitive
 **/
class QCC_DB_LIB_API ccDisc : public ccGenericPrimitive
{
public:
	//! Default drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	 **/
	static const unsigned DEFAULT_DRAWING_PRECISION = 24;

	//! Default constructor
	/** Simple disc constructor
		\param radius disc radius
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
		\param precision drawing precision (angular step = 360/precision)
		\param uniqueID unique ID (handle with care)
	**/
	ccDisc(PointCoordinateType radius,
		   const ccGLMatrix*   transMat  = nullptr,
		   QString             name      = QString("Disc"),
		   unsigned            precision = DEFAULT_DRAWING_PRECISION,
		   unsigned            uniqueID  = ccUniqueIDGenerator::InvalidUniqueID);

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	ccDisc(QString name = QString("Disc"));

	//! Returns top radius
	inline PointCoordinateType getRadius() const
	{
		return m_radius;
	}
	//! Sets top radius
	/** \warning changes primitive content (calls ccGenericPrimitive::updateRepresentation)
	 **/
	void setRadius(PointCoordinateType radius);

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override
	{
		return CC_TYPES::DISC;
	}

	// inherited from ccGenericPrimitive
	virtual QString getTypeName() const override
	{
		return "Disc";
	}
	virtual bool hasDrawingPrecision() const override
	{
		return true;
	}
	virtual ccGenericPrimitive* clone() const override;

protected:
	// inherited from ccGenericPrimitive
	bool  toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool  fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	bool  buildUp() override;

	//! Radius
	PointCoordinateType m_radius;
};

#endif // CCDISC_H
