#pragma once

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
//#                   COPYRIGHT: CloudCompare project                      #
//#                                                                        #
//##########################################################################

//Local
#include "ccPolyline.h"

//! Circle (as a polyline)
/** Extends the ccPolyline class
**/
class QCC_DB_LIB_API ccCircle : public ccPolyline
{
public:

	//! Default constructor
	/** \param radius circle radius
		\param resolution circle displayed resolution
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccCircle(	double radius = 0.0,
						unsigned resolution = 48,
						unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID );

	//! Copy constructor
	/** \param circle circle to copy/clone
	**/
	ccCircle(const ccCircle& circle);

	//! Destructor
	~ccCircle() override = default;

	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CIRCLE; }

	//inherited methods (ccHObject)
	void applyGLTransformation(const ccGLMatrix& trans) override;

	//! Clones this circle
	ccCircle* clone() const;

	//! Sets the radius of the circle
	/**  \param radius the desired radius
	**/
	void setRadius(double radius);

	//! Returns the radius of the circle
	inline double getRadius() const { return m_radius; }

	//! Sets the resolution of the displayed circle
	/**  \param resolution the displayed resolution (>= 4)
	**/
	void setResolution(unsigned resolution);

	//! Returns the resolution of the displayed circle
	inline unsigned getResolution() const { return m_resolution; }

protected:

	//! Updates the internal representation
	void updateInternalRepresentation();

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;

	//! Radius of the circle
	double m_radius;

	//! Resolution of the displayed circle
	unsigned m_resolution;
};
