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

#ifndef ST_FOOTPRINT_HEADER
#define ST_FOOTPRINT_HEADER

#include "ccPolyline.h"

// block
class QCC_DB_LIB_API StFootPrint : public ccPolyline
{
public:
	explicit StFootPrint(StFootPrint& obj);

	explicit StFootPrint(ccPolyline& obj);

	explicit StFootPrint(GenericIndexedCloudPersist* associatedCloud);


	//! Default destructor
	~StFootPrint() override;
	
	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_FOOTPRINT; }

	void reverseVertexOrder();

	inline double getHeight() const;
	void setHeight(double height);
	inline double getGround() const { return m_ground; }
	void setGround(double ground) { m_ground = ground; }

	double m_ground;
};

#endif //ST_FOOTPRINT_HEADER
