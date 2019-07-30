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
class StBlock;

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

	bool reverseVertexOrder();

	inline double getHeight() const;
	void setHeight(double height);	//! attention: will set all the points z to height

	//! the highest plane point
	inline double getHighest() const { return m_highest; }
	void setHighest(double top) { m_highest = top; }

	//! the lowest plane point
	inline double getLowest() const { return m_lowest; }
	void setLowest(double low) { m_lowest = low; }

	//! the bottom of footprint, same as the child block
	inline double getBottom() const { return m_bottom; }
	void setBottom(double bottom);
	
	inline bool isHole() const { return m_hole; }
	void setHoleState(bool state) { m_hole = state; }

	inline int getComponentId() { return m_componentId; }
	void setComponentId(int id) { m_componentId = id; }

	inline QStringList getPlaneNames() { return m_plane_names; }
	void setPlaneNames(QStringList names) { m_plane_names = names; }

	std::vector<StBlock*> getBlocks();

protected:
	//inherited from ccDrawable
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;

	double m_highest;
	double m_lowest;
	double m_bottom;
	bool m_hole;
	int m_componentId;
	QStringList m_plane_names;
};

#endif //ST_FOOTPRINT_HEADER
