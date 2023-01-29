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

#ifndef CC_2D_VIEWPORT_LABEL_HEADER
#define CC_2D_VIEWPORT_LABEL_HEADER

//Local
#include "cc2DViewportObject.h"
//System
#include <array>

//! 2D viewport label
class QCC_DB_LIB_API cc2DViewportLabel : public cc2DViewportObject
{
public:

	//! Default constructor
	explicit cc2DViewportLabel(QString name = QString());

	//! Copy constructor
	explicit cc2DViewportLabel(const cc2DViewportLabel& viewportLabel);

	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::VIEWPORT_2D_LABEL; }
	virtual bool isSerializable() const override { return true; }

	typedef std::array<float, 4> ROI;

	//! Returns ROI (relative to screen)
	inline const ROI& roi() const { return m_roi; }

	//! Sets ROI (relative to screen)
	inline void setRoi(const ROI& roi) { m_roi = roi; }

protected:

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;

	//! Draws the entity only (not its children)
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! label ROI
	/** ROI is relative to the 3D display
	**/
	ROI m_roi;
};

#endif //CC_2D_VIEWPORT_LABEL_HEADER
