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

#ifndef CC_2D_VIEWPORT_LABEL_HEADER
#define CC_2D_VIEWPORT_LABEL_HEADER

//Local
#include "cc2DViewportObject.h"

//! 2D viewport label
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API cc2DViewportLabel : public cc2DViewportObject
#else
class cc2DViewportLabel : public cc2DViewportObject
#endif
{
public:

	//! Default constructor
	cc2DViewportLabel(const char* name=0);

	//inherited from ccHObject
    virtual CC_CLASS_ENUM getClassID() const {return CC_2D_VIEWPORT_LABEL;};
	virtual bool isSerializable() const { return true; }

	//! Returns ROI (relative to screen)
	const float* roi() const { return m_roi; }

	//! Sets ROI (relative to screen)
	void setRoi(const float* roi);

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

    //! Draws the entity only (not its children)
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! label ROI
	/** ROI is relative to screen
	**/
	float m_roi[4];
};

#endif //CC_2D_VIEWPORT_LABEL_HEADER
