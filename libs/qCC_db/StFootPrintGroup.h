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

#ifndef ST_FOOTPRINTGROUP_HEADER
#define ST_FOOTPRINTGROUP_HEADER

#include "ccHObject.h"

// block
class QCC_DB_LIB_API StFootPrintGroup : public ccHObject
{
public:
	explicit StFootPrintGroup(StFootPrintGroup& obj);

	explicit StFootPrintGroup(ccHObject& obj);

	explicit StFootPrintGroup(const QString& name = QString());


	//! Default destructor
	~StFootPrintGroup() override;
	
	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_FOOTPRINTGROUP; }

};

#endif //ST_FOOTPRINTGROUP_HEADER
