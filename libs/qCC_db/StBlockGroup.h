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

#ifndef ST_BLOCKGROUP_HEADER
#define ST_BLOCKGROUP_HEADER

#include "ccHObject.h"
#include "ccPointCloud.h"

// block
class QCC_DB_LIB_API StBlockGroup : public ccHObject
{
public:

	//! Default ccMesh constructor
	/** \param vertices the vertices cloud
	**/
	explicit StBlockGroup(StBlockGroup& obj);

	explicit StBlockGroup(ccHObject& obj);

	explicit StBlockGroup(const QString& name = QString());


	//! Default destructor
	~StBlockGroup() override;
	
	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_BLOCKGROUP; }

	ccHObject::Container GetFootPrints();

};

#endif //ST_BLOCKGROUP_HEADER
