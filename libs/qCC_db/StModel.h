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

#ifndef ST_MODEL_HEADER
#define ST_MODEL_HEADER

#include "ccHObject.h"

// block
class QCC_DB_LIB_API StModel : public ccHObject
{
public:

	//! Default ccMesh constructor
	/** \param vertices the vertices cloud
	**/
	explicit StModel(StModel& obj);

	explicit StModel(ccHObject& obj);

	explicit StModel(const QString& name = QString());


	//! Default destructor
	~StModel() override;
	
	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_MODEL; }

};

#endif //CC_MESH_HEADER
