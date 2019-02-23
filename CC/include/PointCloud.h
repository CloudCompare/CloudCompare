//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_LIB_POINT_CLOUD_HEADER
#define CC_LIB_POINT_CLOUD_HEADER

//Local
#include "GenericIndexedCloudPersist.h"
#include "PointCloudTpl.h"

namespace CCLib
{
	//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
	class CC_CORE_LIB_API PointCloud : public PointCloudTpl<GenericIndexedCloudPersist>
	{
	public:
		//! Default constructor
		PointCloud() = default;

		//! Default destructor
		~PointCloud() override = default;
	};
}

#endif //CC_LIB_POINT_CLOUD_HEADER
