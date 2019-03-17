//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
//#ifdef LP_PCL_PATCH_ENABLED

#ifndef qPCL_COPY_H
#define qPCL_COPY_H

//Local
#include "PCLCloud.h"

//PCL
#include <pcl/PointIndices.h>

class ccPointCloud;

//! Makes a copy of all scalar fields from one cloud to another
/**	This algorithm simply copy the scalar fields from a cloud
	to another using the the mapping contained in a pcl::PointIndicesPtr.
	\param inCloud the input cloud from which to copy scalars
	\param outCloud the output cloud in which to copy the scalar fields
	\param in2outMapping indices of the input cloud for each point in the output
	\param overwrite you can chose to not overwrite existing fields
**/
void copyScalarFields(	const ccPointCloud *inCloud,
						ccPointCloud *outCloud,
						pcl::PointIndicesPtr &in2outMapping,
						bool overwrite = true);

//! Makes a copy of RGB colors from one cloud to another
void copyRGBColors(		const ccPointCloud *inCloud,
						ccPointCloud *outCloud,
						pcl::PointIndicesPtr &in2outMapping,
						bool overwrite = true);

#endif // qPCL_COPY_H

//#endif // LP_PCL_PATCH_ENABLED
