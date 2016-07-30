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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################
//
#ifndef PCL_CONVERSIONS_H
#define PCL_CONVERSIONS_H

//PCL V1.6 or older
#ifdef PCL_VER_1_6_OR_OLDER

#include <pcl/ros/conversions.h>
#define FROM_PCL_CLOUD pcl::fromROSMsg
#define TO_PCL_CLOUD pcl::toROSMsg

#else //Version 1.7 or newer

#include <pcl/PCLPointCloud2.h>
#define FROM_PCL_CLOUD pcl::fromPCLPointCloud2
#define TO_PCL_CLOUD pcl::toPCLPointCloud2

#endif

#endif // PCL_CONVERSIONS_H
