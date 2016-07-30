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
#ifndef PCL_CLOUD_H
#define PCL_CLOUD_H

//PCL V1.6 or older
#ifdef PCL_VER_1_6_OR_OLDER

#include <sensor_msgs/PointCloud2.h>
typedef sensor_msgs::PointCloud2 PCLCloud;

#else //Version 1.7 or newer

#include <pcl/PCLPointCloud2.h>
typedef pcl::PCLPointCloud2 PCLCloud;

#endif

#endif // PCL_CLOUD_H
