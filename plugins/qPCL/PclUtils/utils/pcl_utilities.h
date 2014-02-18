//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################
//
#ifndef PCL_UTILITIES_H
#define PCL_UTILITIES_H

//PCL
#ifdef PCL_VER_1_6_OR_OLDER

#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
typedef sensor_msgs::PointCloud2 PCLCloud;
typedef sensor_msgs::PointField PCLScalarField;
#define FROM_PCL_CLOUD pcl::fromROSMsg
#define TO_PCL_CLOUD pcl::toROSMsg

#else //Version 1.7.0 or newer

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
typedef pcl::PCLPointCloud2 PCLCloud;
typedef pcl::PCLPointField PCLScalarField;
#define FROM_PCL_CLOUD pcl::fromPCLPointCloud2
#define TO_PCL_CLOUD pcl::toPCLPointCloud2

#endif


//Qt
#include <QString>

PCLCloud mergeVectorOfClouds(std::vector<PCLCloud> &clouds);

//! Utility function that loads a given PCD file in a PointCloud2
/** \param[in] filename
	\return a shared pointer to a PCL cloud
**/
PCLCloud::Ptr loadSensorMessage(const QString &filename);

#endif // PCL_UTILITIES_H
