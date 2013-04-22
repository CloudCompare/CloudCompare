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
#include <sensor_msgs/PointCloud2.h>

//Qt
#include <QString>

sensor_msgs::PointCloud2 mergeVectorOfClouds(std::vector<sensor_msgs::PointCloud2> &clouds);

//! Utility function that loads a given PCD file in a sensor_msgs PointCloud2
/** \param[in] filename
	\return a shared pointer to a sensor_msgs cloud
**/
sensor_msgs::PointCloud2::Ptr loadSensorMessage(const QString &filename);

#endif // PCL_UTILITIES_H
