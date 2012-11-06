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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#include "pcl_utilities.h"

//PCL
#include <pcl/io/pcd_io.h>

sensor_msgs::PointCloud2 mergeVectorOfClouds(const std::vector<sensor_msgs::PointCloud2> &clouds)
{
	int n_points = clouds[0].height * clouds[0].width;
	int n_clouds = clouds.size();

	//all the indices
	std::vector<int> indices(n_points);
	for (int i = 0; i < n_points; ++i)
	{
		indices.at(i) = i;
	}
	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2); //out cloud
	sensor_msgs::PointCloud2::Ptr sm_tmp (new sensor_msgs::PointCloud2); //temporary cloud

	*sm_cloud = clouds[0];

	//now loop on scalar fields and merge them
	for (int i = 1; i < n_clouds; ++i)
	{

		pcl::copyPointCloud(*sm_cloud, indices, *sm_tmp);
		pcl::concatenateFields(*sm_tmp, clouds[i], *sm_cloud);
	}

	return *sm_cloud;
}

sensor_msgs::PointCloud2* loadSensorMessage(const QString &filename)
{
	sensor_msgs::PointCloud2 * out_cloud = new sensor_msgs::PointCloud2 ;

	//get filename as std::string
	std::string filename_std = filename.toStdString();
	//Load the given file
	pcl::io::loadPCDFile(filename_std, *out_cloud);
	return out_cloud;
}
