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

sensor_msgs::PointCloud2 mergeVectorOfClouds(std::vector<sensor_msgs::PointCloud2> &clouds)
{
	pcl::uint32_t n_points = clouds[0].height * clouds[0].width;
	size_t n_clouds = clouds.size();

	if (clouds.empty())
		return sensor_msgs::PointCloud2();

	//all the indexes
	std::vector<int> indexes;
	{
		try
		{
			indexes.resize(n_points);
		}
		catch(std::bad_alloc)
		{
			//not enough memory
			return sensor_msgs::PointCloud2();
		}

		for (pcl::uint32_t i = 0; i < n_points; ++i)
		{
			indexes[i] = i;
		}
	}
	
	//now loop on scalar fields and merge them
	{
		for (size_t i = 1; i < n_clouds; ++i)
		{
			sensor_msgs::PointCloud2::Ptr sm_tmp (new sensor_msgs::PointCloud2); //temporary cloud
			pcl::copyPointCloud(clouds[0], indexes, *sm_tmp);
			pcl::concatenateFields(*sm_tmp, clouds[i], clouds[0]);
		}
	}

	return clouds[0];
}

sensor_msgs::PointCloud2::Ptr loadSensorMessage(const QString &filename)
{
	sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);

	//Load the given file
	if (pcl::io::loadPCDFile(filename.toStdString(), *out_cloud) < 0)
	{
		//loading failed
		out_cloud.reset();
	}

	return out_cloud;
}
