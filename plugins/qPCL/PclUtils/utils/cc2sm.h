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
#ifndef Q_PCL_PLUGIN_CC2SM_H
#define Q_PCL_PLUGIN_CC2SM_H

#include "pcl_utilities.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//qCC_db
#include <ccPointCloud.h>

class cc2smReader
{
public:
	cc2smReader();

	PCLCloud getGenericField(std::string field_name);

	PCLCloud getOneOfXYZ(const int coord_ids);

	PCLCloud getOneOfNormal(const int coord_ids);

	PCLCloud getXYZ();

    void getXYZ(pcl::PointCloud<pcl::PointXYZ> & cloud);

	PCLCloud getNormals();

	PCLCloud getFloatScalarField(const std::string field_name);

	PCLCloud getColors();

	int checkIfFieldExist(const std::string field_name);

	void setInputCloud(const ccPointCloud * cc_cloud);

	int getAsSM(std::vector<std::string> requested_fields, PCLCloud &sm_cloud );

	/** \brief this convert all the data in a ccPointCloud to a sesor_msgs::PointCloud2
	* this is useful for saving all the data in a ccPointCloud into a PCD file
	* Is suggested to use other methods for get a cloud for pcl filters (so to get only needed data)
	*/
	int getAsSM(PCLCloud &sm_cloud);

protected:
	const ccPointCloud* m_cc_cloud;

};





#endif // Q_PCL_PLUGIN_CC2SM_H
