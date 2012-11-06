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
#include "cc2sm.h"

#include "my_point_types.h"

//PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>

//CCLib
#include <ChunkedPointCloud.h>

using namespace pcl;

cc2smReader::cc2smReader()
{

}


sensor_msgs::PointCloud2 cc2smReader::getGenericField(std::string field_name)
{
	sensor_msgs::PointCloud2 sm_cloud;

	if (field_name == "x")
	{
		sm_cloud = getOneOfXYZ(0);
		return sm_cloud;
	}

	else if (field_name == "y")
	{
		sm_cloud = getOneOfXYZ(1);
		return sm_cloud;
	}

	else if (field_name == "z")
	{
		sm_cloud = getOneOfXYZ(2);
		return sm_cloud;
	}

	else if (field_name == "xyz")
	{
		sm_cloud = getXYZ();
		return sm_cloud;
	}

	else if (field_name == "normal_x")
	{
		sm_cloud = getOneOfNormal(0);
		return sm_cloud;
	}

	else if (field_name == "normal_y")
	{
		sm_cloud = getOneOfNormal(1);
		return sm_cloud;
	}

	else if (field_name == "normal_z")
	{
		sm_cloud = getOneOfNormal(2);
		return sm_cloud;
	}

	else if (field_name == "normal_xyz")
	{
		sm_cloud = getNormals();
		return sm_cloud;
	}

	else if (field_name == "rgb")
	{
		sm_cloud = getColors();
		return sm_cloud;
	}
	else //load the field from the scalar fields
	{
		sm_cloud = getFloatScalarField(field_name);
		return sm_cloud;
	}
}


sensor_msgs::PointCloud2 cc2smReader::getOneOfXYZ(const int coord_ids)
{
	std::string name;
	if (coord_ids == 0)
		name = "x";
	else if (coord_ids == 1)
		name = "y";
	else if (coord_ids == 2)
		name = "z";


	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<FloatScalar>::Ptr pcl_cloud (new PointCloud<FloatScalar>);


	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);


	CCVector3 this_point;
	for (int i = 0; i < pnumber; ++i)
	{
		m_cc_cloud->getPoint(i, this_point);
        pcl_cloud->at(i).S5c4laR = (float) this_point[coord_ids];
	}

	toROSMsg(*pcl_cloud, *sm_cloud);
	sm_cloud->fields[0].name = name.c_str();
	return *sm_cloud;
}

sensor_msgs::PointCloud2 cc2smReader::getOneOfNormal(const int coord_ids)
{
	std::string name;
	if (coord_ids == 0)
		name = "normal_x";
	else if (coord_ids == 1)
		name = "normal_y";
	else if (coord_ids == 2)
		name = "normal_z";


	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<FloatScalar>::Ptr pcl_cloud (new PointCloud<FloatScalar>);


	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);


	CCVector3 normal;
	for (int i = 0; i < pnumber; ++i)
	{
		normal = m_cc_cloud->getPointNormal(i);
        pcl_cloud->at(i).S5c4laR = (float) normal[coord_ids];
	}

	toROSMsg(*pcl_cloud, *sm_cloud);
	sm_cloud->fields[0].name = name.c_str();
	return *sm_cloud;
}

sensor_msgs::PointCloud2 cc2smReader::getXYZ()
{
	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<PointXYZ>::Ptr pcl_cloud (new PointCloud<PointXYZ>);


	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);


	CCVector3 this_point;
	for (int i = 0; i < pnumber; ++i)
	{
		m_cc_cloud->getPoint(i, this_point);
		pcl_cloud->at(i).x =(float) this_point[0];
		pcl_cloud->at(i).y = (float) this_point[1];
		pcl_cloud->at(i).z = (float) this_point[2];
	}

	toROSMsg(*pcl_cloud, *sm_cloud);
	return *sm_cloud;
}

sensor_msgs::PointCloud2 cc2smReader::getNormals()
{
	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<OnlyNormals>::Ptr pcl_cloud (new PointCloud<OnlyNormals>);

	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);
	CCVector3 normal;

	for (int i = 0; i < pnumber; ++i)
	{
		normal = m_cc_cloud->getPointNormal(i);
		pcl_cloud->at(i).normal_x = normal[0];
		pcl_cloud->at(i).normal_y = normal[1];
		pcl_cloud->at(i).normal_z = normal[2];
	}

	toROSMsg(*pcl_cloud, *sm_cloud);
	return *sm_cloud;
}

sensor_msgs::PointCloud2 cc2smReader::getFloatScalarField(const std::string field_name)
{
	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<FloatScalar>::Ptr pcl_cloud (new PointCloud<FloatScalar>);

	CCLib::ScalarField * scalar_field = m_cc_cloud->getScalarField(m_cc_cloud->getScalarFieldIndexByName(field_name.c_str()));
	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);
	float scalar;
	for (int i = 0; i < pnumber; ++i)
	{
		scalar = scalar_field->getValue(i);
        pcl_cloud->at(i).S5c4laR = scalar;
	}

	toROSMsg(*pcl_cloud, *sm_cloud);

	//Now change the name of the scalar field -> we cannot have any space into the field name
	//NOTE this is a little trick for put any number of scalar fields in a message PointCloud2 object
	//We use a pointtype with a generic scalar field named scalar. we load here scalar field and
	//then we change the name to the needed one

	QString qfield_name = QString(field_name.c_str());
	qfield_name.simplified();
	qfield_name.replace(' ', '_');
	sm_cloud->fields[0].name = qfield_name.toStdString().c_str() ;


	return *sm_cloud;
}

sensor_msgs::PointCloud2 cc2smReader::getColors()
{
	sensor_msgs::PointCloud2::Ptr sm_cloud (new sensor_msgs::PointCloud2);
	PointCloud<OnlyRGB>::Ptr pcl_cloud (new PointCloud<OnlyRGB>);

	int pnumber = m_cc_cloud->size();
	pcl_cloud->resize(pnumber);


	for (int i = 0; i < pnumber; ++i)
	{
		pcl_cloud->at(i).r = m_cc_cloud->getPointColor(i)[0];
		pcl_cloud->at(i).g = m_cc_cloud->getPointColor(i)[1];
		pcl_cloud->at(i).b = m_cc_cloud->getPointColor(i)[2];
	}

	toROSMsg(*pcl_cloud, *sm_cloud);
	return *sm_cloud;
}

void cc2smReader::setInputCloud(const ccPointCloud * cc_cloud)
{
	m_cc_cloud = cc_cloud;
}

int cc2smReader::checkIfFieldExist(const std::string field_name)
{
	if ( (field_name == "x") || (field_name == "y") || (field_name == "z") || (field_name == "xyz") )
		return (m_cc_cloud->size() != 0);

	else if ( (field_name == "normal_x") || (field_name == "normal_y") || (field_name == "normal_z") || (field_name == "normal_xyz") )
		return m_cc_cloud->hasNormals();

	else if (field_name == "rgb")
		return m_cc_cloud->hasColors();

	else
		return (m_cc_cloud->getScalarFieldIndexByName(field_name.c_str()) >= 0);
}

int cc2smReader::getAsSM(std::vector<std::string> requested_fields, sensor_msgs::PointCloud2 &sm_cloud )
{
	//sort requested_fields, needed by binary_search
	std::sort(requested_fields.begin(), requested_fields.end());

	//do some checks
	for (unsigned i = 0; i < requested_fields.size(); ++i)
	{
		std::string field_name = requested_fields[i];
		bool exists = checkIfFieldExist(field_name);

		if (!exists) //all check results must be true
			return -1;
	}

	//are we asking for x, y, and z all togheters?
	bool got_xyz = std::binary_search(requested_fields.begin(), requested_fields.end(), "xyz");
	if (got_xyz)
	{
		//remove from the requested fields lists x y and z as single occurrencies
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("x")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("y")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("z")), requested_fields.end());
	}


	//same for normals
	bool got_normal_xyz = std::binary_search(requested_fields.begin(), requested_fields.end(), "normal_xyz");
	if (got_normal_xyz)
	{
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_x")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_y")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_z")), requested_fields.end());
	}

	//a vector for PointCloud2 clouds
	std::vector<sensor_msgs::PointCloud2> clouds;


	//TODO we should load and merge one-by-one, so to free some memory
	//load all fields
	for (unsigned i = 0; i < requested_fields.size(); ++i)
	{
		clouds.push_back(getGenericField(requested_fields[i]) );
	}

	//merge all these clouds in one
	sm_cloud = mergeVectorOfClouds(clouds);


	return 1;
}

int cc2smReader::getAsSM(sensor_msgs::PointCloud2 &sm_cloud)
{
	//does the cloud have some points?
	if (m_cc_cloud->size() == 0)
		return -1;

	//container
	std::vector<sensor_msgs::PointCloud2> clouds;


	//load the geometry
	clouds.push_back(getXYZ());


	//does we have normals?
	if (m_cc_cloud->hasNormals())
	{
		clouds.push_back(getNormals());
	}


	//colors?
	if (m_cc_cloud->hasColors())

	{
		clouds.push_back(getColors());
	}

	//how many fields does we have?
	int n_fields = m_cc_cloud->getNumberOfScalarFields();

	//declare some vectors
	std::vector<CCLib::ScalarField *> cc_scalar_fields;
	std::vector<std::string> cc_scalar_fields_names;

	//    pcl_scalar_fields.resize(n_fields);


	if (n_fields != 0) // DO THIS ONLY IF WE HAVE FIELDS
	{
		for (int i = 0; i < n_fields; ++i)
		{
			cc_scalar_fields.push_back(m_cc_cloud->getScalarField(i));
			cc_scalar_fields_names.push_back(m_cc_cloud->getScalarFieldName(i));
		}


		for (int i = 0; i < n_fields; ++i)
		{

			clouds.push_back(getFloatScalarField(cc_scalar_fields_names[i]));
		}
	}


	//merge all togheter
	sm_cloud = mergeVectorOfClouds(clouds);
	return 1;
}
