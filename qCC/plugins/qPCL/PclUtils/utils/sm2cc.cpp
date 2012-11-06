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
#include "sm2cc.h"
#include "my_point_types.h"

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointField.h>

using ::sensor_msgs::PointField;

sm2ccReader::sm2ccReader()
{

}

int sm2ccReader::checkIfFieldExist(std::string field_name)
{
	return (pcl::getFieldIndex(*m_sm_cloud, field_name) != -1);
}

void sm2ccReader::updateFieldList()
{
	std::vector<std::string> field_list;
	int n_fields = m_sm_cloud->fields.size();
	field_list.resize(n_fields);
	for (int d = 0; d < n_fields; ++d)
	{
		std::string name = m_sm_cloud->fields[d].name;
		field_list[d] = name;
	}
	std::sort(field_list.begin(), field_list.end());

	//remove fields as "_" -> used by PCL for padding
	for (unsigned i = 0; i < field_list.size(); ++i)
	{
		std::vector<std::string>::iterator it = std::remove(field_list.begin(), field_list.end(), "_");
		field_list.erase( it, field_list.end());

	}
	m_field_list = field_list;

}

int sm2ccReader::getAsCC(ccPointCloud * cloud)
{
	//resize
	unsigned int npoints = getNumberOfPoints();

	cloud->reserveThePointsTable(npoints);
	//update the field-list
	updateFieldList();
	//copy the list for this method
	std::vector<std::string> fields = m_field_list;

	//begin with checks and conversions
	//be sure we have x, y, and z fields


    bool got_xyz = hasXYZ();
        if (!got_xyz)
        {
                return -1;
        }
        else
    {



		addXYZ(cloud);

		//remove x,y,z fields from the vector of field names
		eraseString(fields, "x");
		eraseString(fields, "y");
		eraseString(fields, "z");
	}

	//do we have normals?
	bool got_normal_xyz = hasNormals();
	if (got_normal_xyz)
	{

		addNormals(cloud);
		//remove the fields
		eraseString(fields, "normal_x");
		eraseString(fields, "normal_y");
		eraseString(fields, "normal_z");

	}

	//The same for colors
	bool got_rgb = hasRGB();
	if (got_rgb)
	{

		addRGB(cloud);
		//remove the fields
		eraseString(fields, "rgb");


	}

	//All the field remaining will be stored as float scalars
	//They must be loaded using addIntField or addFloatField depending on the type
	for (unsigned i = 0; i < fields.size(); ++i)
	{
		std::string name = fields[i];

		//get the field
		PointField field = m_sm_cloud->fields.at(pcl::getFieldIndex(*m_sm_cloud, name));

		//check if int or float
		if ((field.datatype == PointField::FLOAT32) || (field.datatype == PointField::FLOAT64))
		  {
		    addFloatField(cloud, name);
		  }
		else if ((field.datatype == PointField::INT8) || (field.datatype == PointField::INT16) || (field.datatype == PointField::INT32))
		  {
		    addIntField(cloud, name);
		  }




	}

	//DGM: handle scalar fields display parameters
	if (cloud->hasScalarFields())
	{
		cloud->setCurrentDisplayedScalarField(0);
		cloud->showSF(true);
	}


	return 1;
}


int sm2ccReader::addXYZ(ccPointCloud *cloud)
{
	cloud->reserveThePointsTable(getNumberOfPoints());
	//add xyz to the given cloud taking xyz infos from the sm cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*m_sm_cloud, *pcl_cloud);

	//loop
	CCVector3 P; //= {0.0,0.0,0.0};
	for (int i = 0; i < getNumberOfPoints(); ++i)
	{
		P[0] = pcl_cloud->at(i).x;
		P[1] = pcl_cloud->at(i).y;
		P[2] = pcl_cloud->at(i).z;

		cloud->addPoint(P);
	}
	return 1;

}



int sm2ccReader::addNormals(ccPointCloud *cloud)
{
	pcl::PointCloud<OnlyNormals>::Ptr pcl_cloud_normals (new pcl::PointCloud<OnlyNormals>);
	pcl::fromROSMsg(*m_sm_cloud, *pcl_cloud_normals);

	if (cloud->hasNormals() == 0)
		cloud->resizeTheNormsTable();


	//loop
	float N[] = {0.0,0.0,0.0};
	for (int i = 0; i < getNumberOfPoints(); ++i)
	{
		N[0] = pcl_cloud_normals->at(i).normal_x;
		N[1] = pcl_cloud_normals->at(i).normal_y;
		N[2] = pcl_cloud_normals->at(i).normal_z;


		cloud->addNormAtIndex(N, i);
	}
	cloud->showNormals(true);
	return 1;
}

int sm2ccReader::addRGB(ccPointCloud * cloud)
{
	pcl::PointCloud<OnlyRGB>::Ptr pcl_cloud_rgb (new pcl::PointCloud<OnlyRGB>);
	pcl::fromROSMsg(*m_sm_cloud, *pcl_cloud_rgb);

	cloud->reserveTheRGBTable();

	colorType C[3]={0,0,0};
	for (int i = 0; i < getNumberOfPoints(); ++i)
	{
		C[0] = pcl_cloud_rgb->points[i].r;
		C[1] = pcl_cloud_rgb->points[i].g;
		C[2] = pcl_cloud_rgb->points[i].b;

		cloud->addRGBColor(C);
	}

	cloud->showColors(true);
	return 1;
}

int sm2ccReader::addFloatField(ccPointCloud * cloud, const std::string name, bool overwrite_if_exist)
{
        //if this field yet exists simply delete it
        int id = cloud->getScalarFieldIndexByName(name.c_str());
        if ((id >= 0) && (overwrite_if_exist == true))
                cloud->deleteScalarField(id); //delete

        if ((id >= 0) && (overwrite_if_exist == false))
            return 1;


        pcl::PointCloud<FloatScalar>::Ptr pcl_scalar (new pcl::PointCloud<FloatScalar>);

        int field_index = pcl::getFieldIndex(*m_sm_cloud, name);

        //temporary change the name of the given field to something else -> S5c4laR should be a pretty uncommon name,
        m_sm_cloud->fields[field_index].name = std::string("S5c4laR");

        //now load
        pcl::fromROSMsg(*m_sm_cloud, *pcl_scalar);


	ccScalarField* cc_scalar_field = new ccScalarField(name.c_str());
	cc_scalar_field->reserve(getNumberOfPoints());


	float scalar = 0.0;
	for (int i = 0; i < getNumberOfPoints(); ++i)
	{
	  scalar = pcl_scalar->points[i].S5c4laR;
	  cc_scalar_field->setValue(i, scalar);
	}


	cc_scalar_field->computeMinAndMax();

	cloud->addScalarField(cc_scalar_field);

	//restore old name for the scalar field
	m_sm_cloud->fields[field_index].name = name;

	return 1;
}


int sm2ccReader::addIntField(ccPointCloud * cloud, const std::string name, bool overwrite_if_exist)
{

        //if this field yet exists simply delete it
        int id = cloud->getScalarFieldIndexByName(name.c_str());
        if ((id >= 0) && (overwrite_if_exist == true))
                cloud->deleteScalarField(id); //delete

	if ((id >= 0) && (overwrite_if_exist == false))
		return 1;


        pcl::PointCloud<IntScalar>::Ptr pcl_scalar (new pcl::PointCloud<IntScalar>);

        int field_index = pcl::getFieldIndex(*m_sm_cloud, name);

        //temporary change the name of the given field to something else -> S5c4laR should be a pretty uncommon name,
        m_sm_cloud->fields[field_index].name = std::string("S5c4laR");


        //now load
        pcl::fromROSMsg(*m_sm_cloud, *pcl_scalar);



	ccScalarField* cc_scalar_field = new ccScalarField(name.c_str());
	cc_scalar_field->reserve(getNumberOfPoints());


	float scalar = 0.0;
	for (int i = 0; i < getNumberOfPoints(); ++i)
	{
	  scalar = pcl_scalar->points[i].S5c4laR;
	  cc_scalar_field->setValue(i, scalar);
	}


	cc_scalar_field->computeMinAndMax();

	cloud->addScalarField(cc_scalar_field);

	//restore old name for the scalar field
	m_sm_cloud->fields[field_index].name = name;

	return 1;
}

int sm2ccReader::getNumberOfPoints()
{
    return (int) (m_sm_cloud->width * m_sm_cloud->height);
}

bool sm2ccReader::existField(std::string name) const
{
        return std::binary_search(m_field_list.begin(), m_field_list.end(), name);
}

bool sm2ccReader::hasNormals() const
{
	bool got_normal_x = existField("normal_x");
	bool got_normal_y = existField("normal_y");
	bool got_normal_z = existField("normal_z");
	return (got_normal_x && got_normal_y && got_normal_z);
}

bool sm2ccReader::hasXYZ() const
{
	bool got_x = existField("x");
	bool got_y = existField("y");
	bool got_z = existField("z");
	return (got_x && got_y && got_z);
}

bool sm2ccReader::hasRGB() const
{
	return existField("rgb");
}

void sm2ccReader::eraseString(std::vector<std::string> &fields, std::string name)
{
	fields.erase(std::remove(fields.begin(), fields.end(), name), fields.end());
}

std::vector<std::string> getFieldList(const sensor_msgs::PointCloud2 &cloud)
{
	std::vector<std::string> field_list;
	for (size_t d = 0; d < cloud.fields.size (); ++d)
	{
		field_list.push_back (cloud.fields[d].name);
	}

	return field_list;
}

