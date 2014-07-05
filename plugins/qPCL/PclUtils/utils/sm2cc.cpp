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

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//system
#include <assert.h>

sm2ccConverter::sm2ccConverter(PCLCloud::Ptr sm_cloud)
	: m_sm_cloud(sm_cloud)
{
	assert(sm_cloud);
}

ccPointCloud* sm2ccConverter::getCloud()
{
	if (!m_sm_cloud)
	{
		assert(false);
		return 0;
	}
	
	//get the fields list
	std::list<std::string> fields;
	for (std::vector< PCLScalarField >::const_iterator it = m_sm_cloud->fields.begin(); it != m_sm_cloud->fields.end(); ++it)
		if (it->name != "_") //PCL padding fields
			fields.push_back(it->name);

	//begin with checks and conversions
	//be sure we have x, y, and z fields
	if (!existField("x") || !existField("y") || !existField("z"))
		return 0;

	//create cloud
	ccPointCloud* cloud = new ccPointCloud();

	//push points inside
	if (!addXYZ(cloud))
	{
		delete cloud;
		return 0;
	}

	//remove x,y,z fields from the vector of field names
	eraseString(fields, "x");
	eraseString(fields, "y");
	eraseString(fields, "z");

	//do we have normals?
	if (existField("normal_x") || existField("normal_y") || existField("normal_z"))
	{
		addNormals(cloud);
		
		//remove the corresponding fields
		eraseString(fields, "normal_x");
		eraseString(fields, "normal_y");
		eraseString(fields, "normal_z");
	}

	//The same for colors
	if (existField("rgb"))
	{
		addRGB(cloud);
		
		//remove the corresponding field
		eraseString(fields, "rgb");
	}

	//All the remaining fields will be stored as scalar fields
	for (std::list<std::string>::const_iterator name = fields.begin(); name != fields.end(); ++name)
	{
		addScalarField(cloud, *name);
	}

	return cloud;
}

bool sm2ccConverter::addXYZ(ccPointCloud *cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	size_t pointCount = getNumberOfPoints();

	if (!cloud->reserve(static_cast<unsigned>(pointCount)))
		return false;

	//add xyz to the given cloud taking xyz infos from the sm cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud);

	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 P(pcl_cloud->at(i).x,
					pcl_cloud->at(i).y,
					pcl_cloud->at(i).z);

		cloud->addPoint(P);
	}

	return true;
}

bool sm2ccConverter::addNormals(ccPointCloud *cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	pcl::PointCloud<OnlyNormals>::Ptr pcl_cloud_normals (new pcl::PointCloud<OnlyNormals>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud_normals);

	if (!cloud->reserveTheNormsTable())
		return false;

	size_t pointCount = getNumberOfPoints();

	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 N(	static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_x),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_y),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_z) );

		cloud->addNorm(N);
	}

	cloud->showNormals(true);
	
	return true;
}

bool sm2ccConverter::addRGB(ccPointCloud * cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	pcl::PointCloud<OnlyRGB>::Ptr pcl_cloud_rgb (new pcl::PointCloud<OnlyRGB>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud_rgb);

	if (!cloud->reserveTheRGBTable())
		return false;

	size_t pointCount = getNumberOfPoints();

	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		colorType C[3] = {	(colorType)pcl_cloud_rgb->points[i].r,
							(colorType)pcl_cloud_rgb->points[i].g,
							(colorType)pcl_cloud_rgb->points[i].b};
		cloud->addRGBColor(C);
	}

	cloud->showColors(true);

	return true;
}

bool sm2ccConverter::addScalarField(ccPointCloud * cloud, const std::string& name, bool overwrite_if_exist/*=true*/)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	//get the field
	PCLScalarField field = m_sm_cloud->fields.at(pcl::getFieldIndex(*m_sm_cloud, name));

	//if this field already exist, simply delete it
	int id = cloud->getScalarFieldIndexByName(name.c_str());
	if (id >= 0)
	{
		if (overwrite_if_exist)
			cloud->deleteScalarField(id);
		else
			return false;
	}

	size_t pointCount = getNumberOfPoints();

	//create new scalar field
	ccScalarField* cc_scalar_field = new ccScalarField(name.c_str());
	if (!cc_scalar_field->reserve((unsigned)pointCount))
	{
		cc_scalar_field->release();
		return false;
	}

	//get PCL field
	PCLScalarField pclField = m_sm_cloud->fields.at(pcl::getFieldIndex(*m_sm_cloud, name));

	//check if int or float
	bool floatField = (pclField.datatype == PCLScalarField::FLOAT32 || pclField.datatype == PCLScalarField::FLOAT64);

	//temporary change the name of the given field to something else -> S5c4laR should be a pretty uncommon name,
	int field_index = pcl::getFieldIndex(*m_sm_cloud, name);
	m_sm_cloud->fields[field_index].name = std::string("S5c4laR");

	if (floatField)
	{
		pcl::PointCloud<FloatScalar>::Ptr pcl_scalar(new pcl::PointCloud<FloatScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = (ScalarType)pcl_scalar->points[i].S5c4laR;
			cc_scalar_field->addElement(scalar);
		}
	}
	else
	{
		pcl::PointCloud<IntScalar>::Ptr pcl_scalar(new pcl::PointCloud<IntScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = (ScalarType)pcl_scalar->points[i].S5c4laR;
			cc_scalar_field->addElement(scalar);
		}
	}

	cc_scalar_field->computeMinAndMax();
	cloud->addScalarField(cc_scalar_field);
	cloud->setCurrentDisplayedScalarField(0);
	cloud->showSF(true);

	//restore old name for the scalar field
	m_sm_cloud->fields[field_index].name = name;

	return true;
}

size_t sm2ccConverter::getNumberOfPoints()
{
	return (size_t) (m_sm_cloud->width * m_sm_cloud->height);
}

bool sm2ccConverter::existField(std::string name) const
{
	if (m_sm_cloud)
		for (std::vector< PCLScalarField >::const_iterator it = m_sm_cloud->fields.begin(); it != m_sm_cloud->fields.end(); ++it)
			if (it->name == name)
				return true;

	return false;
}

void sm2ccConverter::eraseString(std::list<std::string> &fields, std::string name)
{
	fields.remove(name);
}

std::vector<std::string> getFieldList(const PCLCloud &cloud)
{
	std::vector<std::string> field_list;
	for (size_t d = 0; d < cloud.fields.size (); ++d)
	{
		field_list.push_back (cloud.fields[d].name);
	}

	return field_list;
}

