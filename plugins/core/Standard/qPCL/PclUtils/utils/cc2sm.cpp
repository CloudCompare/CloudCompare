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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "cc2sm.h"

//Local
#include "my_point_types.h"
#include "PCLConv.h"

//PCL
#include <pcl/common/io.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//system
#include <assert.h>

using namespace pcl;

cc2smReader::cc2smReader(const ccPointCloud * cc_cloud)
	: m_cc_cloud(cc_cloud)
{
	assert(m_cc_cloud);
}

PCLCloud::Ptr cc2smReader::getGenericField(std::string field_name) const
{
	PCLCloud::Ptr sm_cloud;

	if (field_name == "x")
	{
		sm_cloud = getOneOf(COORD_X);
	}
	else if (field_name == "y")
	{
		sm_cloud = getOneOf(COORD_Y);
	}
	else if (field_name == "z")
	{
		sm_cloud = getOneOf(COORD_Y);
	}
	else if (field_name == "normal_x")
	{
		sm_cloud = getOneOf(NORM_X);
	}
	else if (field_name == "normal_y")
	{
		sm_cloud = getOneOf(NORM_Y);
	}
	else if (field_name == "normal_z")
	{
		sm_cloud = getOneOf(NORM_Z);
	}
	else if (field_name == "xyz")
	{
		sm_cloud = getXYZ();
	}
	else if (field_name == "normal_xyz")
	{
		sm_cloud = getNormals();
	}
	else if (field_name == "rgb")
	{
		sm_cloud = getColors();
	}
	else //try to load the field from the scalar fields
	{
		sm_cloud = getFloatScalarField(field_name);
	}

	return sm_cloud;
}

PCLCloud::Ptr cc2smReader::getOneOf(Fields field) const
{
	assert(m_cc_cloud);

	PCLCloud::Ptr sm_cloud;

	std::string name;
	unsigned char dim = 0;
	switch (field)
	{
	case COORD_X:
		name = "x";
		dim = 0;
		break;
	case COORD_Y:
		name = "y";
		dim = 1;
		break;
	case COORD_Z:
		name = "z";
		dim = 2;
		break;
	case NORM_X:
		if (!m_cc_cloud->hasNormals())
			return sm_cloud;
		name = "normal_x";
		dim = 0;
		break;
	case NORM_Y:
		if (!m_cc_cloud->hasNormals())
			return sm_cloud;
		name = "normal_y";
		dim = 1;
		break;
	case NORM_Z:
		if (!m_cc_cloud->hasNormals())
			return sm_cloud;
		name = "normal_z";
		dim = 2;
		break;
	default:
		//unhandled field?!
		assert(false);
		return sm_cloud;
	};

	assert(/*dim >= 0 && */dim <= 2);

	try
	{
		PointCloud<FloatScalar>::Ptr pcl_cloud (new PointCloud<FloatScalar>);

		unsigned pointCloud = m_cc_cloud->size();
		pcl_cloud->resize(pointCloud);

		for (unsigned i = 0; i < pointCloud; ++i)
		{
			switch(field)
			{
			case COORD_X:
			case COORD_Y:
			case COORD_Z:
				{
					const CCVector3* P = m_cc_cloud->getPoint(i);
					pcl_cloud->at(i).S5c4laR = static_cast<float>(P->u[dim]);
				}
				break;
			case NORM_X:
			case NORM_Y:
			case NORM_Z:
				{
					const CCVector3& N = m_cc_cloud->getPointNormal(i);
					pcl_cloud->at(i).S5c4laR = static_cast<float>(N.u[dim]);
				}
				break;
			default:
				//unhandled field?!
				assert(false);
				break;
			};
		}

		sm_cloud = PCLCloud::Ptr(new PCLCloud);
		TO_PCL_CLOUD(*pcl_cloud, *sm_cloud);
		sm_cloud->fields[0].name = name;
	}
	catch(...)
	{
		//any error (memory, etc.)
		sm_cloud.reset();
	}
	return sm_cloud;
}

PointCloud<PointXYZ>::Ptr cc2smReader::getXYZ2() const
{
	assert(m_cc_cloud);

	PointCloud<PointXYZ>::Ptr pcl_cloud (new PointCloud<PointXYZ>);
	try
	{
		unsigned pointCount = m_cc_cloud->size();
		pcl_cloud->resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = m_cc_cloud->getPoint(i);
			pcl_cloud->at(i).x = static_cast<float>(P->x);
			pcl_cloud->at(i).y = static_cast<float>(P->y);
			pcl_cloud->at(i).z = static_cast<float>(P->z);
		}
	}
	catch(...)
	{
		//any error (memory, etc.)
		pcl_cloud.reset();
	}
	
	return pcl_cloud;
}

PCLCloud::Ptr cc2smReader::getXYZ() const
{
	PCLCloud::Ptr sm_cloud;
	
	PointCloud<PointXYZ>::Ptr pcl_cloud = getXYZ2();
	if (pcl_cloud)
	{
		sm_cloud = PCLCloud::Ptr(new PCLCloud);
		TO_PCL_CLOUD(*pcl_cloud, *sm_cloud);
	}

	return sm_cloud;
}

PCLCloud::Ptr cc2smReader::getNormals() const
{
	if (!m_cc_cloud || !m_cc_cloud->hasNormals())
		return PCLCloud::Ptr(static_cast<PCLCloud*>(0));

	PCLCloud::Ptr sm_cloud (new PCLCloud);
	try
	{
		PointCloud<OnlyNormals>::Ptr pcl_cloud (new PointCloud<OnlyNormals>);

		unsigned pointCount = m_cc_cloud->size();
		pcl_cloud->resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3& N = m_cc_cloud->getPointNormal(i);
			pcl_cloud->at(i).normal_x = N.x;
			pcl_cloud->at(i).normal_y = N.y;
			pcl_cloud->at(i).normal_z = N.z;
		}

		TO_PCL_CLOUD(*pcl_cloud, *sm_cloud);
	}
	catch(...)
	{
		//any error (memory, etc.)
		sm_cloud.reset();
	}
	
	return sm_cloud;
}

PCLCloud::Ptr cc2smReader::getColors() const
{
	if (!m_cc_cloud || !m_cc_cloud->hasColors())
		return PCLCloud::Ptr(static_cast<PCLCloud*>(0));

	PCLCloud::Ptr sm_cloud (new PCLCloud);
	try
	{
		PointCloud<OnlyRGB>::Ptr pcl_cloud (new PointCloud<OnlyRGB>);

		unsigned pointCount = m_cc_cloud->size();
		pcl_cloud->resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const ccColor::Rgb& rgb = m_cc_cloud->getPointColor(i);
			pcl_cloud->at(i).r = static_cast<uint8_t>(rgb.r);
			pcl_cloud->at(i).g = static_cast<uint8_t>(rgb.g);
			pcl_cloud->at(i).b = static_cast<uint8_t>(rgb.b);
		}

		TO_PCL_CLOUD(*pcl_cloud, *sm_cloud);
	}
	catch(...)
	{
		//any error (memory, etc.)
		sm_cloud.reset();
	}

	return sm_cloud;
}

std::string cc2smReader::GetSimplifiedSFName(const std::string& ccSfName)
{
	QString simplified = QString::fromStdString(ccSfName).simplified();
	simplified.replace(' ', '_');
	return simplified.toStdString();
}

PCLCloud::Ptr cc2smReader::getFloatScalarField(const std::string& field_name) const
{
	assert(m_cc_cloud);

	int sfIdx = m_cc_cloud->getScalarFieldIndexByName(field_name.c_str());
	if (sfIdx < 0)
		return PCLCloud::Ptr(static_cast<PCLCloud*>(0));
	CCLib::ScalarField* scalar_field = m_cc_cloud->getScalarField(sfIdx);
	assert(scalar_field);

	PCLCloud::Ptr sm_cloud (new PCLCloud);
	try
	{
		unsigned pointCount = m_cc_cloud->size();

		PointCloud<FloatScalar>::Ptr pcl_cloud (new PointCloud<FloatScalar>);
		pcl_cloud->resize(pointCount);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = scalar_field->getValue(i);
			pcl_cloud->at(i).S5c4laR = static_cast<float>(scalar);
		}

		TO_PCL_CLOUD(*pcl_cloud, *sm_cloud);

		//Now change the name of the scalar field -> we cannot have any space into the field name
		//NOTE this is a little trick to put any number of scalar fields in a message PointCloud2 object
		//We use a point type with a generic scalar field named scalar. we load the scalar field and
		//then we change the name to the needed one
		sm_cloud->fields[0].name = GetSimplifiedSFName(field_name);
	}
	catch(...)
	{
		//any error (memory, etc.)
		sm_cloud.reset();
	}

	return sm_cloud;
}

bool cc2smReader::checkIfFieldExists(const std::string& field_name) const
{
	if ((field_name == "x") || (field_name == "y") || (field_name == "z") || (field_name == "xyz"))
		return true;// (m_cc_cloud->size() != 0);

	else if ( (field_name == "normal_x") || (field_name == "normal_y") || (field_name == "normal_z") || (field_name == "normal_xyz") )
		return m_cc_cloud->hasNormals();

	else if (field_name == "rgb")
		return m_cc_cloud->hasColors();

	else
		return (m_cc_cloud->getScalarFieldIndexByName(field_name.c_str()) >= 0);
}

PCLCloud::Ptr cc2smReader::getAsSM(std::list<std::string>& requested_fields) const
{
	//preliminary check
	{
		for (std::list<std::string>::const_iterator it = requested_fields.begin(); it != requested_fields.end(); ++it)
		{
			bool exists = checkIfFieldExists(*it);
			if (!exists) //all check results must be true
				return PCLCloud::Ptr(static_cast<PCLCloud*>(0));
		}
	}

	//are we asking for x, y, and z all togheters?
	bool got_xyz = (std::find(requested_fields.begin(), requested_fields.end(), "xyz") != requested_fields.end());
	if (got_xyz)
	{
		//remove from the requested fields lists x y and z as single occurrencies
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("x")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("y")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("z")), requested_fields.end());
	}

	//same for normals
	bool got_normal_xyz = (std::find(requested_fields.begin(), requested_fields.end(), "normal_xyz") != requested_fields.end());
	if (got_normal_xyz)
	{
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_x")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_y")), requested_fields.end());
		requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_z")), requested_fields.end());
	}

	//a vector for PointCloud2 clouds
	PCLCloud::Ptr firstCloud;

	//load and merge fields/clouds one-by-one
	{
		for (std::list<std::string>::const_iterator it = requested_fields.begin(); it != requested_fields.end(); ++it)
		{
			if (!firstCloud)
			{
				firstCloud = getGenericField(*it);
			}
			else
			{
				PCLCloud::Ptr otherCloud = getGenericField(*it);
				if (otherCloud)
				{
					PCLCloud::Ptr sm_tmp (new PCLCloud); //temporary cloud
					pcl::concatenateFields(*firstCloud, *otherCloud, *sm_tmp);
					firstCloud = sm_tmp;
				}
			}
		}
	}

	return firstCloud;
}

PCLCloud::Ptr cc2smReader::getAsSM() const
{
	//does the cloud have some points?
	if (!m_cc_cloud)// || m_cc_cloud->size() == 0)
	{
		assert(false);
		return PCLCloud::Ptr(static_cast<PCLCloud*>(0));
	}

	//container
	std::list<std::string> fields;
	try
	{
		fields.push_back("xyz");
		if (m_cc_cloud->hasNormals())
			fields.push_back("normal_xyz");
		if (m_cc_cloud->hasColors())
			fields.push_back("rgb");
		for (unsigned i = 0; i < m_cc_cloud->getNumberOfScalarFields(); ++i)
			fields.push_back(m_cc_cloud->getScalarField(static_cast<int>(i))->getName());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return PCLCloud::Ptr(static_cast<PCLCloud*>(0));
	}

	return getAsSM(fields);
}
