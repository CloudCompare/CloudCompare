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

cc2smReader::cc2smReader(ccPointCloud* cccloud)
	: m_ccCloud(cccloud)
{
	assert(m_ccCloud);
}

std::string cc2smReader::GetSimplifiedSFName(const QString& ccSfName)
{
	QString simplified = ccSfName.simplified();
	simplified.replace(' ', '_');

	return simplified.toStdString();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cc2smReader::getRawXYZ() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<PointXYZ>::Ptr xyzCloud(new PointCloud<PointXYZ>);
	try
	{
		unsigned pointCount = m_ccCloud->size();
		xyzCloud->resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = m_ccCloud->getPoint(i);
			xyzCloud->at(i).x = static_cast<float>(P->x);
			xyzCloud->at(i).y = static_cast<float>(P->y);
			xyzCloud->at(i).z = static_cast<float>(P->z);
		}
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return xyzCloud;
}

PCLCloud::Ptr cc2smReader::getXYZ() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<PointXYZ>::Ptr xyzCloud = getRawXYZ();
	if (!xyzCloud)
	{
		return {};
	}

	PCLCloud::Ptr outputCloud;
	try
	{
		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(*xyzCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}
	
	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getNormals() const
{
	if (!m_ccCloud || !m_ccCloud->hasNormals())
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;
	try
	{
		PointCloud<OnlyNormals> normalsCloud;

		unsigned pointCount = m_ccCloud->size();
		normalsCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3& N = m_ccCloud->getPointNormal(i);
			normalsCloud[i].normal_x = N.x;
			normalsCloud[i].normal_y = N.y;
			normalsCloud[i].normal_z = N.z;
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(normalsCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}
	
	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getColors() const
{
	if (!m_ccCloud || !m_ccCloud->hasColors())
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;

	try
	{
		PointCloud<OnlyRGB> rgbCloud;
		unsigned pointCount = m_ccCloud->size();
		rgbCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			const ccColor::Rgb& rgb = m_ccCloud->getPointColor(i);
			rgbCloud[i].r = static_cast<uint8_t>(rgb.r);
			rgbCloud[i].g = static_cast<uint8_t>(rgb.g);
			rgbCloud[i].b = static_cast<uint8_t>(rgb.b);
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(rgbCloud, *outputCloud);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getFloatScalarField(const QString& fieldName) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	int sfIdx = m_ccCloud->getScalarFieldIndexByName(fieldName.toStdString());
	if (sfIdx < 0)
	{
		return {};
	}

	CCCoreLib::ScalarField* sf = m_ccCloud->getScalarField(sfIdx);
	assert(sf);

	PCLCloud::Ptr outputCloud;

	try
	{
		PointCloud<FloatScalar> sfCloud;

		unsigned pointCount = m_ccCloud->size();
		sfCloud.resize(pointCount);

		for (unsigned i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = sf->getValue(i);
			sfCloud[i].S5c4laR = static_cast<float>(scalar);
		}

		outputCloud.reset(new PCLCloud);
		TO_PCL_CLOUD(sfCloud, *outputCloud);

		//Now change the name of the scalar field -> we cannot have any space into the field name
		//NOTE this is a little trick to put any number of scalar fields in a message PointCloud2 object
		//We use a point type with a generic scalar field named scalar. we load the scalar field and
		//then we change the name to the needed one
		outputCloud->fields[0].name = GetSimplifiedSFName(fieldName);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	return outputCloud;
}

PCLCloud::Ptr cc2smReader::getAsSM() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	//list of scalar fields
	QStringList scalarFields;
	for (unsigned i = 0; i < m_ccCloud->getNumberOfScalarFields(); ++i)
	{
		scalarFields << QString::fromStdString(m_ccCloud->getScalarField(static_cast<int>(i))->getName());
	}

	return getAsSM(true, m_ccCloud->hasNormals(), m_ccCloud->hasColors(), scalarFields);
}

static PCLCloud::Ptr SetOrAdd(PCLCloud::Ptr firstCloud, PCLCloud::Ptr secondCloud)
{
	if (!secondCloud)
	{
		assert(false);
		return {};
	}

	if (firstCloud)
	{
		try
		{
			PCLCloud::Ptr tempCloud(new PCLCloud); //temporary cloud
			pcl::concatenateFields(*firstCloud, *secondCloud, *tempCloud);
			return tempCloud;
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("Not enough memory");
			return nullptr;
		}
	}
	else
	{
		return secondCloud;
	}
}

PCLCloud::Ptr cc2smReader::getAsSM(bool xyz, bool normals, bool rgbColors, const QStringList& scalarFields) const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PCLCloud::Ptr outputCloud;

	unsigned pointCount = m_ccCloud->size();

	try
	{
		if (xyz)
		{
			PCLCloud::Ptr xyzCloud = getXYZ();
			if (!xyzCloud)
			{
				return {};
			}

			outputCloud = SetOrAdd(outputCloud, xyzCloud);
		}

		if (normals && m_ccCloud->hasNormals())
		{
			PCLCloud::Ptr normalsCloud = getNormals();
			if (!normalsCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, normalsCloud);
		}

		if (rgbColors && m_ccCloud->hasColors())
		{
			PCLCloud::Ptr rgbCloud = getColors();
			if (!rgbCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, rgbCloud);
		}

		for (const QString& sfName : scalarFields)
		{
			PCLCloud::Ptr sfCloud = getFloatScalarField(sfName);
			if (!sfCloud)
			{
				return {};
			}
			outputCloud = SetOrAdd(outputCloud, sfCloud);
		}
	}
	catch (...)
	{
		//any error (memory, etc.)
		outputCloud.reset();
	}

	return outputCloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr cc2smReader::getAsPointNormal() const
{
	if (!m_ccCloud)
	{
		assert(false);
		return {};
	}

	PointCloud<pcl::PointNormal>::Ptr pcl_cloud(new PointCloud<pcl::PointNormal>);

	unsigned pointCount = m_ccCloud->size();

	try
	{
		pcl_cloud->resize(pointCount);
	}
	catch (...)
	{
		//any error (memory, etc.)
		return {};
	}

	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* P = m_ccCloud->getPoint(i);
		pcl_cloud->at(i).x = static_cast<float>(P->x);
		pcl_cloud->at(i).y = static_cast<float>(P->y);
		pcl_cloud->at(i).z = static_cast<float>(P->z);
	}

	if (m_ccCloud->hasNormals())
	{
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* N = m_ccCloud->getNormal(i);
			pcl_cloud->at(i).normal_x = static_cast<float>(N->x);
			pcl_cloud->at(i).normal_y = static_cast<float>(N->y);
			pcl_cloud->at(i).normal_z = static_cast<float>(N->z);
		}
	}
	
	return pcl_cloud;
}
