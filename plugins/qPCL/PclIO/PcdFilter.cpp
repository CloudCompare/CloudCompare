//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PcdFilter.h"

//PclUtils
#include <sm2cc.h>
#include <cc2sm.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>

//Qt
#include <QFileInfo>

//Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

//System
#include <iostream>

//pcl
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

bool PcdFilter::canLoadExtension(QString upperCaseExt) const
{
	return (upperCaseExt == "PCD");
}

bool PcdFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//only one cloud per file
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = false;
		exclusive = true;
		return true;
	}

	return false;
}

CC_FILE_ERROR PcdFilter::saveToFile(ccHObject* entity, QString filename)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//the cloud to save
	ccPointCloud* ccCloud = ccHObjectCaster::ToPointCloud(entity);
	if (!ccCloud)
	{
		ccLog::Warning("[PCL] This filter can only save one cloud at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//search for a sensor as child (we take the first if there are several of them)
	ccSensor* sensor(0);
	{
		for (unsigned i=0; i<ccCloud->getChildrenNumber(); ++i)
		{
			ccHObject* child = ccCloud->getChild(i);

			//try to cast to a ccSensor
			sensor = ccHObjectCaster::ToSensor(child);
			if (sensor)
				break;
		}
	}

	PCLCloud::Ptr pclCloud(new PCLCloud);

	cc2smReader converter;
	converter.setInputCloud(ccCloud);
	if (converter.getAsSM(*pclCloud) != 1)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	Eigen::Vector4f pos;
	Eigen::Quaternionf ori;
	if (!sensor)
	{
		//we append to the cloud null sensor informations
		pos = Eigen::Vector4f::Zero();
		ori = Eigen::Quaternionf::Identity();
	}
	else
	{
		//we get out valid sensor informations
		ccGLMatrix mat = sensor->getRigidTransformation();
		CCVector3 trans = mat.getTranslationAsVec3D();
		pos(0) = trans.x;
		pos(1) = trans.y;
		pos(2) = trans.z;

		//also the rotation
		Eigen::Matrix3f eigrot;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				eigrot(i,j) = mat.getColumn(j)[i];

		//now translate to a quaternion notation
		ori = Eigen::Quaternionf(eigrot);
	}

	if (pcl::io::savePCDFile( filename.toStdString(), *pclCloud, pos, ori, true) < 0)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PcdFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	Eigen::Vector4f origin;
	Eigen::Quaternionf orientation;

	boost::shared_ptr<PCLCloud> cloud_ptr_in = loadSensorMessage(filename, origin, orientation);

	if (!cloud_ptr_in) //loading failed?
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;

	PCLCloud::Ptr cloud_ptr;
	if (!cloud_ptr_in->is_dense) //data may contain NaNs --> remove them
	{
		//now we need to remove nans
		pcl::PassThrough<PCLCloud> passFilter;
		passFilter.setInputCloud(cloud_ptr_in);

		cloud_ptr = PCLCloud::Ptr(new PCLCloud);
		passFilter.filter(*cloud_ptr);
	}
	else
	{
		cloud_ptr = cloud_ptr_in;
	}

	//convert to CC cloud
	ccPointCloud* ccCloud = sm2ccConverter(cloud_ptr).getCloud();
	if (!ccCloud)
	{
		ccLog::Warning("[PCL] An error occurred while converting PCD cloud to CloudCompare cloud!");
		return CC_FERR_CONSOLE_ERROR;
	}

	//now we construct a ccGBLSensor
	{
		// get orientation as rot matrix and copy it into a ccGLMatrix
		ccGLMatrix ccRot;
		{
			Eigen::Matrix3f eigrot = orientation.toRotationMatrix();
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					ccRot.getColumn(j)[i] = eigrot(i,j);

			ccRot.getColumn(3)[3] = 1.0f;
			ccRot.setTranslation(origin.data());
		}

		ccGBLSensor* sensor = new ccGBLSensor;
		sensor->setRigidTransformation(ccRot);
		sensor->setYawStep(static_cast<PointCoordinateType>(0.05));
		sensor->setPitchStep(static_cast<PointCoordinateType>(0.05));
		sensor->setVisible(true);
		//uncertainty to some default
		sensor->setUncertainty(static_cast<PointCoordinateType>(0.01));
		//graphic scale
		sensor->setGraphicScale(ccCloud->getBB().getDiagNorm() / 10);

		//Compute parameters
		ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(ccCloud);
		sensor->computeAutoParameters(pc);
	
		sensor->setEnabled(false);
		
		ccCloud->addChild(sensor);
	}

	container.addChild(ccCloud);

	return CC_FERR_NO_ERROR;
}
