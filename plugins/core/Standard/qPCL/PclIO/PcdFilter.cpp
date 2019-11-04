//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#                    COPYRIGHT: CloudCompare project                     #
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

PcdFilter::PcdFilter()
    : FileIOFilter( {
                    "_Point Cloud Library Filter",
                    13.0f,	// priority
                    QStringList{ "pcd" },
                    "pcd",
                    QStringList{ "Point Cloud Library cloud (*.pcd)" },
                    QStringList{ "Point Cloud Library cloud (*.pcd)" },
                    Import | Export
                    } )
{
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

CC_FILE_ERROR PcdFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
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

	PCLCloud::Ptr pclCloud = cc2smReader(ccCloud).getAsSM();
	if (!pclCloud)
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
	if (ccCloud->size() == 0)
	{
		pcl::PCDWriter p;
		QFile file(filename);
		if (!file.open(QFile::WriteOnly | QFile::Truncate))
			return CC_FERR_WRITING;
		QTextStream stream(&file);

		stream << QString(p.generateHeaderBinary(*pclCloud, pos, ori).c_str()) << "DATA binary\n";
		return CC_FERR_NO_ERROR;
	}
	if (pcl::io::savePCDFile( qPrintable(filename), *pclCloud, pos, ori, true) < 0) //DGM: warning, toStdString doesn't preserve "local" characters
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR PcdFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	Eigen::Vector4f origin;
	Eigen::Quaternionf orientation;
	int pcd_version;
	int data_type;
	unsigned int data_idx;
	size_t pointCount = -1;
	PCLCloud::Ptr cloud_ptr_in(new PCLCloud);
	//Load the given file
	pcl::PCDReader p;

	p.readHeader(qPrintable(filename), *cloud_ptr_in, origin, orientation, pcd_version, data_type, data_idx);
	if (cloud_ptr_in)
	{
		pointCount = cloud_ptr_in->width * cloud_ptr_in->height;
		ccLog::Print(QString("%1: Point Count: %2").arg(qPrintable(filename)).arg(pointCount));
	}
	if (pointCount > 0)
	{
		if (pcl::io::loadPCDFile(qPrintable(filename), *cloud_ptr_in, origin, orientation) < 0) //DGM: warning, toStdString doesn't preserve "local" characters
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
	}

	if (!cloud_ptr_in) //loading failed?
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;

	PCLCloud::Ptr cloud_ptr;
	if (!cloud_ptr_in->is_dense) //data may contain NaNs --> remove them
	{
		//now we need to remove NaNs
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
	ccCloud->setName(QStringLiteral("unnamed"));

	//now we construct a ccGBLSensor
	{
		// get orientation as rot matrix and copy it into a ccGLMatrix
		ccGLMatrix ccRot;
		{
			Eigen::Matrix3f eigrot = orientation.toRotationMatrix();
			float* X = ccRot.getColumn(0);
			float* Y = ccRot.getColumn(1);
			float* Z = ccRot.getColumn(2);
			//Warning: Y and Z are inverted
			X[0] =  eigrot(0,0); X[1] =  eigrot(1,0); X[2] =  eigrot(2,0);
			Y[0] = -eigrot(0,2); Y[1] = -eigrot(1,2); Y[2] = -eigrot(2,2);
			Z[0] =  eigrot(0,1); Z[1] =  eigrot(1,1); Z[2] =  eigrot(2,1);

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
		sensor->setGraphicScale(ccCloud->getOwnBB().getDiagNorm() / 10);

		//Compute parameters
		ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(ccCloud);
		sensor->computeAutoParameters(pc);

		sensor->setEnabled(false);

		ccCloud->addChild(sensor);
	}

	container.addChild(ccCloud);

	return CC_FERR_NO_ERROR;
}
