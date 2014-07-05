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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################
//

#include "LoadPCD.h"

//utils
#include <sm2cc.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>

//Qt
#include <QApplication>
#include <QSettings>
#include <QFileDialog>

//Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

//System
#include <iostream>

//pcl
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

LoadPCD::LoadPCD()
	: BaseFilter(FilterDescription(	"LoadPCD",
									"Load PCD FIle",
									"Load a PCD File",
									":/toolbar/PclUtils/icons/load.png"))
{
}

int LoadPCD::checkSelected()
{
	//we don't need any selected entity!
	return 1;
}

int LoadPCD::openInputDialog()
{
	QSettings settings;
	settings.beginGroup("PclUtils/LoadPCD");
	QString currentPath = settings.value("currentPath",QApplication::applicationDirPath()).toString();

	//file choosing dialog
	//We store the result directly in 'm_filenames' as it is simpler.
	//Thus we also bypass getParametersFromDialog, but we avoid keeping
	//the QFileDialog as a member...
	m_filenames = QFileDialog::getOpenFileNames(0,
		tr("Open PCD file(s)"),
		currentPath,
		"PCD file (*.pcd)");

	if (m_filenames.isEmpty())
		return 0;

	//save file loading location
	currentPath = QFileInfo(m_filenames[0]).absolutePath();
	settings.setValue("currentPath",currentPath);
	settings.endGroup();

	return 1;
}

int LoadPCD::compute()
{
	//for each selected filename
	for (int k = 0; k < m_filenames.size(); ++k)
	{
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;

		QString filename = m_filenames[k];

		boost::shared_ptr<PCLCloud> cloud_ptr_in = loadSensorMessage(filename, origin, orientation);

		if (!cloud_ptr_in) //loading failed?
			return 0;

		PCLCloud::Ptr cloud_ptr;
		if (!cloud_ptr_in->is_dense) //data may contain nans. Remove them
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

		//now we construct a ccGBLSensor with these characteristics
		ccGBLSensor * sensor = new ccGBLSensor;

		// get orientation as rot matrix
		Eigen::Matrix3f eigrot = orientation.toRotationMatrix();

		// and copy it into a ccGLMatrix
		ccGLMatrix ccRot;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				ccRot.getColumn(j)[i] = eigrot(i,j);
			}
		}

		ccRot.getColumn(3)[3] = 1.0;

		// now in a format good for CloudComapre
		//ccGLMatrix ccRot = ccGLMatrix::FromQuaternion(orientation.coeffs().data());

		//ccRot = ccRot.transposed();
		ccRot.setTranslation(origin.data());

		sensor->setRigidTransformation(ccRot);
		sensor->setDeltaPhi(static_cast<PointCoordinateType>(0.05));
		sensor->setDeltaTheta(static_cast<PointCoordinateType>(0.05));
		sensor->setVisible(true);

		//uncertainty to some default
		sensor->setUncertainty(static_cast<PointCoordinateType>(0.01));

		ccPointCloud* out_cloud = sm2ccConverter(cloud_ptr).getCloud();
		if (!out_cloud)
			return -31;

		sensor->setGraphicScale(out_cloud->getBB().getDiagNorm() / 10);

		//do the projection on sensor
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(out_cloud);
		int errorCode;
		CCLib::SimpleCloud* projectedCloud = sensor->project(cloud,errorCode,true);
		if (projectedCloud)
		{
			//DGM: we don't use it but we still have to delete it!
			delete projectedCloud;
			projectedCloud = 0;
		}

		QString cloud_name = QFileInfo(filename).baseName();
		out_cloud->setName(cloud_name);

		QFileInfo fi(filename);
		QString containerName = QString("%1 (%2)").arg(fi.fileName()).arg(fi.absolutePath());

		ccHObject* cloudContainer = new ccHObject(containerName);
		out_cloud->addChild(sensor);
		cloudContainer->addChild(out_cloud);

		emit newEntity(cloudContainer);
	}

	return 1;
}

QString LoadPCD::getErrorMessage(int errorCode)
{
	switch(errorCode)
	{
		//THESE CASES CAN BE USED TO OVERRIDE OR ADD FILTER-SPECIFIC ERRORS CODES
		//ALSO IN DERIVED CLASSES DEFAULT MUST BE ""
	case -31:
		return QString("An error occurred while converting PCD to CC cloud!");
	case -21:
		return QString("No filename given, please select a pcd file.");
	}

	return BaseFilter::getErrorMessage(errorCode);
}
