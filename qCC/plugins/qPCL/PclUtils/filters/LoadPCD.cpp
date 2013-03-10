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

//Qt
#include <QApplication>
#include <QSettings>
#include <QFileDialog>

//Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

//System
#include <iostream>

LoadPCD::LoadPCD()
	: BaseFilter(FilterDescription("LoadPCD", "Load PCD FIle", "Load a PCD File", ":/toolbar/PclUtils/icons/load.png", true))
{
}

int LoadPCD::checkSelected()
{
	//we don't need any selected entity!
	return 1;
}

int LoadPCD::openDialog()
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
		QString filename = m_filenames[k];

		sensor_msgs::PointCloud2 * pcd_sensor_cloud = loadSensorMessage(filename);

		boost::shared_ptr<sensor_msgs::PointCloud2> cloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>(*pcd_sensor_cloud);
		ccPointCloud* out_cloud = sm2ccConverter(cloud_ptr).getCCloud();
		if (!out_cloud)
			return -31;

		QString cloud_name = QFileInfo(filename).baseName();
		out_cloud->setName(cloud_name);

		QFileInfo fi(filename);
		QString containerName = QString("%1 (%2)").arg(fi.fileName()).arg(fi.absolutePath());

		ccHObject* cloudContainer = new ccHObject(containerName);
		assert(out_cloud);
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
		return QString("An error occured while converting PCD to CC cloud!");
	case -21:
		return QString("No filename given, please select a pcd file.");
	}

	return BaseFilter::getErrorMessage(errorCode);
}
