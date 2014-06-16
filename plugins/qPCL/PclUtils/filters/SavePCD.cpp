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
//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <ccIncludeGL.h> //Always first normally, but imports 'min' and 'max' definitions that clash with PCL ones (DGM: from where?)
//#include <GL/glut.h>

#include "SavePCD.h"
#include <cc2sm.h>

//qCC_db
#include <ccPolyline.h>
#include <ccPointCloud.h>
#include <ccHObject.h>
#include <ccSensor.h>
#include <ccHObjectCaster.h>

//CCLib
#include <GenericIndexedCloudPersist.h>

//Qt
#include <QFileDialog>
#include <QSettings>
#include <QApplication>

//System
#include <iostream>


SavePCD::SavePCD()
	: BaseFilter(FilterDescription(	"SavePCD",
									"Save as PCD FIle",
									"Save selected entitiy as PCD File",
									":/toolbar/PclUtils/icons/save.png") )
{
}

int SavePCD::openInputDialog()
{
	//persistent settings
	QSettings settings;
	settings.beginGroup("PclUtils/SavePCD");
	QString currentPath = settings.value("currentPath",QApplication::applicationDirPath()).toString();
	QString dir = currentPath+QString("/");

	//file choosing dialog
	//We store the result directly in 'm_filename' as it is simpler.
	//Thus we also bypass getParametersFromDialog, but we avoid keeping
	//the QFileDialog as a member...
	m_filename = QFileDialog::getSaveFileName(	0,
												tr("Save file"),
												dir,
												"PCD file (*.pcd)");
	if (m_filename.isEmpty())
		return 0;

	//save last save destination
	currentPath = QFileInfo(m_filename).absolutePath();
	settings.setValue("currentPath",currentPath);
	settings.endGroup();

	return 1;
}

int SavePCD::compute()
{
	ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	//search for a sensor as child
	size_t n_childs = cloud->getChildrenNumber();
	ccSensor * sensor(0);
	for (size_t i = 0; i < n_childs; ++i)
	{
		ccHObject * child = cloud->getChild(static_cast<unsigned>(i));

		//try to cast to a ccSensor
		if (!child->isKindOf(CC_TYPES::SENSOR))
			continue;

		sensor = ccHObjectCaster::ToSensor(child);
	}

	PCLCloud::Ptr out_cloud(new PCLCloud);

	cc2smReader converter;
	converter.setInputCloud(cloud);
	int result = converter.getAsSM(*out_cloud);

	Eigen::Vector4f pos;
	Eigen::Quaternionf ori;
	if(!sensor)
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
		pos(0) = trans[0];
		pos(1) = trans[1];
		pos(2) = trans[2];

		//also the rotation
		Eigen::Matrix3f eigrot;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				eigrot(i,j) = mat.getColumn(j)[i];

		// now translate to a quaternion notation
		ori = Eigen::Quaternionf(eigrot);
	}

	if (result != 1)
	{
		return -31;
	}

	if (pcl::io::savePCDFile( m_filename.toStdString(), *out_cloud, pos, ori, true) < 0)
	{
		return -32;
	}

	return 1;
}
