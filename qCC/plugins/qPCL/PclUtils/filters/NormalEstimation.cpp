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
#include "NormalEstimation.h"
#include "dialogs/NormalEstimationDlg.h"

#include <cc2sm.h>
#include <filtering.h>
#include <filtering.hpp>
#include <my_point_types.h>
#include <sm2cc.h>

//PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



NormalEstimation::NormalEstimation()
	: BaseFilter(FilterDescription("Estimate Normals", "Estimate Normals and Curvature", "Estimate Normals and Curvature for the selected entity", ":/toolbar/PclUtils/icons/normal_curvature.png", true))
	, m_dialog(0)
	, m_radius(0)
	, m_knn_radius(10)
	, m_useKnn(false)
	, m_overwrite_curvature(false)
{
    m_overwrite_curvature = true;
}

NormalEstimation::~NormalEstimation()
{
	if (m_dialog)
		delete m_dialog;
}

int NormalEstimation::openDialog()
{
	if (!m_dialog)
	{
		m_dialog = new NormalEstimationDialog;
		//initially these are invisible
		m_dialog->surfaceComboBox->setVisible(false);
		m_dialog->searchSurfaceCheckBox->setVisible(false);
	}

	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (cloud)
	{
		ccBBox bBox = cloud->getBB(true,false);
		if (bBox.isValid())
			m_dialog->radiusDoubleSpinBox->setValue(bBox.getDiagNorm() * 0.005);
	}

	return m_dialog->exec() ? 1 : 0;
}

void NormalEstimation::getParametersFromDialog()
{
	assert(m_dialog);
	if (!m_dialog)
		return;


    //fill in parameters from dialog
    m_useKnn = m_dialog->useKnnCheckBox->isChecked();
    m_overwrite_curvature = m_dialog->curvatureCheckBox->isChecked();
    m_knn_radius = m_dialog->knnSpinBox->value();
    m_radius = m_dialog->radiusDoubleSpinBox->value();
}

int NormalEstimation::compute()
{
    //pointer to selected cloud
    ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
        if (!cloud)
                return -1;

    //get xyz in sensor_msgs format
    cc2smReader converter;
    converter.setInputCloud(cloud);
    sensor_msgs::PointCloud2  sm_cloud = converter.getXYZ();

    //get as pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(sm_cloud, *pcl_cloud);

    //create storage for normals
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);

    //now compute
    int result = compute_normals<pcl::PointXYZ, pcl::PointNormal>(pcl_cloud, m_useKnn ? m_knn_radius: m_radius, m_useKnn, normals);

    sensor_msgs::PointCloud2::Ptr sm_normals (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*normals, *sm_normals);

    sm2ccReader reader;
    reader.setInputCloud(sm_normals);


    reader.addNormals(cloud);
    reader.addFloatField(cloud, "curvature", m_overwrite_curvature);

    cloud->showNormals(true);

    emit entityHasChanged(cloud);

    return 1;
}

//INSTANTIATING TEMPLATED FUNCTIONS
template int compute_normals<pcl::PointXYZ, pcl::PointNormal> (const  pcl::PointCloud<pcl::PointXYZ>::Ptr incloud,
                                                               const float radius,
                                                               const bool mode, //true if use knn, false if radius search
                                                               pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
