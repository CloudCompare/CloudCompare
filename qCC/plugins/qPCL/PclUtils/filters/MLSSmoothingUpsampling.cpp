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
#include "MLSSmoothingUpsampling.h"
#include "dialogs/MLSDialog.h"
#include "filtering.h"
#include "filtering.hpp"
#include "cc2sm.h"
#include "sm2cc.h"
#include "my_point_types.h"

MLSSmoothingUpsampling::MLSSmoothingUpsampling()
	: BaseFilter(FilterDescription("MLS smoothing", "Smooth using MLS, optionally upsample", "Smooth the cloud using Moving Least Sqares algorithm, estimate normals and optionally upsample", ":/toolbar/PclUtils/icons/mls_smoothing.png", true))
	, m_dialog(0)
{
	m_parameters = new MLSParameters;
}

MLSSmoothingUpsampling::~MLSSmoothingUpsampling()
{
	if (m_dialog)
		delete m_dialog;

	if (m_parameters)
		delete m_parameters;
}

int MLSSmoothingUpsampling::compute()
{
	//pointer to selected cloud
	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	const char* current_sf_name = cloud->getCurrentDisplayedScalarField()->getName();

	//get xyz in sensor_msgs format
	cc2smReader converter;
	converter.setInputCloud(cloud);

	//take out the current scalar field
	sensor_msgs::PointCloud2  sm_field;
	sm_field = converter.getFloatScalarField(current_sf_name);
	//change its name

	std::string new_name = "scalar";
	sm_field.fields[0].name = new_name.c_str();


	//take out the xyz info
	sensor_msgs::PointCloud2  sm_xyz = converter.getXYZ();
	sensor_msgs::PointCloud2  sm_cloud;

	//put everithing together
	pcl::concatenateFields(sm_xyz, sm_field, sm_cloud);


	//get as pcl point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(sm_cloud, *pcl_cloud);

	//create storage for outcloud
	pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);

	pcl::PointIndicesPtr mapping_indices;
	smooth_mls<pcl::PointXYZ, pcl::PointNormal> (pcl_cloud, *m_parameters, normals, mapping_indices);

	sensor_msgs::PointCloud2::Ptr sm_normals (new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*normals, *sm_normals);

	sm2ccReader reader;
	reader.setInputCloud(sm_normals);

	ccPointCloud * new_cloud = new ccPointCloud;

	reader.setInputCloud(sm_normals);
	reader.getAsCC(new_cloud);
	new_cloud->setName(cloud->getName()+QString("_smoothed")); //original name + suffix

	//copy the original scalar fields here
	copyScalarFields(cloud, new_cloud, mapping_indices, true);

	if (cloud->getParent())
		cloud->getParent()->addChild(new_cloud);

	emit newEntity(new_cloud);
	return 1;
}

int MLSSmoothingUpsampling::openDialog()
{
	if (!m_dialog)
		m_dialog = new MLSDialog();

	return m_dialog->exec() ? 1 : 0;
}

void MLSSmoothingUpsampling::getParametersFromDialog()
{
	//we need to read all the parameters and put them into m_parameters
	m_parameters->search_radius_ = m_dialog->search_radius->value();
	m_parameters->compute_normals_ = m_dialog->compute_normals->checkState() ;
	m_parameters->polynomial_fit_ = m_dialog->use_polynomial->checkState();
	m_parameters->order_ = m_dialog->polynomial_order->value();
	m_parameters->sqr_gauss_param_ = m_dialog->squared_gaussian_parameter->value() ;

	int index_now = m_dialog->upsampling_method->currentIndex();
	QVariant current_status = m_dialog->upsampling_method->itemData(index_now);
	int status = current_status.toInt();

	m_parameters->upsample_method_= ( MLSParameters::UpsamplingMethod) status;

	m_parameters->upsampling_radius_ = m_dialog->upsampling_radius->value();
	m_parameters->upsampling_step_ = m_dialog->upsampling_step_size->value() ;
	m_parameters->step_point_density_ = m_dialog->step_point_density->value() ;
	m_parameters->dilation_voxel_size_ = m_dialog->dilation_voxel_size->value() ;
}

template int smooth_mls<pcl::PointXYZ, pcl::PointNormal>(const pcl::PointCloud<pcl::PointXYZ>::Ptr &incloud,
	const MLSParameters &params,
	pcl::PointCloud<pcl::PointNormal>::Ptr &outcloud,
	pcl::PointIndicesPtr &used_ids);
