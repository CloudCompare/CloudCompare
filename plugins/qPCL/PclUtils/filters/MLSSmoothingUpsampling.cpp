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

#include "MLSSmoothingUpsampling.h"

//Local
#include "dialogs/MLSDialog.h"
#include "filtering.h"
#include "filtering.hpp"
#include "cc2sm.h"
#include "sm2cc.h"
#include "my_point_types.h"

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccScalarField.h>

//Qt
#include <QMainWindow>

MLSSmoothingUpsampling::MLSSmoothingUpsampling()
	: BaseFilter(FilterDescription(	"MLS smoothing",
									"Smooth using MLS, optionally upsample",
									"Smooth the cloud using Moving Least Sqares algorithm, estimate normals and optionally upsample",
									":/toolbar/PclUtils/icons/mls_smoothing.png"))
	, m_dialog(0)
	, m_parameters(new MLSParameters)
{
}

MLSSmoothingUpsampling::~MLSSmoothingUpsampling()
{
	//we must delete parent-less dialogs ourselves!
	if (m_dialog && m_dialog->parent() == 0)
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

	//get xyz in PCL format
	cc2smReader converter;
	converter.setInputCloud(cloud);

	//take out the xyz info
	PCLCloud sm_xyz = converter.getXYZ();
	PCLCloud sm_cloud;

	//take out the current scalar field (if any)
	if (cloud->getCurrentDisplayedScalarField())
	{
		const char* current_sf_name = cloud->getCurrentDisplayedScalarField()->getName();

		PCLCloud sm_field = converter.getFloatScalarField(current_sf_name);
		//change its name
		std::string new_name = "scalar";
		sm_field.fields[0].name = new_name.c_str();
		//put everithing together
		pcl::concatenateFields(sm_xyz, sm_field, sm_cloud);
	}
	else
	{
		sm_cloud = sm_xyz;
	}

	//get as pcl point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
	FROM_PCL_CLOUD(sm_cloud, *pcl_cloud);

	//create storage for outcloud
	pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
#ifdef LP_PCL_PATCH_ENABLED
	pcl::PointIndicesPtr mapping_indices;
	smooth_mls<pcl::PointXYZ, pcl::PointNormal> (pcl_cloud, *m_parameters, normals, mapping_indices);
#else
	smooth_mls<pcl::PointXYZ, pcl::PointNormal> (pcl_cloud, *m_parameters, normals);
#endif

	PCLCloud::Ptr sm_normals (new PCLCloud);
	TO_PCL_CLOUD(*normals, *sm_normals);

	ccPointCloud* new_cloud = sm2ccConverter(sm_normals).getCloud();
	if (!new_cloud)
	{
		//conversion failed (not enough memory?)
		return -1;
	}

	new_cloud->setName(cloud->getName()+QString("_smoothed")); //original name + suffix
	new_cloud->setDisplay(cloud->getDisplay());

#ifdef LP_PCL_PATCH_ENABLED
	//copy the original scalar fields here
	copyScalarFields(cloud, new_cloud, mapping_indices, true);
	//copy the original colors here
	copyRGBColors(cloud, new_cloud, mapping_indices, true);
#endif

	//disable original cloud
	cloud->setEnabled(false);
	if (cloud->getParent())
		cloud->getParent()->addChild(new_cloud);

	emit newEntity(new_cloud);

	return 1;
}

int MLSSmoothingUpsampling::openInputDialog()
{
	if (!m_dialog)
		m_dialog = new MLSDialog(m_app ? m_app->getMainWindow() : 0);

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
	pcl::PointCloud<pcl::PointNormal>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
  ,	pcl::PointIndicesPtr &used_ids
#endif
  );
