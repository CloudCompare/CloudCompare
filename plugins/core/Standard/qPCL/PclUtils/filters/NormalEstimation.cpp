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
#include "NormalEstimation.h"

//Local
#include "dialogs/NormalEstimationDlg.h"
#include "../utils/PCLConv.h"
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//PCL
#include <pcl/features/impl/normal_3d_omp.hpp>

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>

template <typename PointInT, typename PointOutT>
int compute_normals(const typename pcl::PointCloud<PointInT>::Ptr incloud,
					const float radius,
					const bool useKnn, //true if use knn, false if radius search
					typename pcl::PointCloud<PointOutT>::Ptr outcloud)
{
	typename pcl::NormalEstimationOMP<PointInT, PointOutT> normal_estimator;
	//typename pcl::PointCloud<PointOutT>::Ptr normals (new pcl::PointCloud<PointOutT>);

	if (useKnn) //use knn
	{
		int knn_radius = (int) radius; //cast to int
		normal_estimator.setKSearch(knn_radius);
	}
	else //use radius search
	{
		normal_estimator.setRadiusSearch(radius);
	}

	normal_estimator.setInputCloud (incloud);
	//normal_estimator.setNumberOfThreads(4);
	normal_estimator.compute (*outcloud);

	return 1;
}

NormalEstimation::NormalEstimation()
	: BaseFilter(FilterDescription(	"Estimate Normals",
									"Estimate Normals and Curvature",
									"Estimate Normals and Curvature for the selected entity",
									":/toolbar/PclUtils/icons/normal_curvature.png"))
	, m_dialog(nullptr)
	, m_dialogHasParent(false)
	, m_radius(0)
	, m_knn_radius(10)
	, m_useKnn(false)
	, m_overwrite_curvature(false)
{
	m_overwrite_curvature = true;
}

NormalEstimation::~NormalEstimation()
{
	//we must delete parent-less dialogs ourselves!
	if (!m_dialogHasParent && m_dialog && m_dialog->parent() == nullptr)
		delete m_dialog;
}

int NormalEstimation::openInputDialog()
{
	if (!m_dialog)
	{
		m_dialog = new NormalEstimationDialog(m_app ? m_app->getMainWindow() : nullptr);
		m_dialogHasParent = (m_dialog->parent() != nullptr);
		
		//initially these are invisible
		m_dialog->surfaceComboBox->setVisible(false);
		m_dialog->searchSurfaceCheckBox->setVisible(false);
	}

	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (cloud)
	{
		ccBBox bBox = cloud->getOwnBB();
		if (bBox.isValid())
			m_dialog->radiusDoubleSpinBox->setValue(bBox.getDiagNorm() / 200);
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
	m_radius = static_cast<float>(m_dialog->radiusDoubleSpinBox->value());
}

int NormalEstimation::compute()
{
	//pointer to selected cloud
	ccPointCloud* cloud = getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	//if we have normals delete them!
	if (cloud->hasNormals())
		cloud->unallocateNorms();

	//get xyz as pcl point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = cc2smReader(cloud).getXYZ2();
	if (!pcl_cloud)
		return -1;

	//create storage for normals
	pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);

	//now compute
	int result = compute_normals<pcl::PointXYZ, pcl::PointNormal>(pcl_cloud, m_useKnn ? m_knn_radius: m_radius, m_useKnn, normals);
	if (result < 0)
		return -1;

	PCLCloud::Ptr sm_normals (new PCLCloud);
	TO_PCL_CLOUD(*normals, *sm_normals);

	sm2ccConverter converter2(sm_normals);
	converter2.addNormals(cloud);
	converter2.addScalarField(cloud, "curvature", m_overwrite_curvature);

	emit entityHasChanged(cloud);

	return 1;
}

//INSTANTIATING TEMPLATED FUNCTIONS
template int compute_normals<pcl::PointXYZ, pcl::PointNormal> (	const  pcl::PointCloud<pcl::PointXYZ>::Ptr incloud,
																const float radius,
																const bool useKnn, //true if use knn, false if radius search
																pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
