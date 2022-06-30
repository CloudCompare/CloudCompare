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
int ComputeNormals(	const typename pcl::PointCloud<PointInT>::Ptr incloud,
					float radius,
					bool useKnn, //true if use knn, false if radius search
					typename pcl::PointCloud<PointOutT>& outcloud)
{
	typename pcl::NormalEstimationOMP<PointInT, PointOutT> normalEstimator;
	
	if (useKnn) //use knn
	{
		normalEstimator.setKSearch(static_cast<int>(radius)); //cast to int
	}
	else //use radius search
	{
		normalEstimator.setRadiusSearch(radius);
	}

	normalEstimator.setInputCloud(incloud);
	normalEstimator.compute(outcloud);

	return BaseFilter::Success;
}

NormalEstimation::NormalEstimation()
	: BaseFilter(FilterDescription(	"Estimate Normals",
									"Estimate Normals and Curvature",
									"Estimate Normals and Curvature for the selected entity",
									":/toolbar/PclUtils/icons/normal_curvature.png"))
	, m_radius(0)
	, m_knn_radius(10)
	, m_useKnn(false)
	, m_overwrite_curvature(true)
{
}

NormalEstimation::~NormalEstimation()
{
}

int NormalEstimation::getParametersFromDialog()
{
	NormalEstimationDialog dialog(m_app ? m_app->getMainWindow() : nullptr);

	//initially these are invisible
	dialog.surfaceComboBox->setVisible(false);
	dialog.searchSurfaceCheckBox->setVisible(false);

	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (cloud)
	{
		ccBBox bBox = cloud->getOwnBB();
		if (bBox.isValid())
		{
			dialog.radiusDoubleSpinBox->setValue(bBox.getDiagNorm() / 200.0);
		}
	}

	if (!dialog.exec())
	{
		return CancelledByUser;
	}

	//fill in parameters from dialog
	m_useKnn = dialog.useKnnCheckBox->isChecked();
	m_overwrite_curvature = dialog.curvatureCheckBox->isChecked();
	m_knn_radius = dialog.knnSpinBox->value();
	m_radius = static_cast<float>(dialog.radiusDoubleSpinBox->value());

	return Success;
}

int NormalEstimation::compute()
{
	//pointer to selected cloud
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
		return InvalidInput;

	//get xyz as a PCL cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud = cc2smReader(cloud).getRawXYZ();
	if (!xyzCloud)
	{
		return ComputationError;
	}

	//now compute the normals
	pcl::PointCloud<pcl::PointNormal> rawCloudWithNormals;
	int result = ComputeNormals<pcl::PointXYZ, pcl::PointNormal>(xyzCloud, m_useKnn ? m_knn_radius: m_radius, m_useKnn, rawCloudWithNormals);
	if (result < 0)
	{
		return ComputationError;
	}

	//if we have normals delete them!
	if (!cloud->hasNormals())
	{
		if (!cloud->resizeTheNormsTable())
		{
			return NotEnoughMemory;
		}
	}

	//copy the notmals
	unsigned pointCount = cloud->size();
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const pcl::PointNormal& point = rawCloudWithNormals[i];
		CCVector3 N(static_cast<PointCoordinateType>(point.normal_x),
					static_cast<PointCoordinateType>(point.normal_y),
					static_cast<PointCoordinateType>(point.normal_z));

		cloud->setPointNormal(i, N);
	}
	cloud->showNormals(true);

	// FIXME: find a way to avoid converting to a PCLPointCloud2 just to get the curvature values!
	PCLCloud::Ptr cloudWithNormals(new PCLCloud);
	TO_PCL_CLOUD(rawCloudWithNormals, *cloudWithNormals);
	pcl2cc::CopyScalarField(*cloudWithNormals, "curvature", *cloud, m_overwrite_curvature);

	Q_EMIT entityHasChanged(cloud);

	return Success;
}

