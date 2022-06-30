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

#include "MLSSmoothingUpsampling.h"

//Local
#include "dialogs/MLSDialog.h"
#include "../utils/PCLConv.h"
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//PCL
#include <pcl/surface/mls.h>
#include <pcl/common/io.h> // for getFieldIndex
#include <pcl/search/kdtree.h> // for KdTree

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccScalarField.h>
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>

#ifdef LP_PCL_PATCH_ENABLED
#include "../utils/copy.h"
#endif

template <typename PointInT, typename PointOutT>
static int SmoothMLS(	const typename pcl::PointCloud<PointInT>::Ptr &incloud,
						const MLSParameters &params,
						typename pcl::PointCloud<PointOutT>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
				, pcl::PointIndicesPtr &mapping_ids
#endif
	)
{
	typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT>);

#ifdef _OPENMP
	//create the smoothing object
	pcl::MovingLeastSquares< PointInT, PointOutT > smoother;
	smoother.setNumberOfThreads(omp_get_max_threads());
#else
	pcl::MovingLeastSquares< PointInT, PointOutT > smoother;
#endif
	smoother.setInputCloud(incloud);
	smoother.setSearchMethod(tree);	
	smoother.setSearchRadius(params.search_radius_);
	smoother.setComputeNormals(params.compute_normals_);
	if (params.polynomial_fit_)
	{
		smoother.setPolynomialOrder(params.order_);
		smoother.setSqrGaussParam(params.sqr_gauss_param_);
	}

	switch (params.upsample_method_)
	{
	case (MLSParameters::NONE):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::NONE );
			//no need to set other parameters here!
			break;
		}

	case (MLSParameters::SAMPLE_LOCAL_PLANE):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::SAMPLE_LOCAL_PLANE);
			smoother.setUpsamplingRadius(params.upsampling_radius_);
			smoother.setUpsamplingStepSize(params.upsampling_step_);
			break;
		}

	case (MLSParameters::RANDOM_UNIFORM_DENSITY):
		{
			smoother.setUpsamplingMethod( pcl::MovingLeastSquares<PointInT, PointOutT>::RANDOM_UNIFORM_DENSITY );
			smoother.setPointDensity(params.step_point_density_);
			break;
		}

	case (MLSParameters::VOXEL_GRID_DILATION):
		{
			smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::VOXEL_GRID_DILATION);
			smoother.setDilationVoxelSize(static_cast<float>(params.dilation_voxel_size_));
			smoother.setDilationIterations(params.dilation_iterations_);
			break;
		}
	}

	smoother.process(*outcloud);

#ifdef LP_PCL_PATCH_ENABLED
	mapping_ids = smoother.getCorrespondingIndices();
#endif

	return BaseFilter::Success;
}

MLSSmoothingUpsampling::MLSSmoothingUpsampling()
	: BaseFilter(FilterDescription(	"MLS smoothing",
									"Smooth using MLS, optionally upsample",
									"Smooth the cloud using Moving Least Sqares algorithm, estimate normals and optionally upsample",
									":/toolbar/PclUtils/icons/mls_smoothing.png"))
{
}

MLSSmoothingUpsampling::~MLSSmoothingUpsampling()
{
}

int MLSSmoothingUpsampling::compute()
{
	//pointer to selected cloud
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		return InvalidInput;
	}

	ccScalarField* sf = cloud->getCurrentDisplayedScalarField();

	//get xyz in PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud = cc2smReader(cloud).getRawXYZ();
	if (!xyzCloud)
	{
		return NotEnoughMemory;
	}

	//create storage for outcloud
	pcl::PointCloud<pcl::PointNormal>::Ptr rawCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
#ifdef LP_PCL_PATCH_ENABLED
	pcl::PointIndicesPtr mappingIndices;
	SmoothMLS<pcl::PointXYZ, pcl::PointNormal> (xyzCloud, m_parameters, rawCloudWithNormals, mappingIndices);
#else
	SmoothMLS<pcl::PointXYZ, pcl::PointNormal> (xyzCloud, m_parameters, rawCloudWithNormals);
#endif

	PCLCloud cloudWithNormals;
	TO_PCL_CLOUD(*rawCloudWithNormals, cloudWithNormals);

	ccPointCloud* outputCCCloud = pcl2cc::Convert(cloudWithNormals);
	if (!outputCCCloud)
	{
		//conversion failed (not enough memory?)
		return NotEnoughMemory;
	}

	outputCCCloud->setName(cloud->getName() + QString("_smoothed")); //original name + suffix
	outputCCCloud->setDisplay(cloud->getDisplay());

#ifdef LP_PCL_PATCH_ENABLED
	//copy the original scalar fields here
	copyScalarFields(cloud, outputCCCloud, mappingIndices, true);
	//copy the original colors here
	copyRGBColors(cloud, outputCCCloud, mappingIndices, true);
#endif

	//copy global shift & scale
	outputCCCloud->copyGlobalShiftAndScale(*cloud);

	//disable original cloud
	cloud->setEnabled(false);
	if (cloud->getParent())
	{
		cloud->getParent()->addChild(outputCCCloud);
	}

	Q_EMIT newEntity(outputCCCloud);

	return Success;
}

int MLSSmoothingUpsampling::getParametersFromDialog()
{
	MLSDialog dialog(m_app ? m_app->getMainWindow() : nullptr);

	if (!dialog.exec())
	{
		return CancelledByUser;
	}

	//we need to read all the parameters and put them into m_parameters
	m_parameters.search_radius_ = dialog.search_radius->value();
	m_parameters.compute_normals_ = dialog.compute_normals->checkState();
	m_parameters.polynomial_fit_ = dialog.use_polynomial->checkState();
	m_parameters.order_ = dialog.polynomial_order->value();
	m_parameters.sqr_gauss_param_ = dialog.squared_gaussian_parameter->value();

	int index_now = dialog.upsampling_method->currentIndex();
	QVariant current_status = dialog.upsampling_method->itemData(index_now);
	int status = current_status.toInt();

	m_parameters.upsample_method_ = static_cast<MLSParameters::UpsamplingMethod>(status);

	m_parameters.upsampling_radius_ = dialog.upsampling_radius->value();
	m_parameters.upsampling_step_ = dialog.upsampling_step_size->value();
	m_parameters.step_point_density_ = dialog.step_point_density->value();
	m_parameters.dilation_voxel_size_ = dialog.dilation_voxel_size->value();

	return Success;
}

template int SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(	const pcl::PointCloud<pcl::PointXYZ>::Ptr &incloud,
															const MLSParameters &params,
															pcl::PointCloud<pcl::PointNormal>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
															,	pcl::PointIndicesPtr &used_ids
#endif
  );
