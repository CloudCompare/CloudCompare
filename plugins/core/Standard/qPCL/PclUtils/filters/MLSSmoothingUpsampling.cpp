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
int smooth_mls(const typename pcl::PointCloud<PointInT>::Ptr &incloud,
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
	pcl::MovingLeastSquaresOMP< PointInT, PointOutT > smoother;
	int n_threads = omp_get_max_threads();
	smoother.setNumberOfThreads(n_threads);
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

	return 1;
}

MLSSmoothingUpsampling::MLSSmoothingUpsampling()
	: BaseFilter(FilterDescription(	"MLS smoothing",
									"Smooth using MLS, optionally upsample",
									"Smooth the cloud using Moving Least Sqares algorithm, estimate normals and optionally upsample",
									":/toolbar/PclUtils/icons/mls_smoothing.png"))
	, m_dialog(nullptr)
	, m_dialogHasParent(false)
	, m_parameters(new MLSParameters)
{
}

MLSSmoothingUpsampling::~MLSSmoothingUpsampling()
{
	//we must delete parent-less dialogs ourselves!
	if (!m_dialogHasParent && m_dialog && m_dialog->parent() == nullptr)
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

	std::list<std::string> req_fields;
	try
	{
		req_fields.push_back("xyz"); // always needed
		if (cloud->getCurrentDisplayedScalarField())
		{
			//keep the current scalar field (if any)
			req_fields.push_back(cloud->getCurrentDisplayedScalarField()->getName());
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -1;
	}

	//take out the xyz info
	PCLCloud::Ptr sm_cloud = cc2smReader(cloud).getAsSM(req_fields);
	if (!sm_cloud)
		return -1;

	//Now change the name of the field to use to a standard name, only if in OTHER_FIELD mode
	if (cloud->getCurrentDisplayedScalarField())
	{
		int field_index = pcl::getFieldIndex(*sm_cloud, cc2smReader::GetSimplifiedSFName(cloud->getCurrentDisplayedScalarField()->getName()));
		if (field_index >= 0)
			sm_cloud->fields.at(field_index).name = "scalar";
	}

	//get as pcl point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
	FROM_PCL_CLOUD(*sm_cloud, *pcl_cloud);

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

	//copy global shift & scale
	new_cloud->setGlobalScale(cloud->getGlobalScale());
	new_cloud->setGlobalShift(cloud->getGlobalShift());

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
	{
		m_dialog = new MLSDialog(m_app ? m_app->getMainWindow() : nullptr);
		m_dialogHasParent = (m_app->getMainWindow() != nullptr);
	}

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
