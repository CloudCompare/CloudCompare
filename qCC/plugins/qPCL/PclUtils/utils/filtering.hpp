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




#ifndef qPCL_FILTERING_IMPL_H_
#define qPCL_FITLERING_IMPL_H_

#include "filtering.h"
#include "my_point_types.h"

//PCL
#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

#include <pcl/features/intensity_spin.h>
#include <pcl/features/impl/intensity_spin.hpp>


#include <pcl/filters/statistical_outlier_removal.h>



template <typename PointInT, typename PointOutT>
int estimateSIFT(const typename pcl::PointCloud<PointInT>::Ptr in_cloud,
				 typename pcl::PointCloud<PointOutT>::Ptr out_cloud,
				 int nr_octaves = 0,
				 float min_scale = 0,
				 int nr_scales_per_octave = 0,
				 float min_contrast = 0)
{
	pcl::SIFTKeypoint< PointInT, PointOutT > keypoint_detector ;
	keypoint_detector.setInputCloud(in_cloud);

	if ( (nr_octaves!=0) && (min_scale!=0) && (nr_scales_per_octave!=0) )
	{
		keypoint_detector.setScales (min_scale, nr_octaves, nr_scales_per_octave);
	}

	if (min_contrast!=0)
	{
		keypoint_detector.setMinimumContrast(min_contrast);
	}
	//typename pcl::PointCloud<PointOutT>::Ptr out_ptr (new pcl::PointCloud<PointOutT>);


	keypoint_detector.compute(*out_cloud);
	return 1;
}


template <typename PointInT, typename PointOutT>
int
computeIntensitySPINImages(const typename pcl::PointCloud<PointInT>::Ptr incloud,
                           const float radius,
                           const int k_nn,
                           const bool mode, //true if use knn, false if radius search
                           const int n_distance_bins,
                           const int n_intensity_bins,
                           typename pcl::PointCloud<PointOutT>::Ptr outcloud)
{

    if ((n_distance_bins <= 0) || (n_intensity_bins <= 0))
            return -1;

    pcl::IntensitySpinEstimation<PointInT, PointOutT> estimator;
    estimator.setInputCloud(incloud);
    estimator.setNrDistanceBins(n_distance_bins);
    estimator.setNrIntensityBins(n_intensity_bins);

    if (mode == true)
    {
        //knn
        estimator.setKSearch(k_nn);
    }
    else if (mode == false)
    {
        estimator.setRadiusSearch((double) radius);
    }
    else
    {
        return -1;
    }

    estimator.compute(*outcloud);

    return 1;




}


//template <int n_dist, int n_int>
//int
//computeIntensitySPINImages2(const sensor_msgs::PointCloud2Ptr incloud,
//                           const float radius,
//                           const int k_nn,
//                           const bool mode, //true if use knn, false if radius search
//                            sensor_msgs::PointCloud2Ptr outcloud)
//{

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::fromROSMsg(*incloud, *cloud);

//    const int full_size = n_dist * n_int;
//    typename pcl::PointCloud<pcl::Histogram<full_size> >::Ptr features_cloud (new pcl::PointCloud<pcl::Histogram< full_size > >);

//    pcl::IntensitySpinEstimation<pcl::PointXYZI, pcl::Histogram<full_size> > estimator;
//    estimator.setInputCloud(cloud);
//    estimator.setNrDistanceBins(n_dist);
//    estimator.setNrIntensityBins(n_int);

//    if (mode == true)
//    {
//        //knn
//        estimator.setKSearch(k_nn);
//    }
//    else if (mode == false)
//    {
//        estimator.setRadiusSearch((double) radius);
//    }
//    else
//    {
//        return -1;
//    }

//    estimator.compute(*features_cloud);

//    pcl::toROSMsg(*features_cloud, *outcloud);

//    return 1;

//}



template <typename PointInT, typename PointOutT>
int compute_normals(const typename pcl::PointCloud<PointInT>::Ptr incloud,
					const float radius,
					const bool mode, //true if use knn, false if radius search
					typename pcl::PointCloud<PointOutT>::Ptr outcloud)
{
	typename pcl::NormalEstimationOMP<PointInT, PointOutT> normal_estimator;
	//typename pcl::PointCloud<PointOutT>::Ptr normals (new pcl::PointCloud<PointOutT>);

	if (mode) //use knn
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


template <typename PointInT, typename PointOutT>
int smooth_mls(const typename pcl::PointCloud<PointInT>::Ptr &incloud,
			   const MLSParameters &params,
			   typename pcl::PointCloud<PointOutT>::Ptr &outcloud,
			   pcl::PointIndicesPtr &mapping_ids)
{

	typename pcl::search::KdTree<PointInT>::Ptr tree (new pcl::search::KdTree<PointInT>);


	int n_points = incloud->size();

	//create the smoothing object
	pcl::MovingLeastSquares< PointInT, PointOutT > smoother;

	smoother.setInputCloud(incloud);
	smoother.setSearchMethod(tree);	
	smoother.setSearchRadius(params.search_radius_);
	smoother.setComputeNormals(params.compute_normals_);
	smoother.setPolynomialFit(params.polynomial_fit_);



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
			smoother.setDilationVoxelSize(params.dilation_voxel_size_);
			smoother.setDilationIterations(params.dilation_iterations_);
			break;
		}
	}


	smoother.process(*outcloud);

    mapping_ids = smoother.getCorrespondingIndices();

	return 1;
}


#endif


