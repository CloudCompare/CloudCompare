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
#ifndef qPCL_FILTERING_H
#define qPCL_FILTERING_H



//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>


//qCC
#include <ccPointCloud.h>

#define PCL_VERBOSITY_LEVEL = VERBOSE;

struct MLSParameters
{
    ///NOTE: DISTINCT CLOUD METHOD NOT IMPLEMENTED
    enum UpsamplingMethod { NONE, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION };


    MLSParameters():
        order_ (0),
        polynomial_fit_(false),
        search_radius_(0),
        sqr_gauss_param_(0),
        compute_normals_(false),
        upsample_method_(NONE),
        upsampling_radius_(0),
        upsampling_step_(0),
        step_point_density_(0),
        dilation_voxel_size_(0),
        dilation_iterations_(0)
    {
    }


    int order_;
    bool polynomial_fit_;
    double search_radius_;
    double sqr_gauss_param_;
    bool compute_normals_;
    UpsamplingMethod upsample_method_;
    double upsampling_radius_;
    double upsampling_step_;
    int step_point_density_;
    double dilation_voxel_size_;
    int dilation_iterations_;
};


/** \brief Extract SIFT keypoints
  * if only the point cloud is given PCL default parameters are used (that are not really good, so please give parameters)
  *  \note that we can pass different types as input for this function:
  * - PointXYZI
  * - PointNormal
  * - PointXYZRGB
  * \note if a PointType with a scale field is passed as output type, scales will be returned together with the return cloud
*/
template <typename PointInT, typename PointOutT> int
estimateSIFT( const typename pcl::PointCloud<PointInT>::Ptr in_cloud,
              typename pcl::PointCloud<PointOutT>::Ptr out_cloud,
              int nr_scales,
              float min_scale,
              int nr_scales_per_octave,
              float min_contrast);


template <typename PointInT, typename PointOutT> int
compute_normals(                                                const typename pcl::PointCloud<PointInT>::Ptr incloud,
                                                                const float radius,
                                                                const bool mode, //true if use knn, false if radius search
                                                                typename pcl::PointCloud<PointOutT>::Ptr outcloud
);


template <typename PointInT, typename PointOutT>
int
computeIntensitySPINImages(const typename pcl::PointCloud<PointInT>::Ptr incloud,
                           const float radius,
                           const int k_nn,
                           const bool mode, //true if use knn, false if radius search
                           const int n_distance_bins,
                           const int n_intensity_bins,
                           typename pcl::PointCloud<PointOutT>::Ptr);

//template <int n_dist, int n_int>
//int
//computeIntensitySPINImages2(const sensor_msgs::PointCloud2Ptr incloud,
//                           const float radius,
//                           const int k_nn,
//                           const bool mode, //true if use knn, false if radius search
//                           sensor_msgs::PointCloud2Ptr outcloud);


template <typename PointInT, typename PointOutT> int
smooth_mls(
        const typename pcl::PointCloud<PointInT>::Ptr &incloud,
        const MLSParameters &params,
        typename pcl::PointCloud<PointOutT>::Ptr &outcloud,
        pcl::PointIndicesPtr &mapping_ids
        );

int
removeOutliersStatistical(const sensor_msgs::PointCloud2ConstPtr incloud, const int &k, const float &nStds, sensor_msgs::PointCloud2Ptr outcloud);


//! Make a forced copy of all scalar fields from one cloud to another
/** \note This algorithm simply copy the scalar fields from a cloud
 *  to another using the the mammping contained in a pcl::PointIndicesPtr.
 *  \param inCloud the input cloud from which to copy scalars
 *  \param outCloud the output cloud in which to copy the scalar fields
 *  \param in2outMapping indices of the input cloud for each point in the output
 *  \param overwrite you can chose to not overwrite existing fields
 **/
void copyScalarFields(const ccPointCloud *inCloud,
                             ccPointCloud *outCloud,
                            pcl::PointIndicesPtr &in2outMapping,
                      bool overwrite);

#endif // FILTERING_H
