//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qTreeIso                          #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper if you find this tool helpful                #
//#                                                                                     #
//#        Xi, Z.; Hopkinson, C. 3D Graph-Based Individual-Tree Isolation (Treeiso)     #
//#        from Terrestrial Laser Scanning Point Clouds. Remote Sens. 2022, 14, 6116.   #
//#        https://doi.org/10.3390/rs14236116                                           #
//#                                                                                     #
//#		   Our work relies on the cut-pursuit algorithm, please also consider citing:   #
//#        Landrieu, L.; Obozinski, G. Cut Pursuit: Fast Algorithms to Learn Piecewise  #
//#        Constant Functions on General Weighted Graphs. SIAM J. Imaging Sci.          #
//#        2017, 10, 1724–1766.                                                         #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#                  Artemis Lab, Department of Geography & Environment                 #
//#                            University of Lethbridge, Canada                         #
//#                                                                                     #
//#                                                                                     #
//#                           Zhouxin Xi and Chris Hopkinson;                           #
//#                    truebelief2010@gmail.com; c.hopkinson@uleth.ca                   #
//#                                                                                     #
//#######################################################################################

#pragma once
#include <ccPointCloud.h>

#include <API.h>
#include "Eigen/Dense"
#include "knncpp.h"


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_xy;
typedef boost::geometry::model::polygon<point_xy> polygon;
typedef boost::geometry::model::multi_point<point_xy> multi_point;
typedef boost::geometry::model::multi_polygon <polygon> multi_polygon;


void toVec2D(ccPointCloud* pc, std::vector<std::vector<float>>&);
void toVec2DTranslate(ccPointCloud* pc, std::vector<std::vector<float>>& y);
bool perform_cut_pursuit(unsigned K, const float regStrength, std::vector<std::vector<float>>& pc_vec, std::vector<float>& edgeWeight, std::vector<uint32_t>& Eu, std::vector<uint32_t>& Ev, std::vector<uint32_t>& in_component, std::vector<std::vector<uint32_t>>& components);
void perform_cut_pursuit2d(const unsigned K, const float regStrength, std::vector<std::vector<float>>& pc_vec, std::vector<float>& edgeWeight, std::vector<uint32_t>& Eu, std::vector<uint32_t>& Ev, std::vector<uint32_t>&);
ccPointCloud* partial_pcd(ccPointCloud* pc, std::vector<size_t> ia);

template <typename T>size_t arg_min_col(std::vector<T>& arr);
template <typename T>size_t arg_max_col(std::vector<T>& arr);
template <typename T> void min_col(std::vector<std::vector<T>>& arr, std::vector<T>&);
template <typename T> T min_col(std::vector<T>& arr);
template <typename T> void max_col(std::vector<std::vector<T>>& arr, std::vector<T>&);
template <typename T>void mean_col(std::vector<std::vector<T>>& arr, std::vector<T>&);
template <typename T> T mean_col(std::vector<T>& arr);
template <typename T> T median_col(std::vector<T>& arr);
template <typename T> void decimate_vec(std::vector<std::vector<T>>& arr, float res, std::vector<std::vector<T>>&);

template <typename T> void unique_group(std::vector<T>& arr, std::vector<std::vector<T>>& u_group, std::vector<T>& arr_unq, std::vector<T>& ui);
template <typename T> void unique_group(std::vector<T>& idx, std::vector<std::vector<T>>&);
template <typename T> void unique_group(std::vector<T>& arr, std::vector<std::vector<T>>& u_group, std::vector<T>& arr_unq);
template <typename T>void unique_index_by_rows(std::vector<std::vector<T>>& arr, std::vector<size_t>& ia, std::vector<size_t>& ic);
template <typename T> void sort_indexes_by_row(std::vector<std::vector<T>>& v, std::vector<size_t>& idx, std::vector<std::vector<T>>&);
template <typename T> void sort_indexes(std::vector<T>& v, std::vector<size_t>& idx, std::vector<T>&);

template <typename T1, typename T2> void get_subset(std::vector<std::vector<T1>>& arr, std::vector<T2>& indices, std::vector<std::vector<T1>>&);
template <typename T1, typename T2> void get_subset(std::vector<std::vector<T1>>& arr, std::vector<T2>& indices, Eigen::MatrixXf&);
template <typename T1, typename T2> bool get_subset(ccPointCloud* pcd, std::vector<T2>& indices, std::vector<std::vector<T1>>& arr_sub);
template <typename T1, typename T2> void get_subset(std::vector<T1>& arr, std::vector<T2>& indices, std::vector<T1>& arr_sub);

void knn_cpp_nearest_neighbors(const std::vector<std::vector<float>>& dataset, size_t k, std::vector <std::vector<size_t>>& res_idx, std::vector <std::vector<float>>& res_dists, int n_thread);
void knn_cpp_build(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, int n_thread);
void knn_cpp_query(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, Eigen::MatrixXf& query_points, size_t k, std::vector <std::vector<size_t>>& res_idx, std::vector <std::vector<float>>& res_dists);
float knn_cpp_query_min_d(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, Eigen::MatrixXf& query_points, size_t k);

template <typename T> void testWriteLog(std::vector< std::vector<T>>arr, std::string);
template <typename T> void testWriteLog(std::vector<T>arr, std::string fname);
