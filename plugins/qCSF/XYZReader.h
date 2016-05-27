#ifndef XYZ_READER_H_
#define XYZ_READER_H_

#include <string>

#include "csf_h/point_cloud.h"

//从fname文件中读取点云，将点云数据存储在pointcloud中
bool read_xyz(std::string fname, wl::PointCloud &pointcloud);


#endif