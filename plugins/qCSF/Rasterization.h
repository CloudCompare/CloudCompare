#ifndef _RASTERIZATION_H_
#define _RASTERIZATION_H_

#include "Cloth.h"
#include "PointCloud.h"

class Rasterization
{
public:

	//对点云进行最临近搜索，寻找周围最近的N个点  避免求交运算
	static bool RasterTerrain(const Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN);
};

#endif //_RASTERIZATION_H_
