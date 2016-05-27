#ifndef _RASTERIZATION_H_
#define _RASTERIZATION_H_

#include "Cloth.h"
#include "csf_h/point_cloud.h"

class Rasterization
{
public:
	int N;

public:
	Rasterization(int N = 1)
	{
		this->N = N;
	}
	virtual ~Rasterization() {}

	//对点云进行最临近搜索，寻找周围最近的N个点  避免求交运算
	bool RasterTerrian(const Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal);
};

#endif //_RASTERIZATION_H_
