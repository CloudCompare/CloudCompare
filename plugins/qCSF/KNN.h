#ifndef _KNN_H_
#define _KNN_H_

#include "Cloth.h"
#include "csf_h/point_cloud.h"

class Rasterlization
{
public:
	int N;

public:
	Rasterlization(int N = 1)
	{
		this->N = N;
	}
	virtual ~Rasterlization() {}

	//对点云进行最临近搜索，寻找周围最近的N个点  避免求交运算
	bool RasterTerrian(Cloth cloth, const wl::PointCloud& pc, std::vector<double>& heightVal);
};

#endif