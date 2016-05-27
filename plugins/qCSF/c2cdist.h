#ifndef _C2CDIST_H_
#define _C2CDIST_H_

#include "Cloth.h"
#include "PointCloud.h"

//计算点云与点云之间的距离
class c2cdist
{
public:
	
	c2cdist(double threshold) : class_treshold(threshold) {}

	virtual ~c2cdist() {}

	bool c2cdist::calCloud2CloudDist(const Cloth& cloth, const wl::PointCloud& pc, std::vector< std::vector<int> >& output);

protected:
	
	double class_treshold;
};

#endif