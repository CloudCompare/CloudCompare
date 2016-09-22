#ifndef _C2CDIST_H_
#define _C2CDIST_H_

#include "Cloth.h"
#include "PointCloud.h"

//computing distance between clouds
class Cloud2CloudDist
{
public:
	
	static bool Compute(const Cloth& cloth,
						const wl::PointCloud& pc,
						double class_threshold,
						std::vector<int>& groundIndexes,
						std::vector<int>& offGroundIndexes,
						unsigned N = 3);
};

#endif