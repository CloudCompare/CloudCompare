#ifndef _C2CDIST_H_
#define _C2CDIST_H_

#include "Cloth.h"

//计算点云与点云之间的距离
class c2cdist
{
public:
	
	c2cdist(double threshold) : class_treshold(threshold) {}
	virtual ~c2cdist() {}

public:
	bool c2cdist::calCloud2CloudDist(Cloth cloth, PointCloud &pc, unsigned PCSIZE, vector< vector<int> >& output);

private:
	
	double class_treshold;
};




#endif