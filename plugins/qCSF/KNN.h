#ifndef _KNN_H_
#define _KNN_H_

#include "point_cloud.h"
#include "Cloth.h"

class Rasterlization
{
public:
	int N;
public:
	Rasterlization(int N=1){
		this->N = N;
	}
	~Rasterlization(){}

	//对点云进行最临近搜索，寻找周围最近的N个点  避免求交运算
	void RasterTerrian(Cloth cloth, PointCloud &pc, vector<double> &heightVal);

};



#endif