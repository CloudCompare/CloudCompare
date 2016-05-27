#ifndef _TERRIAN_H_
#define _TERRIAN_H_

#include "PointCloud.h"
#include "Vec3.h"

//system
#include <string>

class Terrain
{
public:

	Terrain(wl::PointCloud &pc);
	virtual ~Terrain();

	wl::PointCloud& pc; //associated cloud
	Vec3 bbMin, bbMax;//bounding-box
	double off_avg_x, off_avg_z;//自动shift算法

	//将点云保存到文件
	void saveToFile(const wl::Point& offset, std::string path = "");

protected:
	
	//计算点云的外包围立方体
	void computeBoundingBox();

};


#endif