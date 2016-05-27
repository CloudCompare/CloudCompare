#ifndef _TERRIAN_H_
#define _TERRIAN_H_

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "csf_h/point_cloud.h"

class Terrain
{
public:

	Terrain(wl::PointCloud &pc);
	virtual ~Terrain();

	wl::PointCloud& pc; //点云
	double cube[8];//  包围所有点云的立方体坐标
	double off_avg_x, off_avg_z;//自动shift算法

	//将点云保存到文件
	void saveToFile(const wl::Point& offset, std::string path = "");

protected:
	
	//计算点云的外包围立方体
	void bound_box();

	//计算点云的最大值，最小值
	double getMin(int direction);
	double getMax(int direction);
};


#endif