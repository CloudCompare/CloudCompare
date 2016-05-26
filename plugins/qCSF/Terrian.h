#ifndef _TERRIAN_H_
#define _TERRIAN_H_

#include <string>
#include <vector>
#include <iostream>
#include<iomanip>
#include <fstream>
using namespace std;

#include "point_cloud.h"

class Terrian
{
public:
	PointCloud pc; //点云
	vector<double> cube;//  包围所有点云的立方体坐标
	double off_avg_x, off_avg_z;//自动shift算法

	Terrian(PointCloud &pc);
	~Terrian();

	//计算点云的最大值，最小值
	double getMin(int direction);
	double getMax(int direction);

	//计算点云的外包围立方体
	void bound_box();

	//将点云保存到文件
	void saveToFile(LASPoint offset, string path = "");
};


#endif