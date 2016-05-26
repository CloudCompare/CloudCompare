//cloth simulation filter for airborne lidar filtering
#ifndef _CSF_H_
#define _CSF_H_

#include "point_cloud.h"
#include <iostream>
#include "Cloth.h"
using namespace wl;


class CSF
{
public:
	CSF(wl::PointCloud& cloud);
	virtual ~CSF();

	//input PC from files
	void readPointsFromFile(string filename);
	//save the ground points to file
	void saveGroundPoints(const vector<int>& grp, string path = "");
	void saveOffGroundPoints(const vector<int>& grp, string path = "");
	//get size of point cloud
	size_t size() const { return point_cloud.size(); }

	inline const LASPoint& index(int i) { return point_cloud[i]; }

	//执行滤波处理 得到地面点的在PointCloud 中的序号
	bool do_filtering(unsigned pcsize, vector< vector<int> >& output);

private:
	wl::PointCloud& point_cloud;

public:

	struct{
		//perameters
		//最临近搜索是的点数，一般设置为1
		int k_nearest_points;

		//是否进行边坡后处理
		bool bSloopSmooth;

		//时间步长
		double time_step;

		//分类阈值
		double class_threshold;

		//布料格网大小
		double cloth_resolution;

		//布料硬度参数
		int rigidness;

		//最大迭代次数
		int interations;
	} params;
};

#endif