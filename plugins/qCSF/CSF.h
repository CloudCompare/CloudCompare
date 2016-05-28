//cloth simulation filter for airborne lidar filtering
#ifndef _CSF_H_
#define _CSF_H_

#include "PointCloud.h"
#include "Cloth.h"

//system
#include <vector>
#include <string>

class ccMainAppInterface;
class QWidget;
class ccMesh;

class CSF
{
public:
	CSF(wl::PointCloud& cloud);
	virtual ~CSF();

	//input PC from files
	bool readPointsFromFile(std::string filename);

	//save the ground points to file
	void saveGroundPoints(const std::vector<int>& grp, std::string path = "");
	void saveOffGroundPoints(const std::vector<int>& grp, std::string path = "");
	
	//执行滤波处理 得到地面点的在PointCloud 中的序号
	bool do_filtering(	std::vector<int>& groundIndexes,
						std::vector<int>& offGroundIndexes,
						bool exportClothMesh,
						ccMesh* &clothMesh,
						ccMainAppInterface* app = 0,
						QWidget* parent = 0);

private:
	wl::PointCloud& point_cloud;

public:

	struct Parameters
	{
		//parameters
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
		int iterations;
	};
	
	Parameters params;
};

#endif