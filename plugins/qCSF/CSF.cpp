//CSF
#include "CSF.h"
#include "XYZReader.h"
#include "Terrain.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "c2cdist.h"

//system
#include <cmath>
#include <iomanip>
#include <fstream>

//Qt
#include <QProgressDialog>
#include <QCoreApplication>

CSF::CSF(wl::PointCloud& cloud)
	: point_cloud(cloud)
{
	params.k_nearest_points = 1;
	params.bSloopSmooth = true;
	params.time_step = 0.65;
	params.class_threshold=0.5;
	params.cloth_resolution = 1.5;
	params.rigidness = 3;
	params.iterations = 500;
}

CSF::~CSF()
{
}

//从文件中读取点云，该程序中未用到此函数
void CSF::readPointsFromFile(std::string filename)
{
	read_xyz(filename, this->point_cloud);
}

//CSF主程序 dofiltering
bool CSF::do_filtering(std::vector< std::vector<int> >& output)
{
	//constants
	static const double cloth_y_height = 0.05; //origin cloth height
	static const double clothbuffer_d = 4.0;//set the cloth buffer
	static const double gravity = 0.2;

	try
	{
		//build the terrain
		Terrain terr(point_cloud);

		//computing the number of cloth node
		Vec3 origin_pos1(	terr.bbMin.x - clothbuffer_d,
							terr.bbMax.y + cloth_y_height,
							terr.bbMin.z - clothbuffer_d);
	
		int width_num  = (terr.bbMax.x - terr.bbMin.x + clothbuffer_d * 2) / params.cloth_resolution;
		int height_num = (terr.bbMax.z - terr.bbMin.z + clothbuffer_d * 2) / params.cloth_resolution;
	
		//one Cloth object of the Cloth class
		Cloth cloth1(	terr.bbMax.x - terr.bbMin.x + clothbuffer_d * 2,
						terr.bbMax.z - terr.bbMin.z + clothbuffer_d * 2,
						width_num,
						height_num,
						origin_pos1,
						0.3,
						9999,
						params.rigidness,
						params.time_step);

		std::vector<double> heightvals;
		if (!Rasterization::RasterTerrian(cloth1, point_cloud, heightvals, params.k_nearest_points))
		{
			return false;
		}
	
		cloth1.setheightvals(heightvals);

		double time_step2 = params.time_step * params.time_step;
		bool flag = false;

		//do the filtering
		//显示进度条
		QProgressDialog pDlg2;
		pDlg2.setWindowTitle("CSF");
		pDlg2.setLabelText("Do filtering....");
		pDlg2.setRange(0, params.iterations);
		pDlg2.show();
		QCoreApplication::processEvents();

	#ifdef _DEBUG
		//滤波主循环
		ofstream fout("clothDistDiffMax.txt");
		if(!fout)
		{
			cerr<<"File of clothdist.txt not open!"<<endl;
			exit(1);
		}
	#endif

		for (int i = 0; i < params.iterations; i++)
		{
			//滤波主过程
			cloth1.addForce(Vec3(0, -gravity, 0) * time_step2);
			double maxheight = cloth1.timeStep();
			cloth1.terrCollision(heightvals, &terr, flag);

	#ifdef _DEBUG
			fout.setf(ios::app);
			fout.precision(3);  
			fout<<i<<" "<<maxheight<<endl;
	#endif
			//判断循环终止条件
			if (maxheight != 0 && maxheight < params.class_threshold / 100)
			{
				pDlg2.setValue(params.iterations);
				break;
			}
			pDlg2.setValue(pDlg2.value()+1);
			QCoreApplication::processEvents();
		}
		pDlg2.hide();
		QCoreApplication::processEvents();

		//slope processing
		if (params.bSloopSmooth)
		{
			cloth1.movableFilter();
		}
	
		//classification of the points
		return c2cdist(params.class_threshold).calCloud2CloudDist(cloth1, point_cloud, output);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
}

//输出地面点
void CSF::saveGroundPoints(const std::vector<int>& grp, std::string path)
{
	std::string filepath = "terr_ground.txt";
	if (path != "")
	{
		filepath = path;
	}
	std::ofstream f1(filepath, std::ios::out);
	if (!f1)
		return;
	for (size_t i = 0; i < grp.size(); i++)
	{
		f1 << std::fixed << std::setprecision(8) << point_cloud[grp[i]].x << "	" << point_cloud[grp[i]].z << "	" << -point_cloud[grp[i]].y << std::endl;
	}
	f1.close();
}

void CSF::saveOffGroundPoints(const std::vector<int>& grp, std::string path)
{
	std::string filepath = "off-ground points.txt";
	if (path != "")
	{
		filepath = path;
	}
	std::ofstream f1(filepath, std::ios::out);
	if (!f1)
		return;
	for (size_t i = 0; i < grp.size(); i++)
	{
		f1 << std::fixed << std::setprecision(8) << point_cloud[grp[i]].x << "	" << point_cloud[grp[i]].z << "	" << -point_cloud[grp[i]].y << std::endl;
	}
	f1.close();
}