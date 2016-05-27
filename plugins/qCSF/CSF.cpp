//CSF
#include "CSF.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "Cloud2CloudDist.h"

//system
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>

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

bool CSF::readPointsFromFile(std::string filename)
{
	point_cloud.clear();
	
	try
	{
		std::ifstream fin(filename.c_str(), std::ios::in);

		char line[500];
		std::string x, y, z;
		while (fin.getline(line, sizeof(line)))
		{
			std::stringstream words(line);
			words >> x;
			words >> y;
			words >> z;
			wl::Point P;
			P.x = static_cast<float>(atof(x.c_str()));
			P.y = static_cast<float>(-atof(z.c_str()));
			P.z = static_cast<float>(atof(y.c_str()));
			point_cloud.push_back(P);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	catch (...)
	{
		//other error
		return false;
	}

	return true;
}

//CSF主程序 dofiltering
bool CSF::do_filtering(std::vector<int>& groundIndexes, std::vector<int>& offGroundIndexes)
{
	//constants
	static const double cloth_y_height = 0.05; //origin cloth height
	static const double clothbuffer_d = 4.0;//set the cloth buffer
	static const double gravity = 0.2;

	try
	{
		//compute the terrain (cloud) bounding-box
		wl::Point bbMin, bbMax;
		point_cloud.computeBoundingBox(bbMin, bbMax);

		//computing the number of cloth node
		Vec3 origin_pos1(	bbMin.x - clothbuffer_d,
							bbMax.y + cloth_y_height,
							bbMin.z - clothbuffer_d);
	
		int width_num  = (bbMax.x - bbMin.x + clothbuffer_d * 2) / params.cloth_resolution;
		int height_num = (bbMax.z - bbMin.z + clothbuffer_d * 2) / params.cloth_resolution;
	
		//one Cloth object of the Cloth class
		Cloth cloth1(	bbMax.x - bbMin.x + clothbuffer_d * 2,
						bbMax.z - bbMin.z + clothbuffer_d * 2,
						width_num,
						height_num,
						origin_pos1,
						0.3,
						9999,
						params.rigidness,
						params.time_step);

		if (!Rasterization::RasterTerrain(cloth1, point_cloud, cloth1.getHeightvals(), params.k_nearest_points))
		{
			return false;
		}
	
		double time_step2 = params.time_step * params.time_step;

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
			cloth1.terrainCollision();

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
			pDlg2.setValue(pDlg2.value() + 1);
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
		return Cloud2CloudDist::Compute(cloth1, point_cloud, params.class_threshold, groundIndexes, offGroundIndexes);
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