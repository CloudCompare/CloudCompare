//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

//CSF
#include "CSF.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "Cloud2CloudDist.h"

//CC (for debug)
#include <ccMainAppInterface.h>

//Qt
#include <QProgressDialog>
#include <QCoreApplication>
#include <QElapsedTimer>

//system
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>

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

bool CSF::readPointsFromFile(std::string filename)
{
	point_cloud.clear();
	
	try
	{
		std::ifstream fin(filename.c_str(), std::ios::in);

		char line[500];
		std::string x;
		std::string y;
		std::string z;
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

//CSFÖ÷³ÌÐò dofiltering
bool CSF::do_filtering(	std::vector<int>& groundIndexes,
						std::vector<int>& offGroundIndexes,
						bool exportClothMesh,
						ccMesh* &clothMesh,
						ccMainAppInterface* app/*=0*/,
						QWidget* parent/*=0*/)
{
	//constants
	static const double cloth_y_height = 0.05; //origin cloth height
	static const int clothbuffer = 2; //set the cloth buffer (grid margin size)
	static const double gravity = 0.2;

	try
	{
		QElapsedTimer timer;
		timer.start();

		//compute the terrain (cloud) bounding-box
		wl::Point bbMin;
		wl::Point bbMax;
		point_cloud.computeBoundingBox(bbMin, bbMax);

		//computing the number of cloth node
		Vec3 origin_pos(	bbMin.x - clothbuffer * params.cloth_resolution,
							bbMax.y + cloth_y_height,
							bbMin.z - clothbuffer * params.cloth_resolution);
	
		int width_num = static_cast<int>(floor((bbMax.x - bbMin.x) / params.cloth_resolution)) + 2 * clothbuffer;
		int height_num = static_cast<int>(floor((bbMax.z - bbMin.z) / params.cloth_resolution)) + 2 * clothbuffer;
		
		//Cloth object
		Cloth cloth(origin_pos, 
					width_num,
					height_num,
					params.cloth_resolution,
					params.cloth_resolution,
					0.3,
					9999,
					params.rigidness,
					params.time_step);
		if (app)
		{
			app->dispToConsole(QString("[CSF] Cloth creation: %1 ms").arg(timer.restart()));
		}

		if (!Rasterization::RasterTerrain(cloth, point_cloud, cloth.getHeightvals(), params.k_nearest_points))
		{
			return false;
		}
		//app->dispToConsole("raster cloth", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	
		if (app)
		{
			app->dispToConsole(QString("[CSF] Rasterization: %1 ms").arg(timer.restart()));
		}

		double time_step2 = params.time_step * params.time_step;

		//do the filtering
		QProgressDialog pDlg(parent);
		pDlg.setWindowTitle("CSF");
		pDlg.setLabelText(QString("Cloth deformation\n%1 x %2 particles").arg(cloth.num_particles_width).arg(cloth.num_particles_height));
		pDlg.setRange(0, params.iterations);
		pDlg.show();
		QCoreApplication::processEvents();

		bool wasCancelled = false;
		cloth.addForce(Vec3(0, -gravity, 0) * time_step2);
		for (int i = 0; i < params.iterations; i++)
		{
			//cloth.addForce(Vec3(0, -gravity, 0) * time_step2); //move this outside the main loop
			double maxDiff = cloth.timeStep();
			cloth.terrainCollision();

			//if (app && (i % 50) == 0)
			//{
			//	app->dispToConsole(QString("[CSF] Iteration %1: max delta = %2").arg(i+1).arg(maxDiff));
			//}

			if (maxDiff != 0 && maxDiff < 0.005)
			{
				//early stop
				break;
			}

			pDlg.setValue(i);
			QCoreApplication::processEvents();

			if (pDlg.wasCanceled())
			{
				wasCancelled = true;
				break;
			}
		}
		
		pDlg.close();
		QCoreApplication::processEvents();

		if (app)
		{
			app->dispToConsole(QString("[CSF] Iterations: %1 ms").arg(timer.restart()));
		}

		if (wasCancelled)
		{
			return false;
		}

		//slope processing
		if (params.bSloopSmooth)
		{
			cloth.movableFilter();

			if (app)
			{
				app->dispToConsole(QString("[CSF] Movable filter: %1 ms").arg(timer.restart()));
			}
		}
	
		//classification of the points
		bool result = Cloud2CloudDist::Compute(cloth, point_cloud, params.class_threshold, groundIndexes, offGroundIndexes);
		if (app)
		{
			app->dispToConsole(QString("[CSF] Distance computation: %1 ms").arg(timer.restart()));
		}

		if (exportClothMesh)
		{
			clothMesh = cloth.toMesh();
		}

		return result;
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
}

//Exporting the ground points to file.
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
