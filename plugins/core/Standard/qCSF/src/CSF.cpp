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

//CC
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>

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

#if defined(_OPENMP)
//OpenMP
#include <omp.h>
#endif

bool CSF::Apply(const wl::PointCloud& csfPointCloud,
				const Parameters& params,
				std::vector<bool>& isGround,
				bool exportClothMesh,
				ccMesh*& clothMesh,
				ccMainAppInterface* app/*=nullptr*/,
				QWidget* parent/*=nullptr*/)
{
	if (params.cloth_resolution < std::numeric_limits<double>::epsilon())
	{
		app->dispToConsole("[CSF] Input cloth resolution is too small");
		return false;
	}

	try
	{
		QElapsedTimer timer;
		timer.start();

		//compute the terrain (cloud) bounding-box
		wl::Point bbMin, bbMax;
		csfPointCloud.computeBoundingBox(bbMin, bbMax);

		//computing the number of cloth node
		Vec3 origin_pos(	bbMin.x - params.clothBuffer * params.cloth_resolution,
							bbMax.y + params.clothYHeight,
							bbMin.z - params.clothBuffer * params.cloth_resolution);
	
		int width_num = static_cast<int>((bbMax.x - bbMin.x) / params.cloth_resolution) + 2 * params.clothBuffer; //static_cast is equivalent to floor if value >= 0
		int height_num = static_cast<int>((bbMax.z - bbMin.z) / params.cloth_resolution) + 2 * params.clothBuffer; //static_cast is equivalent to floor if value >= 0
		
		//Cloth object
		Cloth cloth(origin_pos, 
					width_num,
					height_num,
					params.cloth_resolution,
					params.cloth_resolution,
					0.3,
					9999,
					params.rigidness/*,
					params.time_step*/);
		if (app)
		{
			app->dispToConsole(QString("[CSF] Cloth creation: %1 ms").arg(timer.restart()));
		}

		if (!Rasterization::RasterTerrain(cloth, csfPointCloud, params.k_nearest_points))
		{
			return false;
		}
	
		if (app)
		{
			app->dispToConsole(QString("[CSF] Rasterization: %1 ms").arg(timer.restart()));
		}

		double squareTimeStep = params.time_step * params.time_step;

#if defined(_OPENMP)
		//save the current max number of threads before changing it
		int maxThreadCount = omp_get_max_threads();
		omp_set_num_threads(ccQtHelpers::GetMaxThreadCount(maxThreadCount));
#endif

		//do the filtering
		QProgressDialog pDlg(parent);
		pDlg.setWindowTitle("CSF");
		pDlg.setLabelText(QObject::tr("Cloth deformation\n%1 x %2 particles").arg(cloth.num_particles_width).arg(cloth.num_particles_height));
		pDlg.setRange(0, params.iterations);
		pDlg.show();
		QCoreApplication::processEvents();

		bool wasCancelled = false;
		cloth.addForce(-params.gravity * squareTimeStep); // DGM: warning, the force is already mutliplied by dt^2, no need to do it later (in Particle::timeStep())
		for (int i = 0; i < params.iterations; i++)
		{
			double maxDiff = cloth.timeStep();
			cloth.terrainCollision();

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
#if defined(_OPENMP)
			//restore the original max number of threads
			omp_set_num_threads(maxThreadCount);
#endif
			return false;
		}

		//slope processing
		if (params.smoothSlope)
		{
			cloth.movableFilter();

			if (app)
			{
				app->dispToConsole(QString("[CSF] Movable filter: %1 ms").arg(timer.restart()));
			}
		}
	
		//classification of the points
		bool result = Cloud2CloudDist::Compute(cloth, csfPointCloud, params.class_threshold, isGround);
		if (app)
		{
			app->dispToConsole(QString("[CSF] Distance computation: %1 ms").arg(timer.restart()));
		}

		if (exportClothMesh)
		{
			clothMesh = cloth.toMesh();
		}

#if defined(_OPENMP)
		//restore the original max number of threads
		omp_set_num_threads(maxThreadCount);
#endif

		return result;
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
}

bool CSF::Apply(ccPointCloud* cloud,
				const Parameters& params,
				ccPointCloud*& groundCloud,
				ccPointCloud*& offGroundCloud,
				bool exportClothMesh,
				ccMesh*& clothMesh,
				ccMainAppInterface* app/*=nullptr*/)
{
	// just in case
	assert(groundCloud == nullptr);
	groundCloud = nullptr;
	assert(offGroundCloud == nullptr);
	offGroundCloud = nullptr;
	assert(clothMesh == nullptr);

	if (!cloud)
	{
		assert(false);
		return false;
	}

	QElapsedTimer timer;
	timer.start();

	unsigned pointCount = cloud->size();
	if (pointCount == 0)
	{
		// nothing to do
		if (app)
		{
			app->dispToConsole(QString("Cloud %1 is empty").arg(cloud->getName()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
		return true;
	}

	try
	{
		// convert CC point cloud to CSF cloud
		wl::PointCloud csfPC;
		{
			csfPC.resize(pointCount);

			for (unsigned i = 0; i < pointCount; i++)
			{
				const CCVector3* P = cloud->getPoint(i);
				wl::Point& tmpPoint = csfPC[i];
				tmpPoint.x = P->x;
				tmpPoint.y = -P->z;
				tmpPoint.z = P->y;
			}
		}

		//filtering
		std::vector<bool> isGround;
		if (!CSF::Apply(csfPC, params, isGround, exportClothMesh, clothMesh, app))
		{
			if (app)
			{
				app->dispToConsole("Process failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
			return false;
		}

		//count the number of ground points
		unsigned groundPointCount = 0;
		for (size_t i = 0; i < isGround.size(); ++i)
		{
			if (isGround[i])
			{
				++groundPointCount;
			}
		}

		if (app)
		{
			app->dispToConsole(QString("[CSF] %1% of points classified as ground points").arg((groundPointCount * 100.0) / pointCount, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			app->dispToConsole(QString("[CSF] Timing: %1 s.").arg(timer.elapsed() / 1000.0, 0, 'f', 1), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

		//extract ground subset
		if (groundPointCount != 0)
		{
			CCCoreLib::ReferenceCloud groundRef(cloud);
			if (!groundRef.reserve(groundPointCount))
			{
				if (app)
				{
					app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				return false;
			}

			for (size_t i = 0; i < isGround.size(); ++i)
			{
				if (isGround[i])
				{
					groundRef.addPointIndex(static_cast<unsigned>(i));
				}
			}

			groundCloud = cloud->partialClone(&groundRef);
			if (!groundCloud)
			{
				if (app)
				{
					app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				return false;
			}
		}

		//extract off-ground subset
		if (groundPointCount < cloud->size())
		{
			CCCoreLib::ReferenceCloud offgroundRef(cloud);
			if (!offgroundRef.reserve(cloud->size() - groundPointCount))
			{
				if (app)
				{
					app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				return false;
			}

			for (size_t i = 0; i < isGround.size(); ++i)
			{
				if (!isGround[i])
				{
					offgroundRef.addPointIndex(static_cast<unsigned>(i));
				}
			}

			offGroundCloud = cloud->partialClone(&offgroundRef);
			if (!offGroundCloud)
			{
				if (groundCloud)
				{
					delete groundCloud;
					groundCloud = nullptr;
				}
				if (app)
				{
					app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				return false;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		if (app)
		{
			app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		return false;
	}

	return true;
}
