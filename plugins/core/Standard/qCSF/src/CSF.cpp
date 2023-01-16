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
#include <ccPointCloud.h>

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

//OpenMP
#include <omp.h>

CSF::CSF(wl::PointCloud& cloud, const Parameters& params)
	: m_pointCloud(cloud)
	, m_params(params)
{
}

bool CSF::do_filtering(	std::vector<unsigned>& groundIndexes,
						std::vector<unsigned>& offGroundIndexes,
						bool exportClothMesh,
						ccMesh*& clothMesh,
						ccMainAppInterface* app/*=nullptr*/,
						QWidget* parent/*=nullptr*/)
{
	if (m_params.cloth_resolution < std::numeric_limits<double>::epsilon())
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
		m_pointCloud.computeBoundingBox(bbMin, bbMax);

		//computing the number of cloth node
		Vec3 origin_pos(	bbMin.x - m_params.clothBuffer * m_params.cloth_resolution,
							bbMax.y + m_params.clothYHeight,
							bbMin.z - m_params.clothBuffer * m_params.cloth_resolution);
	
		int width_num = static_cast<int>(floor((bbMax.x - bbMin.x) / m_params.cloth_resolution)) + 2 * m_params.clothBuffer;
		int height_num = static_cast<int>(floor((bbMax.z - bbMin.z) / m_params.cloth_resolution)) + 2 * m_params.clothBuffer;
		
		//Cloth object
		Cloth cloth(origin_pos, 
					width_num,
					height_num,
					m_params.cloth_resolution,
					m_params.cloth_resolution,
					0.3,
					9999,
					m_params.rigidness,
					m_params.time_step);
		if (app)
		{
			app->dispToConsole(QString("[CSF] Cloth creation: %1 ms").arg(timer.restart()));
		}

		if (!Rasterization::RasterTerrain(cloth, m_pointCloud, cloth.getHeightvals(), m_params.k_nearest_points))
		{
			return false;
		}
		//app->dispToConsole("raster cloth", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	
		if (app)
		{
			app->dispToConsole(QString("[CSF] Rasterization: %1 ms").arg(timer.restart()));
		}

		double squareTimeStep = m_params.time_step * m_params.time_step;

		omp_set_num_threads(std::max(1, omp_get_max_threads() - 1)); // always leave one thread/core to let the application breath


		//do the filtering
		QProgressDialog pDlg(parent);
		pDlg.setWindowTitle("CSF");
		pDlg.setLabelText(QObject::tr("Cloth deformation\n%1 x %2 particles").arg(cloth.num_particles_width).arg(cloth.num_particles_height));
		pDlg.setRange(0, m_params.iterations);
		pDlg.show();
		QCoreApplication::processEvents();

		bool wasCancelled = false;
		cloth.addForce(Vec3(0, -m_params.gravity, 0) * squareTimeStep);
		for (int i = 0; i < m_params.iterations; i++)
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
			return false;
		}

		//slope processing
		if (m_params.smoothSlope)
		{
			cloth.movableFilter();

			if (app)
			{
				app->dispToConsole(QString("[CSF] Movable filter: %1 ms").arg(timer.restart()));
			}
		}
	
		//classification of the points
		bool result = Cloud2CloudDist::Compute(cloth, m_pointCloud, m_params.class_threshold, groundIndexes, offGroundIndexes);
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

	//Convert CC point cloud to CSF type
	wl::PointCloud csfPC;
	{
		try
		{
			csfPC.resize(pointCount);
		}
		catch (const std::bad_alloc&)
		{
			if (app)
			{
				app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
			return false;
		}

		for (unsigned i = 0; i < pointCount; i++)
		{
			const CCVector3* P = cloud->getPoint(i);
			wl::Point& tmpPoint = csfPC[i];
			tmpPoint.x = P->x;
			tmpPoint.y = -P->z;
			tmpPoint.z = P->y;
		}
	}

	//instantiation a CSF class
	CSF csf(csfPC, params);

	//to do filtering
	std::vector<unsigned> groundIndexes;
	std::vector<unsigned> offGroundIndexes;
	assert(clothMesh == nullptr);
	if (!csf.do_filtering(groundIndexes, offGroundIndexes, exportClothMesh, clothMesh, app))
	{
		if (app)
		{
			app->dispToConsole("Process failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		return false;
	}

	if (app)
	{
		app->dispToConsole(QString("[CSF] %1% of points classified as ground points").arg((groundIndexes.size() * 100.0) / pointCount, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		app->dispToConsole(QString("[CSF] Timing: %1 s.").arg(timer.elapsed() / 1000.0, 0, 'f', 1), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	//extract ground subset
	{
		CCCoreLib::ReferenceCloud groundRef(cloud);
		if (!groundRef.reserve(static_cast<unsigned>(groundIndexes.size())))
		{
			if (app)
			{
				app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
			return false;
		}

		for (unsigned j : groundIndexes)
		{
			groundRef.addPointIndex(j);
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
	{
		CCCoreLib::ReferenceCloud offgroundRef(cloud);
		if (!offgroundRef.reserve(static_cast<unsigned>(offGroundIndexes.size())))
		{
			if (app)
			{
				app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
			return false;
		}

		for (unsigned k : offGroundIndexes)
		{
			offgroundRef.addPointIndex(k);
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

	return true;
}
