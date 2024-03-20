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

#include "Rasterization.h"

// System
#include <iostream>
#include <queue>

// CCPluginAPI
#include <ccQtHelpers.h>

using namespace std;

//Since all the particles in cloth are formed as a regular grid, 
//for each lidar point, its nearest Cloth point can be simply found by Rounding operation
//then record all the correspoinding lidar point for each cloth particle

static double FindHeightValByNeighbor(Particle& p, Cloth &cloth)
{
	if (!p.neighborsList.empty())
	{
		std::queue<Particle*> nqueue;
		for (Particle* neighbor : p.neighborsList)
		{
			nqueue.push(neighbor);
		}
		p.isVisited = true;

		//iterate over the queue
		vector<Particle*> pbacklist;
		while (!nqueue.empty())
		{
			Particle* pneighbor = nqueue.front();
			nqueue.pop();
			pbacklist.push_back(pneighbor);
			if (pneighbor->nearestPointHeight > std::numeric_limits<double>::lowest())
			{
				for (Particle* pp : pbacklist)
				{
					pp->isVisited = false;
				}
				while (!nqueue.empty())
				{
					Particle* pp = nqueue.front();
					pp->isVisited = false;
					nqueue.pop();
				}
				return pneighbor->nearestPointHeight;
			}
			else
			{
				for (Particle* pp : pneighbor->neighborsList)
				{
					if (!pp->isVisited)
					{
						pp->isVisited = true;
						nqueue.push(pp);
					}
				}

			}
		}
	}
	return std::numeric_limits<double>::lowest();
}

static double FindHeightValByScanline(Particle& p, Cloth& cloth)
{
	for (int i = p.pos_x + 1; i < cloth.num_particles_width; i++)
	{
		double crresHeight = cloth.getParticle(i, p.pos_y).nearestPointHeight;
		if (crresHeight > std::numeric_limits<double>::lowest())
			return crresHeight;
	}

	for (int i = p.pos_x - 1; i >= 0; i--)
	{
		double crresHeight = cloth.getParticle(i, p.pos_y).nearestPointHeight;
		if (crresHeight > std::numeric_limits<double>::lowest())
			return crresHeight;
	}

	for (int j = p.pos_y - 1; j >= 0; j--)
	{
		double crresHeight = cloth.getParticle(p.pos_x, j).nearestPointHeight;
		if (crresHeight > std::numeric_limits<double>::lowest())
			return crresHeight;
	}

	for (int j = p.pos_y + 1; j < cloth.num_particles_height; j++)
	{
		double crresHeight = cloth.getParticle(p.pos_x, j).nearestPointHeight;
		if (crresHeight > std::numeric_limits<double>::lowest())
			return crresHeight;
	}

	return FindHeightValByNeighbor(p, cloth);
}

bool Rasterization::RasterTerrain(Cloth& cloth, const wl::PointCloud& pc, unsigned KNN/*=1*/)
{
	std::vector<double>& heightVal = cloth.getHeightvals();

	try
	{
		//find the nearest cloth particle for each lidar point by Rounding operation
		for (int i = 0; i < pc.size(); i++)
		{
			double pc_x = pc[i].x;
			double pc_z = pc[i].z;
			//minus the top-left corner of the cloth
			double deltaX = pc_x - cloth.origin_pos.x;
			double deltaZ = pc_z - cloth.origin_pos.z;
			int col = int(deltaX / cloth.step_x + 0.5);
			int row = int(deltaZ / cloth.step_y + 0.5);
			if (col >= 0 && row >= 0 )
			{
				Particle& pt = cloth.getParticle(col, row);

				double dx = pt.getPos().x - pc_x;
				double dz = pt.getPos().z - pc_z;
				double pc2particleDist = dx * dx + dz * dz;

				if (pc2particleDist < pt.nearestPointDist)
				{
					pt.nearestPointDist = pc2particleDist;
					pt.nearestPointHeight = pc[i].y;
					//pt.nearestPointIndex = i;
				}
			}
		}
		

		heightVal.resize(cloth.getSize());

		for (int i = 0; i < cloth.getSize(); i++)
		{
			Particle& pcur = cloth.getParticleByIndex(i);
			double nearestHeight = pcur.nearestPointHeight;
			
			if (nearestHeight > std::numeric_limits<double>::lowest())
			{
				heightVal[i] = nearestHeight;
			}
			else
			{
				heightVal[i] = FindHeightValByScanline(pcur, cloth);
			}
		
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}
