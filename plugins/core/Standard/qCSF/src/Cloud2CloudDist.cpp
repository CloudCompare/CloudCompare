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

#include "Cloud2CloudDist.h"
 
//system
#include <cmath>


// For each lidar point, we find its neibors in cloth particles by  Rounding operation.
//use for neighbor particles to do bilinear interpolation.
#if 1 

bool Cloud2CloudDist::Compute(const Cloth& cloth,
	const wl::PointCloud& pc,
	double class_threshold,
	std::vector<int>& groundIndexes,
	std::vector<int>& offGroundIndexes,
	unsigned N/*=3*/)
{

	try
	{
		//’“µΩ√ø∏ˆº§π‚¿◊¥Ôµ„µΩ≤º¡œ÷±Ω”µƒæ‡¿Î£¨”√∏√æ‡¿Î„–÷µ¿¥∂‘µ„‘∆Ω¯––∑÷¿‡
		//À´œﬂ–‘≤Â÷µ
		// for each lidar point, find the projection in the cloth grid, and the sub grid which contains it.
		//use the four corner of the subgrid to do bilinear interpolation;
		for (int i = 0; i < pc.size(); i++)
		{
			double pc_x = pc[i].x;
			double pc_z = pc[i].z;
			//Ω´∏√◊¯±Í”Î≤º¡œµƒ◊Û…œΩ«◊¯±Íœ‡ºı
			double deltaX = pc_x - cloth.origin_pos.x;
			double deltaZ = pc_z - cloth.origin_pos.z;
			//µ√µΩº§π‚µ„À˘‘⁄≤º¡œ–°Õ¯∏Ò◊Û…œΩ«µƒ◊¯±Í ºŸ…ËÀƒ∏ˆΩ«µ„∑÷±Œ™0 1 2 3 À≥ ±’Î±‡∫≈
			int col0 = int(deltaX / cloth.step_x);
			int row0 = int(deltaZ / cloth.step_y);
			int col1 = col0 + 1;
			int row1 = row0;
			int col2 = col0 + 1;
			int row2 = row0 + 1;
			int col3 = col0;
			int row3 = row0 + 1;
			//“‘◊”Õ¯∏Ò◊Û…œΩ«Ω®¡¢◊¯±Íœµ£¨≤¢Ω´∆‰πÈ“ªªØµΩ[0,1]
			double subdeltaX = (deltaX - col0*cloth.step_x) / cloth.step_x;
			double subdeltaZ = (deltaZ - row0*cloth.step_y) / cloth.step_y;
			//cout << subdeltaX << " " << subdeltaZ << endl;
			//À´œﬂ–‘≤Â÷µ bilinear interpolation;
			//f(x,y)=f(0,0)(1-x)(1-y)+f(0,1)(1-x)y+f(1,1)xy+f(1,0)x(1-y)
			double fxy = cloth.getParticle(col0, row0).pos.y * (1 - subdeltaX)*(1 - subdeltaZ)
				+ cloth.getParticle(col3, row3).pos.y * (1 - subdeltaX)*subdeltaZ
				+ cloth.getParticle(col2, row2).pos.y * subdeltaX*subdeltaZ
				+ cloth.getParticle(col1, row1).pos.y * subdeltaX*(1 - subdeltaZ);
			double height_var = fxy - pc[i].y;
			if (std::fabs(height_var) < class_threshold)
			{
				groundIndexes.push_back(i);
			}
			else
			{
				offGroundIndexes.push_back(i);
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




#else 


//CGAL //CGAL is much slower for this operation
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_d;
typedef CGAL::Search_traits_2<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

bool Cloud2CloudDist::Compute(const Cloth& cloth,
	const wl::PointCloud& pc,
	double class_threshold,
	std::vector<int>& groundIndexes,
	std::vector<int>& offGroundIndexes,
	unsigned N/*=3*/)
{
	try
	{
		std::list<Point_d> points_2d;
		std::map<std::string, double >mapstring;

		// maping coordinates xy->z  to query the height value of each point
		for (int i = 0; i < cloth.getSize(); i++)
		{
			const Particle& particle = cloth.getParticleByIndex(i);
			std::ostringstream ostrx, ostrz;
			ostrx << particle.pos.x;
			ostrz << particle.pos.z;
			mapstring.insert(std::pair<std::string, double>(ostrx.str() + ostrz.str(), particle.pos.y));
			points_2d.push_back(Point_d(particle.pos.x, particle.pos.z));
		}

		Tree tree(points_2d.begin(), points_2d.end());
		// step two  query the nearest point of cloth for each terrain point
		for (int i = 0; i < pc.size(); i++)
		{
			Point_d query(pc[i].x, pc[i].z);
			Neighbor_search search(tree, query, N);
			double search_min = 0;
			for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
			{
				std::ostringstream ostrx, ostrz;
				ostrx << it->first.x();
				ostrz << it->first.y();
				double y = mapstring[ostrx.str() + ostrz.str()];
				search_min = search_min + y / N;
				//if (y > search_min)
				//{
				//	search_min = y;
				//}
			}
			if (std::fabs(search_min - pc[i].y) < class_threshold)
			{
				groundIndexes.push_back(i);
			}
			else
			{
				offGroundIndexes.push_back(i);
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

//CCLib is always much faster

//CC_CORE_LIB
#include <SimpleCloud.h>
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <QThread>

static bool ComputeMeanNeighborAltitude(const CCLib::DgmOctree::octreeCell& cell,
										void** additionalParameters,
										CCLib::NormalizedProgress* nProgress = 0)
{
	//additional parameters
	const Cloth& cloth = *(const Cloth*)additionalParameters[0];
	const CCLib::DgmOctree* particleOctree = (CCLib::DgmOctree*)additionalParameters[1];
	unsigned N = *(unsigned*)additionalParameters[2];

	//structure for the nearest neighbor search
	CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.minNumberOfNeighbors = N;
	particleOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	particleOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//for each point of the cloud  we look for its nearest neighbours in the set of particles
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		//find the nearest particles around the current point
		cell.points->getPoint(i, nNSS.queryPoint);

		unsigned n = particleOctree->findNearestNeighborsStartingFromCell(nNSS);
		unsigned kNN = std::min(N, n);
		if (kNN == 0)
		{
			assert(false);
			continue;
		}

		//compute the average height
		double search_min = 0;
		for (unsigned k = 0; k < kNN; ++k)
		{
			unsigned particleIndex = nNSS.pointsInNeighbourhood[k].pointIndex;
			double y = cloth.getParticleByIndex(particleIndex).pos.y;
			search_min += y;
		}
		search_min /= kNN;

		cell.points->setPointScalarValue(i, static_cast<ScalarType>(search_min));
	}

	return true;
}

bool Cloud2CloudDist::Compute(	const Cloth& cloth,
								const wl::PointCloud& pc,
								double class_threshold,
								std::vector<int>& groundIndexes,
								std::vector<int>& offGroundIndexes,
								unsigned N/*=3*/)
{
	CCLib::SimpleCloud particlePoints;
	if (!particlePoints.reserve(static_cast<unsigned>(cloth.getSize())))
	{
		//not enough memory
		return false;
	}
	for (int i = 0; i < cloth.getSize(); i++)
	{
		const Particle& particle = cloth.getParticleByIndex(i);
		particlePoints.addPoint(CCVector3(static_cast<PointCoordinateType>(particle.pos.x), 0, static_cast<PointCoordinateType>(particle.pos.z)));
	}

	CCLib::SimpleCloud pcPoints;
	if (!pcPoints.reserve(static_cast<unsigned>(pc.size())))
	{
		//not enough memory
		return false;
	}
	for (size_t i = 0; i < pc.size(); i++)
	{
		const wl::Point& P = pc[i];
		pcPoints.addPoint(CCVector3(P.x, 0, P.z));
	}

	try
	{
		//we spatially 'synchronize' the cloud and particles octrees
		CCLib::DgmOctree *cloudOctree = 0, *particleOctree = 0;
		CCLib::DistanceComputationTools::SOReturnCode soCode =
			CCLib::DistanceComputationTools::synchronizeOctrees
			(
			&particlePoints,
			&pcPoints,
			particleOctree,
			cloudOctree,
			0,
			0
			);

		if (soCode != CCLib::DistanceComputationTools::SYNCHRONIZED && soCode != CCLib::DistanceComputationTools::DISJOINT)
		{
			//not enough memory (or invalid input)
			return false;
		}

		//additional parameters
		void* additionalParameters[] = {(void*)(&cloth),
										(void*)(particleOctree),
										(void*)(&N)
		};

		int octreeLevel = particleOctree->findBestLevelForAGivenPopulationPerCell(std::max<unsigned>(2, N));

		int result = cloudOctree->executeFunctionForAllCellsAtLevel(
			octreeLevel,
			ComputeMeanNeighborAltitude,
			additionalParameters,
			true,
			0,
			"Rasterization",
			QThread::idealThreadCount());

		delete cloudOctree;
		cloudOctree = 0;

		delete particleOctree;
		particleOctree = 0;

		if (result == 0)
		{
			//something went wrong
			return false;
		}

		//now classify the points
		for (unsigned i = 0; i < pcPoints.size(); ++i)
		{
			if (std::fabs(pcPoints.getPointScalarValue(i) - pc[i].y) < class_threshold)
			{
				groundIndexes.push_back(i);
			}
			else
			{
				offGroundIndexes.push_back(i);
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

#endif