#include "Rasterization.h"
#include <iostream>
#include <queue>
using namespace std;
//Since all the particles in cloth are formed as a regular grid, 
//for each lidar point, its nearest Cloth point can be simply found by Rounding operation
//then record all the correspoinding lidar point for each cloth particle
#if 1   



double Rasterization::findHeightValByScanline(Particle *p, Cloth &cloth)
{
	int xpos = p->pos_x;
	int ypos = p->pos_y;
	//横向向右扫描
	for (int i = xpos + 1; i < cloth.num_particles_width; i++)
	{
		double crresHeight = cloth.getParticle(i, ypos).nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}
	//横向向左扫描
	for (int i = xpos - 1; i >= 0; i--)
	{
		double crresHeight = cloth.getParticle(i, ypos).nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}
	//纵向向上扫描
	for (int j = ypos - 1; j >= 0; j--)
	{
		double crresHeight = cloth.getParticle(xpos, j).nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}
	//纵向向下扫描
	for (int j = ypos + 1; j < cloth.num_particles_height; j++)
	{
		double crresHeight = cloth.getParticle(xpos, j).nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}

	return findHeightValByNeighbor(p, cloth);


}

double Rasterization::findHeightValByNeighbor(Particle *p, Cloth &cloth)
{
	queue<Particle*> nqueue;
	vector<Particle *> pbacklist;
	size_t neiborsize = p->neighborsList.size();
	for (size_t i = 0; i < neiborsize; i++)
	{
		p->isVisited = true;
		nqueue.push(p->neighborsList[i]);
	}

	//iterate over the nqueue
	while (!nqueue.empty())
	{
		Particle *pneighbor = nqueue.front();
		nqueue.pop();
		pbacklist.push_back(pneighbor);
		if (pneighbor->nearestPointHeight > MIN_INF)
		{
			for (int i = 0; i < pbacklist.size(); i++)
				pbacklist[i]->isVisited = false;
			while (!nqueue.empty())
			{
				Particle *pp = nqueue.front();
				pp->isVisited = false;
				nqueue.pop();
			}
			return pneighbor->nearestPointHeight;
		}
		else
		{
			size_t nsize = pneighbor->neighborsList.size();
			for (size_t i = 0; i < nsize; i++)
			{
				Particle *ptmp = pneighbor->neighborsList[i];
				if (!ptmp->isVisited)
				{
					ptmp->isVisited = true;
					nqueue.push(ptmp);
				}
			}

		}
	}
	return MIN_INF;
}

bool Rasterization::RasterTerrain(Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN)
{
	try
	{
		//首先对每个lidar点找到在布料网格中对应的节点，并记录下来
		//find the nearest cloth particle for each lidar point by Rounding operation
		for (int i = 0; i < pc.size(); i++)
		{
			double pc_x = pc[i].x;
			double pc_z = pc[i].z;
			//将该坐标与布料的左上角坐标相减 minus the top-left corner of the cloth
			double deltaX = pc_x - cloth.origin_pos.x;
			double deltaZ = pc_z - cloth.origin_pos.z;
			int col = int(deltaX / cloth.step_x + 0.5);
			int row = int(deltaZ / cloth.step_y + 0.5);
			if (col >= 0 && row >= 0 )
			{
				Particle& pt = cloth.getParticle(col, row);
				//Particle pt = cloth.getParticle(col, row); this give wrong results, since it made a copy
				pt.correspondingLidarPointList.push_back(i);
				double pc2particleDist = SQUARE_DIST(pc_x, pc_z, pt.pos.x, pt.pos.z);
				if (pc2particleDist < pt.tmpDist)
				{
					pt.tmpDist = pc2particleDist;
					pt.nearestPointHeight = pc[i].y;
					pt.nearestPointIndex = i;
				}
			}
		}
		

		heightVal.resize(cloth.getSize());
		//#pragma omp parallel for
		for (int i = 0; i < cloth.getSize(); i++)
		{
			Particle& pcur = cloth.getParticleByIndex(i);
			double nearestHeight = pcur.nearestPointHeight;
			
			if (nearestHeight > MIN_INF)
			{
				heightVal[i] = nearestHeight;
			}
			else
			{
				heightVal[i] = findHeightValByScanline(&pcur, cloth);
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
//CGAL is slow but more stable, especially when the point cloud is sparse (relatively to the rectangular raster grid!)
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_d;
typedef CGAL::Search_traits_2<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

bool Rasterization::RasterTerrain(Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN)
{
	try
	{
		//首先建立映射xz->y 即通过xz的坐标能够找到y的坐标，因为所有点云不可能有xz重合的
		std::map<std::string, double > mapstring;
		std::list<Point_d> points_2d;

		for (size_t i = 0; i < pc.size(); i++)
		{
			std::ostringstream ostrx, ostrz;
			ostrx << pc[i].x;
			ostrz << pc[i].z;
			mapstring.insert(std::pair<std::string, double>(ostrx.str() + ostrz.str(), pc[i].y));
			points_2d.push_back(Point_d(pc[i].x, pc[i].z));
		}
		Tree tree(points_2d.begin(), points_2d.end());

		heightVal.resize(cloth.getSize());
		for (int i = 0; i < cloth.getSize(); i++)
		{
			const Particle& particle = cloth.getParticleByIndex(i);
			Point_d query(particle.pos.x, particle.pos.z);
			Neighbor_search search(tree, query, KNN);
			double search_max = 0;
			for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
			{
				std::ostringstream ostrx, ostrz;
				ostrx << it->first.x();
				ostrz << it->first.y();
				double y = mapstring[ostrx.str() + ostrz.str()];
				if (y > search_max || it == search.begin()) //first value
				{
					search_max = y;
				}
			}
			heightVal[i] = search_max;
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}


//we use CC_CORE_LIB --> potentially much faster but very slow when the the point cloud is sparse (relatively to the rectangular raster grid!)

//CC_CORE_LIB
#include <SimpleCloud.h>
#include <DgmOctree.h>
#include <ReferenceCloud.h>
#include <DistanceComputationTools.h>
#include <QThread>

static bool ComputeMaxNeighborAltitude(	const CCLib::DgmOctree::octreeCell& cell,
										void** additionalParameters,
										CCLib::NormalizedProgress* nProgress = 0)
{
	//additional parameters
	const wl::PointCloud& pc = *(const wl::PointCloud*)additionalParameters[0];
	std::vector<double>& heightVal = *(std::vector<double>*)additionalParameters[1];
	const CCLib::DgmOctree* cloudOctree = (CCLib::DgmOctree*)additionalParameters[2];
	unsigned KNN = *(unsigned*)additionalParameters[3];

	//structure for the nearest neighbor search
	CCLib::DgmOctree::NearestNeighboursSearchStruct nNSS;
	nNSS.level = cell.level;
	nNSS.minNumberOfNeighbors = KNN;
	cloudOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
	cloudOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);

	//for each point of the current cell ('particles' octree) we look for its
	//nearest neighbours in the point cloud (not too far!)
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		//find the nearest points around the current particle
		cell.points->getPoint(i, nNSS.queryPoint);
		
		unsigned n = cloudOctree->findNearestNeighborsStartingFromCell(nNSS);

		unsigned particleIndex = cell.points->getPointGlobalIndex(i);

		//determine the (highest) neighbor altitude
		double search_max = heightVal[particleIndex];
		for (unsigned k = 0; k < std::min(KNN, n); ++k)
		{
			unsigned pointIndex = nNSS.pointsInNeighbourhood[k].pointIndex;
			if (std::isnan(search_max) || pc[pointIndex].y > search_max)
			{
				search_max = pc[pointIndex].y;
			}
		}
		heightVal[particleIndex] = search_max;
	}

	return true;
}

bool Rasterization::RasterTerrain(Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN)
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

	//test
	if (false)
	{
		FILE* fp = fopen("particle.asc", "wt");
		for (unsigned i = 0; i < particlePoints.size(); i++)
		{
			const CCVector3* P = particlePoints.getPoint(i);
			fprintf(fp, "%f %f %f\n", P->x, P->y, P->z);
		}
		fclose(fp);
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

	//test
	if (false)
	{
		FILE* fp = fopen("points.asc", "wt");
		for (unsigned i = 0; i < pcPoints.size(); i++)
		{
			const CCVector3* P = pcPoints.getPoint(i);
			fprintf(fp, "%f %f %f\n", P->x, P->y, P->z);
		}
		fclose(fp);
	}

	try
	{
		heightVal.resize(cloth.getSize(), std::numeric_limits<double>::quiet_NaN());

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
		void* additionalParameters[] = {	(void*)(&pc),
											(void*)(&heightVal),
											(void*)(cloudOctree),
											(void*)(&KNN)
		};

		int octreeLevel = particleOctree->findBestLevelForAGivenPopulationPerCell(std::max<unsigned>(2, KNN));
		//int octreeLevel2 = cloudOctree->findBestLevelForComparisonWithOctree(particleOctree);

		int result = particleOctree->executeFunctionForAllCellsAtLevel(
							octreeLevel,
							ComputeMaxNeighborAltitude,
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
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

#endif