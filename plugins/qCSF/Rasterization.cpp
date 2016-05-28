#include "Rasterization.h"

#ifdef WITH_CGAL

//CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_d;
typedef CGAL::Search_traits_2<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

bool Rasterization::RasterTerrain(const Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN)
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

#else //we use CC_CORE_LIB (much faster)

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

	//for each point of the current cell ('particles' octree) we look for its nearest neighbours in the point cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		//find the nearest points around the current particle
		cell.points->getPoint(i, nNSS.queryPoint);
		
		unsigned n = cloudOctree->findNearestNeighborsStartingFromCell(nNSS);

		//determine the (highest) neighbor altitude
		double search_max = 0;
		for (unsigned k = 0; k < std::min(KNN, n); ++k)
		{
			unsigned pointIndex = nNSS.pointsInNeighbourhood[k].pointIndex;
			if (pc[pointIndex].y > search_max || k == 0)
			{
				search_max = pc[pointIndex].y;
			}
		}

		unsigned particleIndex = cell.points->getPointGlobalIndex(i);
		heightVal[particleIndex] = search_max;
	}

	return true;
}

bool Rasterization::RasterTerrain(const Cloth& cloth, const wl::PointCloud& pc, std::vector<double>& heightVal, unsigned KNN)
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
		heightVal.resize(cloth.getSize());

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
		//int octreeLevel = cloudOctree->findBestLevelForComparisonWithOctree(particleOctree);

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