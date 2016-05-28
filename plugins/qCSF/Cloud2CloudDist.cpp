#include "Cloud2CloudDist.h"
 
//system
#include <cmath>

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

#else

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
	if (	!pcPoints.reserve(static_cast<unsigned>(pc.size()))
		||	!pcPoints.enableScalarField())
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