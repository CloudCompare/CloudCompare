#include "Cloud2CloudDist.h"
 
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

//system
#include <cmath>

bool Cloud2CloudDist::Compute(	const Cloth& cloth,
								const wl::PointCloud& pc,
								double class_threshold,
								std::vector<int>& groundIndexes,
								std::vector<int>& offGroundIndexes)
{
	try
	{
		const unsigned int N = 3;
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