#include "KNN.h"

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

bool Rasterlization::RasterTerrian(Cloth cloth, const wl::PointCloud& pc, std::vector<double>& heightVal)
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
			Point_d query(cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]);
			Neighbor_search search(tree, query, N);
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