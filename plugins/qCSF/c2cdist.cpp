#include "c2cdist.h"
 
//system
#include <cmath>

bool c2cdist::calCloud2CloudDist(const Cloth& cloth, const wl::PointCloud& pc, std::vector< std::vector<int> >& c2cre)
{
	try
	{
		c2cre.resize(2);

		const unsigned int N = 3;
		std::list<Point_d> points_2d;
		std::map<std::string, double >mapstring;

		// maping coordinates xy->z  to query the height value of each point
		for (int i = 0; i < cloth.getSize(); i++)
		{
			std::ostringstream ostrx, ostrz;
			ostrx << cloth.getParticle1d(i)->getPos().f[0];
			ostrz << cloth.getParticle1d(i)->getPos().f[2];
			mapstring.insert(std::pair<std::string, double>(ostrx.str() + ostrz.str(), cloth.getParticle1d(i)->getPos().f[1]));
			points_2d.push_back(Point_d( cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]));
		}

		Tree tree(points_2d.begin(), points_2d.end());
		// step two  query the nearest point of cloth for each terr point
		double height_var = 0;

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
			height_var = search_min - pc[i].y;
			if (std::fabs(height_var) < class_treshold)
			{
				c2cre[0].push_back(i);
			}
			else
			{
				c2cre[1].push_back(i);
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