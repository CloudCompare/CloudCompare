#include "c2cdist.h"
#include <cmath>
 
//Qt
#include <QProgressDialog>
#include <QApplication>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>



vector<vector<int>> c2cdist::calCloud2CloudDist(Cloth cloth, PointCloud &pc,unsigned PCSIZE)
{
	vector<vector<int>> c2cre;
	c2cre.resize(PCSIZE);

	const unsigned int N = 3;
	list<Point_d> points_2d;
	map<string, double >mapstring;

	/// maping coordinates xy->z  to query the height value of each point

	for (int i = 0; i < cloth.getSize(); i++)
	{
		ostringstream ostrx, ostrz;
		ostrx << cloth.getParticle1d(i)->getPos().f[0];
		ostrz << cloth.getParticle1d(i)->getPos().f[2];
		mapstring.insert(pair<string, double>(ostrx.str() + ostrz.str(), cloth.getParticle1d(i)->getPos().f[1]));
		points_2d.push_back(Point_d( cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]));
	}


	Tree tree(points_2d.begin(), points_2d.end());
	// step two  query the nearest point of cloth for each terr point
	double height_var=0;

	for (int i = 0; i < pc.size(); i++)
	{
		Point_d query(pc[i].x,pc[i].z);
		Neighbor_search search(tree, query, N);
		double search_min = 0;
		for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
		{
			ostringstream ostrx, ostrz;
			ostrx << it->first.x();
			ostrz << it->first.y();
			double y = mapstring[ostrx.str() + ostrz.str()];
			search_min = search_min + y / double(N);
		//	if (y > search_min){ search_min = y; }
		}
		height_var = search_min - pc[i].y;
		if (std::fabs(height_var) < class_treshold)
		{
			c2cre[1].push_back(i);
		}
		else
		{
			c2cre[2].push_back(i);
		}
	}

	return c2cre;
}