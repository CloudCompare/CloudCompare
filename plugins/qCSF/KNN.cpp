#include "KNN.h"

void Rasterlization::RasterTerrian(Cloth cloth, PointCloud &pc, vector<double> &heightVal)
{
	//首先建立映射xz->y 即通过xz的坐标能够找到y的坐标，因为所有点云不可能有xz重合的
	map<string, double >mapstring;
	list<Point_d> points_2d;
	for (int i = 0; i < pc.size(); i++){
		ostringstream ostrx, ostrz;
		ostrx << pc[i].x;
		ostrz << pc[i].z;
		mapstring.insert(pair<string, double>(ostrx.str() + ostrz.str(), pc[i].y));
		points_2d.push_back(Point_d(pc[i].x, pc[i].z));
	}
	Tree tree(points_2d.begin(), points_2d.end());

	heightVal.resize(cloth.getSize());
	for (int i = 0; i < cloth.getSize(); i++)
	{
		Point_d query(cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]);
		Neighbor_search search(tree, query, N);
		double search_max = -9999;
		for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
		{
			ostringstream ostrx, ostrz;
			ostrx << it->first.x();
			ostrz << it->first.y();
			double y = mapstring[ostrx.str() + ostrz.str()];
			if (y > search_max){ search_max = y; }
		}
		heightVal[i] = search_max;
	}

}