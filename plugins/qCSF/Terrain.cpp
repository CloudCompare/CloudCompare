#include "Terrain.h"

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
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <fstream>

Terrain::Terrain(wl::PointCloud &cloud)
	: pc(cloud)
	, off_avg_x(0)
	, off_avg_z(0)
{
	computeBoundingBox();
}

Terrain::~Terrain()
{
}

void Terrain::computeBoundingBox()
{
	if (pc.empty())
	{
		bbMin = bbMax = Vec3(0, 0, 0);
		return;
	}

	bbMin = bbMax = Vec3(pc[0].x, pc[0].y, pc[0].z);
	for (size_t i = 1; i < pc.size(); i++)
	{
		const wl::Point& P = pc[i];
		for (int d = 0; d < 3; ++d)
		{
			if (P.u[d] < bbMin.f[d])
			{
				bbMin.f[d] = P.u[d];
			}
			else if (P.u[d] > bbMax.f[d])
			{
				bbMax.f[d] = P.u[d];
			}
		}
	}
}

void Terrain::saveToFile(const wl::Point& offset, std::string path)
{
	std::string filepath = "terr_nodes.txt";
	if (path != "")
	{
		filepath = path;
	}
	std::ofstream f1(filepath, std::ios::out);
	if (!f1)
		return;
	for (size_t i = 0; i < pc.size(); i++)
	{
		f1 << std::fixed << std::setprecision(8) << pc[i].x + offset.x << "	" << pc[i].z + offset.z << "	" << -pc[i].y << std::endl;
	}
	f1.close();
}
