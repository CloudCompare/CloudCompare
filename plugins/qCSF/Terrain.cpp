#include "Terrain.h"

//system
#include <assert.h>

Terrain::Terrain(wl::PointCloud &cloud)
	: pc(cloud)
	, off_avg_x(0)
	, off_avg_z(0)
{
	bound_box();
}

Terrain::~Terrain()
{
}

double Terrain::getMin(int direction)
{
	assert(direction >= 0 && direction < 3);

	if (pc.empty())
	{
		return 0.0;
	}

	double minval = pc[0].u[direction];
	for (size_t i = 1; i < pc.size(); i++)
	{
		if (pc[i].u[direction] < minval)
		{
			minval = pc[i].u[direction];
		}
	}
	return minval;
}

double Terrain::getMax(int direction)
{
	assert(direction >= 0 && direction < 3);

	if (pc.empty())
	{
		return 0.0;
	}

	double maxval = pc[0].u[direction];
	for (size_t i = 1; i < pc.size(); i++)
	{
		if (pc[i].u[direction] > maxval)
		{
			maxval = pc[i].u[direction];
		}
	}
	return maxval;
}

void Terrain::bound_box()
{
	cube[0] = getMin(0);
	cube[1] = getMax(0);
	cube[2] = getMin(1);
	cube[3] = getMax(1);
	cube[4] = getMin(2);
	cube[5] = getMax(2);
	cube[6] = (cube[0] + cube[1]) / 2;
	cube[7] = (cube[4] + cube[5]) / 2;
}

void Terrain::saveToFile(wl::LASPoint offset, std::string path)
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
