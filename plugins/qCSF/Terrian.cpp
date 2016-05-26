#include "Terrian.h"

Terrian::Terrian(PointCloud &pc)
{
	off_avg_x = 0;
	off_avg_z = 0;
	this->pc = pc;
	bound_box();
}
Terrian::~Terrian(){}

double Terrian::getMin(int direction)
{
	double minval = 100000000000.0f;
	if (direction == 0) //x
	{
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].x < minval)
			{
				minval = pc[i].x;
			}
		}
	}
	else if (direction == 1){ //y
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].y < minval)
			{
				minval = pc[i].y;
			}
		}
	}
	else
	{
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].z < minval)
			{
				minval = pc[i].z;
			}
		}
	}
	return minval;
}

double Terrian::getMax(int direction)
{
	double maxval = -100000000000.0f;
	if (direction == 0) //x
	{
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].x > maxval)
			{
				maxval = pc[i].x;
			}
		}
	}
	else if (direction == 1){ //y
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].y > maxval)
			{
				maxval = pc[i].y;
			}
		}
	}
	else
	{
		for (size_t i = 0; i < pc.size(); i++)
		{
			if (pc[i].z > maxval)
			{
				maxval = pc[i].z;
			}
		}
	}
	return maxval;
}

void Terrian::bound_box()
{
	double xmin = getMin(0);
	double xmax = getMax(0);
	double ymin = getMin(1);
	double ymax = getMax(1);
	double zmin = getMin(2);
	double zmax = getMax(2);
	cube.push_back(xmin);
	cube.push_back(xmax);
	cube.push_back(ymin);
	cube.push_back(ymax);
	cube.push_back(zmin);
	cube.push_back(zmax);
	cube.push_back((xmin + xmax) / 2);
	cube.push_back((zmin + zmax) / 2);
}

void Terrian::saveToFile(LASPoint offset, string path)
{
	string filepath = "terr_nodes.txt";
	if (path == "")
	{
		filepath = "terr_nodes.txt";
	}
	else
	{
		filepath = path;
	}
	ofstream f1(filepath, ios::out);
	if (!f1)return;
	for (size_t i = 0; i < pc.size(); i++)
	{
		f1 << fixed << setprecision(8) << pc[i].x + offset.x << "	" << pc[i].z + offset.z << "	" << -pc[i].y << endl;
	}
	f1.close();
}
