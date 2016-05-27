#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <fstream>

bool read_xyz(std::string fname, wl::PointCloud &pointcloud)
{
	try
	{
		std::ifstream fin(fname.c_str(), std::ios::in);
		
		char line[500];
		std::string x, y, z;
		while (fin.getline(line, sizeof(line)))
		{
			std::stringstream words(line);
			words >> x;
			words >> y;
			words >> z;
			wl::Point P;
			P.x = static_cast<float>(atof(x.c_str()));
			P.y = static_cast<float>(-atof(z.c_str()));
			P.z = static_cast<float>(atof(y.c_str()));
			pointcloud.push_back(P);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	catch (...)
	{
		//other error
		return false;
	}

	return true;
}