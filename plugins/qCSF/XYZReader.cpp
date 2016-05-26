#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <fstream>

void read_xyz(std::string fname, wl::PointCloud &pointcloud)
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
		wl::LASPoint lasPoint;
		lasPoint.x = static_cast<float>( atof(x.c_str()));
		lasPoint.y = static_cast<float>(-atof(z.c_str()));
		lasPoint.z = static_cast<float>( atof(y.c_str()));
		pointcloud.push_back(lasPoint);
	}

}