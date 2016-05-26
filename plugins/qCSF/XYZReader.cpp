#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <fstream>
using namespace std;

void read_xyz(string fname, PointCloud &pointcloud)
{
	ifstream fin(fname.c_str(), ios::in);
	char line[500];
	string x, y, z;
	while (fin.getline(line, sizeof(line)))
	{
		stringstream words(line);
		words >> x;
		words >> y;
		words >> z;
		LASPoint lasPoint;
		lasPoint.x = atof(x.c_str());
		lasPoint.y = -atof(z.c_str());
		lasPoint.z = atof(y.c_str());
		pointcloud.push_back(lasPoint);
	}

}