/* License Information
 *
 *  Copyright (C) ONERA, The French Aerospace Lab
 *  Author: Alexandre BOULCH
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this
 *  software and associated documentation files (the "Software"), to deal in the Software
 *  without restriction, including without limitation the rights to use, copy, modify, merge,
 *  publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
 *  to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or
 *  substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 *  OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *  Note that this library relies on external libraries subject to their own license.
 *  To use this software, you are subject to the dependencies license, these licenses
 *  applies to the dependency ONLY  and NOT this code.
 *  Please refer below to the web sites for license informations:
 *       PCL, BOOST,NANOFLANN, EIGEN, LUA TORCH
 *
 * When using the software please aknowledge the  corresponding publication:
 * "Deep Learning for Robust Normal Estimation in Unstructured Point Clouds "
 * by Alexandre Boulch and Renaud Marlet
 * Symposium of Geometry Processing 2016, Computer Graphics Forum
 */

#include "pc_io.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;


void pc_load(const std::string& filename,Eigen::MatrixX3d& pc){
	ifstream istr(filename.c_str());
	vector<Eigen::Vector3d> points;
    string line;
	double x,y,z;
    while(getline(istr, line))
    {
		stringstream sstr("");
		sstr << line;
		sstr >> x >> y >> z;
		points.push_back(Eigen::Vector3d(x,y,z));
    }
	istr.close();
	pc.resize(points.size(),3);
	for(int i=0; i<points.size(); i++){
		pc.row(i) = points[i];
	}
}

void pc_save(const std::string& filename,const Eigen::MatrixX3d& pc, const Eigen::MatrixX3d& normals){
	ofstream ofs(filename.c_str());
	for(int i=0; i<pc.rows(); i++){
		ofs << pc(i,0) << " ";
		ofs << pc(i,1) << " ";
		ofs << pc(i,2) << " ";
		ofs << normals(i,0) << " ";
		ofs << normals(i,1) << " ";
		ofs << normals(i,2) << endl;
	}
	ofs.close();
}
