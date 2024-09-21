//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################


#include "qVoxFallTools.h"

//qCC_db
#include <ccPointCloud.h>

//qCC
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//local
#include "qVoxFallDialog.h"

//Qt
#include <QtCore>
#include <QApplication>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtConcurrentMap>



float qVoxFallTransform::GetRotationAngle(double azimuth)
{
	double azimuthRadians = azimuth * 3.14159 / 180;

	std::vector<double> direction = { sin(azimuthRadians), cos(azimuthRadians) };
	std::vector<double> xyView = { 0, 1 };

	// compute dot product of unit vectors
	double dotProduct = 0;
	for (int i = 0; i < direction.size(); i++)
	{
		direction[i] = direction[i] / sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
		xyView[i] = xyView[i] / sqrt(xyView[0] * xyView[0] + xyView[1] * xyView[1]);
		dotProduct += direction[i] * xyView[i];
	}

	float zRot = acos(std::max(-1.0, std::min(dotProduct, 1.0)));

	return zRot;
}


ccBox* qVoxFallTransform::CreateVoxelMesh(CCVector3 V, float voxelSize, int voxelIdx)
{
	CCVector3 dims = { voxelSize, voxelSize, voxelSize };
	QString name = QString("voxel#%1").arg(voxelIdx);

	const Vector3Tpl<float> X(1, 0, 0);
	const Vector3Tpl<float> Y(0, 1, 0);
	const Vector3Tpl<float> Z(0, 0, 1);
	ccGLMatrix* matrix = &ccGLMatrix::ccGLMatrix(X, Y, Z, V);

	ccBox* voxel = new ccBox(dims, matrix, name);
	voxel->computePerTriangleNormals();
	return voxel;
}



std::vector<Tuple3i> qVoxFallTools::FindAdjacents(Tuple3i V, CCVector3 steps, bool facetsOnly=false)
{
	std::vector<Tuple3i> set;
	std::vector<std::vector<int>> adjacencyMatrix;

	if (!facetsOnly)
	{
		adjacencyMatrix = {
			{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
			{0, -1, 0}, {0, 0, 1}, {0, 0, -1},
			{1, 1, 0},  {-1, 1, 0}, {1, -1, 0},
			{-1, -1, 0}, {0, 1, 1}, {0, 1, -1},
			{0, -1, 1}, {0, -1, -1}, {1, 0, 1},
			{1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
			{1, 1, 1}, {-1, -1, -1}, {1, 1, -1},
			{1, -1, 1}, {-1, 1, 1}, {1, -1, -1},
			{-1, -1, 1}, {-1, 1, -1}
		};
	}
	else
	{
		adjacencyMatrix = {
				{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
				{0, -1, 0}, {0, 0, 1},  {0, 0, -1},
		};
	}

	for (unsigned n = 0; n < adjacencyMatrix.size(); n++)
	{

		int x = int(V.x) + adjacencyMatrix[n][0];
		int y = int(V.y) + adjacencyMatrix[n][1];
		int z = int(V.z) + adjacencyMatrix[n][2];

		if (x < 0 || y < 0 || z < 0 || x >= int(steps.x) || y >= int(steps.y) || z >= int(steps.z))
		{
			continue;
		}
							
		set.push_back({ x, y, z });
	}
	
	return set;
}


int qVoxFallTools::Grid2Index(Tuple3i n, CCVector3 steps)
{
	int i = n.x;
	int j = n.y;
	int k = n.z;

	int index = (i)+(j * int(steps.x)) + (k * int(steps.x) * int(steps.y));
	return index;
}


Tuple3i qVoxFallTools::Index2Grid(unsigned index, CCVector3 steps)
{
	int k = std::floor(index / (int(steps.y) * int(steps.x)));
	int remain = index - (int(steps.y) * int(steps.x) * k);
	int j = std::floor(remain / int(steps.x));
	int i = remain - (int(steps.x) * j);

	Tuple3i V(	static_cast<int>(i),
				static_cast<int>(j),
				static_cast<int>(k)	);
	return V;
}



