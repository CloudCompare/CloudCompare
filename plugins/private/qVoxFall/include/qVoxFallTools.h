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

#ifndef Q_VOXFALL_TOOLS_HEADER
#define Q_VOXFALL_TOOLS_HEADER

//CCCoreLib
#include <GenericProgressCallback.h>

//local
#include "qVoxFallDialog.h"

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccBox.h>

#include <unordered_map>


class qVoxFallTransform
{
	double az;
	float zRot;

public:
	ccGLMatrix* matrix;
	ccGLMatrix* inverse;
	qVoxFallTransform(double azimuth)
	{
		az = azimuth;
		zRot = GetRotationAngle(azimuth);

		const Vector3Tpl<float> X(std::cos(zRot), std::sin(zRot), 0);
		const Vector3Tpl<float> Y(-std::sin(zRot), std::cos(zRot), 0);
		const Vector3Tpl<float> Z(0, 0, 1);
		const Vector3Tpl<float> Tr(0, 0, 0);
		matrix = &ccGLMatrix::ccGLMatrix(X, Y, Z, Tr);

		const Vector3Tpl<float> rX(std::cos(-zRot), std::sin(-zRot), 0);
		const Vector3Tpl<float> rY(-std::sin(-zRot), std::cos(-zRot), 0);
		const Vector3Tpl<float> rZ(0, 0, 1);
		const Vector3Tpl<float> rTr(0, 0, 0);
		inverse = &ccGLMatrix::ccGLMatrix(rX, rY, rZ, rTr);
	}

	static ccBox* CreateVoxelMesh(CCVector3 V, float voxelSize, int voxelIdx);

private:
	static float GetRotationAngle(double azimuth);

};


class qVoxFallTools
{
public:

	static std::vector<Tuple3i> FindAdjacents(Tuple3i V, CCVector3 steps, bool facetsOnly);

	static int Grid2Index(Tuple3i n, CCVector3 steps);

	static Tuple3i Index2Grid(unsigned index, CCVector3 steps);

};


#endif //Q_VOXFALL_PROCESS_HEADER
