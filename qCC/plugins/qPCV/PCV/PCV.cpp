//##########################################################################
//#                                                                        #
//#                                PCV                                     #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "PCV.h"
#include "PCVContext.h"

//CCLib
#include <CCMiscTools.h>

using namespace CCLib;

int PCV::Launch(unsigned numberOfRays,
				GenericCloud* vertices,
				GenericMesh* mesh/*=0*/,
				bool meshIsClosed/*=false*/,
				bool mode360/*=true*/,
				unsigned width/*=1024*/,
				unsigned height/*=1024*/,
				CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	//generates light directions
	unsigned lightDirs = numberOfRays*(mode360 ? 1 : 2);
	float *rays = CCMiscTools::sampleSphere(lightDirs);
	if (!rays) //an error occured?
		return -2;

	//we keep only the light directions that meets input parameters (non predictible if not in 360° mode!)
	std::vector<CCVector3> keptRays;
	{
		const float* _ray = rays;
		for (unsigned i=0;i<lightDirs;++i)
		{
			if (mode360 || _ray[2]<=0)
				keptRays.push_back(CCVector3(_ray));
			_ray+=3;
		}
	}

	delete[] rays;
	rays=0;

	if (!Launch(keptRays,vertices, mesh, meshIsClosed, width, height, progressCb))
		return -1;

	return (int)keptRays.size();
}

bool PCV::Launch(std::vector<CCVector3>& rays,
				 CCLib::GenericCloud* vertices,
				 CCLib::GenericMesh* mesh/*=0*/,
				 bool meshIsClosed/*=false*/,
				 unsigned width/*=1024*/,
				 unsigned height/*=1024*/,
				 CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	if (rays.empty())
		return false;

	if (!vertices || !vertices->enableScalarField())
		return false;

	//vertices/points
	unsigned numberOfPoints = vertices->size();
	//rays
	unsigned numberOfRays = rays.size();

	//for each vertex we keep count of the number of light directions for which it is "illuminated"
	int* vertexAccum = new int[numberOfPoints];
	if (!vertexAccum) //not enough memory?
		return false;
	memset(vertexAccum,0,sizeof(int)*numberOfPoints);

	/*** Main illumination loop ***/

	CCLib::NormalizedProgress* nProgress;
	if (progressCb)
	{
		nProgress = new CCLib::NormalizedProgress(progressCb,numberOfRays);
		progressCb->reset();
		progressCb->setMethodTitle("ShadeVis");
		QString infoStr = QString("Rays: %1").arg(numberOfRays);
		if (mesh)
			infoStr.append(QString("\nFaces: %1").arg(mesh->size()));
        else
			infoStr.append(QString("\nVertices: %1").arg(numberOfPoints));
		progressCb->setInfo(qPrintable(infoStr));
		progressCb->start();
	}

	bool success = true;

	//must be done after progress dialog display!
	PCVContext win;
	if (win.init(width,height,vertices,mesh,meshIsClosed))
	{
		for (unsigned i=0;i<numberOfRays;++i)
		{
			//set current 'light' direction
			win.setViewDirection(rays[i].u);

			//flag viewed vertices
			win.GLAccumPixel(vertexAccum);

			if (nProgress && !nProgress->oneStep())
			{
				success = false;
				break;
			}
		}

		if (success)
		{
			//we convert per-vertex accumulators to an 'intensity' scalar field
			for (unsigned j=0;j<numberOfPoints;++j)
				vertices->setPointScalarValue(j,(DistanceType)vertexAccum[j]/(DistanceType)numberOfRays);
		}
	}
	else
	{
		success = false;
	}

	if (nProgress)
		delete nProgress;
	nProgress=0;

	if (vertexAccum)
		delete[] vertexAccum;
	vertexAccum=0;

	return success;
}
