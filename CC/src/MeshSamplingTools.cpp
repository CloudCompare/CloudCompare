//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
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

#include "MeshSamplingTools.h"

#include "GenericProgressCallback.h"
#include "GenericIndexedCloud.h"
#include "GenericMesh.h"
#include "SimpleCloud.h"
#include "CCConst.h"
#include "CCGeom.h"

#include <assert.h>

using namespace CCLib;

double MeshSamplingTools::computeMeshArea(GenericMesh* theMesh)
{
	assert(theMesh);

	//surface totale
	double Stotal = 0.0;

	GenericTriangle* tri = 0;
	CCVector3 O,A,B;

	unsigned n=0;
	theMesh->placeIteratorAtBegining();
	for (;n<theMesh->size();++n)
	{
		tri = theMesh->_getNextTriangle();

		//sommets
		O = *(tri->_getA());
		A = *(tri->_getB());
		B = *(tri->_getC());

		//on calcule la hauteur du triangle par produit vectoriel
		A -= O;
		B -= O;
		O = A.cross(B);
		Stotal += O.norm();
	}

	return Stotal*0.5;
}

SimpleCloud* MeshSamplingTools::samplePointsOnMesh(GenericMesh* theMesh,
													unsigned numberOfPoints,
													GenericProgressCallback* progressCb/*=0*/,
													GenericChunkedArray<1,unsigned>* triIndices/*=0*/)
{
	if (!theMesh)
        return 0;

	//total mesh surface
	double Stotal = computeMeshArea(theMesh);

	if (Stotal < ZERO_TOLERANCE)
        return 0;

	double samplingDensity = double(numberOfPoints)/Stotal;

    //no normal needs to be computed here
	return samplePointsOnMesh(theMesh,samplingDensity,numberOfPoints,progressCb,triIndices);
}

SimpleCloud* MeshSamplingTools::samplePointsOnMesh(GenericMesh* theMesh,
													double samplingDensity,
													GenericProgressCallback* progressCb/*=0*/,
													GenericChunkedArray<1,unsigned>* triIndices/*=0*/)
{
	if (!theMesh)
        return 0;

	//on commence par calculer la surface totale
	double Stotal = computeMeshArea(theMesh);

	unsigned theoricNumberOfPoints = unsigned(Stotal * samplingDensity);

	return samplePointsOnMesh(theMesh,samplingDensity,theoricNumberOfPoints,progressCb,triIndices);
}

SimpleCloud* MeshSamplingTools::samplePointsOnMesh(GenericMesh* theMesh,
													double samplingDensity,
													unsigned theoricNumberOfPoints,
													GenericProgressCallback* progressCb,
													GenericChunkedArray<1,unsigned>* triIndices/*=0*/)
{
	assert(theMesh);
	unsigned triCount = (theMesh ? theMesh->size() : 0);
	if (triCount==0)
		return 0;

	if (theoricNumberOfPoints < 1)
        return 0;


	SimpleCloud* sampledCloud = new SimpleCloud();
	if (!sampledCloud->reserve(theoricNumberOfPoints)) //not enough memory
	{
		delete sampledCloud;
		return 0;
	}

	if (triIndices)
	{
	    triIndices->clear();
		//not enough memory? DGM TODO: we should warn the caller
		if (!triIndices->reserve(theoricNumberOfPoints) || triIndices->capacity() < theoricNumberOfPoints)
		{
			delete sampledCloud;
			triIndices->clear();
			return 0;
		}
	}

	NormalizedProgress* normProgress=0;
    if(progressCb)
    {
		normProgress = new NormalizedProgress(progressCb,triCount);
		progressCb->setMethodTitle("Mesh sampling");
		char buffer[256];
		sprintf(buffer,"Triangles: %i\nPoints: %i",triCount,theoricNumberOfPoints);
		progressCb->setInfo(buffer);
        progressCb->reset();
		progressCb->start();
	}

	unsigned addedPoints=0;

	//for each triangle
	theMesh->placeIteratorAtBegining();
	for (unsigned n=0;n<triCount;++n)
	{
		const GenericTriangle* tri = theMesh->_getNextTriangle();

		//summits (OAB)
		const CCVector3 *O = tri->_getA();
		const CCVector3 *A = tri->_getB();
		const CCVector3 *B = tri->_getC();

		//edges (OA and OB)
		CCVector3 u = *A - *O;
		CCVector3 v = *B - *O;

		//we compute the (twice) the triangle area
		CCVector3 N = u.cross(v);
		double S = N.norm()/2.0;

		//we deduce the number of points to generate on this face
		double fPointsToAdd = S*samplingDensity;
		unsigned pointsToAdd = (unsigned)fPointsToAdd;

        //if the face area is smaller than the surface/random point
		if (pointsToAdd==0)
		{
			//we add a point with the same probability as its (relative) area
			if (double(rand())/double(RAND_MAX) <= fPointsToAdd)
                ++pointsToAdd;
		}

		if (pointsToAdd)
		{
			if (addedPoints + pointsToAdd >= theoricNumberOfPoints)
			{
				theoricNumberOfPoints+=pointsToAdd;
				if (!sampledCloud->reserve(theoricNumberOfPoints)
					|| (triIndices && triIndices->capacity() < theoricNumberOfPoints && !triIndices->reserve(theoricNumberOfPoints))) //not enough memory
				{
					delete sampledCloud;
					sampledCloud=0;
					triIndices->clear();
					break;
				}
			}

			for (unsigned i=0;i<pointsToAdd;++i)
			{
				//we generates random points as in:
				//'Greg Turk. Generating random points in triangles. In A. S. Glassner, editor,Graphics Gems, pages 24-28. Academic Press, 1990.'
				double x = double(rand())/(double)RAND_MAX;
				double y = double(rand())/(double)RAND_MAX;

				//we test if the generated point lies on the right side of (AB)
				if (x+y>1.0)
				{
                    x=1.0-x;
                    y=1.0-y;
                }

				CCVector3 P = (*O) + (PointCoordinateType)x * u + (PointCoordinateType)y * v;

				sampledCloud->addPoint(P);
				if (triIndices)
					triIndices->addElement(n);
				++addedPoints;
			}
		}

		if (normProgress && !normProgress->oneStep())
			break;
	}

	if (normProgress)
	{
        delete normProgress;
		normProgress=0;
	}

	if (sampledCloud) //can be in case of memory overflow!
	{
		if (addedPoints)
		{
			sampledCloud->resize(addedPoints); //should always be ok as addedPoints<theoricNumberOfPoints
			if (triIndices)
				triIndices->resize(addedPoints);
		}
		else
		{
			delete sampledCloud;
			sampledCloud=0;
			if (triIndices)
				triIndices->clear();
		}
	}

	return sampledCloud;
}
