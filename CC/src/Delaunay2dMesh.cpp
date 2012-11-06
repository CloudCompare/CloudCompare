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

#include "Delaunay2dMesh.h"

#include "GenericIndexedCloud.h"

//Triangle Lib
#include <triangle.h>

#include <assert.h>

using namespace CCLib;

Delaunay2dMesh::Delaunay2dMesh()
	: theAssociatedCloud(0)
	, theTrianglesIndexes(0)
	, globalIterator(0)
	, globalIteratorEnd(0)
	, numberOfTriangles(0)
	, cloudIsOwnedByMesh(false)
{
}

Delaunay2dMesh::~Delaunay2dMesh()
{
	linkMeshWith(0);

	if (theTrianglesIndexes)
        delete[] theTrianglesIndexes;
}

void Delaunay2dMesh::linkMeshWith(GenericIndexedCloud* aCloud, bool passOwnership)
{
	if (theAssociatedCloud == aCloud)
		return;

	//previous cloud?
	if (theAssociatedCloud && cloudIsOwnedByMesh)
		delete theAssociatedCloud;

	theAssociatedCloud = aCloud;
	cloudIsOwnedByMesh = passOwnership;
}

bool Delaunay2dMesh::build(CC2DPointsConainer &the2dPoints)
{
	if (the2dPoints.empty())
		return false;

	//reset
	numberOfTriangles=0;
	if (theTrianglesIndexes)
	{
		delete[] theTrianglesIndexes;
		theTrianglesIndexes = 0;
	}

	//we use the external library 'Triangle'
	triangulateio in;
	memset(&in,0,sizeof(triangulateio));

	in.numberofpoints = the2dPoints.size();
	in.pointlist = (REAL*)(&the2dPoints[0]);

	try 
	{ 
		triangulate ( "zQN", &in, &in, 0 );
	}
	catch (...) 
	{
		//cerr << "Exception indéfinie." << endl; 
		return false;
	} 
	//Console::print("Nombre de triangles : %i\n",in.numberoftriangles);

	numberOfTriangles = in.numberoftriangles;
	if (numberOfTriangles>0)
		theTrianglesIndexes	= (int*)in.trianglelist;

	int minIndex = theTrianglesIndexes[0];
	int maxIndex = theTrianglesIndexes[0];
	for (unsigned i=1;i<numberOfTriangles;++i)
	{
		if (minIndex > theTrianglesIndexes[i])
			minIndex = theTrianglesIndexes[i];
		else if (maxIndex < theTrianglesIndexes[i])
			maxIndex = theTrianglesIndexes[i];
	}
	globalIteratorEnd = theTrianglesIndexes+3*numberOfTriangles;

	return true;
}

void Delaunay2dMesh::forEach(genericTriangleAction& anAction)
{
	//TODO
	assert(false);
}

void Delaunay2dMesh::placeIteratorAtBegining()
{
	globalIterator = theTrianglesIndexes;
}

GenericTriangle* Delaunay2dMesh::_getNextTriangle()
{
	if (globalIterator>=globalIteratorEnd)
        return 0;

	theAssociatedCloud->getPoint(*globalIterator++,dumpTriangle.A);
	theAssociatedCloud->getPoint(*globalIterator++,dumpTriangle.B);
	theAssociatedCloud->getPoint(*globalIterator++,dumpTriangle.C);

	return &dumpTriangle; //temporary!
}

TriangleSummitsIndexes* Delaunay2dMesh::getNextTriangleIndexes()
{
	if (globalIterator>=globalIteratorEnd)
        return 0;

	dumpTriangleIndexes.i1 = globalIterator[0];
	dumpTriangleIndexes.i2 = globalIterator[1];
	dumpTriangleIndexes.i3 = globalIterator[2];

	globalIterator+=3;

	return &dumpTriangleIndexes;
}

GenericTriangle* Delaunay2dMesh::_getTriangle(unsigned triangleIndex)
{
	assert(theAssociatedCloud && triangleIndex<numberOfTriangles);

	const int* tri = theTrianglesIndexes + 3*triangleIndex;
	theAssociatedCloud->getPoint(*tri++,dumpTriangle.A);
	theAssociatedCloud->getPoint(*tri++,dumpTriangle.B);
	theAssociatedCloud->getPoint(*tri++,dumpTriangle.C);

	return (GenericTriangle*)&dumpTriangle;
}

void Delaunay2dMesh::getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(theAssociatedCloud && triangleIndex<numberOfTriangles);

	const int* tri = theTrianglesIndexes + 3*triangleIndex;
	theAssociatedCloud->getPoint(*tri++,A);
	theAssociatedCloud->getPoint(*tri++,B);
	theAssociatedCloud->getPoint(*tri++,C);
}

TriangleSummitsIndexes* Delaunay2dMesh::getTriangleIndexes(unsigned triangleIndex)
{
	assert(theAssociatedCloud && triangleIndex<numberOfTriangles);

	return (TriangleSummitsIndexes*)(theTrianglesIndexes + 3*triangleIndex);
}

void Delaunay2dMesh::getBoundingBox(PointCoordinateType Mins[], PointCoordinateType Maxs[])
{
	if (theAssociatedCloud)
		theAssociatedCloud->getBoundingBox(Mins,Maxs);
	else
	{
		Mins[0]=Maxs[0]=0.0;
		Mins[1]=Maxs[1]=0.0;
		Mins[2]=Maxs[2]=0.0;
	}
}
