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

#include "Delaunay2dMesh.h"

//local
#include "GenericIndexedCloud.h"

//Triangle Lib
#include <triangle.h>

//system
#include <assert.h>
#include <string.h>

using namespace CCLib;

Delaunay2dMesh::Delaunay2dMesh()
	: m_associatedCloud(0)
	, m_triIndexes(0)
	, m_globalIterator(0)
	, globalIteratorEnd(0)
	, numberOfTriangles(0)
	, cloudIsOwnedByMesh(false)
{
}

Delaunay2dMesh::~Delaunay2dMesh()
{
	linkMeshWith(0);

	if (m_triIndexes)
        delete[] m_triIndexes;
}

void Delaunay2dMesh::linkMeshWith(GenericIndexedCloud* aCloud, bool passOwnership)
{
	if (m_associatedCloud == aCloud)
		return;

	//previous cloud?
	if (m_associatedCloud && cloudIsOwnedByMesh)
		delete m_associatedCloud;

	m_associatedCloud = aCloud;
	cloudIsOwnedByMesh = passOwnership;
}

bool Delaunay2dMesh::build(CC2DPointsConainer &the2dPoints)
{
	if (the2dPoints.empty())
		return false;

	//reset
	numberOfTriangles=0;
	if (m_triIndexes)
	{
		delete[] m_triIndexes;
		m_triIndexes = 0;
	}

	//we use the external library 'Triangle'
	triangulateio in;
	memset(&in,0,sizeof(triangulateio));

	in.numberofpoints = (int)the2dPoints.size();
	in.pointlist = (REAL*)(&the2dPoints[0]);

	try 
	{ 
		triangulate ( "zQN", &in, &in, 0 );
	}
	catch (...) 
	{
		return false;
	} 

	numberOfTriangles = in.numberoftriangles;
	if (numberOfTriangles>0)
		m_triIndexes = (int*)in.trianglelist;

	int minIndex = m_triIndexes[0];
	int maxIndex = m_triIndexes[0];
	for (unsigned i=1;i<numberOfTriangles;++i)
	{
		if (minIndex > m_triIndexes[i])
			minIndex = m_triIndexes[i];
		else if (maxIndex < m_triIndexes[i])
			maxIndex = m_triIndexes[i];
	}
	globalIteratorEnd = m_triIndexes+3*numberOfTriangles;

	return true;
}

void Delaunay2dMesh::forEach(genericTriangleAction& anAction)
{
	//TODO
	assert(false);
}

void Delaunay2dMesh::placeIteratorAtBegining()
{
	m_globalIterator = m_triIndexes;
}

GenericTriangle* Delaunay2dMesh::_getNextTriangle()
{
	if (m_globalIterator>=globalIteratorEnd)
        return 0;

	m_associatedCloud->getPoint(*m_globalIterator++,dumpTriangle.A);
	m_associatedCloud->getPoint(*m_globalIterator++,dumpTriangle.B);
	m_associatedCloud->getPoint(*m_globalIterator++,dumpTriangle.C);

	return &dumpTriangle; //temporary!
}

TriangleSummitsIndexes* Delaunay2dMesh::getNextTriangleIndexes()
{
	if (m_globalIterator>=globalIteratorEnd)
        return 0;

	dumpTriangleIndexes.i1 = m_globalIterator[0];
	dumpTriangleIndexes.i2 = m_globalIterator[1];
	dumpTriangleIndexes.i3 = m_globalIterator[2];

	m_globalIterator+=3;

	return &dumpTriangleIndexes;
}

GenericTriangle* Delaunay2dMesh::_getTriangle(unsigned triangleIndex)
{
	assert(m_associatedCloud && triangleIndex<numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,dumpTriangle.A);
	m_associatedCloud->getPoint(*tri++,dumpTriangle.B);
	m_associatedCloud->getPoint(*tri++,dumpTriangle.C);

	return (GenericTriangle*)&dumpTriangle;
}

void Delaunay2dMesh::getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(m_associatedCloud && triangleIndex<numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,A);
	m_associatedCloud->getPoint(*tri++,B);
	m_associatedCloud->getPoint(*tri++,C);
}

TriangleSummitsIndexes* Delaunay2dMesh::getTriangleIndexes(unsigned triangleIndex)
{
	assert(m_associatedCloud && triangleIndex<numberOfTriangles);

	return (TriangleSummitsIndexes*)(m_triIndexes + 3*triangleIndex);
}

void Delaunay2dMesh::getBoundingBox(PointCoordinateType bbMin[], PointCoordinateType bbMax[])
{
	if (m_associatedCloud)
		m_associatedCloud->getBoundingBox(bbMin,bbMax);
	else
	{
		bbMin[0] = bbMax[0] = 0;
		bbMin[1] = bbMax[1] = 0;
		bbMin[2] = bbMax[2] = 0;
	}
}
