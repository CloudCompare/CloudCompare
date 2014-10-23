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
#include "ManualSegmentationTools.h"

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
	, m_globalIteratorEnd(0)
	, m_numberOfTriangles(0)
	, m_cloudIsOwnedByMesh(false)
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
	if (m_associatedCloud && m_cloudIsOwnedByMesh)
		delete m_associatedCloud;

	m_associatedCloud = aCloud;
	m_cloudIsOwnedByMesh = passOwnership;
}

bool Delaunay2dMesh::buildMesh(	const std::vector<CCVector2>& the2dPoints,
								size_t pointCountToUse/*=0*/,
								char* outputErrorStr/*=0*/)
{
	size_t pointCount = the2dPoints.size();
	//we will use at most 'pointCountToUse' points (if not 0)
	if (pointCountToUse > 0 && pointCountToUse < pointCount)
		pointCount = pointCountToUse;
	if (pointCount < 3)
		return false;

	//reset
	m_numberOfTriangles = 0;
	if (m_triIndexes)
	{
		delete[] m_triIndexes;
		m_triIndexes = 0;
	}

	//we use the external library 'Triangle'
	triangulateio in;
	memset(&in,0,sizeof(triangulateio));

	in.numberofpoints = static_cast<int>(pointCount);
	in.pointlist = (REAL*)(&the2dPoints[0]);

	//set static variable for 'triunsuitable' (for Triangle lib with '-u' option)
	//s_maxSquareEdgeLength = maxEdgeLength*maxEdgeLength;
	//DGM: doesn't work this way --> triangle lib will simply add new points!
	try 
	{ 
		triangulate ( "zQN", &in, &in, 0 );
	}
	catch (std::exception& e) 
	{
		if (outputErrorStr)
			strcpy(outputErrorStr,e.what());
		return false;
	} 
	catch (...) 
	{
		if (outputErrorStr)
			strcpy(outputErrorStr,"Unknown error");
		return false;
	} 

	m_numberOfTriangles = in.numberoftriangles;
	if (m_numberOfTriangles > 0)
		m_triIndexes = in.trianglelist;

	m_globalIterator = m_triIndexes;
	m_globalIteratorEnd = m_triIndexes + 3*m_numberOfTriangles;
	

	return true;
}

bool Delaunay2dMesh::removeOuterTriangles(	const std::vector<CCVector2>& vertices2D,
											const std::vector<CCVector2>& polygon2D)
{
	if (!m_triIndexes || m_numberOfTriangles == 0)
		return false;

	//we expect the same number of 2D points as the actual number of points in the associated mesh (if any)
	if (m_associatedCloud && static_cast<size_t>(m_associatedCloud->size()) != vertices2D.size())
		return false;

	unsigned lastValidIndex = 0;

	//test each triangle center
	{
		const int* _triIndexes = m_triIndexes;
		for (unsigned i=0; i<m_numberOfTriangles; ++i,_triIndexes+=3)
		{
			//compute the triangle's barycenter
			const CCVector2& A = vertices2D[_triIndexes[0]];
			const CCVector2& B = vertices2D[_triIndexes[1]];
			const CCVector2& C = vertices2D[_triIndexes[2]];
			CCVector2 G = (A + B + C) / 3.0;

			//if G is inside the 'polygon'
			if (CCLib::ManualSegmentationTools::isPointInsidePoly(G,polygon2D))
			{
				//we keep the corresponding triangle
				if (lastValidIndex != i)
					memcpy(m_triIndexes+3*lastValidIndex,_triIndexes,3*sizeof(int));
				++lastValidIndex;
			}
		}
	}

	//new number of triangles
	m_numberOfTriangles = lastValidIndex;
	if (m_numberOfTriangles)
	{
		//shouldn't fail as m_numberOfTriangles is smaller!
		m_triIndexes = static_cast<int*>(realloc(m_triIndexes,sizeof(int)*3*m_numberOfTriangles));
	}
	else
	{
		//no triangle left!
		delete[] m_triIndexes;
		m_triIndexes = 0;
	}

	//update iterators
	m_globalIterator = m_triIndexes;
	m_globalIteratorEnd = m_triIndexes + 3*m_numberOfTriangles;

	return true;
}


bool Delaunay2dMesh::removeTrianglesWithEdgesLongerThan(PointCoordinateType maxEdgeLength)
{
	if (!m_associatedCloud || maxEdgeLength <= 0)
		return false;

	PointCoordinateType squareMaxEdgeLength = maxEdgeLength*maxEdgeLength;

	unsigned lastValidIndex = 0;
	const int* _triIndexes = m_triIndexes;
	for (unsigned i=0; i<m_numberOfTriangles; ++i, _triIndexes+=3)
	{
		const CCVector3* A = m_associatedCloud->getPoint(_triIndexes[0]);
		const CCVector3* B = m_associatedCloud->getPoint(_triIndexes[1]);
		const CCVector3* C = m_associatedCloud->getPoint(_triIndexes[2]);

		if ((*B-*A).norm2() <= squareMaxEdgeLength &&
			(*C-*A).norm2() <= squareMaxEdgeLength &&
			(*C-*B).norm2() <= squareMaxEdgeLength)
		{
			if (lastValidIndex != i)
				memcpy(m_triIndexes+3*lastValidIndex, _triIndexes, sizeof(int)*3);
			++lastValidIndex;
		}
	}

	if (lastValidIndex < m_numberOfTriangles)
	{
		m_numberOfTriangles = lastValidIndex;
		if (m_numberOfTriangles != 0)
		{
			//shouldn't fail as m_numberOfTriangles is smaller than before!
			m_triIndexes = static_cast<int*>(realloc(m_triIndexes,sizeof(int)*3*m_numberOfTriangles));
		}
		else //no more triangles?!
		{
			delete m_triIndexes;
			m_triIndexes = 0;
		}
		m_globalIterator = m_triIndexes;
		m_globalIteratorEnd = m_triIndexes + 3*m_numberOfTriangles;
	}

	return true;
}

void Delaunay2dMesh::forEach(genericTriangleAction& anAction)
{
	if (!m_associatedCloud)
		return;

	CCLib::SimpleTriangle tri;

	const int* _triIndexes = m_triIndexes;
	for (unsigned i=0; i<m_numberOfTriangles; ++i, _triIndexes+=3)
	{
		tri.A = *m_associatedCloud->getPoint(_triIndexes[0]);
		tri.B = *m_associatedCloud->getPoint(_triIndexes[1]);
		tri.C = *m_associatedCloud->getPoint(_triIndexes[2]);
		anAction(tri);
	}
}

void Delaunay2dMesh::placeIteratorAtBegining()
{
	m_globalIterator = m_triIndexes;
}

GenericTriangle* Delaunay2dMesh::_getNextTriangle()
{
	assert(m_associatedCloud);
	if (m_globalIterator>=m_globalIteratorEnd)
        return 0;

	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.A);
	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.B);
	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.C);

	return &m_dumpTriangle; //temporary!
}

TriangleSummitsIndexes* Delaunay2dMesh::getNextTriangleIndexes()
{
	if (m_globalIterator>=m_globalIteratorEnd)
        return 0;

	m_dumpTriangleIndexes.i1 = m_globalIterator[0];
	m_dumpTriangleIndexes.i2 = m_globalIterator[1];
	m_dumpTriangleIndexes.i3 = m_globalIterator[2];

	m_globalIterator+=3;

	return &m_dumpTriangleIndexes;
}

GenericTriangle* Delaunay2dMesh::_getTriangle(unsigned triangleIndex)
{
	assert(m_associatedCloud && triangleIndex<m_numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.A);
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.B);
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.C);

	return (GenericTriangle*)&m_dumpTriangle;
}

void Delaunay2dMesh::getTriangleSummits(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(m_associatedCloud && triangleIndex<m_numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,A);
	m_associatedCloud->getPoint(*tri++,B);
	m_associatedCloud->getPoint(*tri++,C);
}

TriangleSummitsIndexes* Delaunay2dMesh::getTriangleIndexes(unsigned triangleIndex)
{
	assert(triangleIndex < m_numberOfTriangles);

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
