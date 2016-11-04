//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "Delaunay2dMesh.h"

//local
#include "GenericIndexedCloud.h"
#include "ManualSegmentationTools.h"
#include "Polyline.h"
#include "ChunkedPointCloud.h"

#if defined(USE_CGAL_LIB)
//CGAL Lib
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#endif

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

bool Delaunay2dMesh::Available()
{
#if defined(USE_CGAL_LIB)
	return true;
#else
	return false;
#endif
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

bool Delaunay2dMesh::buildMesh(	const std::vector<CCVector2>& points2D,
								const std::vector<int>& segments2D,
								char* outputErrorStr/*=0*/)
{
#if defined(USE_CGAL_LIB)

	//CGAL boilerplate
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	//We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
	typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, K> Vb;
	typedef CGAL::Constrained_triangulation_face_base_2<K> Fb;
	typedef CGAL::No_intersection_tag  Itag; //This tag could ben changed if we decide to handle intersection
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
	typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag> CDT;
	typedef CDT::Point cgalPoint;

	std::vector< std::pair<cgalPoint, size_t > > constraints;
	size_t constrCount = segments2D.size();

	try
	{
		constraints.reserve(constrCount);
	} catch (const std::bad_alloc&)
	{
		if (outputErrorStr)
			strcpy(outputErrorStr, "Not enough memory");
		return false;
	};

	//We create the Constrained Delaunay Triangulation (CDT)
	CDT cdt;

	//We build the constraints
	for(size_t i = 0; i < constrCount; ++i) {
		const CCVector2 * pt = &points2D[segments2D[i]];
		constraints.push_back(std::make_pair(cgalPoint(pt->x, pt->y), segments2D[i]));
	}
	//The CDT  is built according to the constraints
	cdt.insert(constraints.begin(), constraints.end());

	m_numberOfTriangles = static_cast<unsigned >(cdt.number_of_faces());
	m_triIndexes = new int[cdt.number_of_faces()*3];

	//The cgal data structure is converted into CC one
	if (m_numberOfTriangles > 0) {
		int faceCount = 0;
		for (CDT::Face_iterator face = cdt.faces_begin(); face != cdt.faces_end(); ++face, faceCount+=3) {
			m_triIndexes[0+faceCount] = static_cast<int>(face->vertex(0)->info());
			m_triIndexes[1+faceCount] = static_cast<int>(face->vertex(1)->info());
			m_triIndexes[2+faceCount] = static_cast<int>(face->vertex(2)->info());
		};
	}

	m_globalIterator = m_triIndexes;
	m_globalIteratorEnd = m_triIndexes + 3*m_numberOfTriangles;
	return true;

#else

	if (outputErrorStr)
		strcpy(outputErrorStr, "CGAL library not supported");
	return false;

#endif
}

bool Delaunay2dMesh::buildMesh(	const std::vector<CCVector2>& points2D,
								size_t pointCountToUse/*=0*/,
								char* outputErrorStr/*=0*/)
{
#if defined(USE_CGAL_LIB)

	//CGAL boilerplate
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	//We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
	typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, K> Vb;
	typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
	typedef CGAL::Delaunay_triangulation_2<K, Tds> DT;
	typedef DT::Point cgalPoint;

	std::vector< std::pair<cgalPoint, size_t > > pts;
	size_t pointCount = points2D.size();

	//we will use at most 'pointCountToUse' points (if not 0)
	if (pointCountToUse > 0 && pointCountToUse < pointCount)
	{
		pointCount = pointCountToUse;
	}

	if (pointCount < 3)
	{
		if (outputErrorStr)
			strcpy(outputErrorStr, "Not enough points");
		return false;
	}

	try
	{
		pts.reserve(pointCount);
	} catch (const std::bad_alloc&)
	{
		if (outputErrorStr)
			strcpy(outputErrorStr, "Not enough memory");
		return false;
	};

	m_numberOfTriangles = 0;
	if (m_triIndexes)
	{
		delete[] m_triIndexes;
		m_triIndexes = 0;
	}

	for(size_t i = 0; i < pointCount; ++i) {
		const CCVector2 * pt = &points2D[i];
		pts.push_back(std::make_pair(cgalPoint(pt->x, pt->y), i));
	}

	//The delaunay triangulation is built according to the 2D point cloud
	DT dt(pts.begin(), pts.end());

	m_numberOfTriangles = static_cast<unsigned >(dt.number_of_faces());
	m_triIndexes = new int[dt.number_of_faces()*3];

	//The cgal data structure is converted into CC one
	if (m_numberOfTriangles > 0) {
		int faceCount = 0;
		for (DT::Face_iterator face = dt.faces_begin(); face != dt.faces_end(); ++face, faceCount+=3) {
			m_triIndexes[0+faceCount] = static_cast<int>(face->vertex(0)->info());
			m_triIndexes[1+faceCount] = static_cast<int>(face->vertex(1)->info());
			m_triIndexes[2+faceCount] = static_cast<int>(face->vertex(2)->info());
		};
	}

	m_globalIterator = m_triIndexes;
	m_globalIteratorEnd = m_triIndexes + 3*m_numberOfTriangles;
	return true;

#else

	if (outputErrorStr)
		strcpy(outputErrorStr, "CGAL library not supported");
	return false;

#endif
}

bool Delaunay2dMesh::removeOuterTriangles(	const std::vector<CCVector2>& vertices2D,
											const std::vector<CCVector2>& polygon2D,
											bool removeOutside/*=true*/)
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
		for (unsigned i = 0; i < m_numberOfTriangles; ++i, _triIndexes += 3)
		{
			//compute the triangle's barycenter
			const CCVector2& A = vertices2D[_triIndexes[0]];
			const CCVector2& B = vertices2D[_triIndexes[1]];
			const CCVector2& C = vertices2D[_triIndexes[2]];
			CCVector2 G = (A + B + C) / 3.0;

			//if G is inside the 'polygon'
			bool isInside = CCLib::ManualSegmentationTools::isPointInsidePoly(G, polygon2D);
			if ((removeOutside && isInside) || (!removeOutside && !isInside))
			{
				//we keep the corresponding triangle
				if (lastValidIndex != i)
					memcpy(m_triIndexes + 3 * lastValidIndex, _triIndexes, 3 * sizeof(int));
				++lastValidIndex;
			}
		}
	}

	//new number of triangles
	m_numberOfTriangles = lastValidIndex;
	if (m_numberOfTriangles)
	{
		//shouldn't fail as m_numberOfTriangles is smaller!
		m_triIndexes = static_cast<int*>(realloc(m_triIndexes, sizeof(int) * 3 * m_numberOfTriangles));
	}
	else
	{
		//no triangle left!
		delete[] m_triIndexes;
		m_triIndexes = 0;
	}

	//update iterators
	m_globalIterator = m_triIndexes;
	m_globalIteratorEnd = m_triIndexes + 3 * m_numberOfTriangles;

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

void Delaunay2dMesh::forEach(genericTriangleAction& action)
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
		action(tri);
	}
}

void Delaunay2dMesh::placeIteratorAtBegining()
{
	m_globalIterator = m_triIndexes;
}

GenericTriangle* Delaunay2dMesh::_getNextTriangle()
{
	assert(m_associatedCloud);
	if (m_globalIterator >= m_globalIteratorEnd)
        return 0;

	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.A);
	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.B);
	m_associatedCloud->getPoint(*m_globalIterator++,m_dumpTriangle.C);

	return &m_dumpTriangle; //temporary!
}

VerticesIndexes* Delaunay2dMesh::getNextTriangleVertIndexes()
{
	if (m_globalIterator >= m_globalIteratorEnd)
        return 0;

	m_dumpTriangleIndexes.i1 = m_globalIterator[0];
	m_dumpTriangleIndexes.i2 = m_globalIterator[1];
	m_dumpTriangleIndexes.i3 = m_globalIterator[2];

	m_globalIterator += 3;

	return &m_dumpTriangleIndexes;
}

GenericTriangle* Delaunay2dMesh::_getTriangle(unsigned triangleIndex)
{
	assert(m_associatedCloud && triangleIndex < m_numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.A);
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.B);
	m_associatedCloud->getPoint(*tri++,m_dumpTriangle.C);

	return (GenericTriangle*)&m_dumpTriangle;
}

void Delaunay2dMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C)
{
	assert(m_associatedCloud && triangleIndex < m_numberOfTriangles);

	const int* tri = m_triIndexes + 3*triangleIndex;
	m_associatedCloud->getPoint(*tri++,A);
	m_associatedCloud->getPoint(*tri++,B);
	m_associatedCloud->getPoint(*tri++,C);
}

VerticesIndexes* Delaunay2dMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	assert(triangleIndex < m_numberOfTriangles);

	return reinterpret_cast<VerticesIndexes*>(m_triIndexes + 3*triangleIndex);
}

void Delaunay2dMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (m_associatedCloud)
	{
		m_associatedCloud->getBoundingBox(bbMin,bbMax);
	}
	else
	{
		bbMin = bbMax = CCVector3(0,0,0);
	}
}
