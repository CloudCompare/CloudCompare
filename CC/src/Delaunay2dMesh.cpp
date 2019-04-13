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

#include <Delaunay2dMesh.h>

//local
#include <ManualSegmentationTools.h>
#include <PointCloud.h>
#include <Polyline.h>

#if defined(USE_CGAL_LIB)
//CGAL Lib
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#endif


using namespace CCLib;

Delaunay2dMesh::Delaunay2dMesh()
	: m_associatedCloud(nullptr)
	, m_triIndexes(nullptr)
	, m_globalIterator(nullptr)
	, m_globalIteratorEnd(nullptr)
	, m_numberOfTriangles(0)
	, m_cloudIsOwnedByMesh(false)
{
}

Delaunay2dMesh::~Delaunay2dMesh()
{
	linkMeshWith(nullptr);

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
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	//We define a vertex_base with info. The "info" (std::size_t) allow us to keep track of the original point index.
	using Vb = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, K>;
	using Fb = CGAL::Constrained_triangulation_face_base_2<K>;
	using Itag = CGAL::No_intersection_tag; //This tag could ben changed if we decide to handle intersection
	using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
	using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, Tds, Itag>;
	using cgalPoint = CDT::Point;

	std::vector< std::pair<cgalPoint, std::size_t > > constraints;
	std::size_t constrCount = segments2D.size();

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
	for(std::size_t i = 0; i < constrCount; ++i)
	{
		const CCVector2 * pt = &points2D[segments2D[i]];
		constraints.emplace_back(cgalPoint(pt->x, pt->y), segments2D[i]);
	}
	//The CDT  is built according to the constraints
	cdt.insert(constraints.begin(), constraints.end());

	m_numberOfTriangles = static_cast<unsigned>(cdt.number_of_faces());
	m_triIndexes = new int[cdt.number_of_faces() * 3];

	//The cgal data structure is converted into CC one
	if (m_numberOfTriangles > 0) {
		int faceCount = 0;
		for (CDT::Face_iterator face = cdt.faces_begin(); face != cdt.faces_end(); ++face, faceCount += 3)
		{
			m_triIndexes[0 + faceCount] = static_cast<int>(face->vertex(0)->info());
			m_triIndexes[1 + faceCount] = static_cast<int>(face->vertex(1)->info());
			m_triIndexes[2 + faceCount] = static_cast<int>(face->vertex(2)->info());
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
								std::size_t pointCountToUse/*=0*/,
								char* outputErrorStr/*=0*/)
{
#if defined(USE_CGAL_LIB)

	//CGAL boilerplate
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	//We define a vertex_base with info. The "info" (std::size_t) allow us to keep track of the original point index.
	using Vb = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, K>;
	using Tds = CGAL::Triangulation_data_structure_2<Vb>;
	using DT = CGAL::Delaunay_triangulation_2<K, Tds>;
	using cgalPoint = DT::Point;

	std::vector< std::pair<cgalPoint, std::size_t > > pts;
	std::size_t pointCount = points2D.size();

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
	}
	catch (const std::bad_alloc&)
	{
		if (outputErrorStr)
			strcpy(outputErrorStr, "Not enough memory");
		return false;
	};

	m_numberOfTriangles = 0;
	if (m_triIndexes)
	{
		delete[] m_triIndexes;
		m_triIndexes = nullptr;
	}

	for (std::size_t i = 0; i < pointCount; ++i)
	{
		const CCVector2 * pt = &points2D[i];
		pts.emplace_back(cgalPoint(pt->x, pt->y), i);
	}

	//The delaunay triangulation is built according to the 2D point cloud
	DT dt(pts.begin(), pts.end());

	m_numberOfTriangles = static_cast<unsigned >(dt.number_of_faces());
	m_triIndexes = new int[dt.number_of_faces()*3];

	//The cgal data structure is converted into CC one
	if (m_numberOfTriangles > 0)
	{
		int faceCount = 0;
		for (DT::Face_iterator face = dt.faces_begin(); face != dt.faces_end(); ++face, faceCount += 3)
		{
			m_triIndexes[0 + faceCount] = static_cast<int>(face->vertex(0)->info());
			m_triIndexes[1 + faceCount] = static_cast<int>(face->vertex(1)->info());
			m_triIndexes[2 + faceCount] = static_cast<int>(face->vertex(2)->info());
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
	if (m_associatedCloud && static_cast<std::size_t>(m_associatedCloud->size()) != vertices2D.size())
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
		m_triIndexes = nullptr;
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

	PointCoordinateType squareMaxEdgeLength = maxEdgeLength * maxEdgeLength;

	unsigned lastValidIndex = 0;
	const int* _triIndexes = m_triIndexes;
	for (unsigned i = 0; i < m_numberOfTriangles; ++i, _triIndexes += 3)
	{
		const CCVector3* A = m_associatedCloud->getPoint(_triIndexes[0]);
		const CCVector3* B = m_associatedCloud->getPoint(_triIndexes[1]);
		const CCVector3* C = m_associatedCloud->getPoint(_triIndexes[2]);

		if ((*B - *A).norm2() <= squareMaxEdgeLength &&
			(*C - *A).norm2() <= squareMaxEdgeLength &&
			(*C - *B).norm2() <= squareMaxEdgeLength)
		{
			if (lastValidIndex != i)
				memcpy(m_triIndexes + 3 * lastValidIndex, _triIndexes, sizeof(int) * 3);
			++lastValidIndex;
		}
	}

	if (lastValidIndex < m_numberOfTriangles)
	{
		m_numberOfTriangles = lastValidIndex;
		if (m_numberOfTriangles != 0)
		{
			//shouldn't fail as m_numberOfTriangles is smaller than before!
			m_triIndexes = static_cast<int*>(realloc(m_triIndexes, sizeof(int) * 3 * m_numberOfTriangles));
		}
		else //no more triangles?!
		{
			delete m_triIndexes;
			m_triIndexes = nullptr;
		}
		m_globalIterator = m_triIndexes;
		m_globalIteratorEnd = m_triIndexes + 3 * m_numberOfTriangles;
	}

	return true;
}

void Delaunay2dMesh::forEach(genericTriangleAction action)
{
	if (!m_associatedCloud)
		return;

	CCLib::SimpleTriangle tri;

	const int* _triIndexes = m_triIndexes;
	for (unsigned i = 0; i < m_numberOfTriangles; ++i, _triIndexes += 3)
	{
		tri.A = *m_associatedCloud->getPoint(_triIndexes[0]);
		tri.B = *m_associatedCloud->getPoint(_triIndexes[1]);
		tri.C = *m_associatedCloud->getPoint(_triIndexes[2]);
		action(tri);
	}
}

void Delaunay2dMesh::placeIteratorAtBeginning()
{
	m_globalIterator = m_triIndexes;
}

GenericTriangle* Delaunay2dMesh::_getNextTriangle()
{
	assert(m_associatedCloud);
	if (m_globalIterator >= m_globalIteratorEnd)
		return nullptr;

	m_associatedCloud->getPoint(*m_globalIterator++, m_dumpTriangle.A);
	m_associatedCloud->getPoint(*m_globalIterator++, m_dumpTriangle.B);
	m_associatedCloud->getPoint(*m_globalIterator++, m_dumpTriangle.C);

	return &m_dumpTriangle; //temporary!
}

VerticesIndexes* Delaunay2dMesh::getNextTriangleVertIndexes()
{
	if (m_globalIterator >= m_globalIteratorEnd)
        return nullptr;

	m_dumpTriangleIndexes.i1 = m_globalIterator[0];
	m_dumpTriangleIndexes.i2 = m_globalIterator[1];
	m_dumpTriangleIndexes.i3 = m_globalIterator[2];

	m_globalIterator += 3;

	return &m_dumpTriangleIndexes;
}

GenericTriangle* Delaunay2dMesh::_getTriangle(unsigned triangleIndex)
{
	assert(m_associatedCloud && triangleIndex < m_numberOfTriangles);

	const int* tri = m_triIndexes + 3 * triangleIndex;
	m_associatedCloud->getPoint(*tri++, m_dumpTriangle.A);
	m_associatedCloud->getPoint(*tri++, m_dumpTriangle.B);
	m_associatedCloud->getPoint(*tri++, m_dumpTriangle.C);

	return static_cast<GenericTriangle*>(&m_dumpTriangle);
}

void Delaunay2dMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	assert(m_associatedCloud && triangleIndex < m_numberOfTriangles);

	const int* tri = m_triIndexes + 3 * triangleIndex;
	m_associatedCloud->getPoint(*tri++, A);
	m_associatedCloud->getPoint(*tri++, B);
	m_associatedCloud->getPoint(*tri++, C);
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
		m_associatedCloud->getBoundingBox(bbMin, bbMax);
	}
	else
	{
		bbMin = bbMax = CCVector3(0, 0, 0);
	}
}

Delaunay2dMesh* Delaunay2dMesh::TesselateContour(const std::vector<CCVector2>& contourPoints)
{
	size_t count = contourPoints.size();
	if (count < 3)
	{
		//not enough points
		return nullptr;
	}

	//DGM: we check that last vertex is different from the first one!
	//(yes it happens ;)
	if (contourPoints.back().x == contourPoints.front().x &&  contourPoints.back().y == contourPoints.front().y)
		--count;

	char errorStr[1024];
	Delaunay2dMesh* mesh = new Delaunay2dMesh();
	if (!mesh->buildMesh(contourPoints, count, errorStr) || mesh->size() == 0)
	{
		//triangulation failed
		delete mesh;
		return nullptr;
	}

	if (!mesh->removeOuterTriangles(contourPoints, contourPoints, true) || mesh->size() == 0)
	{
		//an error occurred
		delete mesh;
		return nullptr;
	}

	return mesh;
}

Delaunay2dMesh* Delaunay2dMesh::TesselateContour(GenericIndexedCloudPersist* contourPoints, int flatDimension/*=-1*/)
{
	if (!contourPoints)
	{
		assert(false);
		return nullptr;
	}
	
	unsigned count = contourPoints->size();
	if (count < 3)
	{
		//Not enough input points
		return nullptr;
	}

	std::vector<CCVector2> contourPoints2D;
	try
	{
		contourPoints2D.reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		//Not enough memory
		return nullptr;
	}

	if (flatDimension >= 0 && flatDimension <= 2) //X, Y or Z
	{
		const unsigned char Z = static_cast<unsigned char>(flatDimension);
		const unsigned char X = (Z == 2 ? 0 : Z + 1);
		const unsigned char Y = (X == 2 ? 0 : X + 1);
		for (unsigned i = 0; i < contourPoints->size(); ++i)
		{
			const CCVector3* P = contourPoints->getPoint(i);
			contourPoints2D.push_back(CCVector2(P->u[X], P->u[Y]));
		}
	}
	else
	{
		assert(flatDimension < 0);
		Neighbourhood Yk(contourPoints);
		if (!Yk.projectPointsOn2DPlane<CCVector2>(contourPoints2D))
		{
			//something bad happened
			return nullptr;
		}
	}

	CCLib::Delaunay2dMesh* dMesh = CCLib::Delaunay2dMesh::TesselateContour(contourPoints2D);
	return dMesh;
}
