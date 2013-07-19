//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccFacet.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccNormalVectors.h>

//CCLib
#include <Neighbourhood.h>
#include <CCMiscTools.h>
#include <Delaunay2dMesh.h>
#include <DistanceComputationTools.h>
#include <MeshSamplingTools.h>

ccFacet::ccFacet(QString name/*=QString("Facet")*/)
	: ccHObject(name)
	, m_polygonMesh(0)
	, m_contourPolyline(0)
	, m_contourVertices(0)
	, m_originPoints(0)
	, m_center(0,0,0)
	, m_rms(0.0)
	, m_surface(0.0)
{
	m_planeEquation[0] = 0;
	m_planeEquation[1] = 0;
	m_planeEquation[2] = 1;
	m_planeEquation[0] = 0;

	setVisible(true);
    lockVisibility(false);
}

void ccFacet::clearInternalRepresentation()
{
	if (m_polygonMesh)
		delete m_polygonMesh;
	if (m_contourPolyline)
		delete m_contourPolyline;
	if (m_contourVertices)
		delete m_contourVertices;
	m_surface = 0.0;
}

ccFacet::~ccFacet()
{
	clearInternalRepresentation();
	if (m_originPoints)
		delete m_originPoints;
}

void ccFacet::setDisplay_recursive(ccGenericGLDisplay* win)
{
	ccHObject::setDisplay_recursive(win);

	if (m_originPoints)
		m_originPoints->setDisplay_recursive(win);
	if (m_contourVertices)
		m_contourVertices->setDisplay_recursive(win);
	if (m_polygonMesh)
		m_polygonMesh->setDisplay_recursive(win);
	if (m_contourPolyline)
		m_contourPolyline->setDisplay_recursive(win);
}

class IndexedCCVector2 : public CCVector2
{
public:
	unsigned index;
};

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
PointCoordinateType cross(const CCVector2& O, const CCVector2& A, const CCVector2& B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Lexicographic sorting operator
bool LexicographicSort(const CCVector2& a, const CCVector2& b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

//! Returns a list of points on the convex hull in counter-clockwise order.
/** Implementation of Andrew's monotone chain 2D convex hull algorithm.
	Asymptotic complexity: O(n log n).
	Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz machine.
	Note: the last point in the returned list is the same as the first one.
	(retrieved from http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain)
**/
bool Convex_hull_2D(std::vector<IndexedCCVector2>& P, std::vector<CCVector2>& hullPoints, std::vector<unsigned>* hullPointsIndexes = 0)
{
	size_t n = P.size();

	try
	{
		hullPoints.resize(2*n);
		if (hullPointsIndexes)
			hullPointsIndexes->resize(2*n);
	}
	catch (std::bad_alloc)
	{
		//not enough memory
		return false;
	}

	// Sort points lexicographically
	std::sort(P.begin(), P.end(), LexicographicSort);

	// Build lower hull
	size_t k = 0;
	{
		for (size_t i = 0; i < n; i++)
		{
			while (k >= 2 && cross(hullPoints[k-2], hullPoints[k-1], P[i]) <= 0)
				k--;
			if (hullPointsIndexes)
				hullPointsIndexes->at(k) = P[i].index;
			hullPoints[k++] = P[i];
		}
	}

	// Build upper hull
	{
		size_t t = k+1;
		for (int i = (int)n-2; i >= 0; i--)
		{
			while (k >= t && cross(hullPoints[k-2], hullPoints[k-1], P[i]) <= 0)
				k--;
			if (hullPointsIndexes)
				hullPointsIndexes->at(k) = P[i].index;
			hullPoints[k++] = P[i];
		}
	}

	if (hullPointsIndexes)
		hullPointsIndexes->resize(k);
	hullPoints.resize(k);
	return true;
}

ccFacet* ccFacet::Create(ccPointCloud* points, bool transferOwnership/*=false*/)
{
	assert(points);

	//we need at least 3 points to compute a mesh or a plane! ;)
	if (!points || points->size() < 3)
	{
		ccLog::Error("[ccFacet::Create] Need at least 3 points to create a valid facet!");
		return 0;
	}

	//create initial facet structure
	ccFacet* facet = 0;
	{
		//convert input subset as a true point cloud
		ccPointCloud* clonedPoints = (transferOwnership ? points : static_cast<ccPointCloud*>(points->clone()));
		if (!clonedPoints)
		{
			ccLog::Error("[ccFacet::Create] Not enough memory!");
			return 0;
		}

		//get corresponding plane
		CCLib::Neighbourhood Yk(clonedPoints);
		const PointCoordinateType* planeEquation = Yk.getLSQPlane();
		if (!planeEquation)
		{
			delete clonedPoints;
			ccLog::Error("[ccFacet::Create] Failed to compute the LS plane passing through the input points!");
			return 0;
		}

		//create facet structure
		facet = new ccFacet();
		facet->m_originPoints = clonedPoints;
		memcpy(facet->m_planeEquation,planeEquation,sizeof(PointCoordinateType)*4);
		facet->m_center = *Yk.getGravityCenter();
		facet->m_rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(clonedPoints, planeEquation);
	}

	facet->updateInternalRepresentation();
	facet->setDisplay_recursive(points->getDisplay());

	return facet;
}

bool ccFacet::updateInternalRepresentation()
{
	clearInternalRepresentation();

	assert(m_originPoints);
	if (!m_originPoints)
		return false;
	unsigned ptsCount = m_originPoints->size();
	if (ptsCount < 3)
		return false;

	//we construct the plane local frame
	CCVector3 N(m_planeEquation);
	if (N.norm2() < ZERO_TOLERANCE)
		return false;
	CCVector3 u(1.0,0.0,0.0), v(0.0,1.0,0.0);
	CCLib::CCMiscTools::ComputeBaseVectors(N.u,u.u,v.u);

	//compute the cloud centroid
	CCVector3 G(0,0,0);
	{
		CCLib::Neighbourhood Yk(m_originPoints);
		G = *Yk.getGravityCenter();
	}

	//we project the input points on a plane
	std::vector<IndexedCCVector2> the2DPoints;
	{
		//reserve some memory for output
		try
		{
			the2DPoints.resize(ptsCount);
		}
		catch (std::bad_alloc) //out of memory
		{
			ccLog::Error("[ccFacet::updateInternalRepresentation] Not enough memory!");
			return false;
		}

		//project the points
		for (unsigned i=0; i<ptsCount; ++i)
		{
			//we recenter current point
			CCVector3 P = *m_originPoints->getPoint(i) - G;

			//then we project it on plane (with scalar prods)
			the2DPoints[i].x = P.dot(u);
			the2DPoints[i].y = P.dot(v);
			the2DPoints[i].index = i;
		}
	}

	//try to get the points on the convex hull to build the contour and the polygon
	{
		std::vector<CCVector2> hullPoints;
		//std::vector<unsigned> hullPointsIndexes;
		if (!Convex_hull_2D(the2DPoints,hullPoints/*,hullPointsIndexes*/))
		{
			ccLog::Error("[ccFacet::updateInternalRepresentation] Failed to compute the convex hull of the input points!");
		}

		unsigned hullPtsCount = (unsigned)hullPoints.size();
		//assert(hullPtsCount == hullPointsIndexes.size());

		//create vertices
		m_contourVertices = new ccPointCloud("vertices");
		{
			if (!m_contourVertices->reserve(hullPtsCount))
			{
				clearInternalRepresentation();
				ccLog::Error("[ccFacet::updateInternalRepresentation] Not enough memory!");
				return false;
			}
			
			//projection on the LS plane (in 3D)
			for (unsigned i=0; i<hullPtsCount; ++i)
				m_contourVertices->addPoint(G + u*hullPoints[i].x + v*hullPoints[i].y);
			m_contourVertices->setVisible(false);
		}

		//we create the corresponding (3D) polyline
		{
			m_contourPolyline = new ccPolyline(m_contourVertices);
			if (m_contourPolyline->reserve(hullPtsCount))
			{
				m_contourPolyline->addPointIndex(0,hullPtsCount);
				m_contourPolyline->setClosingState(true);
				m_contourPolyline->setVisible(true);
				//m_contourVertices->addChild(m_contourPolyline);
			}
			else
			{
				delete m_contourPolyline;
				m_contourPolyline = 0;
				ccLog::Warning("[ccFacet::updateInternalRepresentation] Not enough memory to create the contour polyline!");
			}
		}

		//we create the corresponding (2D) mesh
		CCLib::Delaunay2dMesh dm;
		if (dm.build(hullPoints))
		{
			unsigned triCount = dm.size();
			assert(triCount != 0);

			m_polygonMesh = new ccMesh(m_contourVertices);
			if (m_polygonMesh->reserve(triCount))
			{
				//import faces
				for (unsigned i=0; i<triCount; ++i)
				{
					const CCLib::TriangleSummitsIndexes* tsi = dm.getTriangleIndexes(i);
					m_polygonMesh->addTriangle(tsi->i1, tsi->i2, tsi->i3);
				}
				m_polygonMesh->setVisible(true);
				m_polygonMesh->enableStippling(true);
				//m_contourVertices->addChild(m_polygonMesh);

				//unique normal for facets
				if (m_polygonMesh->reservePerTriangleNormalIndexes())
				{
					NormsIndexesTableType* normsTable = new NormsIndexesTableType();
					normsTable->reserve(1);
					normsTable->addElement(ccNormalVectors::GetNormIndex(N.u));
					m_polygonMesh->setTriNormsTable(normsTable);
					for (unsigned i=0; i<triCount; ++i)
						m_polygonMesh->addTriangleNormalIndexes(0,0,0); //all triangles will have the same normal!
					m_polygonMesh->showNormals(true);
					m_polygonMesh->addChild(normsTable);
				}
				else
				{
					ccLog::Warning("[ccFacet::updateInternalRepresentation] Not enough memory to create the polygon mesh's normals!");
				}

				//update facet surface
				m_surface = CCLib::MeshSamplingTools::computeMeshArea(m_polygonMesh);
			}
			else
			{
				delete m_polygonMesh;
				m_polygonMesh = 0;
				ccLog::Warning("[ccFacet::updateInternalRepresentation] Not enough memory to create the polygon mesh!");
			}
		}
	}

	return true;
}

ccBBox ccFacet::getMyOwnBB()
{
	if (m_originPoints)
		return m_originPoints->getBB();
	else if (m_contourPolyline)
		return m_contourPolyline->getBB();
	else
		return ccBBox();
}

void ccFacet::setColor(const colorType rgb[])
{
	if (m_contourVertices && m_contourVertices->setRGBColor(rgb))
	{
		m_contourVertices->showColors(true);
		if (m_polygonMesh)
			m_polygonMesh->showColors(true);
	}

	if (m_contourPolyline)
	{
		m_contourPolyline->setColor(rgb);
		m_contourPolyline->showColors(true);
	}
	showColors(true);
}

void ccFacet::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (MACRO_Draw3D(context))
    {
		if (m_originPoints)
			m_originPoints->draw(context);
		if (m_contourPolyline)
			m_contourPolyline->draw(context);
		if (m_polygonMesh)
			m_polygonMesh->draw(context);
    }
}

bool ccFacet::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//origin points (dataVersion>=28)
	{
		bool hasOriginPoints = (m_originPoints != 0);
		if (out.write((const char*)&hasOriginPoints,sizeof(bool))<0)
			return WriteError();

		if (hasOriginPoints)
			if (!m_originPoints->toFile(out))
				return false;
	}

	//plane equation (dataVersion>=28)
	if (out.write((const char*)&m_planeEquation,sizeof(PointCoordinateType)*4)<0)
		return WriteError();

	//center (dataVersion>=28)
	if (out.write((const char*)m_center.u,sizeof(PointCoordinateType)*3)<0)
		return WriteError();

	//RMS (dataVersion>=28)
	if (out.write((const char*)&m_rms,sizeof(double))<0)
		return WriteError();

	//contour vertices (dataVersion>=28)
	{
		bool hasCountourVertices = (m_contourVertices != 0);
		if (out.write((const char*)&hasCountourVertices,sizeof(bool))<0)
			return WriteError();

		if (hasCountourVertices)
			if (!m_contourVertices->toFile(out))
				return false;
	}

	//contour poyline (dataVersion>=28)
	{
		bool hasCountourPolyline = (m_contourVertices != 0 && m_contourPolyline != 0 && m_contourPolyline->getAssociatedCloud() == m_contourVertices);
		if (out.write((const char*)&hasCountourPolyline,sizeof(bool))<0)
			return WriteError();

		if (hasCountourPolyline)
			if (!m_contourPolyline->toFile(out))
				return false;
	}

	//polygon (dataVersion>=28)
	{
		bool hasPolygon = (m_contourVertices != 0 && m_polygonMesh != 0 && m_polygonMesh->getAssociatedCloud() == m_contourVertices);
		if (out.write((const char*)&hasPolygon,sizeof(bool))<0)
			return WriteError();

		if (hasPolygon)
			if (!m_polygonMesh->toFile(out))
				return false;
	}

	//surface (dataVersion>=28)
	if (out.write((const char*)&m_surface,sizeof(double))<0)
		return WriteError();


	return true;
}

bool ccFacet::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion))
		return false;

	if (dataVersion < 28)
		return false;

	//origin points (dataVersion>=28)
	{
		bool hasOriginPoints = (m_originPoints != 0);
		if (in.read((char*)&hasOriginPoints,sizeof(bool))<0)
			return ReadError();

		if (hasOriginPoints)
		{
			unsigned classID=0;
			if (!ccObject::ReadClassIDFromFile(classID, in, dataVersion))
				return ReadError();
			if (classID != CC_POINT_CLOUD)
				return CorruptError();
			m_originPoints = new ccPointCloud();
			if (!m_originPoints->fromFile(in,dataVersion))
				return false;
		}
	}

	//plane equation (dataVersion>=28)
	if (in.read((char*)&m_planeEquation,sizeof(PointCoordinateType)*4)<0)
		return ReadError();

	//center (dataVersion>=28)
	if (in.read((char*)m_center.u,sizeof(PointCoordinateType)*3)<0)
		return ReadError();

	//RMS (dataVersion>=28)
	if (in.read((char*)&m_rms,sizeof(double))<0)
		return ReadError();

	//contour vertices (dataVersion>=28)
	{
		bool hasCountourVertices = false;
		if (in.read((char*)&hasCountourVertices,sizeof(bool))<0)
			return ReadError();

		if (hasCountourVertices)
		{
			unsigned classID=0;
			if (!ccObject::ReadClassIDFromFile(classID, in, dataVersion))
				return ReadError();
			if (classID != CC_POINT_CLOUD)
				return CorruptError();
			m_contourVertices = new ccPointCloud();
			if (!m_contourVertices->fromFile(in,dataVersion))
				return false;
		}
	}

	//contour poyline (dataVersion>=28)
	{
		bool hasCountourPolyline = false;
		if (in.read((char*)&hasCountourPolyline,sizeof(bool))<0)
			return ReadError();

		if (hasCountourPolyline)
		{
			assert(m_contourVertices);
			unsigned classID=0;
			if (!ccObject::ReadClassIDFromFile(classID, in, dataVersion))
				return ReadError();
			if (classID != CC_POLY_LINE)
				return CorruptError();
			m_contourPolyline = new ccPolyline(0);
			if (!m_contourPolyline->fromFile(in,dataVersion))
				return false;
			//associated cloud is reset by fromFile!
			m_contourPolyline->setAssociatedCloud(m_contourVertices);
		}
	}

	//polygon (dataVersion>=28)
	{
		bool hasPolygon = false;
		if (in.read((char*)&hasPolygon,sizeof(bool))<0)
			return ReadError();

		if (hasPolygon)
		{
			assert(m_contourVertices);
			unsigned classID=0;
			if (!ccObject::ReadClassIDFromFile(classID, in, dataVersion))
				return ReadError();
			if (classID != CC_MESH)
				return CorruptError();
			m_polygonMesh = new ccMesh(0);
			if (!m_polygonMesh->fromFile(in,dataVersion))
				return false;
			m_polygonMesh->setAssociatedCloud(m_contourVertices); //associated cloud is reset by fromFile!

			//we must also manually link the normal(s) table
			intptr_t triNormsTableID = (intptr_t)m_polygonMesh->getTriNormsTable();
			if (triNormsTableID > 0)
			{
				ccHObject* triNormsTable = m_polygonMesh->find(triNormsTableID);
				if (triNormsTable && triNormsTable->isA(CC_NORMAL_INDEXES_ARRAY))
					m_polygonMesh->setTriNormsTable(static_cast<NormsIndexesTableType*>(triNormsTable),false);
				else
				{
					ccLog::Warning(QString("[ccFacet::fromFile_MeOnly] Couldn't find shared normals (ID=%1) for the facet's polygon mesh!").arg(triNormsTableID));
					m_polygonMesh->setTriNormsTable(0,false);
					m_polygonMesh->showTriNorms(false);
				}
			}
		}
	}

	//surface (dataVersion>=28)
	if (in.read((char*)&m_surface,sizeof(double))<0)
		return ReadError();

	return true;
}
