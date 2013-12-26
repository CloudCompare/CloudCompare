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
#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccMesh.h"
#include "ccPolyline.h"
#include "ccNormalVectors.h"
#include "ccCylinder.h"
#include "ccCone.h"

//CCLib
#include <Neighbourhood.h>
#include <CCMiscTools.h>
#include <Delaunay2dMesh.h>
#include <DistanceComputationTools.h>
#include <PointProjectionTools.h>
#include <MeshSamplingTools.h>
#include <SimpleCloud.h>

static const char DEFAULT_POLYGON_MESH_NAME[] = "2D polygon";
static const char DEFAULT_CONTOUR_NAME[] = "Contour";
static const char DEFAULT_CONTOUR_POINTS_NAME[] = "Contour points";
static const char DEFAULT_ORIGIN_POINTS_NAME[] = "Origin points";

ccFacet::ccFacet(	PointCoordinateType maxEdgeLength/*=0*/,
					QString name/*=QString("Facet")*/ )
	: ccHObject(name)
	, m_polygonMesh(0)
	, m_contourPolyline(0)
	, m_contourVertices(0)
	, m_originPoints(0)
	, m_center(0,0,0)
	, m_rms(0.0)
	, m_surface(0.0)
	, m_maxEdgeLength(maxEdgeLength)
	, m_showNormalVector(false)
{
	m_planeEquation[0] = 0;
	m_planeEquation[1] = 0;
	m_planeEquation[2] = 1;
	m_planeEquation[3] = 0;

	setVisible(true);
    lockVisibility(false);
}

ccFacet::~ccFacet()
{
}

ccFacet* ccFacet::clone() const
{
	ccFacet* facet = new ccFacet(m_maxEdgeLength, m_name);

	//clone contour
	if (m_contourPolyline)
	{
		assert(m_contourVertices);
		facet->m_contourPolyline = new ccPolyline(*m_contourPolyline);
		facet->m_contourVertices = dynamic_cast<ccPointCloud*>(facet->m_contourPolyline->getAssociatedCloud());

		if (!facet->m_contourPolyline || !facet->m_contourVertices)
		{
			//not enough memory?!
			ccLog::Warning(QString("[ccFacet::clone][%1] Failed to clone countour!").arg(getName()));
			delete facet;
			return 0;
		}

		facet->m_contourPolyline->setLocked(m_contourPolyline->isLocked());
		facet->m_contourVertices->setEnabled(m_contourVertices->isEnabled());
		facet->m_contourVertices->setVisible(m_contourVertices->isVisible());
		facet->m_contourVertices->setLocked(m_contourVertices->isLocked());
		facet->m_contourVertices->setName(m_contourVertices->getName());
		facet->m_contourVertices->addChild(facet->m_contourPolyline);
		facet->addChild(facet->m_contourVertices);
	}

	//clone mesh
	if (m_polygonMesh)
	{
		facet->m_polygonMesh = m_polygonMesh->clone(facet->m_contourVertices);
		if (!facet->m_polygonMesh)
		{
			//not enough memory?!
			ccLog::Warning(QString("[ccFacet::clone][%1] Failed to clone polygon!").arg(getName()));
			delete facet;
			return 0;
		}
		
		facet->m_polygonMesh->setLocked(m_polygonMesh->isLocked());
		facet->m_polygonMesh->setName(m_polygonMesh->getName());
		if (facet->m_contourVertices)
			facet->m_contourVertices->addChild(facet->m_polygonMesh);
		else
			facet->addChild(facet->m_polygonMesh);
	}


	if (m_originPoints)
	{
		facet->m_originPoints = dynamic_cast<ccPointCloud*>(m_originPoints->clone());
		if (!facet->m_originPoints)
		{
			ccLog::Warning(QString("[ccFacet::clone][%1] Failed to clone origin points!").arg(getName()));
			//delete facet;
			//return 0;
		}
		else
		{
			facet->m_originPoints->setLocked(m_originPoints->isLocked());
			facet->m_originPoints->setName(m_originPoints->getName());
			facet->addChild(facet->m_originPoints);
		}
	}

	facet->m_center = m_center;
	facet->m_rms = m_rms;
	facet->m_surface = m_surface;
	facet->m_showNormalVector = m_showNormalVector;
	memcpy(facet->m_planeEquation, m_planeEquation, sizeof(PointCoordinateType)*4);
	facet->setVisible(isVisible());
	facet->lockVisibility(isVisiblityLocked());
	facet->setDisplay_recursive(getDisplay());

	return facet;
}

ccFacet* ccFacet::Create(	CCLib::GenericIndexedCloudPersist* cloud,
							PointCoordinateType maxEdgeLength/*=0*/,
							bool transferOwnership/*=false*/,
							const PointCoordinateType* planeEquation/*=0*/)
{
	assert(cloud);

	//we need at least 3 points to compute a mesh or a plane! ;)
	if (!cloud || cloud->size() < 3)
	{
		ccLog::Error("[ccFacet::Create] Need at least 3 points to create a valid facet!");
		return 0;
	}

	//create facet structure
	ccFacet* facet = new ccFacet(maxEdgeLength,"facet");
	if (!facet->createInternalRepresentation(cloud,planeEquation))
	{
		delete facet;
		return 0;
	}

	ccPointCloud* pc = dynamic_cast<ccPointCloud*>(cloud);
	if (pc)
	{
		facet->setName(pc->getName()+QString(".facet"));
		if (transferOwnership)
		{
			pc->setName(DEFAULT_ORIGIN_POINTS_NAME);
			pc->setEnabled(false);
			pc->setLocked(true);
			facet->addChild(pc);
			facet->setOriginPoints(pc);
		}

		facet->setDisplay_recursive(pc->getDisplay());
	}

	return facet;
}

bool ccFacet::createInternalRepresentation(	CCLib::GenericIndexedCloudPersist* points,
											const PointCoordinateType* planeEquation/*=0*/)
{
	assert(points);
	if (!points)
		return false;
	unsigned ptsCount = points->size();
	if (ptsCount < 3)
		return false;

	CCLib::Neighbourhood Yk(points);

	//get corresponding plane
	if (!planeEquation)
	{
		planeEquation = Yk.getLSQPlane();
		if (!planeEquation)
		{
			ccLog::Warning("[ccFacet::createInternalRepresentation] Failed to compute the LS plane passing through the input points!");
			return false;
		}
	}
	memcpy(m_planeEquation,planeEquation,sizeof(PointCoordinateType)*4);

	//we project the input points on a plane
	std::vector<CCLib::PointProjectionTools::IndexedCCVector2> points2D;
	CCVector3 X,Y; //local base
	if (!Yk.projectPointsOn2DPlane<CCLib::PointProjectionTools::IndexedCCVector2>(points2D,0,&m_center,&X,&Y))
	{
		ccLog::Error("[ccFacet::createInternalRepresentation] Not enough memory!");
		return false;
	}

	//compute resulting RMS
	m_rms = CCLib::DistanceComputationTools::computeCloud2PlaneDistanceRMS(points, m_planeEquation);
	
	//update the points indexes (not done by Neighbourhood::projectPointsOn2DPlane)
	{
		for (unsigned i=0; i<ptsCount; ++i)
			points2D[i].index = i;
	}

	//try to get the points on the convex/concave hull to build the contour and the polygon
	{
		std::list<CCLib::PointProjectionTools::IndexedCCVector2*> hullPoints;
		if (!CCLib::PointProjectionTools::extractConcaveHull2D(	points2D,
																hullPoints,
																m_maxEdgeLength) )
		{
			ccLog::Error("[ccFacet::createInternalRepresentation] Failed to compute the convex hull of the input points!");
		}

		unsigned hullPtsCount = static_cast<unsigned>(hullPoints.size());

		//create vertices
		m_contourVertices = new ccPointCloud();
		{
			if (!m_contourVertices->reserve(hullPtsCount))
			{
				delete m_contourVertices;
				m_contourVertices = 0;
				ccLog::Error("[ccFacet::createInternalRepresentation] Not enough memory!");
				return false;
			}
			
			//projection on the LS plane (in 3D)
			for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
				m_contourVertices->addPoint(m_center + X*(*it)->x + Y*(*it)->y);
			m_contourVertices->setName(DEFAULT_CONTOUR_POINTS_NAME);
			m_contourVertices->setLocked(true);
			m_contourVertices->setVisible(false);
			addChild(m_contourVertices);
		}

		//we create the corresponding (3D) polyline
		{
			m_contourPolyline = new ccPolyline(m_contourVertices);
			if (m_contourPolyline->reserve(hullPtsCount))
			{
				m_contourPolyline->addPointIndex(0,hullPtsCount);
				m_contourPolyline->setClosed(true);
				m_contourPolyline->setVisible(true);
				m_contourPolyline->setLocked(true);
				m_contourPolyline->setName(DEFAULT_CONTOUR_NAME);
				m_contourVertices->addChild(m_contourPolyline);
			}
			else
			{
				delete m_contourPolyline;
				m_contourPolyline = 0;
				ccLog::Warning("[ccFacet::createInternalRepresentation] Not enough memory to create the contour polyline!");
			}
		}

		//we create the corresponding (2D) mesh
		CCLib::Delaunay2dMesh dm;
		std::vector<CCVector2> hullPointsVector;
		try
		{
			hullPointsVector.reserve(hullPoints.size());
			for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
				hullPointsVector.push_back(**it);
		}
		catch(...)
		{
			ccLog::Warning("[ccFacet::createInternalRepresentation] Not enough memory to create the contour mesh!");
		}

		//if we have computed a concave hull, we must remove triangles falling outside!
		bool removePointsOutsideHull = (m_maxEdgeLength > 0);

		if (!hullPointsVector.empty() && dm.build(hullPointsVector,0,removePointsOutsideHull))
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

				//unique normal for facets
				if (m_polygonMesh->reservePerTriangleNormalIndexes())
				{
					NormsIndexesTableType* normsTable = new NormsIndexesTableType();
					normsTable->reserve(1);
					CCVector3 N(m_planeEquation);
					normsTable->addElement(ccNormalVectors::GetNormIndex(N.u));
					m_polygonMesh->setTriNormsTable(normsTable);
					for (unsigned i=0; i<triCount; ++i)
						m_polygonMesh->addTriangleNormalIndexes(0,0,0); //all triangles will have the same normal!
					m_polygonMesh->showNormals(true);
					m_polygonMesh->addChild(normsTable);
					m_polygonMesh->setLocked(true);
					m_polygonMesh->setName(DEFAULT_POLYGON_MESH_NAME);
					m_contourVertices->addChild(m_polygonMesh);
				}
				else
				{
					ccLog::Warning("[ccFacet::createInternalRepresentation] Not enough memory to create the polygon mesh's normals!");
				}

				//update facet surface
				m_surface = CCLib::MeshSamplingTools::computeMeshArea(m_polygonMesh);
			}
			else
			{
				delete m_polygonMesh;
				m_polygonMesh = 0;
				ccLog::Warning("[ccFacet::createInternalRepresentation] Not enough memory to create the polygon mesh!");
			}
		}
	}

	return true;
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

//unit point marker
static QSharedPointer<ccCylinder> c_unitNormalSymbol(0);
static QSharedPointer<ccCone> c_unitNormalHeadSymbol(0);

void ccFacet::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	if (m_showNormalVector && m_contourPolyline)
	{
		if (!c_unitNormalSymbol)
		{
			c_unitNormalSymbol = QSharedPointer<ccCylinder>(new ccCylinder(0.02f,0.9f,0,"UnitNormal",12));
			c_unitNormalSymbol->showColors(true);
			c_unitNormalSymbol->setVisible(true);
			c_unitNormalSymbol->setEnabled(true);
			c_unitNormalSymbol->setTempColor(ccColor::green);
		}
		if (!c_unitNormalHeadSymbol)
		{
			c_unitNormalHeadSymbol = QSharedPointer<ccCone>(new ccCone(0.05f,0.0f,0.1f,0,0,0,"UnitNormalHead",12));
			c_unitNormalHeadSymbol->showColors(true);
			c_unitNormalHeadSymbol->setVisible(true);
			c_unitNormalHeadSymbol->setEnabled(true);
			c_unitNormalHeadSymbol->setTempColor(ccColor::green);
		}

		//build-up point maker own 'context'
		CC_DRAW_CONTEXT markerContext = context;
		markerContext.flags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
		markerContext._win = 0;

		c_unitNormalSymbol->setTempColor(m_contourPolyline->getColor());
		PointCoordinateType scale = m_contourPolyline->getBB().getMinBoxDim();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		ccGL::Translate(m_center.x,m_center.y,m_center.z);
		ccGLMatrix mat = ccGLMatrix::FromToRotation(getNormal(),CCVector3(0,0,1));
		glMultMatrixf(mat.data());
		ccGL::Scale(scale,scale,scale);
		glTranslatef(0,0,0.45f);
		c_unitNormalSymbol->draw(markerContext);
		glTranslatef(0,0,0.45f);
		c_unitNormalHeadSymbol->draw(markerContext);
		glPopMatrix();
	}
}

bool ccFacet::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the origin points here (as it will be automatically saved as a child)
	//so instead we save it's unique ID (dataVersion>=32)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	{
		uint32_t originPointsUniqueID = (m_originPoints ? (uint32_t)m_originPoints->getUniqueID() : 0);
		if (out.write((const char*)&originPointsUniqueID,4)<0)
			return WriteError();
	}

	//we can't save the contour points here (as it will be automatically saved as a child)
	//so instead we save it's unique ID (dataVersion>=32)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	{
		uint32_t contourPointsUniqueID = (m_contourVertices ? (uint32_t)m_contourVertices->getUniqueID() : 0);
		if (out.write((const char*)&contourPointsUniqueID,4)<0)
			return WriteError();
	}

	//we can't save the contour polyline here (as it will be automatically saved as a child)
	//so instead we save it's unique ID (dataVersion>=32)
	//WARNING: the polyline must be saved in the same BIN file! (responsibility of the caller)
	{
		uint32_t contourPolyUniqueID = (m_contourPolyline ? (uint32_t)m_contourPolyline->getUniqueID() : 0);
		if (out.write((const char*)&contourPolyUniqueID,4)<0)
			return WriteError();
	}

	//we can't save the polygon mesh here (as it will be automatically saved as a child)
	//so instead we save it's unique ID (dataVersion>=32)
	//WARNING: the mesh must be saved in the same BIN file! (responsibility of the caller)
	{
		uint32_t polygonMeshUniqueID = (m_polygonMesh ? (uint32_t)m_polygonMesh->getUniqueID() : 0);
		if (out.write((const char*)&polygonMeshUniqueID,4)<0)
			return WriteError();
	}

	//plane equation (dataVersion>=32)
	if (out.write((const char*)&m_planeEquation,sizeof(PointCoordinateType)*4)<0)
		return WriteError();

	//center (dataVersion>=32)
	if (out.write((const char*)m_center.u,sizeof(PointCoordinateType)*3)<0)
		return WriteError();

	//RMS (dataVersion>=32)
	if (out.write((const char*)&m_rms,sizeof(double))<0)
		return WriteError();

	//surface (dataVersion>=32)
	if (out.write((const char*)&m_surface,sizeof(double))<0)
		return WriteError();

	//Max edge length (dataVersion>=31)
	if (out.write((const char*)&m_maxEdgeLength,sizeof(PointCoordinateType))<0)
		return WriteError();

	return true;
}

bool ccFacet::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	if (dataVersion < 32)
		return false;

	//origin points (dataVersion>=32)
	//as the cloud will be saved automatically (as a child)
	//we only store its unique ID --> we hope we will find it at loading time
	{
		uint32_t origPointsUniqueID = 0;
		if (in.read((char*)&origPointsUniqueID,4)<0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'm_originPoints' pointer!!!
		*(uint32_t*)(&m_originPoints) = origPointsUniqueID;
	}

	//contour points
	//as the cloud will be saved automatically (as a child)
	//we only store its unique ID --> we hope we will find it at loading time
	{
		uint32_t contourPointsUniqueID = 0;
		if (in.read((char*)&contourPointsUniqueID,4)<0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the cloud unique ID in the 'm_contourVertices' pointer!!!
		*(uint32_t*)(&m_contourVertices) = contourPointsUniqueID;
	}

	//contour points
	//as the polyline will be saved automatically (as a child)
	//we only store its unique ID --> we hope we will find it at loading time
	{
		uint32_t contourPolyUniqueID = 0;
		if (in.read((char*)&contourPolyUniqueID,4)<0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the polyline unique ID in the 'm_contourPolyline' pointer!!!
		*(uint32_t*)(&m_contourPolyline) = contourPolyUniqueID;
	}

	//polygon mesh
	//as the mesh will be saved automatically (as a child)
	//we only store its unique ID --> we hope we will find it at loading time
	{
		uint32_t polygonMeshUniqueID = 0;
		if (in.read((char*)&polygonMeshUniqueID,4)<0)
			return ReadError();
		//[DIRTY] WARNING: temporarily, we set the polyline unique ID in the 'm_contourPolyline' pointer!!!
		*(uint32_t*)(&m_polygonMesh) = polygonMeshUniqueID;
	}

	//plane equation (dataVersion>=32)
	if (in.read((char*)&m_planeEquation,sizeof(PointCoordinateType)*4)<0)
		return ReadError();

	//center (dataVersion>=32)
	if (in.read((char*)m_center.u,sizeof(PointCoordinateType)*3)<0)
		return ReadError();

	//RMS (dataVersion>=32)
	if (in.read((char*)&m_rms,sizeof(double))<0)
		return ReadError();

	//surface (dataVersion>=32)
	if (in.read((char*)&m_surface,sizeof(double))<0)
		return ReadError();

	//Max edge length (dataVersion>=31)
	if (in.read((char*)&m_maxEdgeLength,sizeof(PointCoordinateType))<0)
		return WriteError();

	return true;
}

void ccFacet::invertNormal()
{
	for (int i=0; i<4; ++i)
	{
		m_planeEquation[i] = -m_planeEquation[i];
	}
}
