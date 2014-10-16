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

//Always first
#include "ccIncludeGL.h"

#include "ccPolyline.h"

//Local
#include "ccPointCloud.h"

//CCLib
#include <Neighbourhood.h>
#include <PointProjectionTools.h>
#include <CCMiscTools.h>

//System
#include <string.h>

ccPolyline::ccPolyline(GenericIndexedCloudPersist* associatedCloud)
	: Polyline(associatedCloud)
	, ccHObject("Polyline")
{
	set2DMode(false);
	setForeground(true);
	setVisible(true);
	lockVisibility(false);
	setColor(ccColor::white);
	showVertices(false);
	setVertexMarkerWidth(3);
	setWidth(0);
}

ccPolyline::ccPolyline(const ccPolyline& poly)
	: Polyline(0)
	, ccHObject(poly)
{
	ccPointCloud* cloud = dynamic_cast<ccPointCloud*>(poly.m_theAssociatedCloud);
	ccPointCloud* clone = cloud ? cloud->partialClone(&poly) : ccPointCloud::From(&poly);
	if (clone)
	{
		if (cloud)
			clone->setName(cloud->getName()); //as 'partialClone' adds the '.extract' suffix by default
	}
	else
	{
		//not enough memory?
		ccLog::Warning("[ccPolyline][copy constructor] Not enough memory!");
	}

	initWith(clone,poly);
}

void ccPolyline::initWith(ccPointCloud* vertices, const ccPolyline& poly)
{
	if (vertices)
	{
		setAssociatedCloud(vertices);
		addChild(vertices);
		//vertices->setEnabled(false);
		assert(m_theAssociatedCloud);
		if (m_theAssociatedCloud)
			addPointIndex(0,m_theAssociatedCloud->size());
	}

	setClosed(poly.m_isClosed);
	set2DMode(poly.m_mode2D);
	setForeground(poly.m_foreground);
	setVisible(poly.isVisible());
	lockVisibility(poly.isVisiblityLocked());
	setColor(poly.m_rgbColor);
	setWidth(poly.m_width);
	showColors(poly.colorsShown());
	showVertices(poly.verticesShown());
	setVertexMarkerWidth(poly.getVertexMarkerWidth());
	setVisible(poly.isVisible());
}

void ccPolyline::set2DMode(bool state)
{
	m_mode2D = state;
}

void ccPolyline::setForeground(bool state)
{
	m_foreground = state;
}

ccBBox ccPolyline::getMyOwnBB()
{
	ccBBox emptyBox;
	getBoundingBox(emptyBox.minCorner().u, emptyBox.maxCorner().u);
	emptyBox.setValidity(!is2DMode() && size() != 0);
	return emptyBox;
}

bool ccPolyline::hasColors() const
{
	return true;
}

void ccPolyline::applyGLTransformation(const ccGLMatrix& trans)
{
	//transparent call
	ccHObject::applyGLTransformation(trans);

	//invalidate the bounding-box
	//(and we hope the vertices will be updated as well!)
	m_validBB = false;
}

void ccPolyline::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//no picking enabled on polylines
	if (MACRO_DrawNames(context))
		return;

	unsigned vertCount = size();
	if (vertCount < 2)
		return;

	bool draw = false;

	if (MACRO_Draw3D(context))
	{
		draw = !m_mode2D;
	}
	else if (m_mode2D)
	{
		bool drawFG = MACRO_Foreground(context);
		draw = ((drawFG && m_foreground) || (!drawFG && !m_foreground));
	}

	if (draw)
	{
		if (colorsShown())
			glColor3ubv(m_rgbColor);

		//display polyline
		{
			if (m_width != 0)
			{
				glPushAttrib(GL_LINE_BIT);
				glLineWidth(static_cast<GLfloat>(m_width));
			}

			glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);
			for (unsigned i=0; i<vertCount; ++i)
			{
				ccGL::Vertex3v(getPoint(i)->u);
			}
			glEnd();

			if (m_width != 0)
			{
				glPopAttrib();
			}
		}

		//display vertices
		if (m_showVertices)
		{
			glPushAttrib(GL_POINT_BIT);
			glPointSize((GLfloat)m_vertMarkWidth);

			glBegin(GL_POINTS);
			for (unsigned i=0; i<vertCount; ++i)
			{
				ccGL::Vertex3v(getPoint(i)->u);
			}
			glEnd();

			glPopAttrib();
		}
	}
}

void ccPolyline::setColor(const colorType col[])
{
	memcpy(m_rgbColor,col,sizeof(colorType)*3);
}

void ccPolyline::setWidth(PointCoordinateType width)
{
	m_width = width;
}

const colorType* ccPolyline::getColor() const
{
	return m_rgbColor;
}

bool ccPolyline::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by multiple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[ccPolyline::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
		return false;
	}
	uint32_t vertUniqueID = (m_theAssociatedCloud ? (uint32_t)vertices->getUniqueID() : 0);
	if (out.write((const char*)&vertUniqueID,4) < 0)
		return WriteError();

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = size();
	if (out.write((const char*)&pointCount,4) < 0)
		return WriteError();

	//points (references to) (dataVersion>=28)
	for (uint32_t i=0; i<pointCount; ++i)
	{
		uint32_t pointIndex = getPointGlobalIndex(i);
		if (out.write((const char*)&pointIndex,4) < 0)
			return WriteError();
	}

	QDataStream outStream(&out);

	//Closing state (dataVersion>=28)
	outStream << m_isClosed;

	//RGB Color (dataVersion>=28)
	outStream << m_rgbColor[0];
	outStream << m_rgbColor[1];
	outStream << m_rgbColor[2];

	//2D mode (dataVersion>=28)
	outStream << m_mode2D;

	//Foreground mode (dataVersion>=28)
	outStream << m_foreground;

	//The width of the line (dataVersion>=31)
	outStream << m_width;

	return true;
}

bool ccPolyline::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	if (dataVersion<28)
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by multiple polylines)
	//we only store its unique ID (dataVersion>=28) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID,4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_theAssociatedCloud) = vertUniqueID;

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = 0;
	if (in.read((char*)&pointCount,4) < 0)
		return ReadError();
	if (!reserve(pointCount))
		return false;

	//points (references to) (dataVersion>=28)
	for (uint32_t i=0; i<pointCount; ++i)
	{
		uint32_t pointIndex = 0;
		if (in.read((char*)&pointIndex,4) < 0)
			return ReadError();
		addPointIndex(pointIndex);
	}

	QDataStream inStream(&in);

	//Closing state (dataVersion>=28)
	inStream >> m_isClosed;

	//RGB Color (dataVersion>=28)
	inStream >> m_rgbColor[0];
	inStream >> m_rgbColor[1];
	inStream >> m_rgbColor[2];

	//2D mode (dataVersion>=28)
	inStream >> m_mode2D;

	//Foreground mode (dataVersion>=28)
	inStream >> m_foreground;

	//Width of the line (dataVersion>=31)
	if (dataVersion >= 31)
		ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_width,1);
	else
		m_width = 0;

	return true;
}

bool ccPolyline::split(	PointCoordinateType maxEdgelLength,
						std::vector<ccPolyline*>& parts)
{
	parts.clear();

	//not enough vertices?
	unsigned vertCount = size();
	if (vertCount <= 2)
	{
		parts.push_back(new ccPolyline(*this));
		return true;
	}

	unsigned startIndex = 0;
	unsigned lastIndex = vertCount-1;
	while (startIndex <= lastIndex)
	{
		unsigned stopIndex = startIndex;
		while (stopIndex < lastIndex && (*getPoint(stopIndex+1) - *getPoint(stopIndex)).norm() <= maxEdgelLength)
		{
			++stopIndex;
		}

		//number of vertices for the current part
		unsigned partSize = stopIndex-startIndex+1;

		//if the polyline is closed we have to look backward for the first segment!
		if (startIndex == 0)
		{
			if (isClosed())
			{
				unsigned realStartIndex = vertCount;
				while (realStartIndex > stopIndex && (*getPoint(realStartIndex-1) - *getPoint(realStartIndex % vertCount)).norm() <= maxEdgelLength)
				{
					--realStartIndex;
				}

				if (realStartIndex == stopIndex)
				{
					//whole loop
					parts.push_back(new ccPolyline(*this));
					return true;
				}
				else if (realStartIndex < vertCount)
				{
					partSize += (vertCount - realStartIndex);
					assert(realStartIndex != 0);
					lastIndex = realStartIndex-1;
					//warning: we shift the indexes!
					startIndex = realStartIndex; 
					stopIndex += vertCount;
				}
			}
			else if (partSize == vertCount)
			{
				//whole polyline
				parts.push_back(new ccPolyline(*this));
				return true;
			}
		}

		if (partSize > 1) //otherwise we skip that point
		{
			//create the corresponding part
			CCLib::ReferenceCloud ref(m_theAssociatedCloud);
			if (!ref.reserve(partSize))
			{
				ccLog::Error("[ccPolyline::split] Not enough memory!");
				return false;
			}

			for (unsigned i=startIndex; i<=stopIndex; ++i)
			{
				ref.addPointIndex(i % vertCount);
			}

			ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
			ccPointCloud* subset = vertices ? vertices->partialClone(&ref) : ccPointCloud::From(&ref);
			ccPolyline* part = new ccPolyline(subset);
			part->initWith(subset,*this);
			part->setClosed(false); //by definition!
			parts.push_back(part);
		}

		//forward
		startIndex = (stopIndex % vertCount) + 1;
	}

	return true;
}

bool ccPolyline::ExtractFlatContour(CCLib::GenericIndexedCloudPersist* points,
									PointCoordinateType maxEdgelLength,
									std::vector<ccPolyline*>& parts,
									bool allowSplitting/*=true*/,
									const PointCoordinateType* preferredDim/*=0*/)
{
	parts.clear();

	//extract whole contour
	ccPolyline* basePoly = ExtractFlatContour(points,maxEdgelLength,preferredDim);
	if (!basePoly)
	{
		return false;
	}
	else if (!allowSplitting)
	{
		parts.push_back(basePoly);
		return true;
	}

	//and split it if necessary
	bool success = basePoly->split(maxEdgelLength,parts);

	delete basePoly;
	basePoly = 0;

	return success;

}

ccPolyline* ccPolyline::ExtractFlatContour(	CCLib::GenericIndexedCloudPersist* points,
											PointCoordinateType maxEdgelLength/*=0*/,
											const PointCoordinateType* preferredDim/*=0*/)
{
	assert(points);
	if (!points)
		return 0;
	unsigned ptsCount = points->size();
	if (ptsCount < 3)
		return 0;

	CCLib::Neighbourhood Yk(points);
	CCVector3 O,X,Y; //local base

	//we project the input points on a plane
	std::vector<CCLib::PointProjectionTools::IndexedCCVector2> points2D;
	PointCoordinateType* planeEq = 0;
	//if the user has specified a default direction, we'll use it as 'projecting plane'
	PointCoordinateType preferredPlaneEq[4] = {0, 0, 0, 0};
	if (preferredDim != 0)
	{
		const CCVector3* G = points->getPoint(0); //any point through which the point pass is ok
		preferredPlaneEq[0] = preferredDim[0];
		preferredPlaneEq[1] = preferredDim[1];
		preferredPlaneEq[2] = preferredDim[2];
		CCVector3::vnormalize(preferredPlaneEq);
		preferredPlaneEq[3] = CCVector3::vdot(G->u,preferredPlaneEq);
		planeEq = preferredPlaneEq;
	}
	if (!Yk.projectPointsOn2DPlane<CCLib::PointProjectionTools::IndexedCCVector2>(points2D,planeEq,&O,&X,&Y))
	{
		ccLog::Warning("[ccPolyline::ExtractFlatContour] Failed to project the points on the LS plane (not enough memory?)!");
		return 0;
	}

	//update the points indexes (not done by Neighbourhood::projectPointsOn2DPlane)
	{
		for (unsigned i=0; i<ptsCount; ++i)
			points2D[i].index = i;
	}

	//try to get the points on the convex/concave hull to build the contour and the polygon
	std::list<CCLib::PointProjectionTools::IndexedCCVector2*> hullPoints;
	if (!CCLib::PointProjectionTools::extractConcaveHull2D(	points2D,
															hullPoints,
															maxEdgelLength*maxEdgelLength) )
	{
		ccLog::Error("[ccPolyline::ExtractFlatContour] Failed to compute the convex hull of the input points!");
	}

	unsigned hullPtsCount = static_cast<unsigned>(hullPoints.size());

	//create vertices
	ccPointCloud* contourVertices = new ccPointCloud();
	{
		if (!contourVertices->reserve(hullPtsCount))
		{
			delete contourVertices;
			contourVertices = 0;
			ccLog::Error("[ccPolyline::ExtractFlatContour] Not enough memory!");
			return 0;
		}

		//projection on the LS plane (in 3D)
		for (std::list<CCLib::PointProjectionTools::IndexedCCVector2*>::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
			contourVertices->addPoint(O + X*(*it)->x + Y*(*it)->y);
		contourVertices->setName("vertices");
		contourVertices->setEnabled(false);
	}

	//we create the corresponding (3D) polyline
	ccPolyline* contourPolyline = new ccPolyline(contourVertices);
	if (contourPolyline->reserve(hullPtsCount))
	{
		contourPolyline->addPointIndex(0,hullPtsCount);
		contourPolyline->setClosed(true);
		contourPolyline->setVisible(true);
		contourPolyline->setName("contour");
		contourPolyline->addChild(contourVertices);
	}
	else
	{
		delete contourPolyline;
		contourPolyline = 0;
		ccLog::Warning("[ccPolyline::ExtractFlatContour] Not enough memory to create the contour polyline!");
	}

	return contourPolyline;
}

PointCoordinateType ccPolyline::computeLength() const
{
	PointCoordinateType length = 0;

	unsigned vertCount = size();
	if (vertCount > 1 && m_theAssociatedCloud)
	{
		unsigned lastVert = isClosed() ? vertCount : vertCount-1;
		for (unsigned i=0; i<lastVert; ++i)
		{
			CCVector3 A;
			getPoint(i,A);
			CCVector3 B;
			getPoint((i+1)%vertCount,B);

			length += (B-A).norm();
		}
	}

	return length;
}

unsigned ccPolyline::getUniqueIDForDisplay() const
{
	if (m_parent && m_parent->getParent() && m_parent->getParent()->isA(CC_TYPES::FACET))
		return m_parent->getParent()->getUniqueID();
	else
		return getUniqueID();
}
