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
}

ccPolyline::ccPolyline(const ccPolyline& poly)
	: Polyline(ccPointCloud::From(poly.getAssociatedCloud()))
	, ccHObject("Polyline")
{
	assert(m_theAssociatedCloud);
	if (m_theAssociatedCloud)
		addPointIndex(0,m_theAssociatedCloud->size());
	setClosingState(poly.m_isClosed);
	set2DMode(poly.m_mode2D);
	setForeground(poly.m_foreground);
	setVisible(poly.isVisible());
	lockVisibility(poly.isVisiblityLocked());
	setColor(poly.m_rgbColor);
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
    emptyBox.setValidity(true);
    return emptyBox;
}

bool ccPolyline::hasColors() const
{
    return true;
}

void ccPolyline::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (size() < 2)
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

        glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);

		unsigned count=size();
		for (unsigned i=0;i<count;++i)
			glVertex3fv(m_theAssociatedCloud->getPoint(m_theIndexes->getValue(i))->u);

        glEnd();
    }
}

void ccPolyline::setColor(const colorType col[])
{
	memcpy(m_rgbColor,col,sizeof(colorType)*3);
}

const colorType* ccPolyline::getColor() const
{
	return m_rgbColor;
}

bool ccPolyline::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//we can't save the associated cloud here (as it may be shared by mutliple polylines)
	//so instead we save it's unique ID (dataVersion>=28)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_theAssociatedCloud);
	if (!vertices)
	{
		ccLog::Warning("[ccPolyline::toFile_MeOnly] Polyline vertices is not a ccPointCloud structure?!");
		return false;
	}
	uint32_t vertUniqueID = (m_theAssociatedCloud ? (uint32_t)vertices->getUniqueID() : 0);
	if (out.write((const char*)&vertUniqueID,4)<0)
		return WriteError();

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = size();
	if (out.write((const char*)&pointCount,4)<0)
		return WriteError();

	//points (references to) (dataVersion>=28)
	for (uint32_t i=0; i<pointCount; ++i)
	{
		uint32_t pointIndex = getPointGlobalIndex(i);
		if (out.write((const char*)&pointIndex,4)<0)
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

	return true;
}

bool ccPolyline::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccHObject::fromFile_MeOnly(in,dataVersion))
		return false;

	if (dataVersion<28)
		return false;

	//as the associated cloud (=vertices) can't be saved directly (as it may be shared by mutliple polylines)
	//we only store its unique ID (dataVersion>=28) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID,4)<0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_associatedCloud' pointer!!!
	*(uint32_t*)(&m_theAssociatedCloud) = vertUniqueID;

	//number of points (references to) (dataVersion>=28)
	uint32_t pointCount = 0;
	if (in.read((char*)&pointCount,4)<0)
		return ReadError();
	if (!reserve(pointCount))
		return false;

	//points (references to) (dataVersion>=28)
	for (uint32_t i=0; i<pointCount; ++i)
	{
		uint32_t pointIndex = 0;
		if (in.read((char*)&pointIndex,4)<0)
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

	return true;
}
