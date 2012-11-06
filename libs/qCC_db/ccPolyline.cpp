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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2265                                                              $
//$LastChangedDate:: 2012-10-13 22:22:51 +0200 (sam., 13 oct. 2012)        $
//**************************************************************************
//

#include "ccIncludeGL.h"

#include "ccPolyline.h"
#include "ccPointCloud.h"

ccPolyline::ccPolyline(GenericIndexedCloudPersist* associatedCloud)
	: Polyline(associatedCloud)
	, ccHObject("Polyline")
{
	set2DMode(false);
	setForeground(true);
    setVisible(true);
    lockVisibility(false);
}

ccPolyline::ccPolyline(const ccPolyline& poly)
	: Polyline(static_cast<CCLib::GenericIndexedCloudPersist*>(new ccPointCloud(poly.getAssociatedCloud())))
	, ccHObject("Polyline")
{
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
