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
//$Rev:: 2242                                                              $
//$LastChangedDate:: 2012-09-22 18:54:21 +0200 (sam., 22 sept. 2012)       $
//**************************************************************************
//

#include "ccPointPickingGenericInterface.h"

//Local
#include "ccGLWindow.h"
#include "ccConsole.h"
#include "mainwindow.h"
#include "db_tree/ccDBRoot.h"

//qCC_db
#include <ccPointCloud.h>
#include <cc2DLabel.h>

ccPointPickingGenericInterface::ccPointPickingGenericInterface(QWidget* parent) : QDialog(parent)
{
    m_win = 0;

    m_processing = false;

#ifndef CC_OPENGL_POINT_PICKING
    m_clouds.clear();
#endif
}

ccPointPickingGenericInterface::~ccPointPickingGenericInterface()
{
    if (m_processing)
        stop(false);

    linkWith(0);

#ifndef CC_OPENGL_POINT_PICKING
    m_clouds.clear();
#endif
}

#ifndef CC_OPENGL_POINT_PICKING
bool ccPointPickingGenericInterface::addCloud(const ccPointCloud* cloud)
{
    if (cloud->getDisplay() != m_win)
    {
        ccConsole::Warning(QString("[Point picking] Can't use cloud '%1' cause it's not displayed in the active 3D view!").arg(cloud->getName()));
        return false;
    }
    else if (!cloud->isVisible() || !cloud->isActive())
    {
        ccConsole::Warning(QString("[Point picking] Can't use cloud '%1' cause it's not visible!").arg(cloud->getName()));
        return false;
    }

    m_clouds.push_back(cloud);

    return true;
}

unsigned ccPointPickingGenericInterface::getDBSize() const
{
    return m_clouds.size();
}
#endif

void ccPointPickingGenericInterface::linkWith(ccGLWindow* win)
{
    if (m_processing)
    {
        ccConsole::Warning("[Point picking] Can't change associated window during picking!");
        return;
    }

    //if the dialog is already linked to a window
    if (m_win)
    {
        //same one? nothing to do!
        if (win == m_win)
            return;

        //ccConsole::Warning("[Point picking] Already linked to a window!");
        disconnect(m_win, 0, this, 0);
    }

    m_win = win;

    if (!m_win)
        return;

    #ifdef CC_OPENGL_POINT_PICKING
    connect(m_win,    SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(handlePickedPoint(int, unsigned, int, int)));
    #else
    connect(m_win,    SIGNAL(leftButtonClicked(int, int)), this, SLOT(handleClickedPixel(int, int)));
    #endif
}

bool ccPointPickingGenericInterface::start()
{
    if (!m_win)
    {
        ccConsole::Error("[Point picking] No associated display!");
        return false;
    }

#ifndef CC_OPENGL_POINT_PICKING
    if (m_clouds.empty())
    {
        ccConsole::Error("[Point picking] No pickable entities DB!");
        return false;
    }
#endif

#ifndef CC_OPENGL_POINT_PICKING
    for(unsigned i=0; i<m_clouds.size(); i++)
    {
        m_clouds[i]->showColors(false);
        m_clouds[i]->setSelected(false);
    }
#endif

    //activate "point picking mode" in associated GL window
    m_win->setPickingMode(ccGLWindow::POINT_PICKING);
    //the user must not close this window!
    m_win->setUnclosable(true);
    m_win->redraw();

    show();

    m_processing = true;

    return true;
}

void ccPointPickingGenericInterface::stop(bool state)
{
    if (!m_processing || !m_win)
        return;

    //deactivate "point picking mode" in all GL windows
    m_win->setPickingMode(ccGLWindow::DEFAULT_PICKING);
    m_win->setUnclosable(false);
    m_win->redraw();

    m_processing = false;

    hide();

    emit processFinished(state);
}

void ccPointPickingGenericInterface::handlePickedPoint(int cloudID, unsigned pointIndex, int x, int y)
{
    if (!m_processing)
        return;

    ccPointCloud* cloud = 0;

    #ifdef CC_OPENGL_POINT_PICKING
	ccHObject* obj = MainWindow::TheInstance()->db()->find(cloudID);
    if (obj->isKindOf(CC_POINT_CLOUD))
        cloud = static_cast<ccPointCloud*>(obj);
    #else
    for(unsigned i=0; i<m_clouds.size(); i++)
    {
        if (m_clouds[i]->getUniqueID() == cloudID)
        {
            cloud = m_clouds[i];
            break;
        }
    }
    #endif

    if (!cloud)
    {
        ccConsole::Warning("[Point picking] Picked point is not in pickable entities DB?!");
        return;
    }

    const CCVector3* P = cloud->getPoint(pointIndex);
    if (!P)
    {
        ccConsole::Warning("[Point picking] Invalid point index!");
        return;
    }

    processPickedPoint(cloud,pointIndex, x, y);
}

#ifndef CC_OPENGL_POINT_PICKING
void ccPointPickingGenericInterface::handleClickedPixel(int x, int y)
{
    if (!m_processing)
        return;

    //selected point
    unsigned pointIndex = 0;
    ccPointCloud* cloud = 0;

    unsigned i, j, bestCloud=0, bestIndex=0;
    DistanceType d, bestDistance, minDepth;
    PointCoordinateType *p;
    GLdouble a, b, c;
    const DistanceType maxDistance = 16.;
    GLdouble modelview[16], projection[16];
    GLint viewport[4];
    bool found=false;

    //Get matrices
    const double* modelview = m_win->getModelViewMatd();
	const double* projection = m_win->getProjectionMatd();

    m_win->getViewportArray(viewport);
    y = viewport[3]-y;

    minDepth = 1.;
    bestDistance = maxDistance;
    for(i=0; i<m_clouds.size(); i++)
    {
        for(j=0; j<m_clouds[i]->size(); j++)
        {
            p = m_clouds[i]->getPoint(j)->u;
            gluProject((GLdouble)p[0], (GLdouble)p[1], (GLdouble)p[2], modelview, projection, viewport, &a, &b, &c);
            d = (((int)a-x)*((int)a-x))+(((int)b-y)*((int)b-y));
            //ccConsole::Print("Picking [x=%d; y=%d; p={%f; %f; %f}; a=%f; b=%f; c=%f; d=%f]", x, y, p[0], p[1], p[2], a, b, c, d);
            if(d<bestDistance || (d==bestDistance && c < minDepth))
            {
                bestIndex = j;
                bestCloud = i;
                bestDistance = d;
                minDepth = c;
                found = true;
            }
        }
    }

    if (found)
    {
        CCVector3* P = m_clouds[bestCloud]->getPoint(bestIndex);
        processPickedPoint(m_clouds[bestCloud],bestIndex, x, y);
    }
}
#endif
