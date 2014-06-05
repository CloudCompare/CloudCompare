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

#include "ccPointPickingGenericInterface.h"

//Local
#include "ccGLWindow.h"
#include "mainwindow.h"
#include "db_tree/ccDBRoot.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <cc2DLabel.h>

bool ccPointPickingGenericInterface::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
		return false;

	//if the dialog is already linked to a window, we must disconnect the 'point picked' signal
	if (oldWin && win != oldWin)
	{
		disconnect(oldWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(handlePickedPoint(int, unsigned, int, int)));
	}
	//then we can connect the new window 'point picked' signal
	if (m_associatedWin)
	{
		connect(m_associatedWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(handlePickedPoint(int, unsigned, int, int)));
	}

	return true;
}

bool ccPointPickingGenericInterface::start()
{
	if (!m_associatedWin)
	{
		ccLog::Error("[Point picking] No associated display!");
		return false;
	}

	//activate "point picking mode" in associated GL window
	m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->redraw();

	ccOverlayDialog::start();

	return true;
}

void ccPointPickingGenericInterface::stop(bool state)
{
	if (m_associatedWin)
	{
		//deactivate "point picking mode" in all GL windows
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		m_associatedWin->redraw();
	}

	ccOverlayDialog::stop(state);
}

void ccPointPickingGenericInterface::handlePickedPoint(int cloudID, unsigned pointIndex, int x, int y)
{
	if (!m_processing)
		return;

	ccPointCloud* cloud = 0;

	ccHObject* obj = MainWindow::TheInstance()->db()->find(cloudID);
	if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
		cloud = static_cast<ccPointCloud*>(obj);

	if (!cloud)
	{
		ccLog::Warning("[Point picking] Picked point is not in pickable entities DB?!");
		return;
	}

	const CCVector3* P = cloud->getPoint(pointIndex);
	if (P)
	{
		processPickedPoint(cloud, pointIndex, x, y);
	}
	else
	{
		ccLog::Warning("[Point picking] Invalid point index!");
	}
}
