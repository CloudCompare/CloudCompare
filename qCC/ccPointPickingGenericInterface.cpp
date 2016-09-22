//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
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

bool ccPointPickingGenericInterface::linkWith(ccGLWindow* win)
{
	if (win == m_associatedWin)
	{
		//nothing to do
		return false;
	}
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	//if the dialog is already linked to a window, we must disconnect the 'point picked' signal
	if (oldWin)
	{
		oldWin->disconnect(this);
	}
	//then we can connect the new window 'point picked' signal
	if (m_associatedWin)
	{
		connect(m_associatedWin, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(handlePickedItem(ccHObject*, unsigned, int, int, const CCVector3&)));
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
	m_associatedWin->redraw(true, false);

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
		m_associatedWin->redraw(true, false);
	}

	ccOverlayDialog::stop(state);
}

void ccPointPickingGenericInterface::handlePickedItem(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3& P)
{
	if (!m_processing || !entity)
		return;

	ccPointCloud* cloud = 0;

	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		cloud = static_cast<ccPointCloud*>(entity);
		if (!cloud)
		{
			assert(false);
			ccLog::Warning("[Item picking] Picked point is not in pickable entities DB?!");
			return;
		}
		processPickedPoint(cloud, itemIdx, x, y);
	}
	else if (entity->isKindOf(CC_TYPES::MESH))
	{
		//NOT HANDLED: 'POINT_PICKING' mode only for now
		assert(false);
	}
	else
	{
		//unhandled entity
		assert(false);
	}
}
