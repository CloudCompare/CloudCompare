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

//common
#include <ccPickingHub.h>

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>

ccPointPickingGenericInterface::ccPointPickingGenericInterface(ccPickingHub* pickingHub, QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, m_pickingHub(pickingHub)
{
	assert(m_pickingHub);
}

bool ccPointPickingGenericInterface::linkWith(ccGLWindow* win)
{
	if (win == m_associatedWin)
	{
		//nothing to do
		return false;
	}
	ccGLWindow* oldWin = m_associatedWin;

	//just in case
	if (m_pickingHub)
	{
		m_pickingHub->removeListener(this);
	}

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	//if the dialog is already linked to a window, we must disconnect the 'point picked' signal
	if (oldWin)
	{
		oldWin->disconnect(this);
	}

	return true;
}

bool ccPointPickingGenericInterface::start()
{
	if (!m_pickingHub)
	{
		ccLog::Error("[Point picking] No associated display!");
		return false;
	}

	//activate "point picking mode" in associated GL window
	if (!m_pickingHub->addListener(this, true, true, ccGLWindow::POINT_PICKING))
	{
		ccLog::Error("Picking mechanism already in use. Close the tool using it first.");
		return false;
	}

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->redraw(true, false);

	ccOverlayDialog::start();

	return true;
}

void ccPointPickingGenericInterface::stop(bool state)
{
	if (m_pickingHub)
	{
		//deactivate "point picking mode" in all GL windows
		m_pickingHub->removeListener(this);

		if ( m_associatedWin != nullptr )
		{
			m_associatedWin->setUnclosable(false);
			m_associatedWin->redraw(true, false);
		}
	}

	ccOverlayDialog::stop(state);
}

void ccPointPickingGenericInterface::onItemPicked(const PickedItem& pi)
{
	if (m_processing && pi.entity)
	{
		processPickedPoint(pi);
	}
}
