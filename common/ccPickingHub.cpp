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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "ccPickingHub.h"

//Qt
#include <QMdiSubWindow>
#include <QMessageBox>

//qCC_gl
#include <ccGLWidget.h>

//qCC_db
#include <ccSphere.h>

//Plugins
#include <ccMainAppInterface.h>

ccPickingHub::ccPickingHub(ccMainAppInterface* app, QObject* parent/*=0*/)
	: QObject(parent)
	, m_app(app)
	, m_activeGLWindow(nullptr)
	, m_pickingMode(ccGLWindow::POINT_OR_TRIANGLE_PICKING)
	, m_autoEnableOnActivatedWindow(true)
	, m_exclusive(false)
{
}

//void ccPickingHub::setPickingMode(ccGLWindow::PICKING_MODE mode, bool autoEnableOnActivatedWindow/*=true*/)
//{
//	m_pickingMode = mode;
//	m_autoEnableOnActivatedWindow = autoEnableOnActivatedWindow;
//}

void ccPickingHub::togglePickingMode(bool state)
{
	//ccLog::Warning(QString("Toggle picking mode: ") + (state ? "ON" : "OFF") + " --> " + (m_activeGLWindow ? QString("View ") + QString::number(m_activeGLWindow->getUniqueID()) : QString("no view")));
	if (m_activeGLWindow)
	{
		m_activeGLWindow->setPickingMode(state ? m_pickingMode : ccGLWindow::DEFAULT_PICKING);
	}
}

void ccPickingHub::onActiveWindowChanged(QMdiSubWindow* mdiSubWindow)
{
	ccGLWindow* glWindow = (mdiSubWindow ? GLWindowFromWidget(mdiSubWindow->widget()) : nullptr);
	//if (glWindow)
	//	ccLog::Warning("New active GL window: " + QString::number(glWindow->getUniqueID()));
	//else
	//	ccLog::Warning("No more active GL window");

	if (m_activeGLWindow == glWindow)
	{
		//nothing to do
		return;
	}

	if (m_activeGLWindow)
	{
		//take care of the previously linked window
		togglePickingMode(false);
		disconnect(m_activeGLWindow);
		m_activeGLWindow = nullptr;
	}

	if (glWindow)
	{
		//link this new window
		connect(glWindow, &ccGLWindow::itemPicked, this, &ccPickingHub::processPickedItem, Qt::UniqueConnection);
		connect(glWindow, &QObject::destroyed, this, &ccPickingHub::onActiveWindowDeleted);
		m_activeGLWindow = glWindow;

		if (m_autoEnableOnActivatedWindow && !m_listeners.empty())
		{
			togglePickingMode(true);
		}
	}
}

void ccPickingHub::onActiveWindowDeleted(QObject* obj)
{
	if (obj == m_activeGLWindow)
	{
		m_activeGLWindow = nullptr;
	}
}

void ccPickingHub::processPickedItem(ccHObject* entity, unsigned itemIndex, int x, int y, const CCVector3& P3D, const CCVector3d& uvw)
{
	if (m_listeners.empty())
	{
		return;
	}

	ccPickingListener::PickedItem item;
	{
		item.clickPoint = QPoint(x, y);
		item.entity = entity;
		item.itemIndex = itemIndex;
		item.P3D = P3D;
		item.uvw = uvw;

		if (entity && entity->isA(CC_TYPES::SPHERE))
		{
			//whether the center of sphere entities should be used when a point is picked on the surface
			static QMessageBox::StandardButton s_pickSphereCenter = QMessageBox::Yes;

			if (s_pickSphereCenter != QMessageBox::YesToAll && s_pickSphereCenter != QMessageBox::NoToAll)
			{
				s_pickSphereCenter = QMessageBox::question(m_activeGLWindow->asWidget(), tr("Sphere picking"), tr("From now on, do you want to pick sphere centers instead of a point on their surface?"), QMessageBox::YesToAll | QMessageBox::Yes | QMessageBox::No | QMessageBox::NoToAll, QMessageBox::YesToAll);
			}
			if (s_pickSphereCenter == QMessageBox::Yes || s_pickSphereCenter == QMessageBox::YesToAll)
			{
				//replace the input point by the sphere center
				item.P3D = static_cast<ccSphere*>(entity)->getOwnBB().getCenter();
				item.entityCenter = true;
			}
		}
	}

	//copy the list of listeners, in case the user call 'removeListener' in 'onItemPicked'
	std::set< ccPickingListener* > listeners = m_listeners;
	for (ccPickingListener* l : listeners)
	{
		if (l)
		{
			l->onItemPicked(item);
		}
	}
}

bool ccPickingHub::addListener(	ccPickingListener* listener,
								bool exclusive/*=false*/,
								bool autoStartPicking/*=true*/,
								ccGLWindow::PICKING_MODE mode/*=ccGLWindow::POINT_OR_TRIANGLE_PICKING*/)
{
	if (!listener)
	{
		assert(false);
		return false;
	}

	//if listeners are already registered
	if (!m_listeners.empty())
	{
		if (m_exclusive) //a previous listener is exclusive
		{
			assert(m_listeners.size() == 1);
			if (m_listeners.find(listener) == m_listeners.end())
			{
				ccLog::Warning("[ccPickingHub::addListener] Exclusive listener already registered: stop the other tool relying on point picking first");
				return false;
			}
		}
		else if (exclusive) //this new listener is exclusive
		{
			if (m_listeners.size() > 1 || m_listeners.find(listener) == m_listeners.end())
			{
				ccLog::Warning("[ccPickingHub::addListener] Attempt to register an exclusive listener while other listeners are already registered");
				return false;
			}
		}
		else if (mode != m_pickingMode)
		{
			if (m_listeners.size() > 1 || m_listeners.find(listener) == m_listeners.end())
			{
				ccLog::Warning("[ccPickingHub::addListener] Other listeners are already registered with a different picking mode");
				return false;
			}
		}
	}
	
	try
	{
		m_listeners.insert(listener);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[ccPickingHub::addListener] Not enough memory");
		return false;
	}

	m_exclusive = exclusive;
	m_pickingMode = mode;

	if (autoStartPicking)
	{
		togglePickingMode(true);
	}

	return true;
}

void ccPickingHub::removeListener(ccPickingListener* listener, bool autoStopPickingIfLast/*=true*/)
{
	m_listeners.erase(listener);

	if (m_listeners.empty())
	{
		m_exclusive = false; //auto drop the 'exclusive' flag
		togglePickingMode(false);
	}
}
