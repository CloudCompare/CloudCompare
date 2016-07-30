//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qAnimation                      #
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
//#             COPYRIGHT: Ryan Wicks, 2G Robotics Inc., 2015              #
//#                                                                        #
//##########################################################################

#include "qAnimation.h"

//Local
#include "qAnimationDlg.h"

//qCC_db
#include <cc2DViewportObject.h>

//Qt
#include <QtGui>
#include <QApplication>
#include <QMainWindow>

qAnimation::qAnimation(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qAnimation::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(!selectedEntities.empty());
}

void qAnimation::getActions(QActionGroup& group)
{
	//default action (if it has not been already created, it's the moment to do it)
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());

		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

//what to do when clicked.
void qAnimation::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	//get active GL window
	ccGLWindow* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		m_app->dispToConsole("No active 3D view!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get the selected viewpots
	std::vector<cc2DViewportObject*> selectedViewports;
	try
	{
		const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
		for (ccHObject::Container::const_iterator entity_iterator = selectedEntities.begin(); entity_iterator != selectedEntities.end(); ++entity_iterator)
		{
			if ((*(entity_iterator))->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT)
			{
				selectedViewports.push_back(static_cast<cc2DViewportObject*>(*entity_iterator));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		m_app->dispToConsole("Not enough memory!");
		return;
	}

	//we need at least two viewports!
	if (selectedViewports.size() < 2)
	{
		m_app->dispToConsole("Animation plugin requires at least two selected viewports to function!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	m_app->dispToConsole(QString("[qAnimation] Selected viewports: %1").arg(selectedViewports.size()));

	qAnimationDlg videoDlg(glWindow, m_app->getMainWindow());
	if (!videoDlg.init(selectedViewports))
	{
		m_app->dispToConsole("Failed to initialize the plugin dialog (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	videoDlg.exec();
}

QIcon qAnimation::getIcon() const
{
	return QIcon(":/CC/plugin/qAnimation/animation.png");
}
