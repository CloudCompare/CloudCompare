//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBroom                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      COPYRIGHT: Wesley Grimes (Collision Engineering Associates)       #
//#                                                                        #
//##########################################################################

#include "qBroom.h"
#include "qBroomDlg.h"
#include "qBroomDisclaimerDialog.h"

//Qt
#include <QtGui>

//qCC_db
#include <ccPointCloud.h>

//qCC
#include <ccGLWindow.h>

//system
#include <assert.h>

qBroom::qBroom(QObject* parent)
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qBroom/info.json" )
	, m_action( nullptr )
{
}

QList<QAction *> qBroom::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qBroom::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qBroom::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		//a single point cloud must be selected
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
	}
}

void qBroom::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
	{
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	if ( !m_app->haveOneSelection() || !selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(selectedEntities.front());

	qBroomDlg broomDlg(m_app);
	
	//the widget should be visible before we add the cloud
	broomDlg.show();
	QCoreApplication::processEvents();
	
	//automatically deselect the input cloud
	m_app->setSelectedInDB(cloud, false);

	if (broomDlg.setCloud(cloud))
	{
		broomDlg.exec();
	}

	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
