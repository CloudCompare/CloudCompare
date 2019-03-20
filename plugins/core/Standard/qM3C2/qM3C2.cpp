//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2.h"

//local
#include "qM3C2Tools.h"
#include "qM3C2Dialog.h"
#include "qM3C2DisclaimerDialog.h"
#include "qM3C2Commands.h"
#include "qM3C2Process.h"

//qCC_db
#include <ccPointCloud.h>


qM3C2Plugin::qM3C2Plugin(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qM3C2Plugin/info.json" )
	, m_action(nullptr)
{
}

void qM3C2Plugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		m_action->setEnabled(	selectedEntities.size() == 2
							&&	selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
							&&	selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD) );
	}

	m_selectedEntities = selectedEntities;
}

QList<QAction *> qM3C2Plugin::getActions()
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		connect(m_action, &QAction::triggered, this, &qM3C2Plugin::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qM3C2Plugin::doAction()
{
	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (m_selectedEntities.size() != 2
		|| !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
		|| !m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select two point clouds!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud1 = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	ccPointCloud* cloud2 = ccHObjectCaster::ToPointCloud(m_selectedEntities[1]);

	//display dialog
	qM3C2Dialog dlg(cloud1, cloud2, m_app);
	if (!dlg.exec())
	{
		//process cancelled by the user
		return;
	}

	QString errorMessage;
	ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
	if (!qM3C2Process::Compute(dlg, errorMessage, outputCloud, true, m_app->getMainWindow(), m_app))
	{
		m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	//'Compute' may change some parameters of the dialog
	dlg.saveParamsToPersistentSettings();
}

void qM3C2Plugin::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandM3C2));
}
