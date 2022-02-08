#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qMeshBoolean                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

//Qt
#include <QObject>

#include "ccStdPluginInterface.h"

class QAction;

//! Mes Boolean Operations (CSG) plugin
/** This plugin is based on ligIGL: https://libigl.github.io/
**/
class qMeshBoolean : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )

	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qMeshBoolean" FILE "../info.json" )

public:

	//! Default constructor
	explicit qMeshBoolean(QObject* parent = nullptr);

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual QList<QAction *> getActions() override;

protected:

	//! Starts main action
	void doAction();

protected:

	//! Associated action
	QAction* m_action;
};
