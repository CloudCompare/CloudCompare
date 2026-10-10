//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: FFDPlugin                       #
//#           Free Form Deformation - Non-rigid Transformation             #
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
//##########################################################################

#pragma once

#include "ccStdPluginInterface.h"

class QAction;

//! Free Form Deformation Plugin
class FFDPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.FFD" FILE "../info.json" )

public:
	explicit FFDPlugin( QObject *parent = nullptr );
	~FFDPlugin() override = default;

	// ccStdPluginInterface methods
	void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	QList<QAction*> getActions() override;

private:
	void createAction();

	QAction *m_deformAction = nullptr;
};
