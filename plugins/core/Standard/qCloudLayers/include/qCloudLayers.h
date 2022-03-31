#pragma once

//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qCloudLayers                    #
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
//#                     COPYRIGHT: WigginsTech 2022                        #
//#                                                                        #
//##########################################################################

#include <ccStdPluginInterface.h>

class ccCloudLayersDlg;

class qCloudLayers : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qCloudLayers" FILE "../info.json" )

public:
	explicit qCloudLayers( QObject* parent = nullptr );
	~qCloudLayers() override = default;

	// Inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container& selectedEntities ) override;
	QList<QAction*> getActions() override;

protected:

	//! Slot called when associated action is triggered
	void doAction();

private:
	//! Default action
	QAction* m_action;

	ccCloudLayersDlg* m_cloudLayersDlg;
};

