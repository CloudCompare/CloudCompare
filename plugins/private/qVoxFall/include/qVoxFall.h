//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
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
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#ifndef Q_VOXFALL_PLUGIN_HEADER
#define Q_VOXFALL_PLUGIN_HEADER

#include "ccStdPluginInterface.h"


//qCC_db
#include <ccHObject.h>


class qVoxFall : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qVoxFall" FILE "../info.json" )

public:
	explicit qVoxFall( QObject *parent = nullptr );
	~qVoxFall() override = default;

	// Inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	QList<QAction *> getActions() override;

private:
	
	void doAction();

	//! Default action
	QAction* m_action;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //Q_VOXFALL_PLUGIN_HEADER
