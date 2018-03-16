//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qDummy                      #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#ifndef Q_DUMMY_PLUGIN_HEADER
#define Q_DUMMY_PLUGIN_HEADER

//qCC
#include "../ccStdPluginInterface.h"

//! Dummy qCC plugin
/** Replace the 'qDummyPlugin' string by your own plugin class name
	and then check 'qDummyPlugin.cpp' for more directions (you
	have to fill-in the blank methods. The most important one is the
	'getActions' method.  This method should return all actions
	(QAction objects). CloudCompare will automatically add them to an
	icon in the plugin toolbar and to an entry in the plugin menu
	(if your plugin returns several actions, CC will create a dedicated
	toolbar and sub-menu). 
	You are responsible to connect these actions to custom slots of your
	plugin.
	Look at the ccStdPluginInterface::m_app attribute to get access to
	most of CC components (database, 3D views, console, etc.).
**/
class qDummyPlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	
	// replace qDummy by the plugin name (IID should be unique - let's hope your plugin name is unique ;)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qDummy")

public:

	//! Default constructor
	explicit qDummyPlugin( QObject* parent = nullptr );

	// inherited from ccPluginInterface
	virtual QString getName() const override;
	virtual QString getDescription() const override;
	virtual QIcon getIcon() const override;

	// inherited from ccStdPluginInterface
	virtual void onNewSelection( const ccHObject::Container& selectedEntities ) override;
	virtual void getActions( QActionGroup& group ) override;

private:

	/*** ADD YOUR CUSTOM ACTIONS HERE ***/
	void doAction();

	//! Default action
	/** You can add as many actions as you want in a plugin.
		All actions will correspond to an icon in the dedicated
		toolbar and an entry in the plugin menu.
	**/
	QAction* m_action;
};

#endif
