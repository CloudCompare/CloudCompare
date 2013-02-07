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
//#               COPYRIGHT: XXX                                           #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#ifndef Q_DUMMY_PLUGIN_HEADER
#define Q_DUMMY_PLUGIN_HEADER

#include <QObject>

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

public:

	//! Default constructor
	qDummyPlugin(QObject* parent=0);

    //inherited from ccPluginInterface
	virtual QString getName() const { return "qDummyPlugin"; }
	virtual QString getDescription() const { return "Dummy plugin (add description here)"; }
    virtual QIcon getIcon() const;

    //inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container& selectedEntities);
    virtual void getActions(QActionGroup& group);

protected slots:

	/*** ADD YOUR CUSTOM ACTIONS' SLOTS HERE ***/
	void doAction();

protected:

	//! Default action
	/** You can add as many actions as you want in a plugin.
		All actions will correspond to an icon in the dedicated
		toolbar and an entry in the plugin menu.
	**/
	QAction* m_action;
};

#endif
