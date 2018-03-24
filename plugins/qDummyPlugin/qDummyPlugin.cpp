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

// First:
//	Replace all occurrences of 'qDummyPlugin' by your own plugin class name in this file!

// Second:
//	Open qDummyPlugin.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>

#include "qDummyPlugin.h"

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
qDummyPlugin::qDummyPlugin( QObject *parent ) :
	  QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qDummyPlugin/info.json" )
	, m_action( nullptr )
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
// For example: if none of the selected entities is a cloud, and your
// plugin deals only with clouds, call 'm_action->setEnabled( false )'
void qDummyPlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	// for example - only enable our action if something is selected
	m_action->setEnabled( !selectedEntities.empty() );
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
void qDummyPlugin::getActions( QActionGroup &group )
{
	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &qDummyPlugin::doAction );
	}

	group.addAction( m_action );
}

// This is an example of an action's method called when the corresponding action
// is triggered (i.e. the corresponding icon or menu entry is clicked in CC's
// main interface). You can access most of CC components (database,
// 3D views, console, etc.) via the 'm_app' variable (see the ccMainAppInterface
// class in ccMainAppInterface.h).
void qDummyPlugin::doAction()
{
	// m_app should have already been initialized by CC when plugin is loaded
	// (--> pure internal check)
	Q_ASSERT(m_app != nullptr);
	
	if ( !m_app )
	{
		return;
	}

	/*** HERE STARTS THE ACTION ***/

	// Put your code here
	// --> you may want to start by asking for parameters (with a custom dialog, etc.)

	// This is how you can output messages
	// Display a standard message in the console
	m_app->dispToConsole( "[qDummyPlugin] Hello world!", ccMainAppInterface::STD_CONSOLE_MESSAGE );
	
	// Display a warning message in the console
	m_app->dispToConsole( "[qDummyPlugin] Warning: dummy plugin shouldn't be used as is!", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
	
	// Display an error message in the console AND pop-up an error box
	m_app->dispToConsole( "Dummy plugin shouldn't be used as is!", ccMainAppInterface::ERR_CONSOLE_MESSAGE );

	/*** HERE ENDS THE ACTION ***/
}
