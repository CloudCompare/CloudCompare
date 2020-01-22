//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScriptingPlugin                      #
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
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################


#include <QtGui>

#include "ChaiScriptingPlugin.h"

#include "extern/ChaiScript/include/chaiscript/chaiscript.hpp"
#include "extern/ChaiScript/include/chaiscript/chaiscript_stdlib.hpp"
//#include "extern/ChaiScript/include/chaiscript/dispatchkit/bootstrap_stl.hpp"
//#include "extern/ChaiScript/include/chaiscript/dispatchkit/function_call.hpp"
chaiscript::ChaiScript* chai;
ChaiScriptingPlugin::ChaiScriptingPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/ChaiScriptingPlugin/info.json" )
	, m_action( nullptr )
{
	using namespace chaiscript;
	chai = new ChaiScript(chaiscript::Std_Lib::library());
	chai->add_global(var(this), "chaiScriptingPlugin");
	chai->add(fun(&ChaiScriptingPlugin::calledAction), "calledAction");
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void ChaiScriptingPlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	// If you need to check for a specific type of object, you can use the methods
	// in ccHObjectCaster.h or loop and check the objects' classIDs like this:
	//
	//	for ( ccHObject *object : selectedEntities )
	//	{
	//		if ( object->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
	//		{
	//			// ... do something with the viewports
	//		}
	//	}
	
	// For example - only enable our action if something is selected.
	m_action->setEnabled(true);// !selectedEntities.empty() );
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> ChaiScriptingPlugin::getActions()
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
		connect( m_action, &QAction::triggered, this, &ChaiScriptingPlugin::doAction );
	}

	return { m_action };
}

// This is an example of an action's method called when the corresponding action
// is triggered (i.e. the corresponding icon or menu entry is clicked in CC's
// main interface). You can access most of CC's components (database,
// 3D views, console, etc.) via the 'm_app' variable (see the ccMainAppInterface
// class in ccMainAppInterface.h).
void ChaiScriptingPlugin::doAction()
{	
	if ( m_app == nullptr )
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT( false );
		
		return;
	}
	int i = chai->eval<int>("5+5");
	if(i==10)
		m_app->dispToConsole("Evaled 5+5 == 10 in chai", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	try {
	chai->eval(R"(chaiScriptingPlugin.calledAction();)");

	}
	catch (...)
	{
		m_app->dispToConsole("Chai Eval Failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
	
}


void ChaiScriptingPlugin::calledAction()
{
	m_app->dispToConsole("Called from Chai Eval", ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

ChaiScriptingPlugin::~ChaiScriptingPlugin()
{
	try
	{
		delete chai;
	}
	catch (...)
	{
			
	}
}