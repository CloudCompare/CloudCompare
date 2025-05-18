//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: E57ImageExtractor                  #
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

#include <QtGui>

#include "qE57ImageExtractor.h"

#include "qE57ImageExtractorAction.h"


qE57ImageExtractor::E57ImageExtractor( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/E57ImageExtractor/info.json" )
	, m_action( nullptr )
{
}

qE57ImageExtractor::qE57ImageExtractor(QObject *parent)
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void qE57ImageExtractor::onNewSelection( const ccHObject::Container &selectedEntities )
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
	m_action->setEnabled( !selectedEntities.empty() );
    m_selectedEntities = selectedEntities;
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> qE57ImageExtractor::getActions()
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
		connect( m_action, &QAction::triggered, this, [this]()
		{
            qE57ImageExtractorAction::extractAndSaveImages( m_app , m_selectedEntities);
		});
	}

	return { m_action };
}
