//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QAction>
#include <QMenu>
#include <QToolBar>
#include <QWidget>

#include "ccConsole.h"
#include "ccGLWindow.h"
#include "ccGLFilterPluginInterface.h"
#include "ccIOFilterPluginInterface.h"
#include "ccMainAppInterface.h"
#include "ccPluginInfoDlg.h"
#include "ccPluginUIManager.h"
#include "ccStdPluginInterface.h"


ccPluginUIManager::ccPluginUIManager( ccMainAppInterface *appInterface, QWidget *parent )
	: QObject( parent )
	, m_parentWidget( parent )
	, m_appInterface( appInterface )
	, m_pluginMenu( nullptr )
	, m_glFilterMenu( nullptr )
	, m_actionRemoveFilter( nullptr )
	, m_glFilterActions( this )
	, m_mainPluginToolbar( nullptr )
	, m_showPluginToolbar( nullptr )
	, m_glFiltersToolbar( nullptr )
	, m_showGLFilterToolbar( nullptr )
{
	setupActions();
	setupMenus();
	setupToolbars();
}

ccPluginUIManager::~ccPluginUIManager()
{
}

void ccPluginUIManager::init( const ccPluginInterfaceList &plugins )
{	
	m_pluginMenu->setEnabled( false );
	m_glFilterMenu->setEnabled( false );
	
	m_mainPluginToolbar->setVisible( false );
	
	bool haveStdPlugin = false;
	
	for ( ccPluginInterface *plugin : plugins )
	{
		if ( plugin == nullptr )
		{
			Q_ASSERT( false );
			continue;
		}
		
		const QString pluginName = plugin->getName();
		
		Q_ASSERT( !pluginName.isEmpty() );
		
		if ( pluginName.isEmpty() )
		{
			// should be unreachable - we have already checked for this in ccPlugins::Find()
			continue;
		}
		
		switch ( plugin->getType() )
		{
			case CC_STD_PLUGIN: //standard plugin
			{
				haveStdPlugin = true;
								
				ccStdPluginInterface *stdPlugin = static_cast<ccStdPluginInterface*>( plugin );
				
				stdPlugin->setMainAppInterface( m_appInterface );
				
				QMenu *menu = m_pluginMenu;
				QToolBar *toolBar = m_mainPluginToolbar;
				
				QActionGroup actions( this );
				stdPlugin->getActions( actions );
				
				if ( actions.actions().size() > 1 ) //more than one action? We create it's own menu and toolbar
				{
					menu = m_pluginMenu->addMenu( pluginName );
					
					if ( menu != nullptr )
					{
						menu->setIcon( stdPlugin->getIcon() );
					}
					
					toolBar = new QToolBar( pluginName + QStringLiteral( " toolbar" ), m_parentWidget );
					
					if ( toolBar != nullptr )
					{
						m_additionalPluginToolbars.push_back( toolBar );

						toolBar->setObjectName( pluginName );
					}
				}
				
				Q_ASSERT( menu != nullptr );
				
				// add actions to menu and toolbar
				const QList<QAction *>	actionList = actions.actions();
				
				for ( QAction* action : actionList )
				{
					menu->addAction( action );
					menu->setEnabled( true );
					
					toolBar->addAction( action );
					toolBar->setEnabled( true );
				}
				
				m_plugins.push_back( stdPlugin );
				
				stdPlugin->onNewSelection( m_appInterface->getSelectedEntities() );
				
				break;
			}
				
			case CC_GL_FILTER_PLUGIN: //GL filter
			{
				ccGLFilterPluginInterface *glPlugin = static_cast<ccGLFilterPluginInterface*>( plugin );
								
				QAction* action = new QAction( pluginName, this );
				
				action->setToolTip( glPlugin->getDescription() );
				action->setIcon( glPlugin->getIcon() );
				action->setCheckable( true );
				
				// store the plugin's interface pointer in the QAction data so we can access it in enableGLFilter()
				QVariant v;
		  
				v.setValue( glPlugin );
		  
				action->setData( v );
				
				connect( action, &QAction::triggered, this, &ccPluginUIManager::enableGLFilter );

				m_glFilterActions.addAction( action );
				
				m_glFilterMenu->addAction( action );
				m_glFilterMenu->setEnabled( true );
				
				m_glFiltersToolbar->addAction( action );				
				m_glFiltersToolbar->setEnabled( true );				

				m_plugins.push_back( glPlugin );
				break;
			}
				
			case CC_IO_FILTER_PLUGIN:
			{
				ccIOFilterPluginInterface *ioPlugin = static_cast<ccIOFilterPluginInterface*>( plugin );

				// there are no menus or toolbars for I/O plugins
				
				m_plugins.push_back( ioPlugin );
				break;
			}	
		}
	}
	
	m_pluginMenu->setEnabled( haveStdPlugin );
	
	if ( m_mainPluginToolbar->isEnabled() )
	{
		m_showPluginToolbar->setEnabled( true );
	}
	
	m_showPluginToolbar->setChecked( m_mainPluginToolbar->isEnabled() );
	
	if ( m_glFiltersToolbar->isEnabled() )
	{
		m_showGLFilterToolbar->setEnabled( true );
	}
	
	m_showGLFilterToolbar->setChecked( m_glFiltersToolbar->isEnabled() );
}

QMenu *ccPluginUIManager::pluginMenu() const
{
	return m_pluginMenu;
}

QMenu *ccPluginUIManager::shaderAndFilterMenu() const
{
	return m_glFilterMenu;
}

QToolBar *ccPluginUIManager::mainPluginToolbar()
{
	return m_mainPluginToolbar;
}

QList<QToolBar *> &ccPluginUIManager::additionalPluginToolbars()
{
	return m_additionalPluginToolbars;
}

QAction *ccPluginUIManager::actionShowMainPluginToolbar()
{
	return m_showPluginToolbar;
}

QToolBar *ccPluginUIManager::glFiltersToolbar()
{
	return m_glFiltersToolbar;
}

QAction *ccPluginUIManager::actionShowGLFilterToolbar()
{
	return m_showGLFilterToolbar;
}

void ccPluginUIManager::updateMenus()
{
	ccGLWindow *active3DView = m_appInterface->getActiveGLWindow();
	const bool hasActiveView = (active3DView != nullptr);

	const QList<QAction*> actionList = m_glFilterActions.actions();
	
	for ( QAction* action : actionList )
	{
		action->setEnabled( hasActiveView );
	}
}

void ccPluginUIManager::handleSelectionChanged()
{
	const ccHObject::Container &selectedEntities = m_appInterface->getSelectedEntities();
	
	for ( ccPluginInterface *plugin : m_plugins )
	{
		if ( plugin->getType() == CC_STD_PLUGIN )
		{
			ccStdPluginInterface	*stdPlugin = static_cast<ccStdPluginInterface *>(plugin);
			
			stdPlugin->onNewSelection( selectedEntities );
		}
	}
}

void ccPluginUIManager::showAboutDialog() const
{
	ccPluginInfoDlg	about;
	
	about.setPluginPaths( ccPluginManager::pluginPaths() );
	about.setPluginList( m_plugins );
	
	about.exec();
}

void ccPluginUIManager::setupActions()
{
	m_actionRemoveFilter = new QAction( QIcon( ":/CC/images/noFilter.png" ), tr( "Remove Filter" ), this );
	m_actionRemoveFilter->setEnabled( false );
	
	connect( m_actionRemoveFilter, &QAction::triggered, this, &ccPluginUIManager::disableGLFilter );
	
	m_showPluginToolbar = new QAction( tr( "Plugins" ), this );
	m_showPluginToolbar->setCheckable( true );
	m_showPluginToolbar->setEnabled( false );
	
	m_showGLFilterToolbar = new QAction( tr( "GL Filters" ), this );
	m_showGLFilterToolbar->setCheckable( true );
	m_showGLFilterToolbar->setEnabled( false );
}

void ccPluginUIManager::setupMenus()
{	
	m_pluginMenu = new QMenu( tr( "Plugins" ), m_parentWidget );
	
	m_glFilterMenu = new QMenu( tr( "Shaders && Filters" ), m_parentWidget );
	
	m_glFilterMenu->addAction( m_actionRemoveFilter );
	
	m_glFilterActions.setExclusive( true );		
}

void ccPluginUIManager::setupToolbars()
{
	m_mainPluginToolbar = new QToolBar( tr( "Plugins" ), m_parentWidget );
	
	m_mainPluginToolbar->setObjectName( QStringLiteral( "Main Plugin Toolbar" ) );
	
	connect( m_showPluginToolbar, &QAction::toggled, m_mainPluginToolbar, &QToolBar::setVisible );
	
	m_glFiltersToolbar = new QToolBar( tr( "GL Filters" ), m_parentWidget );
	
	m_glFiltersToolbar->setObjectName( QStringLiteral( "GL Plugin Toolbar" ) );
	m_glFiltersToolbar->addAction( m_actionRemoveFilter );	

	connect( m_showGLFilterToolbar, &QAction::toggled, m_glFiltersToolbar, &QToolBar::setVisible );
}

void ccPluginUIManager::enableGLFilter()
{
	ccGLWindow *win = m_appInterface->getActiveGLWindow();
	
	if ( win == nullptr )
	{
		ccLog::Warning( "[GL filter] No active 3D view" );
		return;
	}
	
	QAction *action = qobject_cast<QAction*>(sender());
	
	Q_ASSERT( action != nullptr );
	
	ccGLFilterPluginInterface	*plugin = action->data().value<ccGLFilterPluginInterface *>();
	
	if ( plugin == nullptr )
	{
		return;
	}
	
	if ( win->areGLFiltersEnabled() )
	{
		ccGlFilter *filter = plugin->getFilter();
		
		if ( filter != nullptr )
		{
			win->setGlFilter( filter );
			
			m_actionRemoveFilter->setEnabled( true );
			
			ccConsole::Print( "Note: go to << Display > Shaders & Filters > No filter >> to disable GL filter" );
		}
		else
		{
			ccConsole::Error( "Can't load GL filter (an error occurred)!" );
		}
	}
	else
	{
		ccConsole::Error( "GL filters not supported!" );
	}
}

void ccPluginUIManager::disableGLFilter()
{
	ccGLWindow *win = m_appInterface->getActiveGLWindow();
	
	if ( win != nullptr )
	{
		win->setGlFilter( nullptr );
		win->redraw( false );
		
		m_actionRemoveFilter->setEnabled( false );
		
		m_glFilterActions.checkedAction()->setChecked( false );
	}	
}
