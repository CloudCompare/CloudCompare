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
#include "ccMainAppInterface.h"
#include "ccPluginInfoDlg.h"
#include "ccPluginManager.h"


ccPluginManager::ccPluginManager( ccMainAppInterface *appInterface, QWidget *parent )
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

ccPluginManager::~ccPluginManager()
{
}

void ccPluginManager::init( const tPluginInfoList &plugins, const QStringList &pluginPaths )
{
	m_pluginPaths = pluginPaths;	
	
	m_pluginMenu->setEnabled( false );
	m_glFilterMenu->setEnabled( false );
	
	m_mainPluginToolbar->setVisible( false );
	
	bool haveStdPlugin = false;
	
	for ( const tPluginInfo &plugin : plugins )
	{
		if ( plugin.object == nullptr )
		{
			Q_ASSERT( false );
			continue;
		}
		
		QString pluginName = plugin.object->getName();
		
		if ( pluginName.isEmpty() )
		{
			ccLog::Warning( QStringLiteral( "[Plugin] Plugin '%1' has an invalid (empty) name!" ).arg( plugin.filename ) );
			continue;
		}
		
		switch ( plugin.object->getType() )
		{
			case CC_STD_PLUGIN: //standard plugin
			{
				haveStdPlugin = true;
				
				plugin.qObject->setParent( this );
				
				ccStdPluginInterface *stdPlugin = static_cast<ccStdPluginInterface*>( plugin.object );
				
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
				ccGLFilterPluginInterface *glPlugin = static_cast<ccGLFilterPluginInterface*>( plugin.object );
				
				plugin.qObject->setParent( this );
				
				QAction* action = new QAction( pluginName, plugin.qObject );
				action->setToolTip( glPlugin->getDescription() );
				action->setIcon( glPlugin->getIcon() );
				action->setCheckable( true );
				
				connect( action, &QAction::triggered, this, &ccPluginManager::enableGLFilter );

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
				ccIOFilterPluginInterface *ioPlugin = static_cast<ccIOFilterPluginInterface*>( plugin.object );

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

QMenu *ccPluginManager::pluginMenu() const
{
	return m_pluginMenu;
}

QMenu *ccPluginManager::shaderAndFilterMenu() const
{
	return m_glFilterMenu;
}

QToolBar *ccPluginManager::mainPluginToolbar()
{
	return m_mainPluginToolbar;
}

QList<QToolBar *> &ccPluginManager::additionalPluginToolbars()
{
	return m_additionalPluginToolbars;
}

QAction *ccPluginManager::actionShowMainPluginToolbar()
{
	return m_showPluginToolbar;
}

QToolBar *ccPluginManager::glFiltersToolbar()
{
	return m_glFiltersToolbar;
}

QAction *ccPluginManager::actionShowGLFilterToolbar()
{
	return m_showGLFilterToolbar;
}

void ccPluginManager::updateMenus()
{
	ccGLWindow *active3DView = m_appInterface->getActiveGLWindow();
	const bool hasActiveView = (active3DView != nullptr);

	const QList<QAction*> actionList = m_glFilterActions.actions();
	
	for ( QAction* action : actionList )
	{
		action->setEnabled( hasActiveView );
	}
}

void ccPluginManager::handleSelectionChanged()
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

void ccPluginManager::showAboutDialog() const
{
	ccPluginInfoDlg	about;
	
	about.setPluginPaths( m_pluginPaths );
	about.setPluginList( m_plugins );
	
	about.exec();
}

void ccPluginManager::setupActions()
{
	m_actionRemoveFilter = new QAction( QIcon( ":/CC/images/noFilter.png" ), tr( "Remove Filter" ), this );
	m_actionRemoveFilter->setEnabled( false );
	
	connect( m_actionRemoveFilter, &QAction::triggered, this, &ccPluginManager::disableGLFilter );
	
	m_showPluginToolbar = new QAction( tr( "Plugins" ), this );
	m_showPluginToolbar->setCheckable( true );
	m_showPluginToolbar->setEnabled( false );
	
	m_showGLFilterToolbar = new QAction( tr( "GL Filters" ), this );
	m_showGLFilterToolbar->setCheckable( true );
	m_showGLFilterToolbar->setEnabled( false );
}

void ccPluginManager::setupMenus()
{	
	m_pluginMenu = new QMenu( tr( "Plugins" ), m_parentWidget );
	
	m_glFilterMenu = new QMenu( tr( "Shaders && Filters NEW" ), m_parentWidget );
	
	m_glFilterMenu->addAction( m_actionRemoveFilter );
	
	m_glFilterActions.setExclusive( true );		
}

void ccPluginManager::setupToolbars()
{
	m_mainPluginToolbar = new QToolBar( tr( "Plugins" ), m_parentWidget );
	
	m_mainPluginToolbar->setObjectName( QStringLiteral( "Main Plugin Toolbar" ) );
	
	connect( m_showPluginToolbar, &QAction::toggled, m_mainPluginToolbar, &QToolBar::setVisible );
	
	m_glFiltersToolbar = new QToolBar( tr( "GL Filters" ), m_parentWidget );
	
	m_glFiltersToolbar->setObjectName( QStringLiteral( "GL Plugin Toolbar" ) );
	m_glFiltersToolbar->addAction( m_actionRemoveFilter );	

	connect( m_showGLFilterToolbar, &QAction::toggled, m_glFiltersToolbar, &QToolBar::setVisible );
}

void ccPluginManager::enableGLFilter()
{
	ccGLWindow *win = m_appInterface->getActiveGLWindow();
	
	if ( win == nullptr )
	{
		ccLog::Warning( "[GL filter] No active 3D view" );
		return;
	}
	
	QAction *action = qobject_cast<QAction*>(sender());
	
	ccPluginInterface *ccPlugin = ccPlugins::ToValidPlugin( action ? action->parent() : nullptr );
	
	if ( (ccPlugin == nullptr) || (ccPlugin->getType() != CC_GL_FILTER_PLUGIN) )
	{
		return;
	}
	
	if ( win->areGLFiltersEnabled() )
	{
		ccGlFilter *filter = static_cast<ccGLFilterPluginInterface*>(ccPlugin)->getFilter();
		
		if ( filter != nullptr )
		{
			win->setGlFilter(filter);
			
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

void ccPluginManager::disableGLFilter()
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
