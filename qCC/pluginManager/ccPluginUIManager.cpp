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
#include "ccGLPluginInterface.h"
#include "ccGLWindow.h"
#include "ccIOPluginInterface.h"
#include "ccMainAppInterface.h"
#include "ccPluginInfoDlg.h"
#include "ccPluginManager.h"
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

void ccPluginUIManager::init()
{
	auto plugins = ccPluginManager::get().pluginList();
	
	m_pluginMenu->setEnabled( false );
	m_glFilterMenu->setEnabled( false );
	
	m_mainPluginToolbar->setVisible( false );
		
	QVector<ccStdPluginInterface *>	coreStdPlugins;
	QVector<ccStdPluginInterface *>	thirdPartyStdPlugins;
	
	QVector<QAction *>	coreGLActions;
	QVector<QAction *>	thirdPartyGLActions;
	
	for ( ccPluginInterface *plugin : plugins )
	{
		if ( plugin == nullptr )
		{
			Q_ASSERT( false );
			continue;
		}
		
		if ( !ccPluginManager::get().isEnabled( plugin ) )
		{
			m_plugins.push_back( plugin );
			
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
				ccStdPluginInterface *stdPlugin = static_cast<ccStdPluginInterface*>( plugin );
				
				stdPlugin->setMainAppInterface( m_appInterface );
								
				if ( stdPlugin->isCore() )
				{
					coreStdPlugins.append( stdPlugin );
				}
				else
				{
					thirdPartyStdPlugins.append( stdPlugin );
				}
				
				m_plugins.push_back( stdPlugin );
								
				break;
			}
				
			case CC_GL_FILTER_PLUGIN: //GL filter
			{
				ccGLPluginInterface *glPlugin = static_cast<ccGLPluginInterface*>( plugin );
								
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
				
				if ( glPlugin->isCore() )
				{
					coreGLActions.append( action );
				}
				else
				{
					thirdPartyGLActions.append( action );
				}	

				m_plugins.push_back( glPlugin );
				break;
			}
				
			case CC_IO_FILTER_PLUGIN:
			{
				ccIOPluginInterface *ioPlugin = static_cast<ccIOPluginInterface*>( plugin );

				// there are no menus or toolbars for I/O plugins
				
				m_plugins.push_back( ioPlugin );
				break;
			}	
		}
	}
	
	// add core standard plugins to menu & tool bar
	for ( ccStdPluginInterface *plugin : coreStdPlugins )
	{		
		QList<QAction *> actions = plugin->getActions();
		
		addActionsToMenu( plugin, actions );
		addActionsToToolBar( plugin, actions );

		plugin->onNewSelection( m_appInterface->getSelectedEntities() );
	}
	
	// add 3rd standard party plugins to menu & tool bar (if any )
	if ( !thirdPartyStdPlugins.isEmpty() )
	{
		m_pluginMenu->addSection( "3rd Party" );
		
		for ( ccStdPluginInterface *plugin : thirdPartyStdPlugins )
		{			
			QList<QAction *> actions = plugin->getActions();
			
			addActionsToMenu( plugin, actions );
			addActionsToToolBar( plugin, actions );

			plugin->onNewSelection( m_appInterface->getSelectedEntities() );
		}
	}
	
	// add core GL plugins to menu & tool bar
	for ( QAction *action : coreGLActions )
	{
		m_glFilterMenu->addAction( action );
		m_glFiltersToolbar->addAction( action );				
	}
	
	// add 3rd GL party plugins to menu & tool bar (if any )
	if ( !thirdPartyGLActions.isEmpty() )
	{
		m_glFilterMenu->addSection( "3rd Party" );
		
		for ( QAction *action : thirdPartyGLActions )
		{
			m_glFilterMenu->addAction( action );
			m_glFiltersToolbar->addAction( action );				
		}
	}
	
	m_pluginMenu->setEnabled( !m_pluginMenu->isEmpty() );
	
	if ( m_mainPluginToolbar->isEnabled() )
	{
		m_showPluginToolbar->setEnabled( true );
	}
	
	m_glFilterMenu->setEnabled( !m_glFilterMenu->isEmpty() );
	m_glFiltersToolbar->setEnabled( !m_glFilterMenu->isEmpty() );	// [sic] we have toolbar actions if we have them in the menu
	
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
	
	const auto &list = m_plugins;
	
	for ( ccPluginInterface *plugin : list )
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
	
	about.setPluginPaths( ccPluginManager::get().pluginPaths() );
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

void ccPluginUIManager::addActionsToMenu( ccStdPluginInterface *stdPlugin, const QList<QAction *> &actions )
{
	// If the plugin has more than one action we create its own menu
	if ( actions.size() > 1 )
	{
		QMenu	*menu = new QMenu( stdPlugin->getName(), m_parentWidget);
		
		menu->setIcon( stdPlugin->getIcon() );
		menu->setEnabled( true );
		
		for ( QAction* action : actions )
		{
			menu->addAction( action );
		}
		
		m_pluginMenu->addMenu( menu );
	}
	else // otherwise we just add it to the main menu
	{		
		Q_ASSERT( actions.count() == 1 );
		
		m_pluginMenu->addAction( actions.at( 0 ) );
	}
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

void ccPluginUIManager::addActionsToToolBar( ccStdPluginInterface *stdPlugin, const QList<QAction *> &actions )
{	
	const QString pluginName = stdPlugin->getName();
	
	// If the plugin has more than one action we create its own tool bar
	if ( actions.size() > 1 )
	{
		QToolBar *toolBar = new QToolBar( pluginName + QStringLiteral( " toolbar" ), m_parentWidget );
		
		if ( toolBar != nullptr )
		{
			m_additionalPluginToolbars.push_back( toolBar );

			toolBar->setObjectName( pluginName );
			toolBar->setEnabled( true );
			
			for ( QAction* action : actions )
			{
				toolBar->addAction( action );
			}
		}
	}
	else // otherwise we just add it to the main tool bar
	{		
		Q_ASSERT( actions.count() == 1 );
		
		m_mainPluginToolbar->addAction( actions.at( 0 ) );
	}
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
	
	ccGLPluginInterface	*plugin = action->data().value<ccGLPluginInterface *>();
	
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
