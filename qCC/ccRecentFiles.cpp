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
#include <QDir>
#include <QFile>
#include <QMenu>
#include <QSettings>
#include <QString>
#include <QStringList>

#include "ccRecentFiles.h"
#include "mainwindow.h"


QString ccRecentFiles::s_settingKey( "RecentFiles" );


ccRecentFiles::ccRecentFiles( QWidget *parent ) :
	QObject( parent )
{
	m_menu = new QMenu( tr("Open Recent..."), parent );
	
	m_actionClearMenu = new QAction( tr("Clear Menu"), this );
	
	connect( m_actionClearMenu, &QAction::triggered, this, [this]() {
		m_settings.remove( s_settingKey );
		
		updateMenu();		
	});
	
	updateMenu();
}

QMenu *ccRecentFiles::menu()
{
	return m_menu;
}

void ccRecentFiles::addFilePath( const QString &filePath )
{
	QStringList	list = m_settings.value( s_settingKey ).toStringList();
	
	list.removeAll( filePath );
	list.prepend( filePath );
	
	// only save the last ten files
	if ( list.count() > 10 )
	{
		list = list.mid( 0, 10 );
	}
	
	m_settings.setValue( s_settingKey, list );
	
	updateMenu();
}

void ccRecentFiles::updateMenu()
{
	m_menu->clear();
	
	const QStringList	recentList = listRecent();
	
	for ( const QString &recentFile : recentList )
	{
		QAction  *recentAction = new QAction( contractFilePath( recentFile ), this );
		
		recentAction->setData( recentFile );
		
		connect( recentAction, &QAction::triggered, this, &ccRecentFiles::openFileFromAction );
		
		m_menu->addAction( recentAction );
	}
	
	if ( !m_menu->actions().isEmpty() )
	{
		m_menu->addSeparator();
		m_menu->addAction( m_actionClearMenu );
	}
	
	m_menu->setEnabled( !m_menu->actions().isEmpty() );
}

void ccRecentFiles::openFileFromAction()
{
	QAction  *action = qobject_cast<QAction *>( sender() );
	
	Q_ASSERT( action );
	
	QString  fileName = action->data().toString();
	
	if ( !QFile::exists( fileName ) )
	{
		return;
	}
	
	QStringList	fileListOfOne{ fileName };
	
	MainWindow::TheInstance()->addToDB( fileListOfOne );
}

QStringList ccRecentFiles::listRecent()
{	
	QStringList list = m_settings.value( s_settingKey ).toStringList();
	
	QStringList::iterator iter = list.begin();
	
	while ( iter != list.end() )
	{
		const QString filePath = *iter;
		
		if ( !QFile::exists( filePath ) )
		{
			iter = list.erase( iter );
			continue;
		}
		
		++iter;
	}
	
	return list;
}

QString ccRecentFiles::contractFilePath( const QString &filePath )
{
	QString  homePath = QDir::toNativeSeparators( QDir::homePath() );
	QString  newPath = QDir::toNativeSeparators( filePath );
	
	if ( newPath.startsWith( homePath ) )
	{
		return newPath.replace( 0, QDir::homePath().length(), '~' );
	}
	
	return filePath;
}

QString ccRecentFiles::expandFilePath( const QString &filePath )
{
	QString  newPath = QDir::toNativeSeparators( filePath );
	
	if ( newPath.startsWith( '~' ) )
	{
		QString  homePath = QDir::toNativeSeparators( QDir::homePath() );
		
		return newPath.replace( 0, 1, homePath );
	}
	
	return filePath;
}
