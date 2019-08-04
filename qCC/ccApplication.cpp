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

#include <QtGlobal>

#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif

// qCC_io
#include "FileIO.h"

#include "ccApplication.h"
#include "mainwindow.h"

ccApplication::ccApplication( int &argc, char **argv, bool isCommandLine )
	: ccApplicationBase( argc, argv, isCommandLine, QStringLiteral( "2.11 alpha (Anoia)" ) )
{
	setApplicationName( "CloudCompare" );
	
	FileIO::setWriterInfo( applicationName(), versionStr() );
}

bool ccApplication::event(QEvent *inEvent)
{
#ifdef Q_OS_MAC
	switch ( inEvent->type() )
	{
		case QEvent::FileOpen:
		{
			MainWindow* mainWindow = MainWindow::TheInstance();
			
			if ( mainWindow == nullptr )
			{
				return false;
			}
			
			mainWindow->addToDB( QStringList(static_cast<QFileOpenEvent *>(inEvent)->file()) );
			return true;
		}
			
		default:
			break;
	}
#endif
	
	return ccApplicationBase::event( inEvent );
}
