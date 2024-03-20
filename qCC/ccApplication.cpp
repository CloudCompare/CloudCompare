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

//! Map between a file version, and the first version of CloudCompare that was able to load it
struct FileVersionToCCVersion : QMap<short, QString>
{
	FileVersionToCCVersion()
	{
		insert(10, "1.0 (before 2012)");
		insert(20, "2.0 (05/04/2012)");
		insert(21, "2.4 (07/02/2012)");
		insert(22, "2.4 (11/26/2012)");
		insert(23, "2.4 (02/07/2013)");
		insert(24, "2.4 (02/22/2013)");
		insert(25, "2.4 (03/16/2013)");
		insert(26, "2.4 (04/03/2013)");
		insert(27, "2.4 (04/12/2013)");
		insert(28, "2.5.0 (07/12/2013)");
		insert(29, "2.5.0 (08/14/2013)");
		insert(30, "2.5.0 (08/30/2013)");
		insert(31, "2.5.0 (09/25/2013)");
		insert(32, "2.5.1 (10/11/2013)");
		insert(33, "2.5.2 (12/19/2013)");
		insert(34, "2.5.3 (01/09/2014)");
		insert(35, "2.5.4 (02/13/2014)");
		insert(36, "2.5.5 (05/30/2014)");
		insert(37, "2.5.5 (08/24/2014)");
		insert(38, "2.6.0 (09/14/2014)");
		insert(39, "2.6.1 (01/30/2015)");
		insert(40, "2.6.2 (08/06/2015)");
		insert(41, "2.6.2 (09/01/2015)");
		insert(42, "2.6.2 (10/07/2015)");
		insert(43, "2.6.3 (01/07/2016)");
		insert(44, "2.8.0 (07/07/2016)");
		insert(45, "2.8.0 (10/06/2016)");
		insert(46, "2.8.0 (11/03/2016)");
		insert(47, "2.9.0 (12/22/2016)");
		insert(48, "2.10.0 (10/19/2018)");
		insert(49, "2.11.0 (03/31/2019)");
		insert(50, "2.11.0 (10/06/2019)");
		insert(51, "2.12.0 (03/29/2019)");
		insert(52, "2.12.0 (11/30/2020)");
		insert(53, "2.13.alpha (10/02/2022)");
		insert(54, "2.13.alpha (01/29/2023)");
	}

	QString getMinCCVersion(short fileVersion) const
	{
		if (contains(fileVersion))
		{
			return value(fileVersion);
		}
		else
		{
			return "Unknown version";
		}
	}
};
static FileVersionToCCVersion s_fileVersionToCCVersion;

QString ccApplication::GetMinCCVersionForFileVersion(short fileVersion)
{
	return s_fileVersionToCCVersion.getMinCCVersion(fileVersion);
}

ccApplication::ccApplication( int &argc, char **argv, bool isCommandLine )
	: ccApplicationBase( argc, argv, isCommandLine, QString( "2.14.alpha (%1)" ).arg(__DATE__))
{
	setApplicationName( "CloudCompare" );
	
	FileIO::setWriterInfo( applicationName(), versionStr() );
}

bool ccApplication::event(QEvent* inEvent)
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
