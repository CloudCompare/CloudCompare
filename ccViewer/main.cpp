//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

//Qt
#include <QApplication>
#include <QMessageBox>

#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif

//Local
#include "ccviewerlog.h"

//qCC_db
#include <ccIncludeGL.h>
#include <ccTimer.h>
#include <ccNormalVectors.h>
#include <ccColorScalesManager.h>

//qCC_io
#include <FileIOFilter.h>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

#include "ccviewer.h"

class ccApplication : public QApplication
{
public:
	ccApplication( int &argc, char **argv ) :
		QApplication( argc, argv )
	{
		setOrganizationName("CCCorp");
		setApplicationName("CloudCompareViewer");
#ifdef Q_OS_MAC
		mViewer = NULL;

		// Mac OS X apps don't show icons in menus
		setAttribute( Qt::AA_DontShowIconsInMenus );
#endif
	}

#ifdef Q_OS_MAC
	void  setViewer( ccViewer *inViewer ) { mViewer = inViewer; }

protected:
	bool  event( QEvent *inEvent )
	{
		switch ( inEvent->type() )
		{
		case QEvent::FileOpen:
			{
				if ( mViewer == NULL )
					return false;

				QStringList fileName( static_cast<QFileOpenEvent *>(inEvent)->file() );

				mViewer->addToDB( fileName );
				return true;
			}

		default:
			return QApplication::event( inEvent );
		}
	}

private:
	ccViewer *mViewer;
#endif
};

int main(int argc, char *argv[])
{
	ccApplication a(argc, argv);

#ifdef USE_VLD
	VLDEnable();
#endif

	//OpenGL?
	if (!QGLFormat::hasOpenGL())
	{
		QMessageBox::critical(0, "Error", "This application needs OpenGL to run!");
		return EXIT_FAILURE;
	}

	//common data initialization
	ccTimer::Init();
	FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	ccViewer w/*(0,Qt::Window|Qt::CustomizeWindowHint)*/;
#ifdef Q_OS_MAC
	a.setViewer( &w );
#endif

	//register minimal logger to display errors
	ccViewerLog logger(&w);
	ccLog::RegisterInstance(&logger);

	w.show();

	//files to open are passed as argument
	if (argc > 1)
	{
		//parse arguments
		QStringList filenames;
		int i = 1;
		while ( i < argc)
		{
			QString argument = QString(argv[i++]).toUpper();

			//Argument '-WIN X Y W H' (to set window size and position)
			if (argument.toUpper() == "-WIN")
			{
				bool ok = true;
				if (i+3 < argc)
				{
					bool converionOk;
					int x      = QString(argv[i  ]).toInt(&converionOk); ok &= converionOk;
					int y      = QString(argv[i+1]).toInt(&converionOk); ok &= converionOk;
					int width  = QString(argv[i+2]).toInt(&converionOk); ok &= converionOk;
					int height = QString(argv[i+3]).toInt(&converionOk); ok &= converionOk;
					i += 4;

					if (ok)
					{
						//change window position and size
						w.move(x,y);
						w.resize(width,height);
					}
				}
				if (!ok)
				{
					ccLog::Warning("Invalid arguments! 4 integer values are expected after '-win' (X Y W H)");
					break;
				}
			}
			else if (argument == "-TOP")
			{
				w.setWindowFlags(w.windowFlags() | Qt::CustomizeWindowHint | Qt::WindowStaysOnTopHint);
				w.show();
			}
			else //any other argument is assumed to be a filename (that will we try to load)
			{
				filenames << argument;
			}
		}

		if (!filenames.empty())
			w.addToDB(filenames);
	}

#ifdef Q_OS_MAC
	// process events to load any files on the command line
	QCoreApplication::processEvents();
#endif

	w.checkForLoadedEntities();

	int result = a.exec();

	return result;
}
