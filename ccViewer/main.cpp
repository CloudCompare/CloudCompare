//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

//Qt
#include <QApplication>
#include <QDir>
#include <QGLFormat>
#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif

//Local
#include "ccviewerlog.h"

//qCC_db
#include <ccIncludeGL.h>
#include <ccNormalVectors.h>
#include <ccColorScalesManager.h>
#include <ccMaterial.h>

//qCC_io
#include <FileIOFilter.h>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

#include "ccviewer.h"

class ccApplication : public QApplication
{
	Q_OBJECT

public:
	ccApplication( int &argc, char **argv ) :
		QApplication( argc, argv )
	{
		setOrganizationName("CCCorp");
		setApplicationName("CloudCompareViewer");
		setAttribute(Qt::AA_ShareOpenGLContexts);
#ifdef Q_OS_MAC
		mViewer = NULL;

		// Mac OS X apps don't show icons in menus
		setAttribute( Qt::AA_DontShowIconsInMenus );
#endif
		connect(this, &ccApplication::aboutToQuit, [=](){ ccMaterial::ReleaseTextures(); });
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
	//See http://doc.qt.io/qt-5/qopenglwidget.html#opengl-function-calls-headers-and-qopenglfunctions
	/** Calling QSurfaceFormat::setDefaultFormat() before constructing the QApplication instance is mandatory
	on some platforms (for example, OS X) when an OpenGL core profile context is requested. This is to
	ensure that resource sharing between contexts stays functional as all internal contexts are created
	using the correct version and profile.
	**/
	{
		QSurfaceFormat format = QSurfaceFormat::defaultFormat();
		format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
		format.setStencilBufferSize(0);
#ifdef CC_GL_WINDOW_USE_QWINDOW
		format.setStereo(true);
#endif
#ifdef Q_OS_MAC
		format.setVersion( 2, 1 );	// must be 2.1 - see ccGLWindow::functions()
		format.setProfile( QSurfaceFormat::CoreProfile );
#endif
#ifdef QT_DEBUG
		format.setOption(QSurfaceFormat::DebugContext);
#endif
		QSurfaceFormat::setDefaultFormat(format);
	}

	ccApplication a(argc, argv);
	
	//Locale management
	{
		//Force 'english' locale so as to get a consistent behavior everywhere
		QLocale locale = QLocale(QLocale::English);
		locale.setNumberOptions(QLocale::c().numberOptions());
		QLocale::setDefault(locale);

#ifdef Q_OS_UNIX
		//We reset the numeric locale for POSIX functions
		//See http://qt-project.org/doc/qt-5/qcoreapplication.html#locale-settings
		setlocale(LC_NUMERIC, "C");
#endif
	}

#ifdef USE_VLD
	VLDEnable();
#endif
	
	QDir  workingDir = QCoreApplication::applicationDirPath();
	
#ifdef Q_OS_MAC	
	// This makes sure that our "working directory" is not within the application bundle
	if ( workingDir.dirName() == "MacOS" )
	{
		workingDir.cdUp();
		workingDir.cdUp();
		workingDir.cdUp();
	}
#endif
	
	QDir::setCurrent( workingDir.absolutePath() );
	
	if (!QGLFormat::hasOpenGL())
	{
		QMessageBox::critical(0, "Error", "This application needs OpenGL to run!");
		return EXIT_FAILURE;
	}
	if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
	{
		QMessageBox::critical(0, "Error", "This application needs OpenGL 2.1 at least to run!");
		return EXIT_FAILURE;
	}

	//common data initialization
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
			QString argument = QString(argv[i++]);
			QString upperArgument = argument.toUpper();

			//Argument '-WIN X Y W H' (to set window size and position)
			if (upperArgument == "-WIN")
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
			else if (upperArgument == "-TOP")
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
		{
			w.addToDB(filenames);
		}
	}

#ifdef Q_OS_MAC
	// process events to load any files on the command line
	QCoreApplication::processEvents();
#endif

	w.checkForLoadedEntities();

	int result = a.exec();

	//release global structures
	FileIOFilter::UnregisterAll();

	return result;
}

#include "main.moc"
