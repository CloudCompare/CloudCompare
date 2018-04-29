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

#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif
#include <QSurfaceFormat>

// qCC_db
#include "ccMaterial.h"

#include "ccApplication.h"
#include "mainwindow.h"


void ccApplication::init()
{
	//See http://doc.qt.io/qt-5/qopenglwidget.html#opengl-function-calls-headers-and-qopenglfunctions
	/** Calling QSurfaceFormat::setDefaultFormat() before constructing the QApplication instance is mandatory
		on some platforms (for example, OS X) when an OpenGL core profile context is requested. This is to
		ensure that resource sharing between contexts stays functional as all internal contexts are created
		using the correct version and profile.
	**/
	{
		QSurfaceFormat format = QSurfaceFormat::defaultFormat();
		
		format.setSwapBehavior( QSurfaceFormat::DoubleBuffer );
		format.setStencilBufferSize( 0 );
		
#ifdef CC_GL_WINDOW_USE_QWINDOW
		format.setStereo( true );
#endif
		
#ifdef Q_OS_MAC
		format.setVersion( 2, 1 );	// must be 2.1 - see ccGLWindow::functions()
		format.setProfile( QSurfaceFormat::CoreProfile );
#endif
		
#ifdef QT_DEBUG
		format.setOption( QSurfaceFormat::DebugContext, true );
#endif
		
		QSurfaceFormat::setDefaultFormat( format );
	}
	
	// The 'AA_ShareOpenGLContexts' attribute must be defined BEFORE the creation of the Q(Gui)Application
	// DGM: this is mandatory to enable exclusive full screen for ccGLWidget (at least on Windows)
	QCoreApplication::setAttribute( Qt::AA_ShareOpenGLContexts );
}

ccApplication::ccApplication(int &argc, char **argv)
	: QApplication( argc, argv )
{
	setOrganizationName( "CCCorp" );
	setApplicationName( "CloudCompare" );
	
#ifdef Q_OS_MAC
	// Mac OS X apps don't show icons in menus
	setAttribute( Qt::AA_DontShowIconsInMenus );
#endif
	
	// Force 'english' locale so as to get a consistent behavior everywhere
	QLocale::setDefault( QLocale( QLocale::English ) );
	
#ifdef Q_OS_UNIX
	// We reset the numeric locale for POSIX functions
	// See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
	setlocale( LC_NUMERIC, "C" );
#endif
	
	connect( this, &ccApplication::aboutToQuit, [=](){ ccMaterial::ReleaseTextures(); } );
}

#ifdef Q_OS_MAC
bool ccApplication::event(QEvent *inEvent)
{
	switch ( inEvent->type() )
	{
		case QEvent::FileOpen:
		{
			MainWindow* mainWindow = MainWindow::TheInstance();
			
			if ( mainWindow == nullptr )
				return false;
			
			mainWindow->addToDB( QStringList(static_cast<QFileOpenEvent *>(inEvent)->file()) );
			return true;
		}
			
		default:
			return QApplication::event( inEvent );
	}
}
#endif
