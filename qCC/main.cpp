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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <ccIncludeGL.h>

//Qt
#include <QApplication>
#include <QSplashScreen>
#include <QPixmap>
#include <QMessageBox>
#include <QLocale>
#include <QTime>
#include <QTranslator>
#include <QSettings>
#include <QGLFormat>

#ifdef Q_OS_MAC
#include <QFileOpenEvent>
#endif

//qCC_db
#include <ccLog.h>
#include <ccNormalVectors.h>
#include <ccColorScalesManager.h>
#include <ccMaterial.h>

//qCC_io
#include <FileIOFilter.h>
#include <ccGlobalShiftManager.h>

//local
#include "mainwindow.h"
#include "ccGuiParameters.h"
#include "ccCommandLineParser.h"
#include "ccPersistentSettings.h"

//plugins
#include <ccPluginInfo.h>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//! QApplication wrapper
class qccApplication : public QApplication
{
public:
	qccApplication( int &argc, char **argv )
		: QApplication( argc, argv )
	{
		setOrganizationName("CCCorp");
		setApplicationName("CloudCompare");
		//setAttribute( Qt::AA_ShareOpenGLContexts ); //DGM: too late
#ifdef Q_OS_MAC
		// Mac OS X apps don't show icons in menus
		setAttribute( Qt::AA_DontShowIconsInMenus );
#endif
		connect(this, &qccApplication::aboutToQuit, [=](){ ccMaterial::ReleaseTextures(); });
	}

#ifdef Q_OS_MAC
protected:
	bool event( QEvent *inEvent )
	{
		switch ( inEvent->type() )
		{
		case QEvent::FileOpen:
			{
				MainWindow* mainWindow = MainWindow::TheInstance();
				if ( mainWindow == NULL )
					return false;

				mainWindow->addToDB( QStringList(static_cast<QFileOpenEvent *>(inEvent)->file()) );
				return true;
			}

		default:
			return QApplication::event( inEvent );
		}
	}
#endif
};

int main(int argc, char **argv)
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
		format.setStereo(false);
		format.setVersion( 2, 1 );
		format.setProfile( QSurfaceFormat::CoreProfile );
#endif
#ifdef QT_DEBUG
		format.setOption(QSurfaceFormat::DebugContext, true);
#endif
		QSurfaceFormat::setDefaultFormat(format);
	}

	//The 'AA_ShareOpenGLContexts' attribute must be defined BEFORE the creation of the Q(Gui)Application
	//DGM: this is mandatory to enable exclusive full screen for ccGLWidget (at least on Windows)
	QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

	//QT initialiation
	qccApplication app(argc, argv);

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

#ifdef Q_OS_MAC	
	// This makes sure that our "working directory" is not within the application bundle
	QDir  appDir = QCoreApplication::applicationDirPath();
	
	if ( appDir.dirName() == "MacOS" )
	{
		appDir.cdUp();
		appDir.cdUp();
		appDir.cdUp();
		
		QDir::setCurrent( appDir.absolutePath() );
	}
#endif

	//store the log message until a valid logging instance is registered
	ccLog::EnableMessageBackup(true);
	
	//restore some global parameters
	{
		QSettings settings;
		settings.beginGroup(ccPS::GlobalShift());
		double maxAbsCoord = settings.value(ccPS::MaxAbsCoord(), ccGlobalShiftManager::MaxCoordinateAbsValue()).toDouble();
		double maxAbsDiag = settings.value(ccPS::MaxAbsDiag(), ccGlobalShiftManager::MaxBoundgBoxDiagonal()).toDouble();
		settings.endGroup();

		ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2").arg(maxAbsCoord, 0, 'e', 0).arg(maxAbsDiag, 0, 'e', 0));
		
		ccGlobalShiftManager::SetMaxCoordinateAbsValue(maxAbsCoord);
		ccGlobalShiftManager::SetMaxBoundgBoxDiagonal(maxAbsDiag);
	}

	//Command line mode?
	bool commandLine = (argc > 1 && argv[1][0] == '-');
	
	//specific commands
	int lastArgumentIndex = 1;
	QTranslator translator;
	if (commandLine)
	{
		//translation file selection
		if (QString(argv[lastArgumentIndex]).toUpper() == "-LANG")
		{
			QString langFilename = QString(argv[2]);

			//Load translation file
			if (translator.load(langFilename, QCoreApplication::applicationDirPath()))
			{
				qApp->installTranslator(&translator);
			}
			else
			{
				QMessageBox::warning(0, QObject::tr("Translation"), QObject::tr("Failed to load language file '%1'").arg(langFilename));
			}
			commandLine = false;
			lastArgumentIndex += 2;
		}
	}

	//splash screen
	QSplashScreen* splash = 0;
	QTime splashStartTime;

	//standard mode
	if (!commandLine)
	{
		if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
		{
			QMessageBox::critical(0, "Error", "This application needs OpenGL 2.1 at least to run!");
			return EXIT_FAILURE;
		}

		//splash screen
		splashStartTime.start();
		QPixmap pixmap(QString::fromUtf8(":/CC/images/imLogoV2Qt.png"));
		splash = new QSplashScreen(pixmap, Qt::WindowStaysOnTopHint);
		splash->show();
		QApplication::processEvents();
	}

	//global structures initialization
	FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	//load the plugins
	tPluginInfoList plugins;
	QStringList dirFilters;
	QStringList pluginPaths;
	{
		QString appPath = QCoreApplication::applicationDirPath();

#if defined(Q_OS_MAC)
		dirFilters << "*.dylib";

		// plugins are in the bundle
		appPath.remove("MacOS");

		pluginPaths += (appPath + "Plugins/ccPlugins");
#if defined(CC_MAC_DEV_PATHS)
		// used for development only - this is the path where the plugins are built
		// this avoids having to install into the application bundle when developing
		pluginPaths += (appPath + "../../../ccPlugins");
#endif
#elif defined(Q_OS_WIN)
		dirFilters << "*.dll";

		//plugins are in bin/plugins
		pluginPaths << (appPath + "/plugins");
#elif defined(Q_OS_LINUX)
		dirFilters << "*.so";

		// Plugins are relative to the bin directory where the executable is found
		QDir  binDir(appPath);

		if (binDir.dirName() == "bin")
		{
			binDir.cdUp();

			pluginPaths << (binDir.absolutePath() + "/lib/cloudcompare/plugins");
		}
		else
		{
			// Choose a reasonable default to look in
			pluginPaths << "/usr/lib/cloudcompare/plugins";
		}
#else
		#warning Need to specify the plugin path for this OS.
#endif

#ifdef Q_OS_MAC
		// Add any app data paths
		// Plugins in these directories take precendence over the included ones
		QStringList appDataPaths = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);

		for (const QString &appDataPath : appDataPaths)
		{
			pluginPaths << (appDataPath + "/plugins");
		}
#endif
	}

	ccPlugins::LoadPlugins(plugins, pluginPaths, dirFilters);
	
	int result = 0;

	//command line mode
	if (commandLine)
	{
		//command line processing (no GUI)
		result = ccCommandLineParser::Parse(argc, argv);
	}
	else
	{
		//main window init.
		MainWindow* mainWindow = MainWindow::TheInstance();
		if (!mainWindow)
		{
			QMessageBox::critical(0, "Error", "Failed to initialize the main application window?!");
			return EXIT_FAILURE;
		}
		mainWindow->dispatchPlugins(plugins, pluginPaths);
		mainWindow->show();
		QApplication::processEvents();

		//show current Global Shift parameters in Console
		{
			ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
				.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
				.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));
		}

		if (argc > lastArgumentIndex)
		{
			if (splash)
				splash->close();

			//any additional argument is assumed to be a filename --> we try to load it/them
			QStringList filenames;
			for (int i = lastArgumentIndex; i < argc; ++i)
			{
				QString arg(argv[i]);
				//special command: auto start a plugin
				if (arg.startsWith(":start-plugin:"))
				{
					QString pluginName = arg.mid(14);
					QString pluginNameUpper = pluginName.toUpper();
					//look for this plugin
					bool found = false;
					for (const tPluginInfo &plugin : plugins)
					{
						if (plugin.object->getName().replace(' ', '_').toUpper() == pluginNameUpper)
						{
							found = true;
							bool success = plugin.object->start();
							if (!success)
							{
								ccLog::Error(QString("Failed to start the plugin '%1'").arg(plugin.object->getName()));
							}
							break;
						}
					}

					if (!found)
					{
						ccLog::Error(QString("Couldn't find the plugin '%1'").arg(pluginName.replace('_', ' ')));
					}
				}
				else
				{
					filenames << arg;
				}
			}

			mainWindow->addToDB(filenames);
		}
		
		if (splash)
		{
			//we want the splash screen to be visible a minimum amount of time (1000 ms.)
			while (splashStartTime.elapsed() < 1000)
			{
				splash->raise();
				QApplication::processEvents(); //to let the system breath!
			}

			splash->close();
			QApplication::processEvents();

			delete splash;
			splash = 0;
		}

		//let's rock!
		try
		{
			result = app.exec();
		}
		catch (...)
		{
			QMessageBox::warning(0, "CC crashed!", "Hum, it seems that CC has crashed... Sorry about that :)");
		}

		//release the plugins
		for (tPluginInfo &plugin : plugins)
		{
			plugin.object->stop(); //just in case
			if (!plugin.qObject->parent())
			{
				delete plugin.object;
				plugin.object = 0;
				plugin.qObject = 0;
			}
		}

	}

	//release global structures
	MainWindow::DestroyInstance();
	FileIOFilter::UnregisterAll();

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	//for debug purposes
	unsigned alive = CCShareable::GetAliveCount();
	if (alive > 1)
	{
		printf("Error: some shared objects (%u) have not been released on program end!",alive);
		system("PAUSE");
	}
#endif

	return result;
}
