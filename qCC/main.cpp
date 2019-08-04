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
#include <QDir>
#include <QMessageBox>
#include <QPixmap>
#include <QSettings>
#include <QSplashScreen>
#include <QTime>
#include <QTimer>
#include <QTranslator>

//qCC_db
#include <ccColorScalesManager.h>
#include <ccLog.h>
#include <ccNormalVectors.h>

//qCC_io
#include <FileIOFilter.h>
#include <ccGlobalShiftManager.h>

//local
#include "ccApplication.h"
#include "ccCommandLineParser.h"
#include "ccGuiParameters.h"
#include "ccPersistentSettings.h"
#include "mainwindow.h"

//plugins
#include "ccPluginInterface.h"
#include "ccPluginManager.h"

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

#ifdef Q_OS_MAC
#include <unistd.h>
#endif

int main(int argc, char **argv)
{
#ifdef Q_OS_MAC
	bool commandLine = isatty( fileno( stdin ) );
#else
	bool commandLine = (argc > 1) && (argv[1][0] == '-');
#endif
   
	if ( !commandLine )
	{
		ccApplication::initOpenGL();
	}
	
	ccApplication app(argc, argv, commandLine);

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
	QScopedPointer<QSplashScreen> splash(nullptr);
	QTimer splashTimer;

	//standard mode
	if (!commandLine)
	{
		if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
		{
			QMessageBox::critical(0, "Error", "This application needs OpenGL 2.1 at least to run!");
			return EXIT_FAILURE;
		}

		//splash screen
		QPixmap pixmap(QString::fromUtf8(":/CC/images/imLogoV2Qt.png"));
		splash.reset(new QSplashScreen(pixmap, Qt::WindowStaysOnTopHint));
		splash->show();
		QApplication::processEvents();
	}

	//global structures initialization
	FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	//load the plugins
	ccPluginManager::get().loadPlugins();
	
	int result = 0;

	//command line mode
	if (commandLine)
	{
		//command line processing (no GUI)
		result = ccCommandLineParser::Parse(argc, argv, ccPluginManager::get().pluginList());
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
		mainWindow->initPlugins();
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
			{
				splash->close();
			}

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
					for ( ccPluginInterface *plugin : ccPluginManager::get().pluginList() )
					{
						if (plugin->getName().replace(' ', '_').toUpper() == pluginNameUpper)
						{
							found = true;
							bool success = plugin->start();
							if (!success)
							{
								ccLog::Error(QString("Failed to start the plugin '%1'").arg(plugin->getName()));
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
					filenames << QString::fromLocal8Bit(argv[i]);
				}
			}

			mainWindow->addToDB(filenames);
		}
		else if (splash)
		{
			//count-down to hide the timer (only effective once the application will have actually started!)
			QObject::connect(&splashTimer, &QTimer::timeout, [&]() { if (splash) splash->close(); QCoreApplication::processEvents(); splash.reset(); });
			splashTimer.setInterval(1000);
			splashTimer.start();
		}

		//change the default path to the application one (do this AFTER processing the command line)
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

		QDir::setCurrent(workingDir.absolutePath());

		//let's rock!
		try
		{
			result = app.exec();
		}
		catch (const std::exception& e)
		{
			QMessageBox::warning(0, "CC crashed!", QString("Hum, it seems that CC has crashed... Sorry about that :)\n") + e.what());
		}
		catch (...)
		{
			QMessageBox::warning(0, "CC crashed!", "Hum, it seems that CC has crashed... Sorry about that :)");
		}

		//release the plugins
		for ( ccPluginInterface *plugin : ccPluginManager::get().pluginList() )
		{
			plugin->stop(); //just in case
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
