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
#include <QGLFormat>
#include <QMessageBox>
#include <QPixmap>
#include <QSettings>
#include <QSplashScreen>
#include <QTime>
#include <QTimer>
#include <QTranslator>
#ifdef CC_GAMEPAD_SUPPORT
#include <QGamepadManager>
#endif

//qCC_db
#include <ccColorScalesManager.h>
#include <ccLog.h>
#include <ccNormalVectors.h>
#include <ccPointCloud.h>

//qCC_io
#include <FileIOFilter.h>
#include <ccGlobalShiftManager.h>

//local
#include "ccApplication.h"
#include "ccCommandLineParser.h"
#include "ccGuiParameters.h"
#include "ccPersistentSettings.h"
#include "mainwindow.h"
#include "ccTranslationManager.h"

//plugins
#include "ccPluginInterface.h"
#include "ccPluginManager.h"

#ifdef USE_VLD
#include <vld.h>
#endif

static bool IsCommandLine(int argc, char **argv)
{
#ifdef Q_OS_MAC
    // On macOS, when double-clicking the application, the Finder (sometimes!) adds a command-line parameter
    // like "-psn_0_582385" which is a "process serial number".
    // We need to recognize this and discount it when determining if we are running on the command line or not.

    int numRealArgs = argc;

    for ( int i = 1; i < argc; ++i )
    {
        if ( strncmp( argv[i], "-psn_", 5 ) == 0 )
        {
            --numRealArgs;
        }
    }

    return (numRealArgs > 1) && (argv[1][0] == '-');
#else
    return (argc > 1) && (argv[1][0] == '-');
#endif
}

int main(int argc, char **argv)
{
#ifdef _WIN32 //This will allow printf to function on windows when opened from command line
	DWORD stdout_type = GetFileType(GetStdHandle(STD_OUTPUT_HANDLE));
	if (AttachConsole(ATTACH_PARENT_PROCESS))
	{
		if (stdout_type == FILE_TYPE_UNKNOWN) // this will allow std redirection (./executable > out.txt)
		{
			freopen("CONOUT$", "w", stdout);
			freopen("CONOUT$", "w", stderr);
		}
	}
#endif

    bool commandLine = IsCommandLine(argc, argv);

    // Convert the input arguments to QString before the application is initialized
    // (as it will force utf8, which might prevent from properly reading filenames from the command line)
    QStringList argumentsLocal8Bit;
    for (int i = 0; i < argc; ++i)
    {
        argumentsLocal8Bit << QString::fromLocal8Bit(argv[i]);
    }

    //specific commands
    int lastArgumentIndex = 1;
    if (commandLine)
    {
        //translation file selection
        if (	lastArgumentIndex < argumentsLocal8Bit.size()
			&&	argumentsLocal8Bit[lastArgumentIndex].toUpper() == "-LANG")
        {
			//remove verified local option
			argumentsLocal8Bit.removeAt(lastArgumentIndex);

			if (lastArgumentIndex >= argumentsLocal8Bit.size())
			{
				ccLog::Error(QObject::tr("Missing argument after %1: language file").arg("-LANG"));
				return EXIT_FAILURE;
			}

			//remove verified arguments so that -SILENT will be the first one (if present)...
			QString langFilename = argumentsLocal8Bit.takeAt(lastArgumentIndex);

            ccTranslationManager::Get().loadTranslation(langFilename);
            commandLine = false;
        }

		if (	lastArgumentIndex < argumentsLocal8Bit.size()
			&&	argumentsLocal8Bit[lastArgumentIndex].toUpper() == "-VERBOSITY")
		{
			//remove verified local option
			argumentsLocal8Bit.removeAt(lastArgumentIndex);

			if (lastArgumentIndex >= argumentsLocal8Bit.size())
			{
				ccLog::Error(QObject::tr("Missing argument after %1: verbosity level").arg("-VERBOSITY"));
				return EXIT_FAILURE;
			}

			//remove verified arguments so that -SILENT will be the first one (if present)...
			QString verbosityLevelStr = argumentsLocal8Bit.takeAt(lastArgumentIndex);

			bool ok = false;
			int verbosityLevel = verbosityLevelStr.toInt(&ok);
			if (!ok || verbosityLevel < 0)
			{
				ccLog::Warning(QObject::tr("Invalid verbosity level: %1").arg(verbosityLevelStr));
			}
			else
			{
				ccLog::SetVerbosityLevel(verbosityLevel);
			}
		}
    }

#ifdef Q_OS_WIN
	//enables automatic scaling based on the monitor's pixel density
	ccApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    ccApplication::InitOpenGL();

	ccApplication app(argc, argv, commandLine);

	if (!commandLine)
	{
		// if not in CLI mode, we set the default log verbosity level
		ccLog::SetVerbosityLevel(ccGui::Parameters().logVerbosityLevel);
	}

#ifdef CC_GAMEPAD_SUPPORT
#if QT_VERSION >= QT_VERSION_CHECK(5, 9, 0)
#if QT_VERSION < QT_VERSION_CHECK(5, 10, 0)
	QGamepadManager::instance(); //potential workaround to bug https://bugreports.qt.io/browse/QTBUG-61553
#endif
#endif
#endif
	//store the log message until a valid logging instance is registered
	ccLog::EnableMessageBackup(true);

    //splash screen
    QScopedPointer<QSplashScreen> splash(nullptr);

    //standard mode
    if (!commandLine)
    {
        if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
        {
            QMessageBox::critical(nullptr, "Error", "This application needs OpenGL 2.1 at least to run!");
            return EXIT_FAILURE;
        }

        //init splash screen
        QPixmap pixmap(QString::fromUtf8(":/CC/images/imLogoV2Qt.png"));
        splash.reset(new QSplashScreen(pixmap, Qt::WindowStaysOnTopHint));
        splash->show();
    }

    //global structures initialization
    FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
    ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
    ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

    //load the plugins
    ccPluginManager& pluginManager = ccPluginManager::Get();
    pluginManager.loadPlugins();

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

	int result = 0;

	//command line mode
	if (commandLine)
	{
		//command line processing (no GUI)
		result = ccCommandLineParser::Parse(argumentsLocal8Bit, pluginManager.pluginList());
	}
	else
	{
		//main window initialization
		MainWindow* mainWindow = MainWindow::TheInstance();
		if (!mainWindow)
		{
			QMessageBox::critical(nullptr, "Error", "Failed to initialize the main application window?!");
			return EXIT_FAILURE;
		}
		mainWindow->initPlugins();
		mainWindow->show();
		QCoreApplication::processEvents();

		//show current Global Shift parameters in Console
		{
			ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
				.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
				.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));
		}

		if (splash)
		{
			splash->close();
		}

		if (argc > lastArgumentIndex)
		{
			//any additional argument is assumed to be a filename --> we try to load it/them
			QStringList filenames;
			for (int i = lastArgumentIndex; i < argc; ++i)
			{
				QString arg = argumentsLocal8Bit[i];

				//special command: auto start a plugin
				if (arg.startsWith(":start-plugin:"))
				{
					QString pluginName = arg.mid(14);
					QString pluginNameUpper = pluginName.toUpper();
					//look for this plugin
					bool found = false;
					for (ccPluginInterface* plugin : pluginManager.pluginList())
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
					filenames << arg;
				}
			}

			mainWindow->addToDB(filenames);
		}

		//change the default path to the application one (do this AFTER processing the command line)
		QDir workingDir = QCoreApplication::applicationDirPath();

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
			result = QApplication::exec();
		}
		catch (const std::exception& e)
		{
			QMessageBox::warning(nullptr, "CC crashed!", QString("Hum, it seems that CC has crashed... Sorry about that :)\n") + e.what());
		}
		catch (...)
		{
			QMessageBox::warning(nullptr, "CC crashed!", "Hum, it seems that CC has crashed... Sorry about that :)");
		}

		//release the plugins
		for (ccPluginInterface* plugin : pluginManager.pluginList())
		{
			plugin->stop(); //just in case
		}
	}

	//release global structures
	ccPointCloud::ReleaseShaders(); // must be done before the OpenGL context is released (i.e. before the windows is destroyed)
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
