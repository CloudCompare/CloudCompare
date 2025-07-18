// ##########################################################################
// #                                                                        #
// #                   CLOUDCOMPARE LIGHT VIEWER                            #
// #                                                                        #
// #  This project has been initiated under funding from ANR/CIFRE          #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
// #                                                                        #
// ##########################################################################

// Qt
#include <QDir>
#include <QGLFormat>

// Local
#include "ccviewerlog.h"

// qCC_db
#include <ccColorScalesManager.h>
#include <ccIncludeGL.h>
#include <ccMaterial.h>
#include <ccNormalVectors.h>
#include <ccPointCloud.h>

// qCC_io
#include <FileIOFilter.h>

#ifdef USE_VLD
// VLD
#include <vld.h>
#endif

#include "ccViewerApplication.h"
#include "ccviewer.h"

int main(int argc, char* argv[])
{

#ifdef Q_OS_WIN
	// enables automatic scaling based on the monitor's pixel density
	ccViewerApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

	ccViewerApplication::InitOpenGL();

	// Convert the input arguments to QString before the application is initialized
	// (as it will force utf8, which might prevent from properly reading filenmaes from the command line)
	QStringList argumentsLocal8Bit;
	for (int i = 0; i < argc; ++i)
	{
		argumentsLocal8Bit << QString::fromLocal8Bit(argv[i]);
	}

	ccViewerApplication a(argc, argv, false);

#ifdef CC_GAMEPAD_SUPPORT
#if QT_VERSION >= QT_VERSION_CHECK(5, 9, 0)
#if QT_VERSION < QT_VERSION_CHECK(5, 10, 0)
	QGamepadManager::instance(); // potential workaround to bug https://bugreports.qt.io/browse/QTBUG-61553
#endif
#endif
#endif

#ifdef USE_VLD
	VLDEnable();
#endif

	QDir workingDir = QCoreApplication::applicationDirPath();

#ifdef Q_OS_MAC
	// This makes sure that our "working directory" is not within the application bundle
	if (workingDir.dirName() == "MacOS")
	{
		workingDir.cdUp();
		workingDir.cdUp();
		workingDir.cdUp();
	}
#endif

	QDir::setCurrent(workingDir.absolutePath());

	if (!QGLFormat::hasOpenGL())
	{
		QMessageBox::critical(nullptr, "Error", "This application needs OpenGL to run!");
		return EXIT_FAILURE;
	}
	if ((QGLFormat::openGLVersionFlags() & QGLFormat::OpenGL_Version_2_1) == 0)
	{
		QMessageBox::critical(nullptr, "Error", "This application needs OpenGL 2.1 at least to run!");
		return EXIT_FAILURE;
	}

	// common data initialization
	FileIOFilter::InitInternalFilters();       // load all known I/O filters (plugins will come later!)
	ccNormalVectors::GetUniqueInstance();      // force pre-computed normals array initialization
	ccColorScalesManager::GetUniqueInstance(); // force pre-computed color tables initialization

	ccViewer w;
	a.setViewer(&w);

	// register minimal logger to display errors
	ccViewerLog logger(&w);
	ccLog::RegisterInstance(&logger);

	w.show();

	// files to open are passed as argument
	if (argc > 1)
	{
		// parse arguments
		QStringList filenames;
		int         i = 1;
		while (i < argc)
		{
			QString argument      = argumentsLocal8Bit[i++];
			QString upperArgument = argument.toUpper();

			// Argument '-WIN X Y W H' (to set window size and position)
			if (upperArgument == "-WIN")
			{
				bool ok = true;
				if (i + 3 < argc)
				{
					bool converionOk;
					int  x = argumentsLocal8Bit[i].toInt(&converionOk);
					ok &= converionOk;
					int y = argumentsLocal8Bit[i + 1].toInt(&converionOk);
					ok &= converionOk;
					int width = argumentsLocal8Bit[i + 2].toInt(&converionOk);
					ok &= converionOk;
					int height = argumentsLocal8Bit[i + 3].toInt(&converionOk);
					ok &= converionOk;
					i += 4;

					if (ok)
					{
						// change window position and size
						w.move(x, y);
						w.resize(width, height);
					}
				}
				if (!ok)
				{
					ccLog::Warning("Invalid arguments! 4 integer values are expected after '-win' (X Y W H)");
					break;
				}
			}
			else if (upperArgument == "-MAX")
			{
				w.showMaximized();
			}
			else if (upperArgument == "-TOP")
			{
				w.setWindowFlags(w.windowFlags() | Qt::CustomizeWindowHint | Qt::WindowStaysOnTopHint);
				w.show();
			}
			else // any other argument is assumed to be a filename (that will we try to load)
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

	// release global structures
	FileIOFilter::UnregisterAll();
	ccPointCloud::ReleaseShaders();

	return result;
}
