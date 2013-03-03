//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
//#  This program is free software; you can redistribute it and/or modify  #
//#                                                                        #
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
#include <QtGui/QApplication>
#include <QMessageBox>

//qCC
#include <ccConsole.h>

//qCC_db
#include <ccIncludeGL.h>
#include <ccTimer.h>
#include <ccObject.h>
#include <ccNormalVectors.h>
#include <ccColorTablesManager.h>

#include "ccviewer.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	//OpenGL?
    if (!QGLFormat::hasOpenGL())
    {
        QMessageBox::critical(0, "Error", "This application needs OpenGL to run!");
        return EXIT_FAILURE;
    }

	//Global settings 'header'
    QCoreApplication::setOrganizationName("CCCorp");
    QCoreApplication::setApplicationName("CloudCompareViewer");

    //common data initialization
	ccObject::ResetUniqueIDCounter();
    ccTimer::Init();
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorTablesManager::GetUniqueInstance(); //force pre-computed color tables initialization

	ccViewer w/*(0,Qt::Window|Qt::CustomizeWindowHint)*/;
	w.show();

	//init console
	ccConsole::Init(0,&w);

	//files to open are passed as argument
	if (argc>1)
    {
		//parse arguments
		QStringList filenames;
		int i=1;
		while (i<argc)
		{
			QString argument = QString(argv[i++]).toUpper();

			//Argument '-WIN X Y W H' (to set window size and position)
			if (argument.toUpper() == "-WIN")
			{
				bool ok=true;
				if (i+3<argc)
				{
					bool converionOk;
					int x = QString(argv[i]).toInt(&converionOk); ok &= converionOk;
					int y = QString(argv[i+1]).toInt(&converionOk); ok &= converionOk;
					int width = QString(argv[i+2]).toInt(&converionOk); ok &= converionOk;
					int height = QString(argv[i+3]).toInt(&converionOk); ok &= converionOk;
					i+=4;

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

    w.checkForLoadedEntities();

	int result = a.exec();

	ccNormalVectors::ReleaseUniqueInstance();
    ccColorTablesManager::ReleaseUniqueInstance();
	ccConsole::ReleaseInstance();

	return result;
}
