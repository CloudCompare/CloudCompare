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
		//any argument is assumed to be a filename --> we try to load it
		QStringList filenames;
        for (int i=1;i<argc;++i)
			filenames << QString(argv[i]);
		w.addToDB(filenames);
    }

    w.checkForLoadedEntities();

	int result = a.exec();

	ccNormalVectors::ReleaseUniqueInstance();
    ccColorTablesManager::ReleaseUniqueInstance();
	ccConsole::ReleaseInstance();

	return result;
}
