//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#include <ccIncludeGL.h>

//Qt
#include <QApplication>
#include <QSplashScreen>
#include <QPixmap>
#include <QMessageBox>

//qCC_db
#include <ccTimer.h>
#include <ccObject.h>
#include <ccNormalVectors.h>
#include <ccColorTablesManager.h>

#include "mainwindow.h"
#include "ccConsole.h"
#include "ccGuiParameters.h"
#include "ccCommandLineParser.h"

//system
#include <time.h>

int main(int argc, char **argv)
{
	//QT initialisation
    QApplication app(argc, argv);

	//Global settings 'header'
    QCoreApplication::setOrganizationName("CCCorp");
    QCoreApplication::setApplicationName("CloudCompare");

    //Command line mode?
    bool commandLine = (argc>1 && argv[1][0]=='-');

	QSplashScreen* splash = 0;
	clock_t start_time = 0;
    if (!commandLine)
    {
        //OpenGL?
        if (!QGLFormat::hasOpenGL())
        {
            QMessageBox::critical(0, "Error", "This application needs OpenGL to run!");
            return EXIT_FAILURE;
        }

        //splash screen
        start_time = clock();
        QPixmap pixmap(QString::fromUtf8(":/CC/Menu/images/menu/cc_logo_v2_qt.png"));
        splash = new QSplashScreen(pixmap,Qt::WindowStaysOnTopHint);
        splash->show();
        QApplication::processEvents();
    }

    //common data initialization
	ccObject::ResetUniqueIDCounter();
    ccTimer::Init();
	ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
	ccColorTablesManager::GetUniqueInstance(); //force pre-computed color tables initialization

    int result = 0;

    //command line processing
	if (commandLine)
	{
        result = ccCommandLineParser::Parse(argc,argv);
	}
	else
	{
        //main window init.
        MainWindow::TheInstance()->show();
        QApplication::processEvents();

        if (argc>0)
        {
			if (splash)
				splash->close();
            for (int i=1;i<argc;++i)
            {
                //any argument is assumed to be a filename --> we try to load it
                MainWindow::AddToDB(argv[i],UNKNOWN_FILE);
            }
        }
        else if (splash)
        {
            //we want the splash screen to be visible a minimum amount of time (1 s.)
            while((clock() - start_time) < CLOCKS_PER_SEC)
                splash->raise();
            splash->close();
        }

		if (splash)
		{
			delete splash;
			splash=0;
		}

        //let's rock!
        try
        {
            result = app.exec();
        }
        catch(...)
        {
            QMessageBox::warning(0, "CC crashed!","Hum, it seems that CC has crashed... Sorry about that :)");
        }
	}

    MainWindow::DestroyInstance();

    ccGui::ReleaseInstance();
	ccNormalVectors::ReleaseUniqueInstance();
    ccColorTablesManager::ReleaseUniqueInstance();
	ccConsole::ReleaseInstance();

#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
	unsigned alive = CCShareable::GetAliveCount();
	if (alive>1)
	{
		printf("Error: some shared objects (%i) have not been released on program end!",alive);
		system("PAUSE");
	}
#endif

    return result;
}
