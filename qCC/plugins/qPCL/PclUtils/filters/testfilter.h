//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef TESTFILTER_H
#define TESTFILTER_H

#include "BaseFilter.h"

//qCC_db
#include <ccPlatform.h>

//Qt
#include <QThreadPool>

//System
#include <iostream>

#if !defined(CC_WINDOWS)
#include <time.h>
#include <unistd.h>
#endif

class Test:  public QObject
{
private:
    Q_OBJECT
public:
    Test()
    {

    }

   void run()
    {
        for (int i = 0; i < 10; i++)
        {
#ifndef WIN32
            usleep(1);
#endif
            std::cout << "echo: " << i << std::endl;
        }
    }

public slots:
    void hasfinised()
    {
        std::cout << "thread finished" << std::endl;
    }
};

//class Test2: public Test
//{
//private:
//    Q_OBJECT
//public:

//    Test2(){}


//    void run()
//    {
//        for (int i = 0; i < 5; i++)
//        {
//            sleep(1);
//        }
//    }
//};


class testfilter: public BaseFilter
{
    Q_OBJECT

public:
    testfilter();

	//inherited from BaseFilter
	virtual int compute();
protected:

	//inherited from BaseFilter
    virtual int openInputDialog();

    Test* m_app;
    QThreadPool* m_thread_pool;

};

#endif // TESTFILTER_H
