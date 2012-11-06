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
#include "testfilter.h"

//Qt
#include <QRunnable>
#include <QObject>
#include <QThreadPool>
#include <QFuture>
#include <QtConcurrentRun>
#include <QApplication>
#include <QThread>

testfilter::testfilter()
: BaseFilter(FilterDescription("Test Filter", "Test Filter", "TEST!", ":/toolbar/PclUtils/icons/pcl.png", false) )
, m_app()
{
	m_thread_pool = QThreadPool::globalInstance();
}

int testfilter::openDialog()
{
	return 1;
}

int testfilter::compute()
{
	QFuture<void> future = QtConcurrent::run(&(*this->m_app), &Test::run);

	return 1;
}
