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

#include "ccConsole.h"

//Local
#include "mainwindow.h"

//qCC_db
#include <ccSingleton.h>

//Qt
#include <QListWidget>
#include <QMessageBox>
#include <QApplication>
#include <QColor>
#include <QTime>
#include <QThread>
#include <QTextStream>

//system
#include <assert.h>

/***************
 *** Globals ***
 ***************/

//unique console instance
static ccSingleton<ccConsole> s_console;

ccConsole* ccConsole::TheInstance()
{
	if (!s_console.instance)
	{
		s_console.instance = new ccConsole();
		ccLog::RegisterInstance(s_console.instance);
	}

	return s_console.instance;
}

void ccConsole::ReleaseInstance()
{
	if (s_console.instance)
	{
		//DGM: just in case some messages are still in the queue
		s_console.instance->refresh();
	}
	s_console.release();
	ccLog::RegisterInstance(0);
}

ccConsole::ccConsole()
	: m_textDisplay(0)
	, m_parentWidget(0)
	, m_parentWindow(0)
	, m_logStream(0)
{
}

ccConsole::~ccConsole()
{
	setLogFile(QString()); //to close/delete any active stream
}

void ccConsole::Init(	QListWidget* textDisplay/*=0*/,
						QWidget* parentWidget/*=0*/,
						MainWindow* parentWindow/*=0*/)
{
	ccConsole* console = TheInstance();
	assert(console);

	console->m_textDisplay = textDisplay;
	console->m_parentWidget = parentWidget;
	console->m_parentWindow = parentWindow;

	//auto-start
	if (textDisplay)
	{
		console->setAutoRefresh(true);
	}
}

void ccConsole::setAutoRefresh(bool state)
{
	if (state)
	{
		QObject::connect(&m_timer, SIGNAL(timeout()), this, SLOT(refresh()));
		m_timer.start(1000);
	}
	else
	{
		m_timer.stop();
		QObject::disconnect(&m_timer, SIGNAL(timeout()), this, SLOT(refresh()));
	}
}

void ccConsole::refresh()
{
	m_mutex.lock();
	
	if ((m_textDisplay || m_logStream) && !m_queue.isEmpty())
	{
		for (QVector<ConsoleItemType>::const_iterator it=m_queue.begin(); it!=m_queue.end(); ++it)
		{
			 //it->second = message severity
			bool debugMessage = (it->second & LOG_DEBUG);
#ifndef _DEBUG
			//skip debug message in release mode
			if (debugMessage)
				continue;
#endif

			//destination: log file
			if (m_logStream)
			{
				*m_logStream << it->first << endl;
			}

			//destination: console widget
			if (m_textDisplay)
			{
				//it->first = message text
				QListWidgetItem* item = new QListWidgetItem(it->first);

				//set color based on the message severity
				//Error
				if (it->second & LOG_ERROR)
				{
					item->setForeground(Qt::red);
				}
				//Warning
				else if (it->second & LOG_WARNING)
				{
					item->setForeground(Qt::darkRed);
					//we also force the console visibility if a warning message arrives!
					if (m_parentWindow)
						m_parentWindow->forceConsoleDisplay();
				}
				//Standard
				else
				{
#ifdef _DEBUG
					if (debugMessage)
						item->setForeground(Qt::blue);
					else
#endif
						item->setForeground(Qt::black);
				}

				m_textDisplay->addItem(item);
			}
		}

		if (m_logStream)
			m_logFile.flush();

		if (m_textDisplay)
			m_textDisplay->scrollToBottom();
	}

	m_queue.clear();

	m_mutex.unlock();
}

void ccConsole::displayMessage(const QString& message, int level)
{
#ifndef _DEBUG
	//skip debug messages in release mode
	if (level & LOG_DEBUG)
		return;
#endif

	QString formatedMessage = QString("[") + QTime::currentTime().toString() + QString("] ") + message;

	if (m_textDisplay || m_logStream)
	{
		m_mutex.lock();
		m_queue.push_back(ConsoleItemType(formatedMessage,level));
		m_mutex.unlock();
	}
#ifdef _DEBUG
	else
	{
		//Error
		if (level & LOG_ERROR)
		{
			if (level & LOG_DEBUG)
				printf("ERR-DBG: ");
			else
				printf("ERR: ");
		}
		//Warning
		else if (level & LOG_WARNING)
		{
			if (level & LOG_DEBUG)
				printf("WARN-DBG: ");
			else
				printf("WARN: ");
		}
		//Standard
		else
		{
			if (level & LOG_DEBUG)
				printf("MSG-DBG: ");
			else
				printf("MSG: ");
		}
		printf(" %s\n",qPrintable(formatedMessage));
	}
#endif

	//we display the error messages in a popup dialog
	if (	(level & LOG_ERROR)
		&&	qApp
		&&	m_parentWidget
		&&	QThread::currentThread() == qApp->thread()
		)
	{
		QMessageBox::warning(m_parentWidget, "Error", message);
	}
}

bool ccConsole::setLogFile(QString filename)
{
	//close previous stream (if any)
	if (m_logStream)
	{
		m_mutex.lock();
		delete m_logStream;
		m_logStream = 0;
		m_mutex.unlock();

		if (m_logFile.isOpen())
		{
			m_logFile.close();
		}
	}
	
	if (!filename.isEmpty())
	{
		m_logFile.setFileName(filename);
		if (!m_logFile.open(QFile::Text| QFile::WriteOnly))
		{
			return Error(QString("[Console] Failed to open/create log file '%1'").arg(filename));
		}

		m_mutex.lock();
		m_logStream = new QTextStream(&m_logFile);
		m_mutex.unlock();
		setAutoRefresh(true);
	}


	return true;
}
