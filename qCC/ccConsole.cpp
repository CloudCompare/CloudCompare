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
	s_console.release();
	ccLog::RegisterInstance(0);
}

ccConsole::ccConsole()
	: m_textDisplay(0)
	, m_parentWidget(0)
	, m_parentWindow(0)
{
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
	if (textDisplay)
		//auto-start
		console->setAutoRefresh(true);
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
	
	if (m_textDisplay && !m_queue.isEmpty())
	{
		for (QVector<ConsoleItemType>::const_iterator it = m_queue.begin(); it!=m_queue.end(); ++it)
		{
			QListWidgetItem* item = new QListWidgetItem(it->first); //first = message text

			//set color based on the message severity
			Qt::GlobalColor color = Qt::black;
			switch (it->second) //second = message severity
			{
			case LOG_STANDARD:
				//color = Qt::black;
				break;
#ifdef _DEBUG
			case LOG_STANDARD_DEBUG:
#endif
				color = Qt::blue;
				break;
			case LOG_WARNING:	
#ifdef _DEBUG
			case LOG_WARNING_DEBUG:
#endif
				color = Qt::darkRed;
				//we also force the console visibility if a warning message arrives!
				if (m_parentWindow)
					m_parentWindow->forceConsoleDisplay();
				break;
			case LOG_ERROR:	
#ifdef _DEBUG
			case LOG_ERROR_DEBUG:
#endif
				color = Qt::red;
				break;
			default:
#ifndef _DEBUG
				//skip debug message in debug mode
				continue;
#else
				//we shoudn't fall here in debug mode!
				assert(false);
				break;
#endif
			}
			item->setForeground(color);

			m_textDisplay->addItem(item);
		}

		m_textDisplay->scrollToBottom();
	}

	m_queue.clear();

	m_mutex.unlock();
}

void ccConsole::displayMessage(const QString& message, MessageLevel level)
{
	QString formatedMessage = QString("[")+QTime::currentTime().toString()+QString("] ")+message;

	if (m_textDisplay)
	{
		m_mutex.lock();
		m_queue.push_back(ConsoleItemType(formatedMessage,level));
		m_mutex.unlock();
	}
#ifdef _DEBUG
	else
	{
		switch(level)
		{
		case LOG_STANDARD:
			printf("MSG: ");
			break;
		case LOG_STANDARD_DEBUG:
			printf("MSG-DBG: ");
			break;
		case LOG_WARNING:	
			printf("WARNING: ");
			break;
		case LOG_WARNING_DEBUG:
			printf("WARNING-DBG: ");
			break;
		case LOG_ERROR:	
			printf("ERROR: ");
			break;
		case LOG_ERROR_DEBUG:
			printf("ERROR-DBG: ");
			break;
		}
		printf(" %s\n",qPrintable(formatedMessage));
	}
#endif

	if (m_parentWidget && (level==LOG_ERROR || level==LOG_ERROR_DEBUG))
	{
		//we display error message in a popup dialog
		if( qApp && QThread::currentThread() == qApp->thread() )
			QMessageBox::warning(m_parentWidget, "Error", message);
	}
}
