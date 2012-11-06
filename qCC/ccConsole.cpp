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

#include "ccConsole.h"

//Qt
#include <QListWidget>
#include <QMessageBox>
#include <QApplication>
#include <QColor>
#include <QTime>

//system
#include <assert.h>

#if !defined(_WIN32) && !defined(WIN32)
#define _vsnprintf vsnprintf
#endif

/***************
 *** Globals ***
 ***************/

//buffer for formated string generation
static char s_buffer[4096];

//unique console instance
static ccConsole* s_console = 0;

ccConsole* ccConsole::TheInstance()
{
    if (!s_console)
	{
        s_console = new ccConsole();
		ccLog::RegisterInstance(s_console);
	}

    return s_console;
}

void ccConsole::ReleaseInstance()
{
    if (s_console)
	{
		ccLog::RegisterInstance(0);
        delete s_console;
    s_console=0;
	}
}

ccConsole::ccConsole()
	: m_textDisplay(0)
	, m_parentWidget(0)
{
}

void ccConsole::Init(QListWidget* textDisplay/*=0*/, QWidget* parentWidget/*=0*/)
{
    ccConsole* console = TheInstance();
    assert(console);
    console->m_textDisplay = textDisplay;
	console->m_parentWidget = parentWidget;
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
		for (QVector<ConsoleItemType>::const_iterator it = m_queue.begin();it!=m_queue.end();++it)
		{
			QListWidgetItem* item = new QListWidgetItem(it->first);
			item->setForeground(QBrush(it->second));
			m_textDisplay->addItem(item);
		}

		m_textDisplay->scrollToBottom();
	}

	m_queue.clear();

	m_mutex.unlock();
}

void ccConsole::displayMessage(const QString& message, MessageLevel level)
{
	Qt::GlobalColor color = Qt::black;
	switch(level)
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
		break;
	case LOG_ERROR:	
#ifdef _DEBUG
	case LOG_ERROR_DEBUG:
#endif
		color = Qt::red;
		break;
#ifndef _DEBUG
	default:
		return;
#endif
	}

	QString formatedMessage = QString("[")+QTime::currentTime().toString()+QString("] ")+message;

    if (m_textDisplay)
    {
		m_mutex.lock();
		m_queue.push_back(ConsoleItemType(formatedMessage,color));
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
		QMessageBox::warning(m_parentWidget, "Error", message);
	}
}
