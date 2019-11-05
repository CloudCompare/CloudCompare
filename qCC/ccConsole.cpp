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

#include "ccConsole.h"

//Local
#include "ccPersistentSettings.h"
#include "mainwindow.h"

//qCC_db
#include <ccSingleton.h>

//Qt
#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QKeyEvent>
#include <QMessageBox>
#include <QSettings>
#include <QTextStream>
#include <QThread>
#include <QTime>

//system
#include <cassert>
#ifdef QT_DEBUG
#include <iostream>
#endif

/***************
 *** Globals ***
 ***************/

//unique console instance
static ccSingleton<ccConsole> s_console;

bool ccConsole::s_showQtMessagesInConsole = false;
bool ccConsole::s_redirectToStdOut = false;


// ccCustomQListWidget
ccCustomQListWidget::ccCustomQListWidget(QWidget *parent)
	: QListWidget(parent)
{
}

void ccCustomQListWidget::keyPressEvent(QKeyEvent *event)
{
	if (event->matches(QKeySequence::Copy))
	{
		int itemsCount = count();
		QStringList strings;
		for (int i = 0; i < itemsCount; ++i)
		{
			if (item(i)->isSelected())
			{
				strings << item(i)->text();
			}
		}
		
		QApplication::clipboard()->setText(strings.join("\n"));
	}
	else
	{
		QListWidget::keyPressEvent(event);
	}
}


// ccConsole
ccConsole* ccConsole::TheInstance(bool autoInit/*=true*/)
{
	if (!s_console.instance && autoInit)
	{
		s_console.instance = new ccConsole;
		ccLog::RegisterInstance(s_console.instance);
	}

	return s_console.instance;
}

void ccConsole::ReleaseInstance(bool flush/*=true*/)
{
	if (flush && s_console.instance)
	{
		//DGM: just in case some messages are still in the queue
		s_console.instance->refresh();
	}
	ccLog::RegisterInstance(nullptr);
	s_console.release();
}

ccConsole::ccConsole()
	: m_textDisplay(nullptr)
	, m_parentWidget(nullptr)
	, m_parentWindow(nullptr)
	, m_logStream(nullptr)
{
}

ccConsole::~ccConsole()
{
	setLogFile(QString()); //to close/delete any active stream
}

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
#ifndef QT_DEBUG
	if (!ccConsole::QtMessagesEnabled())
	{
		return;
	}

	if (type == QtDebugMsg)
	{
		return;
	}
#endif

	QString message = QString("[%1] ").arg(context.function) + msg; // QString("%1 (%1:%1, %1)").arg(msg).arg(context.file).arg(context.line).arg(context.function);

	//in this function, you can write the message to any stream!
	switch (type)
	{
	case QtDebugMsg:
		ccLog::PrintDebug(msg);
		break;
	case QtWarningMsg:
		message.prepend("[Qt WARNING] ");
		ccLog::Warning(message);
		break;
	case QtCriticalMsg:
		message.prepend("[Qt CRITICAL] ");
		ccLog::Warning(message);
		break;
	case QtFatalMsg:
		message.prepend("[Qt FATAL] ");
		ccLog::Warning(message);
		break;
	case QtInfoMsg:
		message.prepend("[Qt INFO] ");
		ccLog::Warning(message);
		break;
	}
	
#ifdef QT_DEBUG
	// Also send the message to the console so we can look at the output when CC has quit
	//	(in Qt Creator's Application Output for example)
	switch (type)
	{
		case QtDebugMsg:
		case QtWarningMsg:
		case QtInfoMsg:
			std::cout << message.toStdString() << std::endl;
			break;
			
		case QtCriticalMsg:
		case QtFatalMsg:
			std::cerr << message.toStdString() << std::endl;
			break;
	}
	
#endif
}

void ccConsole::EnableQtMessages(bool state)
{
	s_showQtMessagesInConsole = state;

	//save to persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::Console());
	settings.setValue("QtMessagesEnabled", s_showQtMessagesInConsole);
	settings.endGroup();
}

void ccConsole::Init(	QListWidget* textDisplay/*=0*/,
						QWidget* parentWidget/*=0*/,
						MainWindow* parentWindow/*=0*/,
						bool redirectToStdOut/*=false*/)
{
	//should be called only once!
	if (s_console.instance)
	{
		assert(false);
		return;
	}
	
	s_console.instance = new ccConsole;
	s_console.instance->m_textDisplay = textDisplay;
	s_console.instance->m_parentWidget = parentWidget;
	s_console.instance->m_parentWindow = parentWindow;
	s_redirectToStdOut = redirectToStdOut;
	//auto-start
	if (textDisplay)
	{
		//load from persistent settings
		QSettings settings;
		settings.beginGroup(ccPS::Console());
		s_showQtMessagesInConsole = settings.value("QtMessagesEnabled", false).toBool();
		settings.endGroup();

		//install : set the callback for Qt messages
		qInstallMessageHandler(myMessageOutput);

		s_console.instance->setAutoRefresh(true);
	}
	ccLog::RegisterInstance(s_console.instance);
}

void ccConsole::setAutoRefresh(bool state)
{
	if (state)
	{
		connect(&m_timer, &QTimer::timeout, this, &ccConsole::refresh);
		m_timer.start(1000);
	}
	else
	{
		m_timer.stop();
		disconnect(&m_timer, &QTimer::timeout, this, &ccConsole::refresh);
	}
}

void ccConsole::refresh()
{
	m_mutex.lock();
	
	if ((m_textDisplay || m_logStream) && !m_queue.isEmpty())
	{
		for (QVector<ConsoleItemType>::const_iterator it=m_queue.constBegin(); it!=m_queue.constEnd(); ++it)
		{
			 //it->second = message severity
			bool debugMessage = (it->second & LOG_DEBUG);
#ifndef QT_DEBUG
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
#ifdef QT_DEBUG
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

void ccConsole::logMessage(const QString& message, int level)
{
#ifndef QT_DEBUG
	//skip debug messages in release mode
	if (level & LOG_DEBUG)
	{
		return;
	}
#endif

	QString formatedMessage = QStringLiteral("[") + QTime::currentTime().toString() + QStringLiteral("] ") + message;
	if (s_redirectToStdOut)
	{
		printf("%s\n", qPrintable(message));
	}
	if (m_textDisplay || m_logStream)
	{
		m_mutex.lock();
		m_queue.push_back(ConsoleItemType(formatedMessage,level));
		m_mutex.unlock();
	}
#ifdef QT_DEBUG
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

bool ccConsole::setLogFile(const QString& filename)
{
	//close previous stream (if any)
	if (m_logStream)
	{
		m_mutex.lock();
		delete m_logStream;
		m_logStream = nullptr;
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
