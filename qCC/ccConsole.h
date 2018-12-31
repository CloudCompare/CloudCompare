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

#ifndef CC_CONSOLE_HEADER
#define CC_CONSOLE_HEADER

//qCC_db
#include <ccLog.h>

//Qt
#include <QFile>
#include <QListWidget>
#include <QMutex>
#include <QTimer>

class MainWindow;
class QTextStream;

//! Custom QListWidget to allow for the copy of all selected elements when using CTRL+C
class ccCustomQListWidget : public QListWidget
{
	Q_OBJECT
	
public:
	ccCustomQListWidget(QWidget* parent = nullptr);

protected:
	void keyPressEvent(QKeyEvent *event) override;
};

//! Console
class ccConsole : public QObject, public ccLog
{
	Q_OBJECT

public:

	//! Destructor
	~ccConsole() override;

	//! Inits console (and optionaly associates it with a text output widget)
	/** WARNING: in release mode, no message will be output if no 'textDisplay'
		widget is defined. Moreover, error messages will only appear in a
		(blocking) QMessageBox if a 'parentWidget' widget is defined.
		In debug mode, all message are sent to system console (with 'printf').
		\param textDisplay text output widget (optional)
		\param parentWidget parent widget (optional)
		\param parentWindow parent window (if any - optional)
	**/
	static void Init(	QListWidget* textDisplay = nullptr,
						QWidget* parentWidget = nullptr,
						MainWindow* parentWindow = nullptr);

	//! Returns the (unique) static instance
	/** \param autoInit automatically initialize the console instance (with no widget!) if not done already
	**/
	static ccConsole* TheInstance(bool autoInit = true);

	//! Releases unique instance
	static void ReleaseInstance(bool flush = true);

	//! Sets auto-refresh state
	void setAutoRefresh(bool state);

	//! Sets log file
	bool setLogFile(const QString& filename);

	//! Whether to show Qt messages (qDebug / qWarning / etc.) in Console
	static void EnableQtMessages(bool state);

	//! Returns whether to show Qt messages (qDebug / qWarning / etc.) in Console or not
	static bool QtMessagesEnabled() { return s_showQtMessagesInConsole; }

	//! Returns the parent widget (if any)
	inline QWidget* parentWidget() { return m_parentWidget; }

public slots:

	//! Refreshes console (display all messages still in queue)
	void refresh();

protected:

	//! Default constructor
	/** Constructor is protected to avoid using this object as a non static class.
	**/
	ccConsole();

	//inherited from ccLog
	void logMessage(const QString& message, int level) override;

	//! Associated text display widget
	QListWidget* m_textDisplay;

	//! Parent widget
	QWidget* m_parentWidget;

	//! Parent window (if any)
	MainWindow* m_parentWindow;

	//! Mutex for concurrent thread access to console
	QMutex m_mutex;

	//! Queue element type (message + color)
	using ConsoleItemType = QPair<QString,int>;

	//! Queue for incoming messages
	QVector<ConsoleItemType> m_queue;

	//! Timer for auto-refresh
	QTimer m_timer;

	//! Log file
	QFile m_logFile;
	//! Log file stream
	QTextStream* m_logStream;

	//! Whether to show Qt messages (qDebug / qWarning / etc.) in Console
	static bool s_showQtMessagesInConsole;
};

#endif
