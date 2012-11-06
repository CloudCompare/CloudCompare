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

#ifndef CC_CONSOLE_HEADER
#define CC_CONSOLE_HEADER

//system
#include <stdio.h>
#include <string.h>

//qCC_db
#include <ccLog.h>

//Qt
#include <QObject>
#include <QMutex>
#include <QStringList>
#include <QTimer>
#include <QVector>
#include <QPair>

class QListWidget;
class QWidget;

//! Console
class ccConsole : public QObject, public ccLog
{

	Q_OBJECT

public:

    //! Inits console (and optionaly associates it with a text output widget)
    /** WARNING: in release mode, no message will be output if no 'textDisplay'
		widget is defined. Moreover, error messages will only appear in a
		(blocking) QMessageBox if a 'parentWidget' widget is defined.
		In debug mode, all message are sent to system console (with 'printf').
		\param textDisplay text output widget (optional)
		\param parentWidget parent widget (optional)
    **/
	static void Init(QListWidget* textDisplay=0, QWidget* parentWidget=0);

    //! Returns the (unique) static instance
	static ccConsole* TheInstance();

    //! Releases unique instance
    static void ReleaseInstance();

	//! Sets auto-refresh state
	void setAutoRefresh(bool state);

public slots:

	//! Refreshes console (display all messages still in queue)
	void refresh();

protected:

    //! Default constructor
    /** Constructor is protected to avoid using this object as a
        non static class.
    **/
	ccConsole();

    //inherited from ccLog
    virtual void displayMessage(const QString& message, MessageLevel level);

    //! Associated text display widget
	QListWidget* m_textDisplay;

	//! Corresponding widget
	QWidget* m_parentWidget;

	//! Mutex for concurrent thread access to console
	QMutex m_mutex;

	//! Queue element type (message + color)
	typedef QPair<QString,Qt::GlobalColor> ConsoleItemType;

	//! Queue for incoming messages
	QVector<ConsoleItemType> m_queue;

	//! Timer for auto-refresh
	QTimer m_timer;
};

#endif
