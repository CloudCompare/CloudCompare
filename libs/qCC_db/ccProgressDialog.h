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
//$Rev:: 2280                                                              $
//$LastChangedDate:: 2012-10-19 00:12:14 +0200 (ven., 19 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_PROGRESS_DIALOG_HEADER
#define CC_PROGRESS_DIALOG_HEADER

//Qt
#include <QProgressDialog>
#include <QMutex>
#include <QTimer>

//CCLib
#include <GenericProgressCallback.h>

//! Graphical progress indicator (thread-safe)
/** Implements the GenericProgressCallback interface, in order
	to be passed to the CCLib algorithms (check the
    CCLib documentation for more information about the
	inherited methods).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccProgressDialog : public QProgressDialog, public CCLib::GenericProgressCallback
#else
class ccProgressDialog : public QProgressDialog, public CCLib::GenericProgressCallback
#endif
{

Q_OBJECT

public:

	//! Default constructor
	/** By default, a cancel button is always displayed on the
		progress interface. It is only possible to activate or
		deactivate this button. Sadly, the fact that this button is
		activated doesn't mean it will be possible to stop the ongoing
		process: it depends only on the client algorithm implementation.
		\param cancelButton activates or deactivates the cancel button
		\param parent parent widget
	**/
	ccProgressDialog(bool cancelButton = false,
						QWidget *parent = 0,
						Qt::WindowFlags flags = Qt::SubWindow | Qt::Popup);

	//! Destructor (virtual)
	virtual ~ccProgressDialog() {}

	//inherited method
	virtual void reset();
	virtual void update(float percent);
	virtual void setMethodTitle(const char* methodTitle);
	virtual void setInfo(const char* infoStr);
	virtual bool isCancelRequested();
	virtual void start();
	virtual void stop();

public slots:

	//! Refreshes widget
	/** Should only be called in the main Qt thread!
		--> Job automatically done by 'm_timer'
	**/
	void refresh();

protected:

    //! Current progress value (percent)
	int m_currentValue;

    //! Last displayed progress value (percent)
	int m_lastValue;

	//! Mutex for concurrent access
	QMutex m_mutex;

	//! Timer for automatic update
	QTimer m_timer;
};

#endif
