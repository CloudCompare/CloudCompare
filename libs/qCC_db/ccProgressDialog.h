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

#ifndef CC_PROGRESS_DIALOG_HEADER
#define CC_PROGRESS_DIALOG_HEADER

//Local
#include "qCC_db.h"

//Qt
#include <QProgressDialog>
#include <QAtomicInt>
#include <QTimer>

//CCLib
#include <GenericProgressCallback.h>

//! Graphical progress indicator (thread-safe)
/** Implements the GenericProgressCallback interface, in order
	to be passed to the CCLib algorithms (check the
	CCLib documentation for more information about the
	inherited methods).
**/
class QCC_DB_LIB_API ccProgressDialog : public QProgressDialog, public CCLib::GenericProgressCallback
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
	ccProgressDialog(	bool cancelButton = false,
						QWidget *parent = 0 );

	//! Destructor (virtual)
	virtual ~ccProgressDialog() {}

	//inherited method
	virtual void update(float percent) override;
	inline virtual void setMethodTitle(const char* methodTitle) override { setMethodTitle(QString(methodTitle)); }
	inline virtual void setInfo(const char* infoStr) override { setInfo(QString(infoStr)); }
	inline virtual bool isCancelRequested() override { return wasCanceled(); }
	virtual void start() override;
	virtual void stop() override;

	//! setMethodTitle with a QString as argument
	virtual void setMethodTitle(QString methodTitle);
	//! setInfo with a QString as argument
	virtual void setInfo(QString infoStr);

protected slots:

	//! Refreshes the progress
	/** Should only be called in the main Qt thread!
		This slot is automatically called by 'update' (in Qt::QueuedConnection mode).
	**/
	void refresh();

signals:

	//! Schedules a call to refresh
	void scheduleRefresh();

protected:

	//! Current progress value (percent)
	QAtomicInt m_currentValue;

	//! Last displayed progress value (percent)
	QAtomicInt m_lastRefreshValue;
};

#endif //CC_PROGRESS_DIALOG_HEADER
