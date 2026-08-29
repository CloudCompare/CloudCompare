// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccProgressDialog.h"

// Qt
#include <QCoreApplication>
#include <QPushButton>
#include <QThread>

ccProgressDialog::ccProgressDialog(bool     showCancelButton,
                                   QWidget* parent /*=nullptr*/)
    : QProgressDialog(parent)
    , m_currentValue(0)
    , m_lastRefreshValue(-1)
{
	// Make sure the dialog doesn't steal focus
	setAttribute(Qt::WA_ShowWithoutActivating);
	setWindowFlag(Qt::WindowDoesNotAcceptFocus);

	setAutoClose(true);

	resize(400, 200);
	setRange(0, 100);
	setMinimumWidth(400);

	QPushButton* cancelButton = nullptr;
	if (showCancelButton)
	{
		cancelButton = new QPushButton("Cancel");
		cancelButton->setDefault(false);
		cancelButton->setFocusPolicy(Qt::NoFocus);
	}
	setCancelButton(cancelButton);
}

void ccProgressDialog::refresh()
{
	int value = m_currentValue;
	if (m_lastRefreshValue != value)
	{
		m_lastRefreshValue = value;
		setValue(value); // See Qt doc: if the progress dialog is modal, setValue() calls QApplication::processEvents()
	}
}

void ccProgressDialog::update(float percent)
{
	// thread-safe
	int value = static_cast<int>(percent);
	if (value != m_currentValue)
	{
		m_currentValue = value;
		if (QThread::currentThread() == thread())
		{
			// called from the GUI thread: refresh directly, and let the event loop
			// breathe so that the dialog is actually repainted
			refresh();
			QCoreApplication::processEvents();
		}
		else
		{
			// called from a worker thread: the refresh has to happen in the GUI thread
			QTimer::singleShot(0, this, [this]()
			                   { refresh(); });
		}
	}
}

void ccProgressDialog::setMethodTitle(QString methodTitle)
{
	if (QThread::currentThread() == thread())
	{
		setWindowTitle(methodTitle);
		QCoreApplication::processEvents();
	}
	else
	{
		QTimer::singleShot(0, this, [this, methodTitle]()
		                   { setWindowTitle(methodTitle); });
	}
}

void ccProgressDialog::setInfo(QString infoStr)
{
	if (QThread::currentThread() == thread())
	{
		setLabelText(infoStr);
		if (isVisible())
		{
			QProgressDialog::update();
			QCoreApplication::processEvents();
		}
	}
	else
	{
		QTimer::singleShot(0, this, [this, infoStr]()
		                   { setLabelText(infoStr); });
	}
}

void ccProgressDialog::start()
{
	// thread-safe: algorithms may call this from a worker thread, and a widget can
	// only be shown from the thread it lives in
	m_lastRefreshValue = -1;
	if (QThread::currentThread() == thread())
	{
		show();
		QCoreApplication::processEvents();
	}
	else
	{
		QTimer::singleShot(0, this, [this]()
		                   { show(); });
	}
}

void ccProgressDialog::stop()
{
	// thread-safe, see start()
	if (QThread::currentThread() == thread())
	{
		hide();
		QCoreApplication::processEvents();
	}
	else
	{
		QTimer::singleShot(0, this, [this]()
		                   { hide(); });
	}
}
