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
#include <CCPlatform.h>
#include <QCoreApplication>
#include <QProgressBar>
#include <QPushButton>
#if defined(CC_WINDOWS)
#include <QThread>
#include <windows.h>
#endif

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
#if defined(CC_WINDOWS)
		if (QThread::currentThread() && QThread::currentThread()->isMainThread())
		{
			refresh();
			::Sleep(1);
		}
		else
#endif
		{
			QTimer::singleShot(0, this, [this]()
			                   { refresh(); });
			QCoreApplication::processEvents();
		}
	}
}

void ccProgressDialog::setMethodTitle(QString methodTitle)
{
	setWindowTitle(methodTitle);
}

void ccProgressDialog::setInfo(QString infoStr)
{
	setLabelText(infoStr);
	if (isVisible())
	{
		QProgressDialog::update();
		QCoreApplication::processEvents();
	}
}

void ccProgressDialog::start()
{
	m_lastRefreshValue = -1;
	show();
	QCoreApplication::processEvents();
}

void ccProgressDialog::stop()
{
	hide();
	QCoreApplication::processEvents();
}
