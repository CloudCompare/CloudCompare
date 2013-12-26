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

#include "ccProgressDialog.h"

//Qt
#include <QApplication>
#include <QPushButton>
#include <QProgressBar>

ccProgressDialog::ccProgressDialog(bool showCancelButton,
								   QWidget *parent/*=0*/,
								   Qt::WindowFlags flags/*=Qt::SubWindow|Qt::Popup*/)
	: QProgressDialog(parent,flags)
    , m_currentValue(0)
	, m_lastValue(-1)
	, m_timer(this)
	, m_refreshInterval(1)
{
    setAutoClose(true);
	setWindowModality(Qt::ApplicationModal); //not compatible with Qt::QueuedConnection?!

    setRange(0,100);
    setMinimumDuration(0);

	QPushButton* cancelButton = 0;
    if (showCancelButton)
	{
		cancelButton = new QPushButton("Cancel");
		cancelButton->setDefault(false);
		cancelButton->setFocusPolicy(Qt::NoFocus);
	}
	setCancelButton(cancelButton);

	connect(&m_timer,	SIGNAL(timeout()),			this,	SLOT(refresh())/*, Qt::DirectConnection*/);
}

void ccProgressDialog::reset()
{
    QProgressDialog::reset();

    setValue(0);
    m_currentValue = 0;
	m_lastValue = -1;
    QApplication::processEvents();
}

void ccProgressDialog::refresh()
{
	if (m_mutex.tryLock())
	{
		if (m_lastValue == m_currentValue)
		{
			m_mutex.unlock();
		}
		else
		{
			int value = m_lastValue = m_currentValue;
			m_mutex.unlock();

			setValue(value);
			//repaint();
		}
	}
}

//Thread-safe!
void ccProgressDialog::update(float percent)
{
    int value = static_cast<int>(percent);

	m_mutex.lock();
    if (value != m_currentValue)
    {
		m_currentValue = value;

		//every now and then we let the dialog (and more generally the GUI) re-display itself and process events!
		bool refresh = (abs(m_lastValue-m_currentValue) >= m_refreshInterval);
		m_mutex.unlock();

		if (refresh)
			QApplication::processEvents();
    }
	else
	{
		m_mutex.unlock();
	}
}

void ccProgressDialog::setMinRefreshInterval(int i)
{
	m_mutex.lock();
	m_refreshInterval = std::max(1,i);
	m_mutex.unlock();
}

void ccProgressDialog::setMethodTitle(const char* methodTitle)
{
    setWindowTitle(QString(methodTitle));
}

void ccProgressDialog::setInfo(const char* infoStr)
{
    setLabelText(infoStr);
    if (isVisible())
        QApplication::processEvents();
}

bool ccProgressDialog::isCancelRequested()
{
    return wasCanceled();
}

void ccProgressDialog::start()
{
    show();
	m_timer.start(0);
}

void ccProgressDialog::stop()
{
	m_timer.stop();
    hide();
}
