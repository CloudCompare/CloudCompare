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
//$Rev:: 2266                                                              $
//$LastChangedDate:: 2012-10-15 00:07:12 +0200 (lun., 15 oct. 2012)        $
//**************************************************************************
//

#include "ccProgressDialog.h"

#include <QApplication>

ccProgressDialog::ccProgressDialog(bool cancelButton,
								   QWidget *parent/*=0*/,
								   Qt::WindowFlags flags/*=Qt::SubWindow|Qt::Popup*/)
	: QProgressDialog(parent,flags)
    , m_currentValue(0)
	, m_lastValue(-1)
	, m_timer(this)
{
    setAutoClose(true);
	setWindowModality(Qt::ApplicationModal); //not compatible with Qt::QueuedConnection?!

    setRange(0,100);
    setMinimumDuration(0);

    if (!cancelButton)
        setCancelButton(0);

	//QObject::connect(this, SIGNAL(doUpdate(int)), this, SLOT(setValue(int)), Qt::QueuedConnection); 
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(refresh()));
}

void ccProgressDialog::reset()
{
    QProgressDialog::reset();

    setValue(0);
    m_currentValue=0;
	m_lastValue=-1;
    QApplication::processEvents();
}

void ccProgressDialog::refresh()
{
	if (m_lastValue != m_currentValue)
	{
		setValue(m_currentValue);
		m_lastValue=m_currentValue;
	}
}

//Thread-safe!
void ccProgressDialog::update(float percent)
{
    int value = (int)percent;

	m_mutex.lock();
    if (value != m_currentValue)
    {
		m_currentValue = value;

		//every 5% (or more) we let the dialog (and more generally the GUI) re-display itself and treat events!
		bool refresh=(abs(m_lastValue-m_currentValue)>4);
		m_mutex.unlock();

		if (refresh)
            QApplication::processEvents();
    }
	else
	{
		m_mutex.unlock();
	}
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
